## @ingroup Components-Energy-Distributors
# Electronic_Speed_Controller_eta.py
#
# Created:  Jun 2014, E. Botero
# Modified: Jan 2016, T. MacDonald
#           Sep 2021, J. Mangold

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE
import numpy as np

from SUAVE.Components.Energy.Energy_Component import Energy_Component
from scipy.interpolate import LinearNDInterpolator

# ----------------------------------------------------------------------
#  Electronic Speed Controller Class
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Distributors
class Electronic_Speed_Controller_eta_Inverter_Mid_Fid(Energy_Component):
    
    def __defaults__(self):
        """ This sets the default values.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            None
            """         
        

    
    def voltageout(self,conditions,eta):
        """ The voltage out of the electronic speed controller
        
            Assumptions:
            The ESC's output voltage is linearly related to throttle setting
    
            Source:
            N/A
    
            Inputs:
            conditions.propulsion.throttle [0-1] 
            self.inputs.voltage            [volts]
    
            Outputs:
            voltsout                       [volts]
            self.outputs.voltageout        [volts]
    
            Properties Used:
            None
           
        """
        # Unpack, don't modify the throttle
        #eta = (conditions.propulsion.throttle[:,0,None])*1.0
        #eta = conditions.electric_throttle_sum[:,0,None]
        
        # Negative throttle is bad
        #eta[eta<=0.0] = 0.0 # maybe not for battery charging
        
        # Cap the throttle
        #eta[eta>=1.0] = 1.0
        
        voltsin  = self.inputs.voltagein
        voltsout = abs(eta)*voltsin
        
        # Pack the output
        self.outputs.voltageout = voltsout # fix for voltage is always postive, that with negativ current battery charging is possible
        
        return voltsout
    
    def currentin(self,conditions,eta):
        """ The current going into the speed controller
        
            Assumptions:
                The ESC draws current.
            
            Inputs:
                self.inputs.currentout [amps]
               
            Outputs:
                outputs.currentin      [amps]
            
            Properties Used:
                self.efficiency - [0-1] efficiency of the ESC
               
        """
        
        # Unpack, don't modify the throttle
        #eta = (conditions.propulsion.throttle[:,0,None])*1.0
        #eta = conditions.electric_throttle_sum[:, 0, None]
        currentout = self.inputs.currentout

        # Unpack the surrogate
        eta_surrogate = self.eta_surrogate

        torque_surrogate =  abs(self.inputs.torque)/self.torque_input_scale


        if any(torque_surrogate < self.torque_in_min_max[0]) or any(torque_surrogate > self.torque_in_min_max[1]):
            print(f'replacing dcac_torque out of bounds\n{torque_surrogate}')
            torque_surrogate[torque_surrogate < self.torque_in_min_max[0]] = self.torque_in_min_max[0]
            torque_surrogate[torque_surrogate > self.torque_in_min_max[1]] = self.torque_in_min_max[1]

        if any(self.inputs.rpm < self.rpm_out_min_max[0]) or any(self.inputs.rpm > self.rpm_out_min_max[1]):
            print(f'replacing dcac_inputs.rpm out of bounds\n{self.inputs.rpm}')
            self.inputs.rpm[self.inputs.rpm < self.rpm_out_min_max[0]] = self.rpm_out_min_max[0]
            self.inputs.rpm[self.inputs.rpm > self.rpm_out_min_max[1]] = self.rpm_out_min_max[1]


        cond = np.hstack([torque_surrogate, self.inputs.rpm])

        eff = eta_surrogate(cond)



        #eff        = self.efficiency * np.ones_like(currentout)

        currentin = currentout * abs(eta) * eff**(-np.sign(currentout))
        #currentin[eta>0]  = currentout[eta>0]*abs(eta[eta>0])/eff
        
        # Pack
        self.outputs.currentin = currentin
        self.outputs.power_in  = self.outputs.voltageout*currentin
        
        return currentin


    def build_surrogate(self):
        """ Build a surrogate. Multiple options for models are available including:

         Assumptions:
         None

         Source:
         N/A

         Inputs:
         state [state()]

         Outputs:
         self.sfc_surrogate    [fun()]



         Properties Used:
         Defaulted values
        """


        # file name to look for
        file_name = self.input_file

        raw_data = np.genfromtxt(file_name, delimiter=',')

        y = raw_data[0][1:]
        my_data = np.empty((0, 3))

        for i, x in enumerate(raw_data[1:, 0]):
            voltage = np.ones(np.shape(raw_data)[1] - 1) * x
            my_data = np.append(my_data, np.array([voltage, y, raw_data[i+1][1:]]).T, axis=0)

        xy  = my_data[:,:2] # Torque, RPM

        eta = np.transpose(np.atleast_2d(my_data[:, 2]))/100  #Efficiency

        self.torque_input_scale = np.max(xy[:,0])
        self.eta_input_scale = np.max(eta)

        # normalize for better surrogate performance
        xy[:,0] /= self.torque_input_scale


        eta_surrogate = LinearNDInterpolator(xy, eta)

        self.torque_in_min_max = np.array([np.amin(xy[:, 0]), np.amax(xy[:, 0])])
        self.rpm_out_min_max     = np.array([np.amin(xy[:, 1]), np.amax(xy[:, 1])])

        # Save the output
        self.eta_surrogate    = eta_surrogate
