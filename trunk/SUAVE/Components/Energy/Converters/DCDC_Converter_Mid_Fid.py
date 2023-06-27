## @ingroup Components-Energy-Converters
# DCDC_Converter_Lo_Fid.py
#
# Created:  Feb 2022, B. Zaghari and J. Mangold
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np


# suave imports
import SUAVE

# package imports
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from scipy.interpolate import LinearNDInterpolator

# ----------------------------------------------------------------------
#  Converter DC/DC Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class DCDC_Converter_Mid_Fid(Energy_Component):
    """This is a low-fidelity DCDC converter component.
    
    Assumptions:
    None

    Source:
    None
    """      
    def __defaults__(self):
        """This sets the default values for the component to function.

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
        self.bus_voltage        = 0.0
        self.inputs.eta         = 1.0


    def covert_mid(self,conditions):
        """Calculates the coverters's XXX

        Assumptions:
    
        Source:
        N/A
    
        Inputs: 
        self.
            inputs.currentOut       [A]
            inputs.voltageIn       [V]
    
        Outputs:
        self.outputs.
            outputs.powerIn         [W]
            outputs.currentIn       [A]
           
        Properties Used:
        self.
            bus_voltage             [V]

        """          

        dcdc_voltageOut = self.bus_voltage

        dcdc_currentOut = self.inputs.currentOut
        dcdc_voltageIn = self.inputs.voltageIn

        dcdc_powerOut = abs(dcdc_currentOut * dcdc_voltageOut)

        # dcdc_efficiency = 1 - 0.9* np.abs(np.log10(dcdc_voltageOut / dcdc_voltageIn))
        # this is based on a basic relation between voltage of the battery
        # and voltage of the bus. The efficiency is defined based in the bus voltage
        # dcdc_VoltageOut and the voltage comming from the battery (dcdc_voltageIn)
        # later this can be changed to an efficiency map based on the component and
        # topology of the converter. bus voltage will be studied depends on the
        # simulation.

        # Unpack the surrogate
        eta_surrogate = self.eta_surrogate

        dcdc_voltageIn_surrogtae = dcdc_voltageIn/self.voltage_input_scale

        #/self.voltage_input_scale
        if any(dcdc_voltageIn_surrogtae < self.voltage_in_min_max[0]) or any(dcdc_voltageIn_surrogtae > self.voltage_in_min_max[1]):
            #print(f'replacing dcdc_voltageIn out of bounds\n{dcdc_voltageIn_surrogtae}')
            dcdc_voltageIn_surrogtae[dcdc_voltageIn_surrogtae < self.voltage_in_min_max[0]] = self.voltage_in_min_max[0]
            dcdc_voltageIn_surrogtae[dcdc_voltageIn_surrogtae > self.voltage_in_min_max[1]] = self.voltage_in_min_max[1]

        if any(dcdc_powerOut < self.power_out_min_max[0]) or any(dcdc_powerOut > self.power_out_min_max[1]):
            #print(f'replacing dcdc_voltageIn out of bounds\n{dcdc_powerOut}')
            dcdc_powerOut[dcdc_powerOut < self.power_out_min_max[0]] = self.power_out_min_max[0]
            dcdc_powerOut[dcdc_powerOut > self.power_out_min_max[1]] = self.power_out_min_max[1]


        cond = np.hstack([dcdc_voltageIn_surrogtae, dcdc_powerOut])


        dcdc_efficiency = eta_surrogate(cond)
        #dcdc_efficiency = 1 - 0.9* np.abs(np.log10(dcdc_voltageOut / dcdc_voltageIn))

        dcdc_powerIn =  dcdc_powerOut / dcdc_efficiency**(np.sign(dcdc_currentOut)) * np.sign(dcdc_currentOut)

        dcdc_currentIn = dcdc_powerIn / dcdc_voltageIn
        
        self.outputs.powerIn = dcdc_powerIn
        self.outputs.currentIn = dcdc_currentIn


        return dcdc_powerIn, dcdc_currentIn

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

        xy  = my_data[:,:2] # Voltage_In, Power_Out

        eta = np.transpose(np.atleast_2d(my_data[:, 2]))/100  #Efficiency

        self.voltage_input_scale = np.max(xy[:,0])
        self.eta_input_scale = np.max(eta)

        # normalize for better surrogate performance
        xy[:,0] /= self.voltage_input_scale

        
        eta_surrogate = LinearNDInterpolator(xy, eta)

        self.voltage_in_min_max = np.array([np.amin(xy[:, 0]), np.amax(xy[:, 0])])
        self.power_out_min_max     = np.array([np.amin(xy[:, 1]), np.amax(xy[:, 1])])

        # Save the output
        self.eta_surrogate    = eta_surrogate

