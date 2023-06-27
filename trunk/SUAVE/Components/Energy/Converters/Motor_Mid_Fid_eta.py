## @ingroup Components-Energy-Converters
# Motor_Lo_Fid_eta.py
#
# Created:  Jun 2014, E. Botero
# Modified: Jan 2016, T. MacDonald  
#           Mar 2020, M. Clarke
#           Mar 2020, E. Botero
#           Feb 2022, J. Mangold

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np

# suave imports
import SUAVE
import time
from SUAVE.Core import Data, Units
import pickle


# package imports
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from scipy.interpolate import LinearNDInterpolator

# ----------------------------------------------------------------------
#  Motor Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class Motor_Mid_Fid_eta(Energy_Component):
    """This is a low-fidelity motor component.
    
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
        self.resistance         = 0.0
        self.no_load_current    = 0.0
        self.speed_constant     = 0.0
        self.gear_ratio         = 1.0 #0.0
        self.gearbox_efficiency = 1.0 #0.0
        self.expected_current   = 0.0
        self.motor_efficiency   = 0.0
        self.rated_power        = 0.0
        self.rated_voltage      = 0.0
        self.inputs.eta         = 1.0
    
    def omega(self,conditions):
        """Calculates the motor's rotation rate
    
        Assumptions:
        Cp (power coefficient) is constant
    
        Source:
        N/A
    
        Inputs:
        conditions.
          freestream.velocity                    [m/s]
          freestream.density                     [kg/m^3]
        self.inputs.voltage                      [V]
    
        Outputs:
        self.outputs.
          torque                                 [Nm]
          omega                                  [radian/s]
    
        Properties Used:
        self.
          resistance                             [ohms]
          gearbox_efficiency                     [-]
          expected_current                       [A]
          no_load_current                        [A]
          gear_ratio                             [-]
          speed_constant                         [radian/s/V]
        """  
        # Unpack
        Res   = self.resistance
        etaG  = self.gearbox_efficiency
        exp_i = self.expected_current
        io    = self.no_load_current + exp_i*(1-etaG)
        G     = self.gear_ratio
        Kv    = self.speed_constant/G
        etam  = self.motor_efficiency
        v     = self.inputs.voltage
        
        inside = Res*Res*io*io - 2.*Res*etam*io*v - 2.*Res*io*v + etam*etam*v*v - 2.*etam*v*v + v*v
        
        inside[inside<0.] = 0.
        
        # Omega
        omega1 = (Kv*v)/2. + (Kv*(inside)**(1./2.))/2. - (Kv*Res*io)/2. + (Kv*etam*v)/2.

        # If the voltage supplied is too low this function will NaN. However, that really means it won't spin
        omega1[np.isnan(omega1)] = 0.0
        
        # The torque
        Q      = ((v-omega1/Kv)/Res -io)/Kv
        
        omega1[v==0] = 0.

        # store to outputs
        self.outputs.omega  = omega1
        self.outputs.torque = Q
        
        return omega1
    
    def current(self,conditions):
        """Calculates the motor's rotation rate
    
        Assumptions:
        Cp (power coefficient) is constant
    
        Source:
        N/A
    
        Inputs:
        self.inputs.voltage    [V]
    
        Outputs:
        self.outputs.current   [A]
        conditions.
          propulsion.etam      [-]
        i                      [A]
    
        Properties Used:
        self.
          gear_ratio           [-]
          speed_constant       [radian/s/V]
          resistance           [ohm]
          omega(conditions)    [radian/s] (calls the function)
          gearbox_efficiency   [-]
          expected_current     [A]
          no_load_current      [A]
        """       
        
        # Unpack
        G     = self.gear_ratio
        Kv    = self.speed_constant
        Res   = self.resistance
        v     = self.inputs.voltage
        omeg  = self.omega(conditions)*G
        etaG  = self.gearbox_efficiency
        exp_i = self.expected_current
        io    = self.no_load_current + exp_i*(1-etaG)
        
        i=(v-omeg/Kv)/Res
        
        # This line means the motor cannot recharge the battery
        i[i < 0.0] = 0.0

        # Pack
        self.outputs.current = i
         
        etam=(1-io/i)*(1-i*Res/v)
        conditions.propulsion.etam = etam
        
        return i
    
    def power_mid(self,conditions):
        """Calculates the motor's power output
    
        Assumptions: 
    
        Source:
        N/A
    
        Inputs: 
        self.
           inputs.voltage         [V]
           inputs.eta
    
        Outputs:
        self.outputs.
           outputs.power          [W]
           outputs.current        [A]
           
        Properties Used:
        self.
        
          motor_efficiency        [-] 
          rated_power             [W] 
          rated_voltage           [V] 
        """          
        

        power  = self.rated_power
        v_rate = self.rated_voltage
        v_in   = self.inputs.voltage
        eta  = self.inputs.eta
        etam = self.inputs.etam

        # Unpack the surrogate
        eta_surrogate = self.eta_surrogate

        #power_{nominal}, voltage_{nominal}, rpm_{nominal}, rpm_{depend on throttle}, Y_{depend on throttle}

        power_nom_sur = power/self.power_nom_input_scale * np.ones_like(self.inputs.etam)
        voltage_nom_sur = v_rate/self.voltage_nom_input_scale * np.ones_like(self.inputs.etam)
        rpm_nom_sur = self.nominal_rpm/self.rpm_nom_input_scale * np.ones_like(self.inputs.etam)
        # rpm_sur = self.inputs.rpm/self.rpm_input_scale


        power_out = abs(power*v_in/v_rate*etam)
        torque = power_out / (self.inputs.rpm * Units.rpm)
        torque_sur = torque/self.torque_input_scale

        if any(power_nom_sur < self.power_nom_min_max[0]) or any(power_nom_sur > self.power_nom_min_max[1]):
            print(f'replacing power_nom_sur out of bounds\n{power_nom_sur}')
            power_nom_sur[power_nom_sur < self.power_nom_min_max[0]] = self.power_nom_min_max[0]
            power_nom_sur[power_nom_sur > self.power_nom_min_max[1]] = self.power_nom_min_max[1]

        if any(voltage_nom_sur < self.voltage_nom_min_max[0]) or any(voltage_nom_sur > self.voltage_nom_min_max[1]):
            print(f'replacing voltage_nom_sur out of bounds\n{voltage_nom_sur}')
            voltage_nom_sur[voltage_nom_sur < self.voltage_nom_min_max[0]] = self.voltage_nom_min_max[0]
            voltage_nom_sur[voltage_nom_sur > self.voltage_nom_min_max[1]] = self.voltage_nom_min_max[1]

        if any(rpm_nom_sur < self.rpm_nom_min_max[0]) or any(rpm_nom_sur > self.rpm_nom_min_max[1]):
            print(f'replacing rpm_nom_sur out of bounds\n{rpm_nom_sur}')
            rpm_nom_sur[rpm_nom_sur < self.rpm_nom_min_max[0]] = self.rpm_nom_min_max[0]
            rpm_nom_sur[rpm_nom_sur > self.rpm_nom_min_max[1]] = self.rpm_nom_min_max[1]

        # if any(rpm_sur < self.rpm_min_max[0]) or any(rpm_sur > self.rpm_min_max[1]):
        #     print(f'replacing rpm_sur out of bounds\n{rpm_sur}')
        #     rpm_sur[rpm_sur < self.rpm_min_max[0]] = self.rpm_min_max[0]
        #     rpm_sur[rpm_sur > self.rpm_min_max[1]] = self.rpm_min_max[1]

        if any(torque_sur < self.torque_min_max[0]) or any(torque_sur > self.torque_min_max[1]):
            print(f'replacing torque_sur out of bounds\n{torque_sur}')
            torque_sur[torque_sur < self.torque_min_max[0]] = self.torque_min_max[0]
            torque_sur[torque_sur > self.torque_min_max[1]] = self.torque_min_max[1]

        # cond = np.hstack([power_nom_sur, voltage_nom_sur, rpm_nom_sur, rpm_sur, torque_sur])
        cond = np.hstack([power_nom_sur, voltage_nom_sur, rpm_nom_sur, torque_sur])

        etam = eta_surrogate(cond)

        #power_out = power*v_in/v_rate*etam # wird vorher gerechnet

        i = power_out/(etam**(np.sign(eta))*v_in)
        
        i[power_out == 0.] = 0.
        
        self.outputs.power   = power_out * (np.sign(eta))
        self.outputs.current = i * (np.sign(eta))
        self.outputs.torque = torque * (np.sign(eta))
        self.outputs.omega = self.inputs.rpm * Units.rpm
        self.outputs.etam = etam
        
        return power_out, i


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

        if file_name.split('.')[-1] == 'csv':
            my_data = np.genfromtxt(file_name, delimiter=',')
            # power_{nominal}, voltage_{nominal}, rpm_{nominal}, rpm_{depend on throttle}, torque_{depend on throttle}, eff_{depend on throttle}, specific_power_{nominal}, motor_index

            xy  = my_data[:,:5] # power_{nominal}, voltage_{nominal}, rpm_{nominal}, rpm_{depend on throttle}, Y_{depend on throttle}

            xy = np.delete(xy, 3, 1)

            eta = np.transpose(np.atleast_2d(my_data[:, 5]))  #eff_{depend on throttle}
            specific_power = np.transpose(np.atleast_2d(my_data[:, 6]))  #eff_{depend on throttle}

            self.power_nom_input_scale = np.max(xy[:,0])
            self.voltage_nom_input_scale = np.max(xy[:,1])
            self.rpm_nom_input_scale = np.max(xy[:,2])
            # self.rpm_input_scale = np.max(xy[:,3])
            self.torque_input_scale = np.max(xy[:,3])

            #self.eta_input_scale = np.max(eta)

            # normalize for better surrogate performance
            xy[:,0] /= self.power_nom_input_scale
            xy[:,1] /= self.voltage_nom_input_scale
            xy[:,2] /= self.rpm_nom_input_scale
            # xy[:,3] /= self.rpm_input_scale
            xy[:,3] /= self.torque_input_scale

            # start_time = time.time()
            eta_surrogate = LinearNDInterpolator(xy, eta)
            # stop_time = time.time()
            # duration = stop_time - start_time
            #print(f"Emotor surrogate eta in {duration:,.2f} s.")

            specific_power_surrogate = LinearNDInterpolator(xy[:,:3], specific_power)

            self.power_nom_min_max = np.array([np.amin(xy[:, 0]), np.amax(xy[:, 0])])
            self.voltage_nom_min_max     = np.array([np.amin(xy[:, 1]), np.amax(xy[:, 1])])
            self.rpm_nom_min_max     = np.array([np.amin(xy[:, 2]), np.amax(xy[:, 2])])
            # self.rpm_min_max     = np.array([np.amin(xy[:, 3]), np.amax(xy[:, 3])])
            self.torque_min_max     = np.array([np.amin(xy[:, 3]), np.amax(xy[:, 3])])

            # Save the output
            self.eta_surrogate    = eta_surrogate
            self.specific_power_surrogate = specific_power_surrogate

        elif file_name.split('.')[-1] == 'p':
            surrogate = pickle.load(open(file_name, "rb"))
            self.power_nom_input_scale = surrogate.power_nom_input_scale
            self.voltage_nom_input_scale = surrogate.voltage_nom_input_scale
            self.rpm_nom_input_scale = surrogate.rpm_nom_input_scale
            self.torque_input_scale = surrogate.torque_input_scale

            self.power_nom_min_max = surrogate.power_nom_min_max
            self.voltage_nom_min_max = surrogate.voltage_nom_min_max
            self.rpm_nom_min_max = surrogate.rpm_nom_min_max
            self.torque_min_max = surrogate.torque_min_max

            # Save the output
            self.eta_surrogate = surrogate.eta_surrogate
            self.specific_power_surrogate = surrogate.specific_power_surrogate

        else:
            raise AttributeError('Surrogate data type unknown')

    
        
        