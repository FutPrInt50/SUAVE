## @ingroup Components-Energy-Networks
# Battery_Propeller.py
# 
# Created:  Jul 2015, E. Botero
# Modified: Feb 2016, T. MacDonald
#           Mar 2020, M. Clarke
#           Apr 2021, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from SUAVE.Components.Propulsors.Propulsor import Propulsor

from SUAVE.Core import Data , Units

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class Battery_Propeller_withBUS(Propulsor):
    """ This is a simple network with a battery powering a propeller through
        an electric motor
        
        This network adds 2 extra unknowns to the mission. The first is
        a voltage, to calculate the thevenin voltage drop in the pack.
        The second is torque matching between motor and propeller.
    
        Assumptions:
        None
        
        Source:
        None
    """  
    def __defaults__(self):
        """ This sets the default values for the network to function.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            N/A
        """             
        self.motor                     = None
        self.propeller                 = None
        self.esc                       = None
        self.avionics                  = None
        self.payload                   = None
        self.battery                   = None
        self.bus                       = 700  #500 chosen as an example
        self.nacelle_diameter          = None
        self.engine_length             = None
        self.number_of_engines         = None
        self.voltage                   = None
        self.thrust_angle              = 0.0
        self.pitch_command             = 0.0 
        self.tag                       = 'Battery_Propeller_withBUS'
        self.use_surrogate             = False
        self.generative_design_minimum = 0
    
    # manage process with a driver function
    def evaluate_thrust(self,state):
        """ Calculate thrust given the current state of the vehicle
    
            Assumptions:
            Caps the throttle at 110% and linearly interpolates thrust off that
    
            Source:
            N/A
    
            Inputs:
            state [state()]
    
            Outputs:
            results.thrust_force_vector [newtons]
            results.vehicle_mass_rate   [kg/s]
            conditions.propulsion:
                rpm                  [radians/sec]
                current              [amps]
                battery_draw         [watts]
                battery_energy       [joules]
                voltage_open_circuit [volts]
                voltage_under_load   [volts]
                motor_torque         [N-M]
                propeller_torque     [N-M]
    
            Properties Used:
            Defaulted values
        """          
    
        # unpack
        conditions = state.conditions
        numerics   = state.numerics
        motor      = self.motor
        propeller  = self.propeller
        esc        = self.esc
        avionics   = self.avionics
        payload    = self.payload
        battery    = self.battery
        num_engines= self.number_of_engines
        
        # Set battery energy
        battery.current_energy = conditions.propulsion.battery_energy  

        # Step 1 battery power
#        esc.inputs.voltagein = state.unknowns.battery_voltage_under_load
        esc.inputs.voltagein = self.bus * np.ones_like(conditions.propulsion.throttle)
        # Step 2
        esc.voltageout(conditions)
        #eta_sum = conditions.propulsion.throttle[:, 0, None] # new because of hybrid
        #esc.voltageout(conditions, eta_sum) # new because of hybrid
        
        # link
        motor.inputs.voltage = esc.outputs.voltageout
        
        # step 3
        #motor.omega(conditions)
        motor.power_lo(conditions)
        motor_power = motor.outputs.power
        motor_current = motor.outputs.current
        motor.outputs.omega = 1200 * np.ones_like(motor_power) * Units.rpm
        motor_torque = motor_power / (motor.outputs.omega)
        motor.outputs.torque =  motor_torque
        
        
        # link
        propeller.inputs.omega  = motor.outputs.omega# 1200 * Units.rpm #motor.outputs.omega
        propeller.thrust_angle  = self.thrust_angle
        propeller.pitch_command = self.pitch_command
        propeller.inputs.torque = motor.outputs.power / propeller.inputs.omega
        
        # step 4
        #F, Q, P, Cp, outputs , etap = propeller.spin(conditions)
        F, Q, P, Cp = propeller.spin(conditions)
            
        # Check to see if magic thrust is needed, the ESC caps throttle at 1.1 already
        #eta        = conditions.propulsion.throttle[:,0,None]
        #P[eta>1.0] = P[eta>1.0]*eta[eta>1.0]
        #F[eta>1.0] = F[eta>1.0]*eta[eta>1.0]
        
        # link
        #propeller.outputs = outputs
        
        # Run the avionics
        #avionics.power()

        # Run the payload
        #payload.power()

        # Run the motor for current
        #motor.current(conditions)

        
        # link
        esc.inputs.currentout =  motor.outputs.current

        # Run the esc
        esc.currentin(conditions)
        
        #dcdc converter
        dcdc_currentOut = esc.outputs.currentin*num_engines
        dcdc_voltageOut = esc.inputs.voltagein

        dcdc_powerOut = dcdc_currentOut * dcdc_voltageOut
        
        dcdc_voltageIn = state.unknowns.battery_voltage_under_load

        dcdc_efficiency = 1 - 0.9* np.abs(np.log10(dcdc_voltageOut / dcdc_voltageIn))
        # this is based on a basic relation between voltage of the battery 
        # and voltage of the bus. The efficiency is defined based in the bus voltage
        # dcdc_VoltageOut and the voltage comming from the battery (dcdc_voltageIn)
        # later this can be changed to an efficiency map based on the component and 
        # topology of the converter. bus voltage will be studied depends on the 
        # simulation.

        dcdc_powerIn =  dcdc_powerOut / dcdc_efficiency
        
        dcdc_currentIn = dcdc_powerIn / dcdc_voltageIn


        # Calculate avionics and payload power
        #avionics_payload_power = avionics.outputs.power + payload.outputs.power

        # Calculate avionics and payload current
        #avionics_payload_current = avionics_payload_power/self.voltage

        # link
        battery.inputs.current  = dcdc_currentIn #+ avionics_payload_current
        # battery.inputs.power_in = -(esc.outputs.voltageout*esc.outputs.currentin*num_engines) #+ avionics_payload_power)
        battery.inputs.power_in = -(dcdc_powerIn) #+ avionics_payload_power) #new changes Jonas recomendation
        battery.energy_calc(numerics)        
    
        # Pack the conditions for outputs
        a                    = conditions.freestream.speed_of_sound
        R                    = propeller.tip_radius
        rpm                  = motor.outputs.omega / Units.rpm
        #current              = esc.outputs.currentin
        current              = battery.inputs.current
        battery_draw         = battery.inputs.power_in 
        battery_energy       = battery.current_energy
        voltage_open_circuit = battery.voltage_open_circuit
        voltage_under_load   = battery.voltage_under_load
        state_of_charge      = battery.state_of_charge
          
        conditions.propulsion.propeller_rpm                 = rpm
        conditions.propulsion.battery_current               = current
        conditions.propulsion.battery_draw                  = battery_draw
        conditions.propulsion.battery_energy                = battery_energy
        conditions.propulsion.battery_voltage_open_circuit  = voltage_open_circuit
        conditions.propulsion.battery_voltage_under_load    = voltage_under_load
        conditions.propulsion.state_of_charge               = state_of_charge
        conditions.propulsion.propeller_motor_torque        = motor.outputs.torque
        conditions.propulsion.propeller_torque              = Q
        conditions.propulsion.battery_specfic_power         = -battery_draw/battery.mass_properties.mass # Wh/kg
        conditions.propulsion.propeller_tip_mach            = (R*rpm*Units.rpm)/a
        conditions.propulsion.propeller_power_coefficient   = Cp
        
        # noise
        #outputs.number_of_engines                   = num_engines
        #conditions.noise.sources.propeller          = outputs

        # Create the outputs
        F                                           = num_engines* F * [np.cos(self.thrust_angle),0,-np.sin(self.thrust_angle)]      
        mdot                                        = state.ones_row(1)*0.0
        F_mag                                       = np.atleast_2d(np.linalg.norm(F, axis=1))  
        conditions.propulsion.disc_loading          = (F_mag.T)/ (num_engines*np.pi*(R)**2) # N/m^2                  
        conditions.propulsion.power_loading         = (F_mag.T)/(P)
        conditions.propulsion.propeller_thrust      = F_mag.T
        conditions.propulsion.power                 = P * num_engines
        
        results = Data()
        results.thrust_force_vector = F
        results.vehicle_mass_rate   = mdot
        
        
        return results
    
    
    def unpack_unknowns(self,segment):
        """ This is an extra set of unknowns which are unpacked from the mission solver and send to the network.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state.unknowns.propeller_power_coefficient [None]
            state.unknowns.battery_voltage_under_load  [volts]
    
            Outputs:
            state.conditions.propulsion.propeller_power_coefficient [None]
            state.conditions.propulsion.battery_voltage_under_load  [volts]
    
            Properties Used:
            N/A
        """                  
        
        # Here we are going to unpack the unknowns (Cp) provided for this network
        # segment.state.conditions.propulsion.propeller_power_coefficient = segment.state.unknowns.propeller_power_coefficient
        # segment.state.conditions.propulsion.propeller_omega = segment.state.unknowns.propeller_omega
        #segment.state.conditions.propulsion.battery_voltage_under_load  = segment.state.unknowns.battery_voltage_under_load
        
        return
    
    def residuals(self,segment):
        """ This packs the residuals to be send to the mission solver.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state.conditions.propulsion:
                motor_torque                          [N-m]
                propeller_torque                      [N-m]
                voltage_under_load                    [volts]
            state.unknowns.battery_voltage_under_load [volts]
            
            Outputs:
            None
    
            Properties Used:
            self.voltage                              [volts]
        """        
        
        # Here we are going to pack the residuals (torque,voltage) from the network
        
        # Unpack
        q_motor   = segment.state.conditions.propulsion.propeller_motor_torque
        q_prop    = segment.state.conditions.propulsion.propeller_torque
        v_actual  = segment.state.conditions.propulsion.battery_voltage_under_load
        v_predict = segment.state.unknowns.battery_voltage_under_load
        v_max     = self.voltage
        
        # Return the residuals
        #segment.state.residuals.network[:,0] = q_motor[:,0] - q_prop[:,0]
        #segment.state.residuals.network[:,0] = (v_predict[:,0] - v_actual[:,0])/0.5#/v_max
        segment.state.residuals.network[:,0] = (v_predict[:,0] - v_actual[:,0])/700#/v_max

        return    
            
    __call__ = evaluate_thrust


