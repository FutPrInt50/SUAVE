## @ingroup Components-Energy-Networks
# Hybrid_Turboprop_Battery_Propeller_Bus.py
# 
# Created:  Jul 2015, E. Botero
# Modified: Feb 2016, T. MacDonald
#           Mar 2020, M. Clarke
#           Apr 2021, M. Clarke
#           Feb 2022, J. Mangold

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
class Hybrid_Turboprop_Battery_Propeller_Bus(Propulsor):
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
        self.nacelle_diameter          = None
        self.engine_length             = None
        self.number_of_engines         = None
        self.voltage                   = None
        self.thrust_angle              = 0.0
        self.pitch_command             = 0.0 
        self.tag                       = 'Hybrid_Turboprop_Battery_Propeller'
        self.use_surrogate             = False
        self.generative_design_minimum = 0

        #hybrid
        self.engine = None
        self.dcdc    = None
        self.cable = None
        self.TMS = None
    
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

        # hybrid
        engine = self.engine
        #eta_elc = self.electric_throttle
        eta_elc = conditions.electric_throttle
        dcdc = self.dcdc
        TMS = self.TMS

        # hybird mit WTP
        motorWTP = self.motorWTP
        escWTP   = self.escWTP
        eta_elc_WTP = conditions.electric_throttle_WTP
        
        # Set battery energy
        battery.current_energy = conditions.propulsion.battery_energy  

        # Step 1 battery power
        eta = conditions.propulsion.throttle[:, 0, None]
        #esc.inputs.voltagein = state.unknowns.battery_voltage_under_load * eta *0.3
        #if max(eta) < 0.5:
        #    eta_sum = eta * eta_elc * np.ones_like(eta)
        #else:
        #    eta_sum = eta_elc * np.ones_like(eta)
        eta_sum = eta_elc * np.ones_like(eta)

        conditions.electric_throttle_sum = eta_sum
        #esc.inputs.voltagein = state.unknowns.battery_voltage_under_load * eta_elc * eta
        #esc.inputs.voltagein = state.unknowns.battery_voltage_under_load #* eta_sum
        esc.inputs.voltagein = dcdc.bus_voltage * np.ones_like(conditions.propulsion.throttle)
        
        # Step 2
        esc.voltageout(conditions,eta_sum)
        
        # link
        motor.inputs.voltage = esc.outputs.voltageout
        #motor.inputs.voltage = motor.rated_voltage * 0.2 #* np.ones_like(eta)
        motor.inputs.eta = eta_sum
        
        # step 3
        #motor.omega(conditions)
        motor.power_lo(conditions)
        #motor_power = motor.outputs.powe

        #motor_power = motor.outputs.power * eta_sum / abs(eta_sum) # to set that negativ throttle menas negativ power (charging)
        # if min(eta_sum) < 0.:
        #     motor_power = motor.outputs.power * (-1)
        #     motor_current = motor.outputs.current * (-1)
        # else:
        #     motor_power = motor.outputs.power
        #     motor_current = motor.outputs.current

        motor_power = motor.outputs.power
        motor_current = motor.outputs.current

        motor_power[eta_sum<0.] = motor_power[eta_sum<0.] * (-1)
        motor_current[eta_sum<0.] = motor_current[eta_sum<0.] * (-1)

        #motor_current = motor.outputs.current
        #motor.outputs.omega = 10*1200 * np.ones_like(motor_power) * Units.rpm #1200
        motor.outputs.omega = state.conditions.propulsion.rpm * Units.rpm # that engine and motor has the same speed
        motor_torque = motor_power / (motor.outputs.omega)
        motor.outputs.torque =  motor_torque

        # hybrid combustion engine
        # Throttle the engine
        engine.inputs.speed = state.conditions.propulsion.rpm * Units.rpm
        conditions.propulsion.combustion_engine_throttle = conditions.propulsion.throttle

        # Run the engine
        engine.power(conditions)
        power_output = engine.outputs.power
        sfc = engine.outputs.power_specific_fuel_consumption
        mdot = engine.outputs.fuel_flow_rate
        torque = engine.outputs.torque

        # link emotor and engone
        propeller.inputs.omega  = motor.outputs.omega# 1200 * Units.rpm #motor.outputs.omega
        #propeller.thrust_angle  = self.thrust_angle
        #propeller.pitch_command = self.pitch_command
        #propeller.inputs.torque = motor.outputs.power / propeller.inputs.omega
        #propeller.inputs.torque = motor.outputs.power / propeller.inputs.omega + engine.outputs.torque
        propeller.inputs.torque = motor_torque + engine.outputs.torque
        
        # step 4
        #F, Q, P, Cp, outputs , etap = propeller.spin(conditions)
        F_TP, Q_TP, P_TP, Cp_TP = propeller.spin(conditions)
            
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
        #esc.inputs.currentout =  motor.outputs.current
        esc.inputs.currentout = motor_current

        # Run the esc
        esc.currentin(conditions,eta_sum)

        # Calculate avionics and payload power
        #avionics_payload_power = avionics.outputs.power + payload.outputs.power

        # Calculate avionics and payload current
        #avionics_payload_current = avionics_payload_power/self.voltage

        # ################################################################################################################
        # hybrid mit WTP - also Electric Branch nochmal fuer WTP - siehe Skizze
        # ################################################################################################################

        #escWTP.inputs.voltagein = state.unknowns.battery_voltage_under_load
        escWTP.inputs.voltagein = dcdc.bus_voltage * np.ones_like(conditions.propulsion.throttle)

        etaWTP_sum = eta_elc_WTP * np.ones_like(eta)

        conditions.electric_throttleWTP_sum = etaWTP_sum
        #escWTP.inputs.voltagein = state.unknowns.battery_voltage_under_load  # * eta_sum

        # Step 2
        escWTP.voltageout(conditions,etaWTP_sum)

        # link
        motorWTP.inputs.voltage = escWTP.outputs.voltageout
        # motor.inputs.voltage = motor.rated_voltage * 0.2 #* np.ones_like(eta)
        motorWTP.inputs.eta = etaWTP_sum

        # step 3
        # motor.omega(conditions)
        motorWTP.power_lo(conditions)
        # motor_power = motor.outputs.powe

        # motor_power = motor.outputs.power * eta_sum / abs(eta_sum) # to set that negativ throttle menas negativ power (charging)
        # if min(etaWTP_sum) < 0.:
        #     motorWTP_power = motorWTP.outputs.power * (-1)
        # else:
        #     motorWTP_power = motorWTP.outputs.power
        # #motorWTP_power = motorWTP.outputs.power

        motorWTP_current = motorWTP.outputs.current
        motorWTP_power = motorWTP.outputs.power

        motorWTP_power[etaWTP_sum<0.] = motorWTP_power[etaWTP_sum<0.] * (-1)
        motorWTP_current[etaWTP_sum<0.] = motorWTP_current[etaWTP_sum<0.] * (-1)

        # motor.outputs.omega = 10*1200 * np.ones_like(motor_power) * Units.rpm #1200
        motorWTP.outputs.omega = state.conditions.propulsion.rpm * Units.rpm  # that engine and motor has the same speed
        motorWTP_torque = motorWTP_power / (motorWTP.outputs.omega)
        motorWTP.outputs.torque = motorWTP_torque

        propellerWTP = self.propeller
        # link
        propellerWTP.inputs.omega = state.conditions.propulsion.rpm * Units.rpm
        propellerWTP.inputs.torque = motorWTP_torque

        # step 4
        F_WTP, Q_WTP, P_WTP, Cp_WTP = propellerWTP.spin(conditions)

        conditions.propulsion.power_WTP = P_WTP

        # link
        #escWTP.inputs.currentout =  motorWTP.outputs.current
        escWTP.inputs.currentout =motorWTP_current

        # Run the esc
        escWTP.currentin(conditions,etaWTP_sum)


        # run the bus - dcdc converter
        dcdc.inputs.voltageIn = state.unknowns.battery_voltage_under_load
        dcdc.inputs.currentOut = esc.outputs.currentin * num_engines + escWTP.outputs.currentin * num_engines

        dcdc.covert_lo(conditions)

        # Sum for heat load
        esc_heat = abs(esc.inputs.voltagein * esc.outputs.currentin - esc.outputs.voltageout * esc.inputs.currentout)
        escWTP_heat = abs(escWTP.inputs.voltagein * escWTP.outputs.currentin - escWTP.outputs.voltageout * escWTP.inputs.currentout)

        motor_heat = abs(motor.inputs.voltage * motor.outputs.current - motor.outputs.power)
        motorWTP_heat = abs(motorWTP.inputs.voltage * motorWTP.outputs.current - motorWTP.outputs.power)

        dcdc_heat = abs(dcdc.inputs.voltageIn * dcdc.outputs.currentIn - dcdc.inputs.currentOut * dcdc.bus_voltage)

        battery_heat = state.unknowns.resistive_losses

        # Run the TMS
        TMS.inputs.heat_load = (esc_heat + motor_heat) * num_engines + (escWTP_heat + motorWTP_heat) * num_engines + dcdc_heat + battery_heat
        TMS.power_consumption(conditions)

        TMS_current = TMS.outputs.total_power_consumption / dcdc.inputs.voltageIn



        # link
        #battery.inputs.current  = esc.outputs.currentin*num_engines #+ avionics_payload_current
        #battery.inputs.current = esc.outputs.currentin * num_engines + escWTP.outputs.currentin * num_engines
        battery.inputs.current = dcdc.outputs.currentIn + TMS_current

        #battery.inputs.power_in = -(esc.outputs.voltageout*esc.outputs.currentin*num_engines) #+ avionics_payload_power)
        #battery.inputs.power_in = -(esc.outputs.voltageout * esc.outputs.currentin * num_engines) - (escWTP.outputs.voltageout * escWTP.outputs.currentin * num_engines)
        #battery.inputs.power_in = -(esc.inputs.voltagein * esc.outputs.currentin * num_engines) - (escWTP.inputs.voltagein * escWTP.outputs.currentin * num_engines)
        battery.inputs.power_in = -(dcdc.outputs.powerIn) - TMS.outputs.total_power_consumption
        battery.energy_calc(numerics)
        
        
        # ################################################################################################################
        # # Second Propeller - Try
        # ################################################################################################################
        # # hybrid step 4
        # # Throttle the engine
        # engine.inputs.speed = state.conditions.propulsion.rpm * Units.rpm
        # conditions.propulsion.combustion_engine_throttle = conditions.propulsion.throttle
        #
        # # Run the engine
        # engine.power(conditions)
        # power_output = engine.outputs.power
        # sfc = engine.outputs.power_specific_fuel_consumption
        # mdot = engine.outputs.fuel_flow_rate
        # torque = engine.outputs.torque
        #
        # propellerWTP = self.propeller
        # # link
        # #propellerWTP.inputs.omega = np.ones_like(torque)*1200 * Units.rpm
        # propeller.inputs.omega = state.conditions.propulsion.rpm * Units.rpm
        # #propellerWTP.thrust_angle = self.thrust_angle
        # propellerWTP.inputs.torque = torque
        #
        # # step 4
        # # F, Q, P, Cp ,  outputs  , etap  = propeller.spin(conditions)
        # F_WTP, Q_WTP, P_WTP, Cp_WTP = propellerWTP.spin(conditions)
        #
        # conditions.propulsion.power_WTP = P_WTP

        ################################################################################################################
        
        
        
    
        # Pack the conditions for outputs
        a                    = conditions.freestream.speed_of_sound
        R                    = propeller.tip_radius
        rpm                  = motor.outputs.omega / Units.rpm
        #current              = esc.outputs.currentin # just for one esc and one engine
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
        conditions.propulsion.propeller_torque              = Q_TP
        conditions.propulsion.battery_specfic_power         = -battery_draw/battery.mass_properties.mass # Wh/kg
        conditions.propulsion.propeller_tip_mach            = (R*rpm*Units.rpm)/a
        conditions.propulsion.propeller_power_coefficient   = Cp_TP

        conditions.propulsion.electric_throttle             = eta_sum
        conditions.propulsion.electric_throttle_WTP         = etaWTP_sum
        
        # noise
        #outputs.number_of_engines                   = num_engines
        #conditions.noise.sources.propeller          = outputs

        #################################################################
        #################################################################

        energy = battery_energy#conditions.propulsion.battery_energy[:, 0]
        volts = voltage_under_load#conditions.propulsion.battery_voltage_under_load[:, 0]
        current = current#conditions.propulsion.battery_current[:, 0]
        battery_amp_hr = (energy / Units.Wh) / volts
        C_rating = current / battery_amp_hr

        #################################################################
        #################################################################


        # Create the outputs
        #F                                           = num_engines* F * [np.cos(self.thrust_angle),0,-np.sin(self.thrust_angle)]
        #F                                           = (num_engines * F + 2*F_WTP) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]
        #F                                            = (num_engines * F) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]
        F                                           = (num_engines * F_TP + 2*F_WTP) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]

        #mdot                                        = state.ones_row(1)*0.0
        F_mag                                       = np.atleast_2d(np.linalg.norm(F, axis=1))  
        conditions.propulsion.disc_loading          = (F_mag.T)/ (num_engines*np.pi*(R)**2) # N/m^2                  
        conditions.propulsion.power_loading         = (F_mag.T)/(P_TP)
        conditions.propulsion.propeller_thrust      = F_mag.T
        conditions.propulsion.power                 = P_TP * num_engines + P_WTP * 2
        conditions.propulsion.heat_load             =       TMS.inputs.heat_load
        conditions.propulsion.battery_resistive_losses =    battery.resistive_losses
        conditions.propulsion.thrust_turboprop =           (F_TP) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]
        conditions.propulsion.thrust_WTP =                 (F_WTP) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]

        conditions.propulsion.power_propeller_turboprop = P_TP
        conditions.propulsion.power_turboshaft = engine.outputs.power
        conditions.propulsion.power_motor_turboprop = motor_power

        conditions.propulsion.power_propeller_WTP = P_WTP


        results = Data()
        results.thrust_force_vector = F
        results.vehicle_mass_rate   = mdot * 2
        
        
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
        #segment.state.conditions.propulsion.propeller_power_coefficient = segment.state.unknowns.propeller_power_coefficient
        #segment.state.conditions.propulsion.propeller_omega = segment.state.unknowns.propeller_omega
        segment.state.conditions.propulsion.battery_voltage_under_load  = segment.state.unknowns.battery_voltage_under_load
        #hybrid
        segment.state.conditions.propulsion.rpm = segment.state.unknowns.rpm
        #tms
        segment.state.conditions.propulsion.battery_resistive_losses = segment.state.unknowns.resistive_losses
        
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
        segment.state.residuals.network[:,0] = (v_predict[:,0] - v_actual[:,0])/v_actual[:,0]#0.5#/v_max

        # fake propeller rpm
        #rpm_actual = segment.state.conditions.propulsion.rpm
        #rpm_actual = segment.parameter_rpm * np.ones_like(v_predict)
        rpm_actual = 1200 * np.ones_like(v_predict)
        rpm_predict = segment.state.unknowns.rpm
        segment.state.residuals.network[:,1] = (rpm_actual[:,0] - rpm_predict[:,0])/rpm_actual[:,0]

        #TMS
        heat_actual = segment.state.conditions.propulsion.battery_resistive_losses
        heat_predict = segment.state.unknowns.resistive_losses
        segment.state.residuals.network[:,2] = (heat_predict[:,0] - heat_actual[:,0])/10000

        return    
            
    __call__ = evaluate_thrust


