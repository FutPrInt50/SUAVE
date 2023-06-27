## @ingroup Components-Energy-Networks
#Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus.py
#
# Created:  Mar 2017, E. Botero
# Modified: Jan 2020, T. MacDonald
#           May 2021, E. Botero
#           Jul 2022, J. Mangold

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from copy import deepcopy
from SUAVE.Components.Propulsors.Propulsor import Propulsor
from SUAVE.Methods.Utilities.Cubic_Spline_Blender import Cubic_Spline_Blender

from SUAVE.Core import Data, Units
import sklearn
from sklearn import gaussian_process
from sklearn.gaussian_process.kernels import RationalQuadratic, ConstantKernel, RBF, Matern
from sklearn import neighbors
from sklearn import svm, linear_model

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus(Propulsor):
    """ This is a way for you to load engine data from a source.
        A .csv file is read in, a surrogate made, that surrogate is used during the mission analysis.
        
        You need to use build surrogate first when setting up the vehicle to make this work.
        
        Assumptions:
        The input format for this should be Altitude, Mach, Throttle, Power, SFC(power-related)
        
        Source:
        None
    """        
    def __defaults__(self):
        """ This sets the default values for the network to function
        
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
        self.nacelle_diameter         = None
        self.engine_length            = None
        self.number_of_engines        = None
        self.tag                      = 'Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus'
        self.input_file               = None
        self.sfc_surrogate            = None
        self.thrust_surrogate         = None
        self.thrust_angle             = 0.0
        self.areas                    = Data()
        self.surrogate_type           = 'gaussian'
        self.altitude_input_scale     = 1.
        self.power_input_scale        = 1.
        self.power_anchor_scale        = 1.
        self.sfc_anchor               = None
        self.sfc_anchor_scale         = 1.
        self.sfc_anchor_conditions    = np.array([[1.,1.,1.]])
        self.thrust_anchor            = None
        self.thrust_anchor_scale      = 1.
        self.thrust_anchor_conditions = np.array([[1.,1.,1.]])
        self.sfc_rubber_scale         = 1.
        self.use_extended_surrogate   = False
        self.sealevel_static_thrust   = 0.0
        self.negative_throttle_values = False

        # for hybrid
        self.dcdc = None
        self.motor = None
        self.cable = None
        self.TMS = None
        self.propellerWTP = None

   
    # manage process with a driver function
    def evaluate_thrust(self,state):
        """ Calculate thrust given the current state of the vehicle
        
            Assumptions:
            None
            
            Source:
            N/A
            
            Inputs:
            state [state()]
            
            Outputs:
            results.thrust_force_vector [newtons]
            results.vehicle_mass_rate   [kg/s]
            
            Properties Used:
            Defaulted values
        """

        conditions = state.conditions
        numerics   = state.numerics

        emotor       = self.emotor
        esc         = self.esc
        battery     = self.battery
        turboshaft  = self.turboshaft
        eta_elc = conditions.electric_throttle
        dcdc = self.dcdc
        tms_vcs = self.tms_vcs
        tms_liquid = self.tms_liquid
        emotorWTP = self.emotorWTP
        escWTP   = self.escWTP
        eta_elc_WTP = conditions.electric_throttle_WTP

        #num_engines = self.number_of_engines
        num_engines = (self.number_of_engines - self.one_engine_propeller_inoperative)
        #num_WTP = self.number_of_WTP
        num_WTP = (self.number_of_WTP - self.number_of_WTP * self.one_engine_propeller_inoperative_WTP)
        #TODO: for wtp failure, both will fail

        # Conditions
        eta = conditions.propulsion.throttle[:, 0, None]
        if np.any(eta < 0.15):
            #print('Throttle < 0.15')
            eta_elc = 0#eta_elc * abs(eta)
            eta_elc_WTP = 0#eta_elc_WTP * abs(eta)

        # if np.any(eta > 1.05):
        #     print('Throttle > 1.05')

        # Battery
        battery.current_energy = conditions.propulsion.battery_energy

        eta_sum = eta_elc * np.ones_like(eta)
        conditions.electric_throttle_sum = eta_sum

        esc.inputs.voltagein = dcdc.bus_voltage * np.ones_like(conditions.propulsion.throttle)

        # Step 2
        esc.voltageout(conditions,eta_sum)
        # link
        emotor.inputs.voltage = esc.outputs.voltageout
        emotor.inputs.eta = eta_sum
        emotor.inputs.rpm = emotor.nominal_rpm * np.ones_like(conditions.propulsion.throttle)
        emotor.inputs.etam = state.unknowns.emotor_efficiency

        # step 3
        emotor.power_mid(conditions)

        #emotor_power = emotor.outputs.power
        #emotor_current = emotor.outputs.current

        #emotor_power[eta_sum<0.] = emotor_power[eta_sum<0.] * (-1)
        #emotor_current[eta_sum<0.] = emotor_current[eta_sum<0.] * (-1)

        #emotor.outputs.omega = emotor.inputs.rpm * Units.rpm#state.conditions.propulsion.rpm * Units.rpm # that engine and motor has the same speed
        #emotor_torque = emotor.outputs.torque #emotor_power / (emotor.outputs.omega)
        #emotor.outputs.torque =  emotor_torque # wirdn

        # ##############################################################################################################
        # Run the engine
        # ##############################################################################################################

        conditions.propulsion.combustion_engine_throttle = eta # benötigt?

        # Unpack the conditions for engine surrogate
        conditions = state.conditions
        # rescale altitude for proper surrogate performance
        altitude   = conditions.freestream.altitude
        mach       = conditions.freestream.mach_number
        throttle   = conditions.propulsion.throttle
        
        ################################################################################################################
        turboshaft = self.turboshaft
        # Run the engine

        if np.any((throttle < 0.)) == True:
            print('Info: Throttle < 0')
            conditions.propulsion.throttle = np.ones_like(throttle) * 0.5
        elif np.isnan(throttle).any():
            print('Throttle NAN')
            #conditions.propulsion.throttle = np.ones_like(throttle) * 0.5
            conditions.propulsion.throttle[np.isnan(conditions.propulsion.throttle)] = 0.5
        elif np.any((throttle > 2.)) == True:
            # print('Info: Throttle > 2')
            conditions.propulsion.throttle = np.ones_like(throttle) * 1.0

        if np.isnan(mach).any():
            # print('Mach NAN')
            conditions.freestream.mach_number = np.ones_like(mach) * 0.3

        if np.isnan(altitude).any():
            # print('Altitude NAN')
            conditions.freestream.altitude = np.ones_like(altitude) * 3000

        turboshaft.power(conditions)
        power = turboshaft.outputs.power
        sfc          = turboshaft.outputs.power_specific_fuel_consumption
        mdot_one_engine = turboshaft.outputs.mdot_one_engine
        residual_thrust_one_engine = turboshaft.outputs.residual_thrust_one_engine

        if turboshaft.p3t3_method == True:
            p3  = turboshaft.outputs.p3
            t3  = turboshaft.outputs.t3
            far = turboshaft.outputs.far

        ################################################################################################################
        # Propeller and Gearbox
        propeller = self.propeller
        gearbox = self.gearbox

        # link the propeller with the electric motor and the combusiton engine and gearbox
        propeller.inputs.omega = state.conditions.propulsion.rpm * Units.rpm
        propeller.inputs.torque = (power / propeller.inputs.omega  + emotor.outputs.torque * (emotor.outputs.omega / propeller.inputs.omega)) * gearbox.efficiency

        # step 4
        F_TP, Q_TP, P_TP, Cp_TP = propeller.spin(conditions)

        if (np.isfinite(F_TP).all()) != True:
            #print('Info: Thrust is NAN')
            F_TP = np.ones_like(F_TP)*20000

        # link
        esc.inputs.currentout = emotor.outputs.current
        esc.inputs.torque = emotor.outputs.torque #for Mid Fid
        esc.inputs.rpm = emotor.outputs.omega / Units.rpm #for Mid Fid

        # Run the esc
        esc.currentin(conditions,eta_sum)

        ################################################################################################################
        # Wing Tip Propeller
        ################################################################################################################

        escWTP.inputs.voltagein = dcdc.bus_voltage * np.ones_like(conditions.propulsion.throttle)

        etaWTP_sum = eta_elc_WTP * np.ones_like(eta)

        conditions.electric_throttleWTP_sum = etaWTP_sum

        # Step 2
        escWTP.voltageout(conditions,etaWTP_sum)

        # link
        emotorWTP.inputs.voltage = escWTP.outputs.voltageout
        emotorWTP.inputs.eta = etaWTP_sum
        emotorWTP.inputs.rpm = emotorWTP.nominal_rpm * np.ones_like(conditions.propulsion.throttle)
        emotorWTP.inputs.etam = state.unknowns.emotorWTP_efficiency

        # step 3
        emotorWTP.power_mid(conditions)

        #emotorWTP_current = emotorWTP.outputs.current
        #emotorWTP_power = emotorWTP.outputs.power

        #emotorWTP_power[etaWTP_sum<0.] = emotorWTP_power[etaWTP_sum<0.] * (-1)
        #emotorWTP_current[etaWTP_sum<0.] = emotorWTP_current[etaWTP_sum<0.] * (-1)

        #emotorWTP.outputs.omega = state.conditions.propulsion.rpm_wtp * Units.rpm  # that engine and motor has the same speed
        #emotorWTP_torque = emotorWTP_power / (emotorWTP.outputs.omega)
        #emotorWTP.outputs.torque = emotorWTP_torque

        # Propeller WTP and Gearbox WTP
        propellerWTP = self.propellerWTP
        gearboxWTP = self.gearboxWTP

        # link the propeller WTP with the electric motor WTP and the gearbox WTP
        propellerWTP.inputs.omega = state.conditions.propulsion.rpm_wtp * Units.rpm
        #propellerWTP.inputs.torque = emotorWTP_torque * gearboxWTP.efficiency
        propellerWTP.inputs.torque = emotorWTP.outputs.torque * emotor.outputs.omega / propellerWTP.inputs.omega * gearboxWTP.efficiency

        # step 4
        F_WTP, Q_WTP, P_WTP, Cp_WTP = propellerWTP.spin(conditions)

        if (np.isfinite(F_WTP).all()) != True:
            #print('Info: Thrust WTP is NAN')
            F_WTP = np.ones_like(F_WTP)*5000

        conditions.propulsion.power_WTP = P_WTP

        # link
        escWTP.inputs.currentout = emotorWTP.outputs.current
        escWTP.inputs.torque = emotorWTP.outputs.torque #for Mid Fid
        escWTP.inputs.rpm = emotorWTP.outputs.omega / Units.rpm#state.conditions.propulsion.rpm_wtp #for Mid Fid

        # Run the escWTP
        escWTP.currentin(conditions,etaWTP_sum)

        # run the bus - dcdc converter
        dcdc.inputs.voltageIn = state.unknowns.battery_voltage_under_load
        dcdc.inputs.currentOut = esc.outputs.currentin * num_engines + escWTP.outputs.currentin * num_WTP

        #dcdc.covert_lo(conditions)
        dcdc.covert_mid(conditions)


        ################################################################################################################
        # Thermal Management System
        ################################################################################################################

        # Sum for heat load
        esc_heat = abs(esc.inputs.voltagein * esc.outputs.currentin - esc.outputs.voltageout * esc.inputs.currentout)
        escWTP_heat = abs(escWTP.inputs.voltagein * escWTP.outputs.currentin - escWTP.outputs.voltageout * escWTP.inputs.currentout)

        emotor_heat = 0#abs(emotor.inputs.voltage * emotor.outputs.current - emotor.outputs.power)
        emotorWTP_heat = 0#abs(emotorWTP.inputs.voltage * emotorWTP.outputs.current - emotorWTP.outputs.power)

        dcdc_heat = abs(dcdc.inputs.voltageIn * dcdc.outputs.currentIn - dcdc.inputs.currentOut * dcdc.bus_voltage)

        battery_heat = state.unknowns.resistive_losses

        # Run the TMS - Case 1 VCS
        tms_vcs.inputs.heat_load = (dcdc_heat + battery_heat) / tms_vcs.number_system #each side of the wing
        tms_vcs.power_consumption(conditions)
        tms_vcs.tms_air_consumption(conditions)
        tms_vcs_current = tms_vcs.outputs.total_power_consumption / dcdc.inputs.voltageIn * tms_vcs.number_system #each side of the wing

        # Run the TMS - Case 4 Liquid
        #tms_liquid.inputs.heat_load = (esc_heat + emotor_heat) * num_engines + (escWTP_heat + emotorWTP_heat) * num_WTP # with motor
        tms_liquid.inputs.heat_load = ((esc_heat) * num_engines + (escWTP_heat) * num_WTP) / tms_liquid.number_system #each side of the wing
        tms_liquid.power_consumption(conditions)
        tms_liquid.tms_air_consumption(conditions)
        tms_liquid_current = tms_liquid.outputs.total_power_consumption / dcdc.inputs.voltageIn * tms_liquid.number_system #each side of the wing

        # link
        battery.inputs.current = dcdc.outputs.currentIn + tms_vcs_current + tms_liquid_current
        battery.inputs.power_in = -(dcdc.outputs.powerIn) - tms_vcs.outputs.total_power_consumption * tms_vcs.number_system  - tms_liquid.outputs.total_power_consumption * tms_liquid.number_system #each side of the wing
        battery.energy_calc(numerics)


        ################################################################################################################
        # Results
        ################################################################################################################

        conditions.propulsion.emotor_efficiency = emotor.outputs.etam
        conditions.propulsion.emotorWTP_efficiency = emotorWTP.outputs.etam

        # Pack the conditions for outputs
        a = conditions.freestream.speed_of_sound
        R = propeller.tip_radius
        rpm = propeller.inputs.omega / Units.rpm
        R_wtp = propellerWTP.tip_radius
        rpm_wtp = propellerWTP.inputs.omega / Units.rpm

        current              = battery.inputs.current
        battery_draw         = battery.inputs.power_in
        battery_energy       = battery.current_energy
        voltage_open_circuit = battery.voltage_open_circuit
        voltage_under_load   = battery.voltage_under_load
        state_of_charge      = battery.state_of_charge


        conditions.propulsion.rpm = rpm
        conditions.propulsion.propeller_rpm                 = rpm
        conditions.propulsion.battery_current               = current
        conditions.propulsion.battery_draw                  = battery_draw
        conditions.propulsion.battery_energy                = battery_energy
        conditions.propulsion.battery_voltage_open_circuit  = voltage_open_circuit
        conditions.propulsion.battery_voltage_under_load    = voltage_under_load
        conditions.propulsion.state_of_charge               = state_of_charge
        conditions.propulsion.propeller_motor_torque        = emotor.outputs.torque
        conditions.propulsion.propeller_torque              = Q_TP
        conditions.propulsion.battery_specfic_power         = -battery_draw/battery.mass_properties.mass # Wh/kg
        conditions.propulsion.propeller_tip_mach            = (R*rpm*Units.rpm)/a
        conditions.propulsion.propeller_power_coefficient   = Cp_TP

        if turboshaft.p3t3_method == True:
            conditions.propulsion.gas_turbine_p3 = p3
            conditions.propulsion.gas_turbine_t3  = t3
            conditions.propulsion.gas_turbine_far = far

        conditions.propulsion.electric_throttle             = eta_sum
        conditions.propulsion.electric_throttle_WTP         = etaWTP_sum

        energy = battery_energy
        volts = voltage_under_load
        current = current
        battery_amp_hr = (energy / Units.Wh) / volts
        C_rating = current / battery_amp_hr

        # Create the outputs
        F                                           = (num_engines * F_TP + num_WTP * F_WTP) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]
        F_mag                                       = np.atleast_2d(np.linalg.norm(F, axis=1))
        conditions.propulsion.disc_loading          = (F_mag.T)/ (num_engines*np.pi*(R)**2) # N/m^2
        conditions.propulsion.power_loading         = (F_mag.T)/(P_TP)
        conditions.propulsion.propeller_thrust      = F_mag.T
        conditions.propulsion.power                 = P_TP * num_engines + P_WTP * num_WTP

        conditions.propulsion.heat_load_vcs                     =       tms_vcs.inputs.heat_load #each system
        conditions.propulsion.tms_mdot_air_vcs                  =       tms_vcs.outputs.tms_air_consumption #each system
        conditions.propulsion.heat_load_liquid                  =       tms_liquid.inputs.heat_load #each system
        conditions.propulsion.tms_mdot_air_liquid               =       tms_liquid.outputs.tms_air_consumption #each system

        conditions.propulsion.battery_resistive_losses =    battery.resistive_losses
        conditions.propulsion.thrust_turboprop =           (F_TP) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]
        conditions.propulsion.thrust_WTP =                 (F_WTP) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]

        conditions.propulsion.power_propeller_turboprop = P_TP
        conditions.propulsion.power_turboshaft = power
        conditions.propulsion.power_motor_turboprop = emotor.outputs.power

        conditions.propulsion.power_propeller_WTP = P_WTP
        conditions.propulsion.propellerWTP_tip_mach            = (R_wtp*rpm_wtp*Units.rpm)/a


        results = Data()
        results.thrust_force_vector = F
        results.vehicle_mass_rate   = mdot_one_engine * num_engines


        return results          

    def unpack_unknowns(self, segment):
        """Unpacks the unknowns set in the mission to be available for the mission.

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:
        state.unknowns.rpm                 [RPM]

        Outputs:
        state.conditions.propulsion.rpm    [RPM]


        Properties Used:
        N/A
        """

        segment.state.conditions.propulsion.rpm = segment.state.unknowns.rpm
        segment.state.conditions.propulsion.rpm_wtp = segment.state.unknowns.rpm_wtp

        #hybrid
        segment.state.conditions.propulsion.battery_voltage_under_load  = segment.state.unknowns.battery_voltage_under_load
        #tms
        segment.state.conditions.propulsion.battery_resistive_losses = segment.state.unknowns.resistive_losses
        #emotor mid fid
        segment.state.conditions.propulsion.emotor_efficiency = segment.state.unknowns.emotor_efficiency
        #emotorWTP mid fid
        segment.state.conditions.propulsion.emotorWTP_efficiency = segment.state.unknowns.emotorWTP_efficiency


        return

    def residuals(self, segment):
        """ Calculates a residual based on torques

        Assumptions:

        Inputs:

        Outputs:

        Properties Used:
            N/A

        """

        # Battery Voltage
        # Unpack
        v_actual  = segment.state.conditions.propulsion.battery_voltage_under_load
        v_predict = segment.state.unknowns.battery_voltage_under_load
        v_max     = self.voltage
        segment.state.residuals.network[:,0] = (v_predict[:,0] - v_actual[:,0])/v_actual[:,0]#0.5#/v_max

        # fake propeller rpm
        #rpm_actual = 1200 * np.ones_like(v_predict)
        rpm_actual = self.propeller.nominal_rpm * np.ones_like(v_predict)
        rpm_predict = segment.state.unknowns.rpm
        segment.state.residuals.network[:,1] = (rpm_actual[:,0] - rpm_predict[:,0])/rpm_actual[:,0]

        # fake propeller rpm wtp
        #rpm_actual_wtp = 5000 * np.ones_like(v_predict)
        rpm_actual_wtp = self.propellerWTP.nominal_rpm * np.ones_like(v_predict)
        rpm_predict_wtp = segment.state.unknowns.rpm_wtp
        segment.state.residuals.network[:, 2] = (rpm_actual_wtp[:, 0] - rpm_predict_wtp[:, 0]) / rpm_actual_wtp[:, 0]

        #TMS
        heat_actual = segment.state.conditions.propulsion.battery_resistive_losses
        heat_predict = segment.state.unknowns.resistive_losses
        segment.state.residuals.network[:, 3] = (heat_predict[:,0] - heat_actual[:,0])/10000

        # Efficiency of Electric Motor
        eff_actual = segment.state.conditions.propulsion.emotor_efficiency
        eff_predict = segment.state.unknowns.emotor_efficiency
        segment.state.residuals.network[:,4] = eff_predict[:,0] - eff_actual[:,0]

        # Efficiency of Electric Motor WTP
        eff_actualWTP = segment.state.conditions.propulsion.emotorWTP_efficiency
        eff_predictWTP = segment.state.unknowns.emotorWTP_efficiency
        segment.state.residuals.network[:,5] = eff_predictWTP[:,0] - eff_actualWTP[:,0]

        return