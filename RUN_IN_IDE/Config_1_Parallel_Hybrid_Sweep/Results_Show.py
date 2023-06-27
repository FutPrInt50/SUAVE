## Config_1_parallel_hybrid.py
# 
# This code was contributed under project FUT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Jonas Mangold, mangold@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Jul 2022, J. Mangold
# Modified:

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# Python Imports
import numpy as np
import pylab as plt
from matplotlib import ticker
from copy import deepcopy
try:
    import cPickle as pickle
except ImportError:
    import pickle

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Plots.Mission_Plots import *
from Plots import plot_mission, plot_mission_single

def results_show(results,vehicle,iteration_setup,first_climb_segment,last_climb_segment,last_descent_segment,first_descent_segment,\
                 first_reserve_climb_segment,last_reserve_descent_segment,first_reserve_descent_segment,last_reserve_climb_segment,\
                 climb_segments):
    plot_mission(results)
    plot_flight_conditions(results, 'bo-', False)
    plot_aerodynamic_forces(results, 'bo-', False)
    plot_mission_single(results)
    plot_stability_coefficients(results, 'bo-', False)
    # Plot Aircraft Electronics
    plot_electronic_conditions(results, 'bo-')

    # Plot Propeller Conditions
    plot_delft_propeller_conditions(results, 'propeller', 'bo-')


    # Plot Electric Motor and Propeller Efficiencies
    #plot_eMotor_Prop_efficiencies(results, 'bo-')

    #plot_altitude_sfc_weight(results, 'bo-', False)

    # Mission summary
    print('\n%s' % iteration_setup.mission_iter.Mission_name)

    # Fuel
    block_fuel = results.segments[first_climb_segment].conditions.weights.total_mass[0] - \
                 results.segments[last_descent_segment].conditions.weights.total_mass[-1]

    climb_fuel = results.segments[first_climb_segment].conditions.weights.total_mass[0] - \
                 results.segments[last_climb_segment].conditions.weights.total_mass[-1]

    cruise_fuel = results.segments.cruise.conditions.weights.total_mass[0] - \
                  results.segments.cruise.conditions.weights.total_mass[-1]

    descent_fuel = results.segments[first_descent_segment].conditions.weights.total_mass[0] - \
                   results.segments[last_descent_segment].conditions.weights.total_mass[-1]

    if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        second_leg_block_fuel = results.segments[first_second_leg_climb_segment].conditions.weights.total_mass[0] - \
                                results.segments[last_second_leg_descent_segment].conditions.weights.total_mass[-1]

        second_leg_climb_fuel = results.segments[first_second_leg_climb_segment].conditions.weights.total_mass[0] - \
                                results.segments[last_second_leg_climb_segment].conditions.weights.total_mass[-1]

        second_leg_cruise_fuel = results.segments.second_leg_cruise.conditions.weights.total_mass[0] - \
                                 results.segments.second_leg_cruise.conditions.weights.total_mass[-1]

        second_leg_descent_fuel = results.segments[first_second_leg_descent_segment].conditions.weights.total_mass[0] - \
                                  results.segments[last_second_leg_descent_segment].conditions.weights.total_mass[-1]

    reserve_block_fuel = results.segments[first_reserve_climb_segment].conditions.weights.total_mass[0] - \
                         results.segments[last_reserve_descent_segment].conditions.weights.total_mass[-1]

    reserve_climb_fuel = results.segments[first_reserve_climb_segment].conditions.weights.total_mass[0] - \
                         results.segments[last_reserve_climb_segment].conditions.weights.total_mass[-1]

    reserve_cruise_fuel = results.segments.reserve_cruise.conditions.weights.total_mass[0] - \
                          results.segments.reserve_cruise.conditions.weights.total_mass[-1]

    reserve_descent_fuel = results.segments[first_reserve_descent_segment].conditions.weights.total_mass[0] - \
                           results.segments[last_reserve_descent_segment].conditions.weights.total_mass[-1]

    reserve_hold_fuel = results.segments.hold.conditions.weights.total_mass[0] - \
                        results.segments.hold.conditions.weights.total_mass[-1]

    reserve_additional_fuel = block_fuel * iteration_setup.mission_iter.reserve_trip_pct
    if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        reserve_additional_fuel += second_leg_block_fuel * iteration_setup.mission_iter.reserve_trip_pct
    reserve_total_fuel = reserve_block_fuel + reserve_hold_fuel + reserve_additional_fuel

    # Time
    block_time = (results.segments[last_descent_segment].conditions.frames.inertial.time[-1] - \
                  results.segments[first_climb_segment].conditions.frames.inertial.time[0]) / Units['min']

    climb_time = (results.segments[last_climb_segment].conditions.frames.inertial.time[-1] - \
                  results.segments[first_climb_segment].conditions.frames.inertial.time[0]) / Units['min']

    cruise_time = (results.segments.cruise.conditions.frames.inertial.time[-1] - \
                   results.segments.cruise.conditions.frames.inertial.time[0]) / Units['min']

    descent_time = (results.segments[last_descent_segment].conditions.frames.inertial.time[-1] - \
                    results.segments[first_descent_segment].conditions.frames.inertial.time[0]) / Units['min']

    if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        second_leg_block_time = (results.segments[last_second_leg_descent_segment].conditions.frames.inertial.time[-1] - \
                                 results.segments[first_second_leg_climb_segment].conditions.frames.inertial.time[0]) / \
                                Units['min']

        second_leg_climb_time = (results.segments[last_second_leg_climb_segment].conditions.frames.inertial.time[-1] - \
                                 results.segments[first_second_leg_climb_segment].conditions.frames.inertial.time[0]) / \
                                Units['min']

        second_leg_cruise_time = (results.segments.second_leg_cruise.conditions.frames.inertial.time[-1] - \
                                  results.segments.second_leg_cruise.conditions.frames.inertial.time[0]) / Units['min']

        second_leg_descent_time = (results.segments[last_second_leg_descent_segment].conditions.frames.inertial.time[-1] - \
                                   results.segments[first_second_leg_descent_segment].conditions.frames.inertial.time[0]) / \
                                  Units['min']

    reserve_block_time = (results.segments[last_reserve_descent_segment].conditions.frames.inertial.time[-1] - \
                          results.segments[first_reserve_climb_segment].conditions.frames.inertial.time[0]) / Units['min']

    reserve_climb_time = (results.segments[last_reserve_climb_segment].conditions.frames.inertial.time[-1] - \
                          results.segments[first_reserve_climb_segment].conditions.frames.inertial.time[0]) / Units['min']

    reserve_cruise_time = (results.segments.reserve_cruise.conditions.frames.inertial.time[-1] - \
                           results.segments.reserve_cruise.conditions.frames.inertial.time[0]) / Units['min']

    reserve_descent_time = (results.segments[last_reserve_descent_segment].conditions.frames.inertial.time[-1] - \
                            results.segments[first_reserve_descent_segment].conditions.frames.inertial.time[0]) / Units[
                               'min']

    reserve_hold_time = (results.segments.hold.conditions.frames.inertial.time[-1] - \
                         results.segments.hold.conditions.frames.inertial.time[0]) / Units['min']

    reserve_total_time = reserve_block_time + reserve_hold_time

    # Distance
    block_distance = (results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                      results.segments[first_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units[
                         'nautical_mile']

    climb_distance = (results.segments[last_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                      results.segments[first_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units[
                         'nautical_mile']

    cruise_distance = (results.segments.cruise.conditions.frames.inertial.position_vector[-1][0] - \
                       results.segments.cruise.conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

    descent_distance = (results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                        results.segments[first_descent_segment].conditions.frames.inertial.position_vector[0][0]) / Units[
                           'nautical_mile']

    if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        second_leg_block_distance = (results.segments[
                                         last_second_leg_descent_segment].conditions.frames.inertial.position_vector[-1][
                                         0] - \
                                     results.segments[
                                         first_second_leg_climb_segment].conditions.frames.inertial.position_vector[0][0]) / \
                                    Units['nautical_mile']

        second_leg_climb_distance = (results.segments[
                                         last_second_leg_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                                     results.segments[
                                         first_second_leg_climb_segment].conditions.frames.inertial.position_vector[0][0]) / \
                                    Units['nautical_mile']

        second_leg_cruise_distance = (results.segments.second_leg_cruise.conditions.frames.inertial.position_vector[-1][0] - \
                                      results.segments.second_leg_cruise.conditions.frames.inertial.position_vector[0][0]) / \
                                     Units['nautical_mile']

        second_leg_descent_distance = (results.segments[
                                           last_second_leg_descent_segment].conditions.frames.inertial.position_vector[-1][
                                           0] - \
                                       results.segments[
                                           first_second_leg_descent_segment].conditions.frames.inertial.position_vector[0][
                                           0]) / Units['nautical_mile']

    reserve_block_distance = (results.segments[last_reserve_descent_segment].conditions.frames.inertial.position_vector[-1][
                                  0] - \
                              results.segments[first_reserve_climb_segment].conditions.frames.inertial.position_vector[0][
                                  0]) / Units['nautical_mile']

    reserve_climb_distance = (results.segments[last_reserve_climb_segment].conditions.frames.inertial.position_vector[-1][
                                  0] - \
                              results.segments[first_reserve_climb_segment].conditions.frames.inertial.position_vector[0][
                                  0]) / Units['nautical_mile']

    reserve_cruise_distance = (results.segments.reserve_cruise.conditions.frames.inertial.position_vector[-1][0] - \
                               results.segments.reserve_cruise.conditions.frames.inertial.position_vector[0][0]) / Units[
                                  'nautical_mile']

    reserve_descent_distance = (results.segments[last_reserve_descent_segment].conditions.frames.inertial.position_vector[
                                    -1][0] - \
                                results.segments[first_reserve_descent_segment].conditions.frames.inertial.position_vector[
                                    0][0]) / Units['nautical_mile']

    reserve_hold_distance = (results.segments.hold.conditions.frames.inertial.position_vector[-1][0] - \
                             results.segments.hold.conditions.frames.inertial.position_vector[0][0]) / Units[
                                'nautical_mile']

    reserve_total_distance = reserve_block_distance + reserve_hold_distance

    print('\nBOW:  %7.1f kg' % iteration_setup.weight_iter.BOW / Units.kg)
    print('PAX:  %7.1f kg' % iteration_setup.weight_iter.Design_Payload / Units.kg)
    print('FUEL: %7.1f kg' % iteration_setup.weight_iter.FUEL / Units.kg)
    print('TOW:  %7.1f kg' % iteration_setup.weight_iter.TOW / Units.kg)

    print('\nSegment      Fuel [kg]    Time [min]    Distance [nm]    SR [nm/kg]')
    print('Climb        %7.2f      %7.2f         %7.2f        %7.3f' % (
    climb_fuel, climb_time, climb_distance, climb_distance / climb_fuel))
    print('Cruise       %7.2f      %7.2f         %7.2f        %7.3f' % (
    cruise_fuel, cruise_time, cruise_distance, cruise_distance / cruise_fuel))
    print('Descent      %7.2f      %7.2f         %7.2f        %7.3f' % (
    descent_fuel, descent_time, descent_distance, descent_distance / descent_fuel))
    print('Total        %7.2f      %7.2f         %7.2f        %7.3f\n' % (
    block_fuel, block_time, block_distance, block_distance / block_fuel))
    if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        print('Climb2       %7.2f      %7.2f         %7.2f        %7.3f' % (
        second_leg_climb_fuel, second_leg_climb_time, second_leg_climb_distance,
        second_leg_climb_distance / second_leg_climb_fuel))
        print('Cruise2      %7.2f      %7.2f         %7.2f        %7.3f' % (
        second_leg_cruise_fuel, second_leg_cruise_time, second_leg_cruise_distance,
        second_leg_cruise_distance / second_leg_cruise_fuel))
        print('Descent2     %7.2f      %7.2f         %7.2f        %7.3f' % (
        second_leg_descent_fuel, second_leg_descent_time, second_leg_descent_distance,
        second_leg_descent_distance / second_leg_descent_fuel))
        print('Total2       %7.2f      %7.2f         %7.2f        %7.3f\n' % (
        second_leg_block_fuel, second_leg_block_time, second_leg_block_distance,
        second_leg_block_distance / second_leg_block_fuel))
    print('Climb Alt.   %7.2f      %7.2f         %7.2f        %7.3f' % (
    reserve_climb_fuel, reserve_climb_time, reserve_climb_distance, reserve_climb_distance / reserve_climb_fuel))
    print('Cruise Alt.  %7.2f      %7.2f         %7.2f        %7.3f' % (
    reserve_cruise_fuel, reserve_cruise_time, reserve_cruise_distance, reserve_cruise_distance / reserve_cruise_fuel))
    print('Descent Alt. %7.2f      %7.2f         %7.2f        %7.3f' % (
    reserve_descent_fuel, reserve_descent_time, reserve_descent_distance, reserve_descent_distance / reserve_descent_fuel))
    print('Alternate    %7.2f      %7.2f         %7.2f        %7.3f' % (
    reserve_block_fuel, reserve_block_time, reserve_block_distance, reserve_block_distance / reserve_block_fuel))
    print('Hold         %7.2f      %7.2f         %7.2f        %7.3f' % (
    reserve_hold_fuel, reserve_hold_time, reserve_hold_distance, reserve_hold_distance / reserve_hold_fuel))
    print('Additional   %7.2f            -               -              -' % (reserve_additional_fuel))
    print('Total        %7.2f      %7.2f         %7.2f        %7.3f' % (
    reserve_total_fuel, reserve_total_time, reserve_total_distance, reserve_total_distance / reserve_total_fuel))

    velocity = results.segments.climb_1.conditions.freestream.velocity[0][0]
    thrust = results.segments.climb_1.conditions.frames.body.thrust_force_vector[0][0]
    drag = -results.segments.climb_1.conditions.frames.wind.drag_force_vector[0][0]
    weight = results.segments.climb_1.conditions.weights.total_mass[0][0]
    ROC_1 = (velocity * (thrust - drag) / (weight * 9.81)) * 196.8504  # ft/min

    velocity = results.segments.climb_2.conditions.freestream.velocity[0][0]
    thrust = results.segments.climb_2.conditions.frames.body.thrust_force_vector[0][0]
    drag = -results.segments.climb_2.conditions.frames.wind.drag_force_vector[0][0]
    weight = results.segments.climb_2.conditions.weights.total_mass[0][0]
    ROC_2 = (velocity * (thrust - drag) / (weight * 9.81)) * 196.8504  # ft/min

    ROC = (ROC_1)
    print('\nROC (ISA, SL, MTOW): %.1f ft/min' % ROC)
    print('\nROC2 (ISA, SL, MTOW): %.1f ft/min' % ROC_2)

    velocity = results.segments[last_climb_segment].conditions.freestream.velocity[-1][0]
    thrust = results.segments[last_climb_segment].conditions.frames.body.thrust_force_vector[-1][0]
    drag = -results.segments[last_climb_segment].conditions.frames.wind.drag_force_vector[-1][0]
    weight = results.segments[last_climb_segment].conditions.weights.total_mass[-1][0]
    ROC_TOC = (velocity * (thrust - drag) / (weight * 9.81)) * 196.8504  # ft/min
    print('ROC (end of climb): %.1f ft/min' % ROC_TOC)

    altitude = []
    time = []
    for segment in climb_segments:
        time.append(results.segments[segment].conditions.frames.inertial.time[0][0] / Units.min)
        altitude.append(results.segments[segment].conditions.freestream.altitude[0][0] / Units.ft)
    time.append(results.segments[segment].conditions.frames.inertial.time[-1][0] / Units.min)
    altitude.append(results.segments[segment].conditions.freestream.altitude[-1][0] / Units.ft)
    ttc_170 = np.interp(17000, altitude, time)
    print('Time to climb to FL170: %.2f min' % ttc_170)

    print('Cruise fuel flow: %.1f kg/h' % (cruise_fuel / (cruise_time / 60)))

    m4_m3 = results.segments[last_climb_segment].conditions.weights.total_mass[-1] / \
            results.segments[first_climb_segment].conditions.weights.total_mass[0]

    m5_m4 = results.segments.cruise.conditions.weights.total_mass[-1] / \
            results.segments.cruise.conditions.weights.total_mass[0]

    m6_m5 = results.segments[last_descent_segment].conditions.weights.total_mass[-1] / \
            results.segments[first_descent_segment].conditions.weights.total_mass[0]

    m7_m6 = results.segments[last_reserve_climb_segment].conditions.weights.total_mass[-1] / \
            results.segments[first_reserve_climb_segment].conditions.weights.total_mass[0]

    m8_m7 = results.segments[last_reserve_descent_segment].conditions.weights.total_mass[-1] / \
            results.segments.reserve_cruise.conditions.weights.total_mass[0]

    m9_m8 = results.segments.hold.conditions.weights.total_mass[-1] / \
            results.segments.hold.conditions.weights.total_mass[0]


    print('m4/m3: %.3f' % m4_m3)
    print('m5/m4: %.3f' % m5_m4)
    print('m6/m5: %.3f' % m6_m5)
    print('m7/m6: %.3f' % m7_m6)
    print('m8/m7: %.3f' % m8_m7)
    print('m9/m8: %.3f' % m9_m8)


    mdot = results.segments.cruise.conditions.weights.vehicle_mass_rate[:, 0]
    power = results.segments.cruise.conditions.propulsion.power[:, 0]
    sfc_p = (mdot / Units.lb) / (power / Units.hp) * Units.hr
    print('SFC_P %.3f %.3f %.3f %.3f'  % (sfc_p[0], sfc_p[1], sfc_p[2], sfc_p[3]))

    #plt.show()
    # run payload diagram
    # vehicle = configs.base
    # cruise_segment_tag = "cruise"
    # reserves = 0.# 0.03 * vehicle.mass_properties.fuel #+ 3500
    # payload_range_results = payload_range(vehicle,mission,cruise_segment_tag,reserves)

    a = vehicle



    print('\n','*'* 74, '\n{:^76s}\n'.format('Mass Properties'),'','*'* 74)
    print('Wing Mass %.2f kg' % vehicle.wings.main_wing.mass_properties.mass)
    print('Hor Mass %.2f kg' % vehicle.wings.horizontal_stabilizer.mass_properties.mass)
    print('Ver Mass %.2f kg' % vehicle.wings.vertical_stabilizer.mass_properties.mass)
    print('Fuse Mass %.2f kg' % vehicle.fuselages.fuselage.mass_properties.mass)
    print('Gear Mass %.2f kg' % (vehicle.landing_gear.main.mass_properties.mass+vehicle.landing_gear.nose.mass_properties.mass))
    print('*'* 74)
    print('Str %.2f kg' % vehicle.mass_properties.structures)
    print('Prop Mass %.2f kg' % vehicle.mass_properties.propulsion)
    print('Sys Mass %.2f kg' % vehicle.mass_properties.systems)
    print('Op Mass %.2f kg' % vehicle.mass_properties.operationals)
    print('Battery Mass %.2f kg' % vehicle.propulsors.network.battery.mass_properties.mass)
    print('*'* 74)
    print('OEM %.2f kg' % vehicle.mass_properties.operating_empty)
    print('Pay Mass %.2f kg' % vehicle.mass_properties.payload)
    print('Fuel Mass %.2f kg' % vehicle.systems.fuel.mass_properties.mass)
    print('*'* 74)
    print('MTOM %.2f kg' % vehicle.mass_properties.max_takeoff)
    print('\n')

    print('\n','*'* 74, '\n{:^76s}\n'.format('Electric Component Mass Properties'),'','*'* 74)
    print('Motor Mass %.2f kg' % vehicle.systems.emotor.mass_properties.mass)
    print('MotorWTP Mass %.2f kg' % vehicle.systems.emotorWTP.mass_properties.mass)
    print('esc Mass %.2f kg' % vehicle.systems.esc.mass_properties.mass)
    print('escWTP Mass %.2f kg' % vehicle.systems.escWTP.mass_properties.mass)
    print('dcdc Mass %.2f kg' % vehicle.systems.dcdc.mass_properties.mass)
    print('Engine Mass %.2f kg' % vehicle.systems.turboshaft.mass_properties.mass)
    print('GearBox Mass %.2f kg' % vehicle.systems.gearbox.mass_properties.mass)
    print('GearBoxWTP Mass %.2f kg' % vehicle.systems.gearboxWTP.mass_properties.mass)
    print('Propeller Mass %.2f kg' % vehicle.systems.propeller.mass_properties.mass)
    print('PropellerWTP Mass %.2f kg' % vehicle.systems.propellerWTP.mass_properties.mass)
    print('Cable Mass %.2f kg' % vehicle.systems.cable.mass_properties.mass)
    print('TMS VCS Mass %.2f kg' % vehicle.systems.tms_vcs.mass_properties.mass)
    print('TMS Liquid Mass %.2f kg' % vehicle.systems.tms_liquid.mass_properties.mass)



    print('\n','*'* 74, '\n{:^76s}\n'.format('CoG and Stability'),'','*'* 74)
    print('Center of Gravity %.2f m' % vehicle.mass_properties.center_of_gravity[0][0])
    print('xMAC t/4 %.2f m' % (vehicle.wings.main_wing.origin[0][0] +
                               vehicle.wings.main_wing.aerodynamic_center[0]))
    Percent_MAC = (vehicle.mass_properties.center_of_gravity[0][0] - (vehicle.wings.main_wing.origin[0][0] +
                                                                      vehicle.wings.main_wing.aerodynamic_center[0] - 0.25 * vehicle.wings.main_wing.chords.mean_aerodynamic))/vehicle.wings.main_wing.chords.mean_aerodynamic * 100

    print('CG %.2f %%MAC' % Percent_MAC)
    print('l_vt %.2f' % vehicle.wings.vertical_stabilizer.l_vt)
    print('l_ht %.2f' % vehicle.wings.horizontal_stabilizer.l_ht)
    print('S_vt %.2f' % vehicle.wings.vertical_stabilizer.areas.reference)
    print('S_ht %.2f' % vehicle.wings.horizontal_stabilizer.areas.reference)
    print('Span Wing %.2f' % vehicle.wings.main_wing.spans.projected)



    ####################################################################################################################
    # Output in txt file
    ####################################################################################################################

    output_write = False

    if output_write == True:
        time_output = []
        altitude_output = []
        machnumber_output = []
        power_battery = []
        thrust_aircraft = []
        thrust_turboprop = []
        thrust_WTP = []
        power_propeller_turboprop = []
        power_turboshaft = []
        power_motor_turboprop = []
        power_propeller_WTP = []
        propeller_rpm = []
        heat_load = []

        for segment in results.segments.values():
            time = segment.conditions.frames.inertial.time[:,0]
            time_output.extend(time)

            altitude = segment.conditions.freestream.altitude[:,0]
            altitude_output.extend(altitude)

            machnumber = segment.conditions.freestream.mach_number[:,0]
            machnumber_output.extend(machnumber)

            thrust = segment.conditions.frames.body.thrust_force_vector[:,0]
            thrust_aircraft.extend(thrust)

            turboprop_thrust = segment.conditions.propulsion.thrust_turboprop[:,0]
            thrust_turboprop.extend(turboprop_thrust)

            WTP_thrust = segment.conditions.propulsion.thrust_WTP[:,0]
            thrust_WTP.extend(WTP_thrust)

            battery_draw = -segment.conditions.propulsion.battery_draw[:,0]
            power_battery.extend(battery_draw)

            power_propeller_turboprop.extend(segment.conditions.propulsion.power_propeller_turboprop[:,0])

            power_turboshaft.extend(segment.conditions.propulsion.power_turboshaft[:,0])

            power_motor_turboprop.extend(segment.conditions.propulsion.power_motor_turboprop[:,0])

            power_propeller_WTP.extend(segment.conditions.propulsion.power_propeller_WTP[:,0])

            propeller_rpm.extend(segment.conditions.propulsion.propeller_rpm[:,0])

            heat_load.extend(segment.conditions.propulsion.heat_load[:,0])






        # Append to one variable
        parameter_over_time = []

        parameter_over_time.append((np.array(time_output)).T)
        parameter_over_time.append((np.array(altitude_output)).T)
        parameter_over_time.append((np.array(machnumber_output)).T)
        parameter_over_time.append(np.array(thrust_aircraft).T)
        parameter_over_time.append(np.array(thrust_turboprop).T)
        parameter_over_time.append(np.array(thrust_WTP).T)
        parameter_over_time.append(np.array(power_battery).T)
        parameter_over_time.append(np.array(power_propeller_turboprop).T)
        parameter_over_time.append(np.array(power_turboshaft).T)
        parameter_over_time.append(np.array(power_motor_turboprop).T)
        parameter_over_time.append(np.array(power_propeller_WTP).T)
        parameter_over_time.append(np.array(propeller_rpm).T)
        parameter_over_time.append(np.array(heat_load).T)

        # Convert and transpose
        parameter_over_time_txt = (np.array(parameter_over_time)).T

        np.savetxt('Parameter_over_time.txt', parameter_over_time_txt,delimiter='; ',header='Time [s]; Altitude [m]; Mach number [-]; Thrust_Aircraft [N]; Thrust_Turboprop (1x) [N]; Thrust_WTP (1x) [N]; Battery Power (total) [W]; Power_Propeller_Turboprop (1x) [W]; Power_Turboshaft (1x) [W]; Power_Motor_Turboprop (1x) [W]; Power_Propeller_WTP (1x) [W]; Propeller_rpm [min^-1]; Heat Load (total) [W]')


    ####################################################################################################################
    # Open Plots
    ####################################################################################################################
    plt.show()
