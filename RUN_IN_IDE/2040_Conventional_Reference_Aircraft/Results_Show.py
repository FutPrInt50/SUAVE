## Conventional_Reference_Aircraft.py
# 
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Jonas Mangold, mangold@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Sep 2021, J. Mangold
# Modified: Jul 2022 F.Brenner

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
from SUAVE.Methods.Performance  import payload_range_variable_reserve

def results_show(results,vehicle,iteration_setup,first_climb_segment,last_climb_segment,last_descent_segment,first_descent_segment,\
                 first_reserve_climb_segment,last_reserve_descent_segment,first_reserve_descent_segment,last_reserve_climb_segment,\
                 climb_segments):
    plot_mission(results,vehicle)
    plot_flight_conditions(results, 'bo-', False)
    plot_aerodynamic_forces(results, 'bo-', False)
    plot_stability_coefficients(results, 'bo-', False)
    plot_mission_single(results)

#    plot_delft_propeller_conditions(results, 'propeller')

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

    print('\nOEW1:  %7.1f kg' % vehicle.mass_properties.operating_empty)
    print('PAX1:  %7.1f kg' % (vehicle.payload.passengers.mass_properties.mass + vehicle.payload.baggage.mass_properties.mass + vehicle.payload.cargo.mass_properties.mass))
    print('FUEL1: %7.1f kg' % vehicle.systems.fuel.mass_properties.mass)
    print('TOW1:  %7.1f kg' % vehicle.mass_properties.takeoff)

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

    altitude = np.array([])
    time = np.array([])
    for segment in climb_segments:
        time = np.append(time, results.segments[segment].conditions.frames.inertial.time[:, 0] / Units.min)
        altitude = np.append(altitude, results.segments[segment].conditions.freestream.altitude[:, 0] / Units.ft)
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

    mdot = results.segments.reserve_descent_5.conditions.weights.vehicle_mass_rate[:, 0]
    power = results.segments.reserve_descent_5.conditions.propulsion.power[:, 0]
    sfc_p = (mdot / Units.lb) / (power / Units.hp) * Units.hr
    print('SFC_P %.3f %.3f %.3f %.3f'  % (sfc_p[0], sfc_p[1], sfc_p[2], sfc_p[3]))

    mdot = results.segments.reserve_descent_6.conditions.weights.vehicle_mass_rate[:, 0]
    power = results.segments.reserve_descent_6.conditions.propulsion.power[:, 0]
    sfc_p = (mdot / Units.lb) / (power / Units.hp) * Units.hr
    print('SFC_P %.3f %.3f %.3f %.3f'  % (sfc_p[0], sfc_p[1], sfc_p[2], sfc_p[3]))

    #plt.show()
    # run payload diagram
    # vehicle = configs.base
    # cruise_segment_tag = "cruise"
    # reserves = 0.# 0.03 * vehicle.mass_properties.fuel #+ 3500
    # payload_range_results = payload_range(vehicle,mission,cruise_segment_tag,reserves)

    a = vehicle

    cl = results.segments.cruise.conditions.aerodynamics.lift_coefficient[:,0,None]
    cd = results.segments.cruise.conditions.aerodynamics.drag_coefficient[:,0,None]
    LoverD = cl/cd
    sumLoverD = sum(LoverD)
    number = len(LoverD)
    mittelwert = sumLoverD/number
    print('LoD mid_Cruise %.2f' % mittelwert)


    print(' ','*'* 74, '\n{:^76s}\n'.format('Mass Properties'),' ','*'* 74  )
    print('OEM %.2f kg' % vehicle.mass_properties.operating_empty)
    print('-')
    print('Wing Mass %.2f kg' % vehicle.wings.main_wing.mass_properties.mass)
    print('Wing total CG %.2f m' % ((vehicle.wings.main_wing.origin[0][0]) + (
    vehicle.wings.main_wing.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('HTP Mass %.2f kg' % vehicle.wings.horizontal_stabilizer.mass_properties.mass)
    print('HTP total CG %.2f m' % ((vehicle.wings.horizontal_stabilizer.origin[0][0]) + (
    vehicle.wings.horizontal_stabilizer.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('VTP Mass %.2f kg' % vehicle.wings.vertical_stabilizer.mass_properties.mass)
    print('VTP total CG %.2f m' % ((vehicle.wings.vertical_stabilizer.origin[0][0]) + (
    vehicle.wings.vertical_stabilizer.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Tail Mass %.2f kg' % (
                vehicle.wings.horizontal_stabilizer.mass_properties.mass + vehicle.wings.vertical_stabilizer.mass_properties.mass))
    print('-')
    print('ENG Mass %.2f kg' % vehicle.propulsors.network.mass_properties.mass)
    print('Eng total CG %.2f m' % ((vehicle.propulsors.network.origin[0][0]) + (
    vehicle.propulsors.network.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Propeller Mass %.2f kg' % vehicle.systems.propeller.mass_properties.mass)
    print('Propeller total CG %.2f m' % ((vehicle.systems.propeller.origin[0][0]) + (
    vehicle.systems.propeller.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Gearbox Mass %.2f kg' % vehicle.systems.gearbox.mass_properties.mass)
    print('Gearbox total CG %.2f m' % ((vehicle.systems.gearbox.origin[0][0]) + (
    vehicle.systems.gearbox.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Turboshaft Mass %.2f kg' % vehicle.systems.turboshaft.mass_properties.mass)
    print('Turboshaft total CG %.2f m' % ((vehicle.systems.turboshaft.origin[0][0]) + (
    vehicle.systems.turboshaft.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Fuse Mass %.2f kg' % vehicle.fuselages.fuselage.mass_properties.mass)
    print('Fuse total CG %.2f m' % vehicle.fuselages.fuselage.mass_properties.center_of_gravity[0][0])
    print('-')
    print('Main Gear Mass %.2f kg' % (vehicle.landing_gear.main.mass_properties.mass))
    print('Main Gear total CG %.2f m' % ((vehicle.landing_gear.main.origin[0][0]) + (
    vehicle.landing_gear.main.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Nose Gear Mass %.2f kg' % (vehicle.landing_gear.nose.mass_properties.mass))
    print('Nose Gear total CG %.2f m' % ((vehicle.landing_gear.nose.origin[0][0]) + (
    vehicle.landing_gear.nose.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Gear Mass %.2f kg' % (
                (vehicle.landing_gear.main.mass_properties.mass) + (vehicle.landing_gear.nose.mass_properties.mass)))
    print('-')
    print('Str %.2f kg' % vehicle.mass_properties.structures)
    print('-')
    print('System Mass total %.2f kg' % vehicle.mass_properties.systems)
    print('-')
    print('Control System Mass %.2f kg' % vehicle.systems.control_systems.mass_properties.mass)
    print('Control System total CG %.2f m' % ((vehicle.systems.control_systems.origin[0][0]) + (
    vehicle.systems.control_systems.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Electrical System Mass %.2f kg' % vehicle.systems.electrical_systems.mass_properties.mass)
    print('Electrical System total CG %.2f m' % ((vehicle.systems.electrical_systems.origin[0][0]) + (
    vehicle.systems.electrical_systems.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Avionics Mass %.2f kg' % vehicle.systems.avionics.mass_properties.mass)
    print('Avionics total CG %.2f m' % ((vehicle.systems.avionics.origin[0][0]) + (
    vehicle.systems.avionics.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Air Con Mass %.2f kg' % vehicle.systems.air_conditioner.mass_properties.mass)
    print('Air Con total CG %.2f m' % ((vehicle.systems.air_conditioner.origin[0][0]) + (
    vehicle.systems.air_conditioner.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Hydraulics Mass %.2f kg' % vehicle.systems.hydraulics.mass_properties.mass)
    print('Hydraulics total CG %.2f m' % ((vehicle.systems.hydraulics.origin[0][0]) + (
    vehicle.systems.hydraulics.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Design Fuel Mass %.2f kg' % vehicle.systems.fuel.mass_properties.mass)
    print('Design Fuel total CG %.2f m' % (
                (vehicle.systems.fuel.origin[0][0]) + (vehicle.systems.fuel.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Furnishing Mass %.2f kg' % vehicle.systems.furnishings.mass_properties.mass)
    print('Furnishing total CG %.2f m' % ((vehicle.systems.furnishings.origin[0][0]) + (
    vehicle.systems.furnishings.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Payload Mass %.2f kg' % vehicle.mass_properties.payload)
    print('Payload total CG %.2f m' % ((vehicle.payload.passengers.origin[0][0]) + (
    vehicle.payload.passengers.mass_properties.center_of_gravity[0][0])))
    print('Oprational Items %.2f kg' % vehicle.systems.optionals.mass_properties.mass)
    print('-')

    print('Crew %.2f kg' % vehicle.systems.crew.mass_properties.mass)
    print('Crew total CG %.2f m' % (
                (vehicle.systems.crew.origin[0][0]) + (vehicle.systems.crew.mass_properties.center_of_gravity[0][0])))
    print('Oprational Items total CG %.2f m' % ((vehicle.systems.optionals.origin[0][0]) + (
    vehicle.systems.optionals.mass_properties.center_of_gravity[0][0])))
    print('-')
    print('Oprational Items Mass %.2f kg' % vehicle.mass_properties.operationals)
    # print('Center of Gravity check %.2f m' %  ((1601.03*9.19) + (184.50*20.38) + (270.07*18.81) + (1737.00*8.43) + (2085.71*9.07) + (554.96*10.17) + (115.43*1.86) + (328.25*9.42) + (465.15*12.41) + (988.89 * 1.36) + (431.74* 3.41) + (90.55 * 12.73)+ (2789.98 * 9.93) + (1196.49 * 11.11) + (4560.00 * 10.70) + (1200.31 * 11.11)) / (1601.03+454.57+1737.00+2085.71+670.39+3501.06+4560.00+1200.31))

    print('Center of Gravity %.4f m' % vehicle.mass_properties.center_of_gravity[0][0])
    print('ZF Center of Gravity %.2f m' % vehicle.mass_properties.zero_fuel_center_of_gravity[0][0])

    print('xMAC t/4 %.2f m' % (vehicle.wings.main_wing.origin[0][0] +
                               vehicle.wings.main_wing.aerodynamic_center[0]))
    Percent_MAC = (vehicle.mass_properties.center_of_gravity[0][0] - (vehicle.wings.main_wing.origin[0][0] +
                                                                      vehicle.wings.main_wing.aerodynamic_center[0])) \
                  / vehicle.wings.main_wing.chords.mean_aerodynamic * 100
    print('CG %.2f %%MAC' % Percent_MAC)
    print('l_vt %.2f' % vehicle.wings.vertical_stabilizer.l_vt)
    print('l_ht %.2f' % vehicle.wings.horizontal_stabilizer.l_ht)
    print('S_vt %.2f' % vehicle.wings.vertical_stabilizer.areas.reference)
    print('S_ht %.2f' % vehicle.wings.horizontal_stabilizer.areas.reference)
    print('Span Wing %.2f' % vehicle.wings.main_wing.spans.projected)

    print('Power Sea Level Static %.2f' % vehicle.propulsors.network.sea_level_power)
    plt.show()
