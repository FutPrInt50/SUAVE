## @ingroup Methods-Performance
# sizing_chart.py
#
# Created:  May 2022, J.Mangold
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Data, Units
import time
import numpy as np
import SUAVE
# ----------------------------------------------------------------------
#  Calculate Vehicle Sizing Chart
# ----------------------------------------------------------------------

## @ingroup Methods-Performance
def sizing_chart_mission_evaluate(vehicle,mission,iteration_setup):
    """Calculates a vehicle's sizing chart. Includes plotting.

    Assumptions:
    tbd

    Source:
    N/A

    Inputs:
    vehicle.mass_properties.
      operating_empty                     [kg]
      max_zero_fuel                       [kg]
      max_takeoff                         [kg]
      max_payload                         [kg]
      max_fuel                            [kg]
      takeoff                             [kg]
    mission.segments[0].analyses.weights.
      vehicle.mass_properties.takeoff     [kg]
    cruise_segment_tag                    <string>

    Outputs:
    payload_range.
      range                             [m]
      payload                           [kg]
      fuel                              [kg]
      takeoff_weight                    [kg]
    PayloadRangeDiagram.dat (text file)

    Properties Used:
    N/A
    """

    ####################################################################################################################
    # load sizing chart mission beforehand
    ####################################################################################################################


    ####################################################################################################################
    # definition
    ####################################################################################################################
    #print('Running Sizing Chart')

    MTOM =  vehicle.mass_properties.max_takeoff
    mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff = MTOM

    number_of_engines = vehicle.propulsors.network.number_of_engines
    ref_area= vehicle.wings.main_wing.areas.reference

    sizing_chart_mission_evaluate.power = Data()
    sizing_chart_mission_evaluate.power_SL_static_one_engine = Data()


    ####################################################################################################################
    # run sizing chart mission
    ####################################################################################################################


    results = mission.evaluate()



    ####################################################################################################################
    # postprocessing
    ####################################################################################################################

    sizing_chart_mission_evaluate.results = results

    #print('-')
    ####################################################################################################################
    # Second Segment OEI


    #m2_m0 = 0.99

    power_second_segment_OEI = results.segments.second_segment_oei.conditions.propulsion.power[:, 0]

    power_second_segment_OEI_average = sum(power_second_segment_OEI)/len(power_second_segment_OEI)

    #print('Power Second Segemnt OEI %.2f kW' % (power_second_segment_OEI_average/1000))

    sizing_chart_mission_evaluate.power.second_segment_OEI = power_second_segment_OEI_average

    rho_average  = sum(results.segments.second_segment_oei.conditions.freestream.density[:,0])/len(results.segments.second_segment_oei.conditions.freestream.density[:,0])

    sizing_chart_mission_evaluate.power_SL_static_one_engine.second_segment_OEI = sizing_chart_mission_evaluate.power.second_segment_OEI / (rho_average/1.225 )** 0.5


    ####################################################################################################################
    # Take off Field Length

    v_stall = 45
    v2 = 1.2 * v_stall

    k_TO = 2.45
    s_TOFL = 1165
    eta_TO = 0.74672
    cL_max_TO_new = MTOM * 9.81 / (1.225/2 * (51)**2 * ref_area)
    cL_max_TO = 0.8 * 2.7
    rho_airport = 1.225

    sizing_chart_mission_evaluate.power.take_off = MTOM * 9.81 * v2 / eta_TO * k_TO / (cL_max_TO * rho_airport * s_TOFL) * (MTOM / ref_area) / number_of_engines
    sizing_chart_mission_evaluate.power.take_off_new = MTOM * 9.81 * (51*1.2) / eta_TO * k_TO / (cL_max_TO_new * rho_airport * s_TOFL) * (MTOM / ref_area) / number_of_engines

    #print('Power Take Off %.2f kW' % (sizing_chart_mission_evaluate.power.take_off/1000))#
    #print('Power Take Off New %.2f kW' % (sizing_chart_mission_evaluate.power.take_off_new/1000))

    sizing_chart_mission_evaluate.power_SL_static_one_engine.take_off = sizing_chart_mission_evaluate.power.take_off

    ####################################################################################################################
    # Cruise

    power_cruise = results.segments.cruise.conditions.propulsion.power[:, 0]

    power_cruise_average = sum(power_cruise)/len(power_cruise)

    #print('Power Cruise %.2f kW' % (power_cruise_average/1000/number_of_engines))


    sizing_chart_mission_evaluate.power.cruise = power_cruise_average/number_of_engines

    rho_average  = sum(results.segments.cruise.conditions.freestream.density[:,0])/len(results.segments.cruise.conditions.freestream.density[:,0])

    sizing_chart_mission_evaluate.power_SL_static_one_engine.cruise = sizing_chart_mission_evaluate.power.cruise / (rho_average/1.225 )** 0.5


    ####################################################################################################################
    # Climb Requirment 1850 ft/min MTOM, SL, ISA

    power_climb_1850 = results.segments.climb_1850.conditions.propulsion.power[:, 0]

    power_climb_1850_average = sum(power_climb_1850)/len(power_climb_1850)

    #print('Power Climb 1850ft/min %.2f kW' % (power_climb_1850_average/1000/number_of_engines))

    sizing_chart_mission_evaluate.power.climb_1850 = power_climb_1850_average/number_of_engines

    sizing_chart_mission_evaluate.power_SL_static_one_engine.climb_1850 = sizing_chart_mission_evaluate.power.climb_1850

    ####################################################################################################################
    # Ceiling OEI 18.000 ft +10ISA

    power_ceiling_oei = results.segments.ceiling_oei.conditions.propulsion.power[:, 0]

    power_ceiling_oei_average = sum(power_ceiling_oei)/len(power_ceiling_oei)

    #print('Power Ceiling OEI %.2f kW' % (power_ceiling_oei_average/1000))

    sizing_chart_mission_evaluate.power.ceiling_oei = power_ceiling_oei_average

    rho_average  = sum(results.segments.ceiling_oei.conditions.freestream.density[:,0])/len(results.segments.ceiling_oei.conditions.freestream.density[:,0])

    sizing_chart_mission_evaluate.power_SL_static_one_engine.ceiling_oei = sizing_chart_mission_evaluate.power.ceiling_oei / (rho_average/1.225 )** 0.5


    ####################################################################################################################
    # Service Ceiling 25.000 ft with cruise speed

    power_service_ceiling = results.segments.service_ceiling.conditions.propulsion.power[:, 0]

    power_service_ceiling_average = sum(power_service_ceiling)/len(power_service_ceiling)

    #print('Power ServiceCeiling %.2f kW' % (power_service_ceiling_average/1000/number_of_engines))

    sizing_chart_mission_evaluate.power.service_ceiling = power_service_ceiling_average/number_of_engines

    rho_average  = sum(results.segments.service_ceiling.conditions.freestream.density[:,0])/len(results.segments.service_ceiling.conditions.freestream.density[:,0])

    sizing_chart_mission_evaluate.power_SL_static_one_engine.service_ceiling = sizing_chart_mission_evaluate.power.service_ceiling / (rho_average/1.225 )** 0.5


    ####################################################################################################################
    # Initial Cruise Altitude - Top of Climb - 300 ft/min with climb speed

    power_initial_cruise_altitude = results.segments.initial_cruise_altitude.conditions.propulsion.power[:, 0]

    power_initial_cruise_altitude_average = sum(power_initial_cruise_altitude)/len(power_initial_cruise_altitude)

    #print('Power Initial Cruise Altitude %.2f kW' % (power_initial_cruise_altitude_average/1000/number_of_engines))

    sizing_chart_mission_evaluate.power.initial_cruise_altitude = power_initial_cruise_altitude_average/number_of_engines

    rho_average  = sum(results.segments.initial_cruise_altitude.conditions.freestream.density[:,0])/len(results.segments.initial_cruise_altitude.conditions.freestream.density[:,0])

    sizing_chart_mission_evaluate.power_SL_static_one_engine.initial_cruise_altitude = sizing_chart_mission_evaluate.power.initial_cruise_altitude / (rho_average/1.225 )** 0.5


    # print('-')
    #
    # print('Power One Engine SL Static Takeoff  %.2f kW' % (sizing_chart_mission_evaluate.power_SL_static_one_engine.take_off/1000))
    # print('Power One Engine SL Static 2ng Segment OEI  %.2f kW' % (sizing_chart_mission_evaluate.power_SL_static_one_engine.second_segment_OEI/1000))
    # print('Power One Engine SL Static Climb 1850  %.2f kW' % (sizing_chart_mission_evaluate.power_SL_static_one_engine.climb_1850/1000))
    # print('Power One Engine SL Static Cruise  %.2f kW' % (sizing_chart_mission_evaluate.power_SL_static_one_engine.cruise/1000))
    # print('Power One Engine SL Static Ceiling OEI %.2f kW' % (sizing_chart_mission_evaluate.power_SL_static_one_engine.ceiling_oei/1000))
    # print('Power One Engine SL Static Service Ceiling %.2f kW' % (sizing_chart_mission_evaluate.power_SL_static_one_engine.service_ceiling/1000))
    # print('Power One Engine SL Static Inital Cruise Alt %.2f kW' % ((sizing_chart_mission_evaluate.power_SL_static_one_engine.initial_cruise_altitude/1000)))
    #
    # print('-')
    # power_loadind_max = max(sizing_chart_mission_evaluate.power_SL_static_one_engine)/vehicle.mass_properties.max_takeoff * number_of_engines
    #
    # print('Power Loading Max %.2f W/kg' % power_loadind_max)


    return sizing_chart_mission_evaluate
