## Config_1_parallel_hybrid.py
# 
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Jonas Mangold, mangold@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Jul 2022, J. Mangold
# Modified:

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



def mission_setup_Sizing_Chart(analyses,iteration_setup,parameters=None):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude = 0.0 * Units.ft
    airport.delta_isa = 0.0
    airport.atmosphere = SUAVE.Attributes.Atmospheres.Earth.US_Standard_1976()

    mission.airport = airport

    # unpack Segments module
    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()
    base_segment.state.numerics.number_control_points = 4
    base_segment.temperature_deviation = airport.delta_isa

    number_control_climb = 4
    climb_throttle = 0.9
    idle_throttle = 0.17
    idle_throttle_descent = 0.10



    ########################################################################
    # Start Stuff for Battery_Propeller
    vehicle = analyses.vehicle
    ones_row = base_segment.state.ones_row
    base_segment.process.iterate.unknowns.network        = vehicle.propulsors.network.unpack_unknowns
    base_segment.process.iterate.residuals.network       = vehicle.propulsors.network.residuals
    base_segment.state.residuals.network = 0 * ones_row(6)  # 1 #2

    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.state.unknowns.battery_voltage_under_load = vehicle.propulsors.network.battery.max_voltage * ones_row(1)
    base_segment.battery_energy = vehicle.propulsors.network.battery.max_energy * 0.80

    #hybird
    base_segment.state.unknowns.rpm                             = 1200 * ones_row(1) #nominal rpm propeller --> vehicle setup
    base_segment.state.unknowns.rpm_wtp                         = 4300 * ones_row(1) #nominal rpm propellerWTP --> vehicle setup

    # tms
    base_segment.state.unknowns.resistive_losses                = 30000 * ones_row(1)

    # Electric Component Mid Fid --> E-Motor
    base_segment.state.unknowns.emotor_efficiency                = 0.98 * ones_row(1)
    base_segment.state.unknowns.emotorWTP_efficiency             = 0.98 * ones_row(1)

    if parameters.ems == 1:
        ems = [[1, 1], [1, 1], [1, 1], [0, 0], [-1, 1], [-1, 1]]
    elif parameters.ems == 2:
        ems = [[1, 1], [1, 1], [-1, 1], [0, 0], [-1, 1], [-1, 1]]
    elif parameters.ems == 4:
        ems = [[-1, 1], [1, 1], [-1, 1], [0, 0], [-1, 1], [-1, 1]]
    elif parameters.ems == 3:
        ems = [[-1, 1], [-1, 1], [-1, 1], [0, 0], [-1, 1], [-1, 1]]
    elif parameters.ems == 5:
        ems = [[1, 1], [1, 1], [-1, 1], [0, -0.1], [-1, 1], [-1, 1]]
    elif parameters.ems == 6:
        ems = [[1, 1], [1, 1], [-1, 1], [-0.1, -0.1], [-1, 1], [-1, 1]]
    elif parameters.ems == 7: #linear increase during climb [-1,1]
        ems = [[-1, 1], [1, 1], [-1, 1], [0, 0], [-1, 1], [-1, 1]]
    elif parameters.ems == 8: #linear increase during climb [-1,0.5]
        ems = [[-1, 1], [0.5, 1], [-1, 1], [0, 0], [-1, 1], [-1, 1]]
    elif parameters.ems == 9: #linear increase during climb [-1,0]
        ems = [[-1, 1], [0, 1], [-1, 1], [0, 0], [-1, 1], [-1, 1]]

    #emotor turboprop
    base_segment.state.conditions.electric_throttle = 0.
    climb_electric_throttle_lower11000 = ems[0][0]#1.  # 1.
    climb_electric_throttle = ems[1][0]#1.#0.5#1.
    cruise_electric_throttle = ems[2][0]#-1.
    descent_electric_throttle = ems[3][0]#0.
    reserve_climb_electric_throttle = ems[4][0]#-1.  # -0.1
    reserve_electric_throttle = ems[5][0]#-1.


    #hybird wtp
    base_segment.state.conditions.electric_throttle_WTP = 0.# 1.
    climb_electric_throttle_WTP_lower11000 = ems[0][1]#1.  # 1.
    climb_electric_throttle_WTP = ems[1][1]#1#1.#1.
    cruise_electric_throttle_WTP = ems[2][1]#1.
    descent_electric_throttle_WTP = ems[3][1]#0.
    reserve_climb_electric_throttle_WTP = ems[4][1]  # 1.#1.
    reserve_electric_throttle_WTP = ems[5][1]#1.





    # End Stuff for Battery_Propeller
    ########################################################################

    # ------------------------------------------------------------------
    #   Climb Requirment 1850 ft/min MTOM, SL, ISA
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "climb_1850"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 0.0 * Units.ft
    segment.altitude_end = 400 * Units.ft
    segment.air_speed = 160 * Units.knots
    segment.climb_rate     = 1850.  * Units.ft / Units.min

    #segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
    #segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * segment.altitude_start/Units.ft + ems[0][0]
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    else:
        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   2ng Segment OEI for Main Propeller
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "second_segment_oei" #just small letters

    # connect vehicle configuration
    segment.analyses.extend(analyses.second_segment_oei)

    segment.altitude_start = 0 * Units.m
    segment.altitude_end   = 400. * Units.m
    if parameters.number_of_engines_for_oei == 2:
        segment.climb_angle    = 0.024  * Units.rad
    elif parameters.number_of_engines_for_oei == 3:
        segment.climb_angle    = 0.027  * Units.rad
    elif parameters.number_of_engines_for_oei == 4:
        segment.climb_angle    = 0.03  * Units.rad
    else:
        segment.climb_angle    = 0.024  * Units.rad
    #segment.analyses.aerodynamics.settings.oswald_efficiency_factor = 0.75

    MTOM = iteration_setup.weight_iter.MTOW
    cL_max_takeoff = 0.8 * 2.7

    ref_area= vehicle.wings.main_wing.areas.reference

    v_stall_takeoff = (MTOM * 9.81 / ( 1.225 / 2 * ref_area * cL_max_takeoff))**0.5
    v2 = 1.2 * v_stall_takeoff

    segment.air_speed      = v2

    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    else:
        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   2ng Segment OEI for Wing Tip Propeller --> both fail
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "second_segment_oei_wtp" #just small letters

    # connect vehicle configuration
    segment.analyses.extend(analyses.second_segment_oei_wtp)

    segment.altitude_start = 0 * Units.m
    segment.altitude_end   = 400. * Units.m
    if parameters.number_of_engines_for_oei == 2:
        segment.climb_angle    = 0.024  * Units.rad
    elif parameters.number_of_engines_for_oei == 3:
        segment.climb_angle    = 0.027  * Units.rad
    elif parameters.number_of_engines_for_oei == 4:
        segment.climb_angle    = 0.03  * Units.rad
    else:
        segment.climb_angle    = 0.024  * Units.rad

    #segment.analyses.aerodynamics.settings.oswald_efficiency_factor = 0.75

    MTOM = iteration_setup.weight_iter.MTOW
    cL_max_takeoff = 0.8 * 2.7

    ref_area= vehicle.wings.main_wing.areas.reference

    v_stall_takeoff = (MTOM * 9.81 / ( 1.225 / 2 * ref_area * cL_max_takeoff))**0.5
    v2 = 1.2 * v_stall_takeoff

    segment.air_speed      = v2

    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    else:
        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

    # add to misison
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------


    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)


    cruise_speed = iteration_setup.mission_iter.cruise_speed
    cruise_distance = 10 * Units.km #iteration_setup.mission_iter.cruise_distance

    segment.air_speed = cruise_speed
    segment.distance = cruise_distance
    segment.altitude = iteration_setup.mission_iter.cruise_altitude * Units.ft

    segment.state.conditions.electric_throttle = cruise_electric_throttle
    segment.state.conditions.electric_throttle_WTP = cruise_electric_throttle_WTP

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    # Ceiling OEI 18.000 ft +10ISA
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "ceiling_oei"

    # connect vehicle configuration
    segment.analyses.extend(analyses.ceiling_oei)

    cruise_speed = 195 * Units.knots #240 210.445 cas 160 210.445
    cruise_distance = 10 * Units.km

    #segment.state.conditions.weights.total_mass[-1,0] = 18000 * Units.kg

    segment.air_speed = cruise_speed
    segment.distance = cruise_distance
    segment.altitude = 13125 * Units.ft
    segment.temperature_deviation = 10

    #segment.state.conditions.electric_throttle = climb_electric_throttle
    #segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP
    segment.state.conditions.electric_throttle = cruise_electric_throttle
    segment.state.conditions.electric_throttle_WTP = cruise_electric_throttle_WTP

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    # Service Ceiling 25.000 ft with cruise speed
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "service_ceiling"

    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = 24000 * Units.ft
    segment.altitude_end   = 24500 * Units.ft
    segment.climb_rate     = 100 * Units.ft / Units.min
    segment.air_speed      = 0.89 * iteration_setup.mission_iter.cruise_speed#267 * Units.knots

    segment.state.conditions.electric_throttle = climb_electric_throttle
    segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    # Initial Cruise Altitude - Top of Climb - 300 ft/min with climb speed
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "initial_cruise_altitude"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 24000 * Units.ft#17000 * Units.ft #24000 * Units.ft
    segment.altitude_end   = 24500 * Units.ft#18000 * Units.ft #25000 * Units.ft
    segment.climb_rate     = 300 * Units.ft / Units.min
    segment.air_speed      = 0.89 * iteration_setup.mission_iter.cruise_speed#267  * Units.knots #300 236.196

    segment.state.conditions.electric_throttle = climb_electric_throttle
    segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP

    # add to mission
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Reserve Hold Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "hold"

    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)

    reserve_hold_speed = iteration_setup.mission_iter.reserve_hold_speed

    reserve_hold_altitude = iteration_setup.mission_iter.reserve_hold_altitude
    segment.altitude = reserve_hold_altitude

    segment.air_speed = reserve_hold_speed
    segment.distance = 10 * Units.km
    segment.state.conditions.electric_throttle = reserve_electric_throttle
    segment.state.conditions.electric_throttle_WTP = reserve_electric_throttle_WTP

    # add to mission
    mission.append_segment(segment)




    return mission