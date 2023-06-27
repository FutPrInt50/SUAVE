## Reference_ATR42_06_2022.py
# 
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Jonas Mangold, mangold@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Sep 2021, J. Mangold
# Modified: Jul 2022, F. Brenner

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



def mission_setup_FutPrInt50(analyses,iteration_setup):
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
    base_segment.state.numerics.number_control_points = 6
    base_segment.temperature_deviation = airport.delta_isa

    number_control_climb = 4
    climb_throttle = 0.83 #0.821
    idle_throttle = 0.06

    # # Stuff from ICE EMB
    vehicle = analyses.vehicle
    ones_row     = base_segment.state.ones_row
    base_segment.process.iterate.unknowns.network            = vehicle.propulsors.network.unpack_unknowns
    base_segment.process.iterate.residuals.network           = vehicle.propulsors.network.residuals
    base_segment.state.residuals.network                     = 0 * ones_row(1)
    base_segment.state.unknowns.rpm                          = 1200 * ones_row(1)

    # ------------------------------------------------------------------
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_1"

    # connect vehicle configuration
    segment.analyses.extend(analyses.takeoff)

    segment.altitude_start = 0.0 * Units.ft
    segment.altitude_end = 400 * Units.ft
    segment.calibrated_air_speed= 160* Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_2"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 400.0 * Units.ft
    segment.altitude_end = 1500 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_3"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 1500 * Units.ft
    segment.altitude_end = 3000 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   Fourth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------
    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_4"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 3000 * Units.ft
    segment.altitude_end = 6000 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Sixth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_5"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 6000 * Units.ft
    segment.altitude_end = 10000 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Seventh Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_6"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)
    segment.state.numerics.number_control_points = 16

    segment.altitude_start = 10000 * Units.ft
    segment.altitude_end = 15000 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Eighth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_7"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 15000 * Units.ft
    segment.altitude_end = 18000 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Eighth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_8"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 18000 * Units.ft
    segment.altitude_end = 21000 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)
    # ------------------------------------------------------------------
    #   Eighth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climb_9"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 21000 * Units.ft
    segment.altitude_end = 25000 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots
    segment.throttle = climb_throttle
    # add to misison
    mission.append_segment(segment)


    cruise_altitude = iteration_setup.mission_iter.cruise_altitude

    # ------------------------------------------------------------------
    #   Cruise Segment: Constant Speed, Constant Altitude
    # ------------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    # connect vehicle configuration
    segment.analyses.extend(analyses.cruise)


    cruise_speed = iteration_setup.mission_iter.cruise_speed
    cruise_distance = iteration_setup.mission_iter.cruise_distance

    segment.air_speed = cruise_speed
    segment.distance = cruise_distance

    # add to mission
    mission.append_segment(segment)

    if cruise_altitude > 23000:
        # ------------------------------------------------------------------
        #   First Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 25000.0 * Units.ft
        segment.altitude_end = 23000.0 * Units.ft
        segment.calibrated_air_speed = 160 * Units.knots
        segment.throttle = idle_throttle


        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 21000:
        # ------------------------------------------------------------------
        #   Second Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_2"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 23000.0 * Units.ft
        segment.altitude_end = 21000.0 * Units.ft
        segment.calibrated_air_speed = 160 * Units.knots
        segment.throttle = idle_throttle

        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 19000:
        # ------------------------------------------------------------------
        #   Third Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_3"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 21000.0 * Units.ft
        segment.altitude_end = 19000.0 * Units.ft
        segment.calibrated_air_speed = 160 * Units.knots
        segment.throttle = idle_throttle
        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 17000:
        # ------------------------------------------------------------------
        #   Fourth Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_4"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 19000.0 * Units.ft
        segment.altitude_end = 17000.0 * Units.ft
        segment.calibrated_air_speed = 160 * Units.knots
        segment.throttle = idle_throttle
        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 15000:
        # ------------------------------------------------------------------
        #   Fifth Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_5"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 17000.0 * Units.ft
        segment.altitude_end = 15000.0 * Units.ft
        segment.calibrated_air_speed = 160 * Units.knots
        segment.throttle = idle_throttle

        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 13000:
        # ------------------------------------------------------------------
        #   Sixth Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_6"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 15000.0 * Units.ft
        segment.altitude_end = 13000.0 * Units.ft
        segment.calibrated_air_speed = 160 * Units.knots
        segment.throttle = idle_throttle


        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 11000:
        # ------------------------------------------------------------------
        #   Seventh Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_7"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 13000.0 * Units.ft
        segment.altitude_end = 11000.0 * Units.ft
        segment.calibrated_air_speed = 160 * Units.knots
        segment.throttle = idle_throttle

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Eighth Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)

    segment.tag = "descent_8"

    # connect vehicle configuration
    segment.analyses.extend(analyses.descent)

    segment.altitude_start = 11000.0 * Units.ft
    segment.altitude_end = 9000.0 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots

    segment.throttle = idle_throttle


    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Ninth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "descent_9"

    # connect vehicle configuration
    segment.analyses.extend(analyses.descent)

    segment.altitude_start = 9000.0 * Units.ft
    segment.altitude_end = 7000.0 * Units.ft
    segment.calibrated_air_speed = 160.0 * Units.knots

    segment.throttle = idle_throttle

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Tenth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "descent_10"

    # connect vehicle configuration
    segment.analyses.extend(analyses.descent)

    segment.altitude_start = 7000.0 * Units.ft
    segment.altitude_end = 4000.0 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots

    segment.throttle = idle_throttle

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Eleventh Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "descent_11"

    # connect vehicle configuration
    segment.analyses.extend(analyses.landing)

    segment.altitude_start = 4000.0 * Units.ft
    segment.altitude_end = 1500.0 * Units.ft
    segment.calibrated_air_speed = 160 * Units.knots

    segment.throttle = idle_throttle

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Twelveth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "descent_12"

    # connect vehicle configuration
    segment.analyses.extend(analyses.landing)

    segment.altitude_start = 1500.0 * Units.ft
    segment.altitude_end = 0.0 * Units.ft
    segment.air_speed = 160.0 * Units.knots

    segment.descent_angle = 3. * Units.deg

    # add to misison
    mission.append_segment(segment)

    reserve_flag = iteration_setup.mission_iter.reserve_flag

    if reserve_flag:
        # ------------------------------------------------------------------
        #   First Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "reserve_climb_1"

        # connect vehicle configuration
        segment.analyses.extend(analyses.takeoff)

        segment.altitude_start = 0.0 * Units.ft
        segment.altitude_end = 400.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = climb_throttle

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Second Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "reserve_climb_2"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 400.0 * Units.ft
        segment.altitude_end = 4000.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = climb_throttle
        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Third Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "reserve_climb_3"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 4000.0 * Units.ft
        segment.altitude_end = 7000.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = climb_throttle

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Fourth Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "reserve_climb_4"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 7000.0 * Units.ft
        segment.altitude_end = 9000.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = climb_throttle

        # add to misison
        mission.append_segment(segment)
        # ------------------------------------------------------------------
        #   Fifth Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "reserve_climb_5"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 9000.0 * Units.ft
        segment.altitude_end = 11000.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = climb_throttle

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Reserve Cruise Segment: Constant Speed, Constant Altitude
        # ------------------------------------------------------------------

        segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
        segment.tag = "reserve_cruise"

        # connect vehicle configuration
        segment.analyses.extend(analyses.cruise)

        segment.air_speed = 188* Units.knots

        reserve_cruise_distance = iteration_setup.mission_iter.reserve_cruise_distance
        segment.distance = reserve_cruise_distance

        # add to mission
        mission.append_segment(segment)


        # ------------------------------------------------------------------
        #   Third Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "reserve_descent_3"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 11000.0 * Units.ft
        segment.altitude_end = 9000.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = idle_throttle

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   4# Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)

        segment.tag = "reserve_descent_4"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 9000.0 * Units.ft
        segment.altitude_end = 7000.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = idle_throttle

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   5# Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "reserve_descent_5"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 7000.0 * Units.ft
        segment.altitude_end = 4000.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = idle_throttle

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   6# Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)

        segment.tag = "reserve_descent_6"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 4000.0 * Units.ft
        segment.altitude_end = 1500.0 * Units.ft
        segment.calibrated_air_speed = 160.0 * Units.knots
        segment.throttle = idle_throttle

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   7# Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "reserve_descent_7"

        # connect vehicle configuration
        segment.analyses.extend(analyses.landing)

        segment.altitude_start = 1500.0 * Units.ft
        segment.altitude_end = 0.0 * Units.ft
        segment.air_speed = 160.0 * Units.knots
        segment.descent_angle = 3 * Units.deg

        # add to mission
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Reserve Hold Segment: Constant Speed, Constant Altitude
        # ------------------------------------------------------------------

        segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
        segment.tag = "hold"

        # connect vehicle configuration
        segment.analyses.extend(analyses.cruise)

        reserve_hold_time = iteration_setup.mission_iter.reserve_hold_time
        reserve_hold_speed = iteration_setup.mission_iter.reserve_hold_speed

        reserve_hold_distance = reserve_hold_speed * reserve_hold_time

        reserve_hold_altitude = iteration_setup.mission_iter.reserve_hold_altitude
        segment.altitude =  reserve_hold_altitude

        segment.air_speed = reserve_hold_speed
        segment.distance = reserve_hold_distance

        # add to mission
        mission.append_segment(segment)

    return mission