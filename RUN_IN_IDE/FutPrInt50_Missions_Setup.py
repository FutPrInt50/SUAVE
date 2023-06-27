# FutPrInt50_Missions_Setup.py
#
# This code was contributed under project FutPrint50 (www.futprint50.eu) that has received funding from the
# European Union's Horizon 2020 Research and Innovation programme under Grant Agreement No. 875551.
#
# Contributed by:
#  - Dominik Eisenhut, University of Stuttgart
#
# Created:  Feb 2022, D. Eisenhut
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np

# SUAVE imports
import SUAVE

# Units
from SUAVE.Core import Units, Data


# ---------------------------------------------------------------------------------------------------------------------
# definition of the basis missions - single leg (1 flight + 1 reserve) and twin leg (2 flights + 1 reserve)
# ---------------------------------------------------------------------------------------------------------------------

def base_mission_single_leg(mission_tag, departure, destination, alternate, altitudes, distances, holding_duration,
                            target, analyses):
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = mission_tag

    mission.target = target

    mission.departure = departure
    mission.destination = destination
    mission.alternate = alternate

    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()

    # ---------------------------------------------------------------
    # Taxi Out Segment
    # ---------------------------------------------------------------
    segment = Segments.Ground.Taxi_Constant_Speed_Set_Time(base_segment)

    segment.tag = "taxi_out"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = departure.altitude
    segment.air_speed = 25 * Units.kt
    segment.friction_coefficient = 0.04
    segment.time = 10 * Units.min

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Take-Off Segment
    # ---------------------------------------------------------------
    # TODO: Add Take-Off

    # ---------------------------------------------------------------
    # Climbout Segment
    # ---------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climbout"

    segment.temperature_deviation = departure.delta_isa

    segment.analyses.extend(analyses.takeoff)

    segment.altitude_start = departure.altitude
    segment.altitude_end = segment.altitude_start + altitudes.agl.climbout

    segment.calibrated_air_speed = 160 * Units.kt
    segment.throttle = 1.

    mission.append_segment(segment)

    if departure.altitude + altitudes.agl.climbout < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Climb Segment
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1"

        segment.temperature_deviation = departure.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Climb Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_2"

        segment.temperature_deviation = departure.delta_isa

        segment.analyses.extend(analyses.climb_2)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Climb Segment
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1"

        segment.temperature_deviation = departure.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Cruise Segment
    # ---------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.temperature_deviation = (departure.delta_isa + destination.delta_isa) / 2

    segment.analyses.extend(analyses.cruise)

    segment.air_speed = 500 * Units.km / Units.h
    segment.distance = distances.cruise

    # add to mission
    mission.append_segment(segment)

    if destination.altitude + altitudes.agl.final_approach_fix < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Descent Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Descent Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_2"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.descent_2)

        segment.altitude_end = destination.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Descent Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = destination.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Final Approach
    # ---------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach"

    segment.temperature_deviation = destination.delta_isa

    segment.analyses.extend(analyses.landing)

    segment.altitude_end = destination.altitude

    segment.air_speed = 160 * Units.kt
    segment.descent_angle = 3 * Units.degree

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Aborted Landing - Climb Out
    # ---------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climbout_reserve"

    segment.temperature_deviation = destination.delta_isa

    segment.analyses.extend(analyses.landing)

    segment.altitude_end = destination.altitude + altitudes.agl.climbout

    segment.calibrated_air_speed = 160 * Units.kt
    segment.throttle = 1.

    mission.append_segment(segment)

    if destination.altitude + altitudes.agl.climbout < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Climb Segment
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1_reserve"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Climb Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_2_reserve"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.climb_2)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Climb Segment
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1_reserve"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Cruise to Alternate Segment
    # ---------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_reserve"

    segment.temperature_deviation = (destination.delta_isa + alternate.delta_isa) / 2

    segment.analyses.extend(analyses.cruise)

    segment.air_speed = 500 * Units.km / Units.h
    segment.distance = distances.reserve_cruise

    # add to mission
    mission.append_segment(segment)

    if alternate.altitude + altitudes.agl.final_approach_fix < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Descent Segment to Holding Pattern
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1_reserve"

        segment.temperature_deviation = alternate.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Descent Segment to Holding Pattern
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_2_reserve"

        segment.temperature_deviation = alternate.delta_isa

        segment.analyses.extend(analyses.descent_2)

        segment.altitude_end = alternate.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Descent Segment to Holding Pattern
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1_reserve"

        segment.temperature_deviation = alternate.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = alternate.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Holding
    # ---------------------------------------------------------------
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "holding"

    segment.temperature_deviation = alternate.delta_isa

    segment.air_speed = 300 * Units.km / Units.h  # TODO: Update value
    segment.distance = segment.air_speed * holding_duration

    # add to mission
    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Final Approach to Alternate
    # ---------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach_reserve"

    segment.temperature_deviation = alternate.delta_isa

    segment.analyses.extend(analyses.landing)

    segment.altitude_end = alternate.altitude

    segment.air_speed = 160 * Units.kt
    segment.descent_angle = 3 * Units.degree

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Landing
    # ---------------------------------------------------------------
    # TODO: Add Landing

    # ---------------------------------------------------------------
    # Taxi In Segment
    # ---------------------------------------------------------------
    segment = Segments.Ground.Taxi_Constant_Speed_Set_Time(base_segment)

    segment.tag = "taxi_in"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = destination.altitude
    segment.air_speed = 25 * Units.kt
    segment.friction_coefficient = 0.04
    segment.time = 5 * Units.min

    mission.append_segment(segment)


def base_mission_twin_leg(mission_tag, departure, intermediate, destination, alternate, altitudes, distances,
                          holding_duration, target, analyses):
    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = mission_tag

    mission.target = target

    mission.departure = departure
    mission.intermediate = intermediate
    mission.destination = destination
    mission.alternate = alternate

    Segments = SUAVE.Analyses.Mission.Segments

    # base segment
    base_segment = Segments.Segment()

    # ---------------------------------------------------------------
    # Taxi Out Segment
    # ---------------------------------------------------------------
    segment = Segments.Ground.Taxi_Constant_Speed_Set_Time(base_segment)

    segment.tag = "taxi_out"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = departure.altitude
    segment.air_speed = 25 * Units.kt
    segment.friction_coefficient = 0.04
    segment.time = 10 * Units.min

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Take-Off Segment
    # ---------------------------------------------------------------
    # TODO: Add Take-Off

    # ---------------------------------------------------------------
    # Climbout Segment
    # ---------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climbout"

    segment.temperature_deviation = departure.delta_isa

    segment.analyses.extend(analyses.takeoff)

    segment.altitude_start = departure.altitude
    segment.altitude_end = segment.altitude_start + altitudes.agl.climbout

    segment.calibrated_air_speed = 160 * Units.kt
    segment.throttle = 1.

    mission.append_segment(segment)

    if departure.altitude + altitudes.agl.climbout < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Climb Segment
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1"

        segment.temperature_deviation = departure.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Climb Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_2"

        segment.temperature_deviation = departure.delta_isa

        segment.analyses.extend(analyses.climb_2)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Climb Segment
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1"

        segment.temperature_deviation = departure.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Cruise Segment
    # ---------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise"

    segment.temperature_deviation = (departure.delta_isa + intermediate.delta_isa) / 2

    segment.analyses.extend(analyses.cruise)

    segment.air_speed = 500 * Units.km / Units.h
    segment.distance = distances.cruise

    # add to mission
    mission.append_segment(segment)

    if intermediate.altitude + altitudes.agl.final_approach_fix < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Descent Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1"

        segment.temperature_deviation = intermediate.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Descent Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_2"

        segment.temperature_deviation = intermediate.delta_isa

        segment.analyses.extend(analyses.descent_2)

        segment.altitude_end = intermediate.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Descent Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1"

        segment.temperature_deviation = intermediate.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = intermediate.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Final Approach
    # ---------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach"

    segment.temperature_deviation = intermediate.delta_isa

    segment.analyses.extend(analyses.landing)

    segment.altitude_end = intermediate.altitude

    segment.air_speed = 160 * Units.kt
    segment.descent_angle = 3 * Units.degree

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Landing
    # ---------------------------------------------------------------
    # TODO: Add Landing

    # ---------------------------------------------------------------
    # Taxi In Segment
    # ---------------------------------------------------------------
    segment = Segments.Ground.Taxi_Constant_Speed_Set_Time(base_segment)

    segment.tag = "taxi_in"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = destination.altitude
    segment.air_speed = 25 * Units.kt
    segment.friction_coefficient = 0.04
    segment.time = 5 * Units.min

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Taxi Out Segment
    # ---------------------------------------------------------------
    segment = Segments.Ground.Taxi_Constant_Speed_Set_Time(base_segment)

    segment.tag = "taxi_out_second_leg"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = departure.altitude
    segment.air_speed = 25 * Units.kt
    segment.friction_coefficient = 0.04
    segment.time = 10 * Units.min

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Take-Off Segment
    # ---------------------------------------------------------------
    # TODO: Add Take-Off

    # ---------------------------------------------------------------
    # Climbout Segment - Second Leg
    # ---------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climbout_second_leg"

    segment.temperature_deviation = intermediate.delta_isa

    segment.analyses.extend(analyses.takeoff)

    segment.altitude_end = intermediate.altitude + altitudes.agl.climbout

    segment.calibrated_air_speed = 160 * Units.kt
    segment.throttle = 1.

    mission.append_segment(segment)

    if intermediate.altitude + altitudes.agl.climbout < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Climb Segment - Second Leg
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1_second_leg"

        segment.temperature_deviation = intermediate.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Climb Segment - Second Leg
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_2_second_leg"

        segment.temperature_deviation = intermediate.delta_isa

        segment.analyses.extend(analyses.climb_2)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Climb Segment - Second Leg
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1_second_leg"

        segment.temperature_deviation = intermediate.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Cruise Segment - Second Leg
    # ---------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_second_leg"

    segment.temperature_deviation = (intermediate.delta_isa + destination.delta_isa) / 2

    segment.analyses.extend(analyses.cruise)

    segment.air_speed = 500 * Units.km / Units.h
    segment.distance = distances.cruise2

    # add to mission
    mission.append_segment(segment)

    if destination.altitude + altitudes.agl.final_approach_fix < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Descent Segment - Second Leg
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1_second_leg"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Descent Segment - Second Leg
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_2_second_leg"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.descent_2)

        segment.altitude_end = destination.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Descent Segment - Second Leg
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1_second_leg"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = destination.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Final Approach - Second Leg
    # ---------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach_second_leg"

    segment.temperature_deviation = destination.delta_isa

    segment.analyses.extend(analyses.landing)

    segment.altitude_end = destination.altitude

    segment.air_speed = 160 * Units.kt
    segment.descent_angle = 3 * Units.degree

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Aborted Landing - Climb Out
    # ---------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
    segment.tag = "climbout_reserve"

    segment.temperature_deviation = destination.delta_isa

    segment.analyses.extend(analyses.landing)

    segment.altitude_end = destination.altitude + altitudes.agl.climbout

    segment.calibrated_air_speed = 160 * Units.kt
    segment.throttle = 1.

    mission.append_segment(segment)

    if destination.altitude + altitudes.agl.climbout < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Climb Segment
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1_reserve"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Climb Segment
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_2_reserve"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.climb_2)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Climb Segment
        # ---------------------------------------------------------------
        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "climb_1_reserve"

        segment.temperature_deviation = destination.delta_isa

        segment.analyses.extend(analyses.climb_1)

        segment.altitude_end = altitudes.asl.cruise

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 1.

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Cruise to Alternate Segment
    # ---------------------------------------------------------------

    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "cruise_reserve"

    segment.temperature_deviation = (destination.delta_isa + alternate.delta_isa) / 2

    segment.analyses.extend(analyses.cruise)

    segment.air_speed = 500 * Units.km / Units.h
    segment.distance = distances.reserve_cruise

    # add to mission
    mission.append_segment(segment)

    if alternate.altitude + altitudes.agl.final_approach_fix < altitudes.asl.transition:

        # ---------------------------------------------------------------
        # 1st Descent Segment to Holding Pattern
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1_reserve"

        segment.temperature_deviation = alternate.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = altitudes.asl.transition

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

        # ---------------------------------------------------------------
        # 2nd Descent Segment to Holding Pattern
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_2_reserve"

        segment.temperature_deviation = alternate.delta_isa

        segment.analyses.extend(analyses.descent_2)

        segment.altitude_end = alternate.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    else:

        # ---------------------------------------------------------------
        # 1st Descent Segment to Holding Pattern
        # ---------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_CAS(base_segment)
        segment.tag = "descent_1_reserve"

        segment.temperature_deviation = alternate.delta_isa

        segment.analyses.extend(analyses.descent_1)

        segment.altitude_end = alternate.altitude + altitudes.agl.final_approach_fix

        segment.calibrated_air_speed = 160 * Units.kt
        segment.throttle = 0.  # TODO: Update value

        mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Holding
    # ---------------------------------------------------------------
    segment = Segments.Cruise.Constant_Speed_Constant_Altitude(base_segment)
    segment.tag = "holding"

    segment.temperature_deviation = alternate.delta_isa

    segment.air_speed = 300 * Units.km / Units.h  # TODO: Update value
    segment.distance = segment.air_speed * holding_duration

    # add to mission
    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Final Approach to Alternate
    # ---------------------------------------------------------------

    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "final_approach_reserve"

    segment.temperature_deviation = alternate.delta_isa

    segment.analyses.extend(analyses.landing)

    segment.altitude_end = alternate.altitude

    segment.air_speed = 160 * Units.kt
    segment.descent_angle = 3 * Units.degree

    mission.append_segment(segment)

    # ---------------------------------------------------------------
    # Landing
    # ---------------------------------------------------------------
    # TODO: Add Landing

    # ---------------------------------------------------------------
    # Taxi In Segment
    # ---------------------------------------------------------------
    segment = Segments.Ground.Taxi_Constant_Speed_Set_Time(base_segment)

    segment.tag = "taxi_in_second_leg"

    segment.analyses.extend(analyses.cruise)

    segment.altitude_start = destination.altitude
    segment.air_speed = 25 * Units.kt
    segment.friction_coefficient = 0.04
    segment.time = 5 * Units.min

    mission.append_segment(segment)


def update_mission_ranges(results):

    if results.tag in ('design_range', 'maximum_range', 'cold_ops', 'extreme_cold', 'hot_and_high', 'mountain_ops',
                       'stol_ops'):
        # these are the defined single leg missions of FutPrInt50
        distances, converged, deltas = update_mission_ranges_single_leg(results)
    elif results.tag in ('island_ops'):
        # these are the defined twin leg missions of FutPrInt50
        distances, converged, deltas = update_mission_ranges_twin_leg(results)
    else:
        raise AttributeError('Mission Type unknown!')

    return distances, converged, deltas


# ---------------------------------------------------------------------------------------------------------------------
# definition of helper functions that update all cruise segment distances for the two base missions
# ---------------------------------------------------------------------------------------------------------------------

def update_mission_ranges_single_leg(results):
    # start with the normal segments for flight to destination
    climb_segments = [key for key in results.segments.keys() if
                      (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key) and (
                                  'climbout' not in key))]
    first_climb_segment = climb_segments[0]
    last_climb_segment = climb_segments[-1]
    climb_distance = (results.segments[last_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                      results.segments[first_climb_segment].conditions.frames.inertial.position_vector[0][0])

    descent_segments = [key for key in results.segments.keys() if
                        (('descent' in key) and ('reserve' not in key) and ('second_leg' not in key))]
    first_descent_segment = descent_segments[0]
    last_descent_segment = descent_segments[-1]
    descent_distance = (results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                        results.segments[first_descent_segment].conditions.frames.inertial.position_vector[0][0])

    # now calculate the reserve mission
    reserve_climb_segments = [key for key in results.segments.keys() if
                              (('climb' in key) and ('reserve' in key) and ('second_leg' not in key) and (
                                          'climbout' not in key))]
    first_reserve_climb_segment = reserve_climb_segments[0]
    last_reserve_climb_segment = reserve_climb_segments[-1]
    reserve_climb_distance = (
                results.segments[last_reserve_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                results.segments[first_reserve_climb_segment].conditions.frames.inertial.position_vector[0][0])

    reserve_descent_segments = [key for key in results.segments.keys() if
                                (('descent' in key) and ('reserve' in key) and ('second_leg' not in key))]
    first_reserve_descent_segment = reserve_descent_segments[0]
    last_reserve_descent_segment = reserve_descent_segments[-1]

    reserve_descent_distance = (
                results.segments[last_reserve_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                results.segments[first_reserve_descent_segment].conditions.frames.inertial.position_vector[0][0])

    distances = Data()

    distances.cruise = results.target.range - climb_distance - descent_distance
    distances.reserve_cruise = results.target.reserve - reserve_climb_distance - reserve_descent_distance

    # calculate old distances to calculate convergence
    cruise_distance = (results.segments.cruise.conditions.frames.inertial.position_vector[-1][0] -
                       results.segments.cruise.conditions.frames.inertial.position_vector[0][0])
    reserve_cruise_distance = (results.segments.cruise_reserve.conditions.frames.inertial.position_vector[-1][0] -
                               results.segments.cruise_reserve.conditions.frames.inertial.position_vector[0][0])

    deltas = np.array([cruise_distance - distances.cruise, reserve_cruise_distance - distances.reserve_cruise])
    converged = all(deltas < 0.1)

    return distances, converged, deltas


def update_mission_ranges_twin_leg(results):
    # start with the normal segments for flight to destination
    climb_segments = [key for key in results.segments.keys() if
                      (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key) and (
                              'climbout' not in key))]
    first_climb_segment = climb_segments[0]
    last_climb_segment = climb_segments[-1]
    climb_distance = (results.segments[last_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                      results.segments[first_climb_segment].conditions.frames.inertial.position_vector[0][0])

    descent_segments = [key for key in results.segments.keys() if
                        (('descent' in key) and ('reserve' not in key) and ('second_leg' not in key))]
    first_descent_segment = descent_segments[0]
    last_descent_segment = descent_segments[-1]
    descent_distance = (results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                        results.segments[first_descent_segment].conditions.frames.inertial.position_vector[0][0])

    # now calculate second leg
    climb_segments_second_leg = [key for key in results.segments.keys() if
                                 (('climb' in key) and ('reserve' not in key) and ('second_leg' in key) and (
                                         'climbout' not in key))]
    first_climb_segment_second_leg = climb_segments_second_leg[0]
    last_climb_segment_second_leg = climb_segments_second_leg[-1]
    climb_distance_second_leg = (
                results.segments[last_climb_segment_second_leg].conditions.frames.inertial.position_vector[-1][0] - \
                results.segments[first_climb_segment_second_leg].conditions.frames.inertial.position_vector[0][0])

    descent_segments_second_leg = [key for key in results.segments.keys() if
                                   (('descent' in key) and ('reserve' not in key) and ('second_leg' in key))]
    first_descent_segment_second_leg = descent_segments_second_leg[0]
    last_descent_segment_second_leg = descent_segments_second_leg[-1]
    descent_distance_second_leg = (
                results.segments[last_descent_segment_second_leg].conditions.frames.inertial.position_vector[-1][0] - \
                results.segments[first_descent_segment_second_leg].conditions.frames.inertial.position_vector[0][0])

    # now calculate the reserve mission
    reserve_climb_segments = [key for key in results.segments.keys() if
                              (('climb' in key) and ('reserve' in key) and ('second_leg' not in key) and (
                                      'climbout' not in key))]
    first_reserve_climb_segment = reserve_climb_segments[0]
    last_reserve_climb_segment = reserve_climb_segments[-1]
    reserve_climb_distance = (
            results.segments[last_reserve_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
            results.segments[first_reserve_climb_segment].conditions.frames.inertial.position_vector[0][0])

    reserve_descent_segments = [key for key in results.segments.keys() if
                                (('descent' in key) and ('reserve' in key) and ('second_leg' not in key))]
    first_reserve_descent_segment = reserve_descent_segments[0]
    last_reserve_descent_segment = reserve_descent_segments[-1]

    reserve_descent_distance = (
            results.segments[last_reserve_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
            results.segments[first_reserve_descent_segment].conditions.frames.inertial.position_vector[0][0])

    distances = Data()

    distances.cruise = results.target.range - climb_distance - descent_distance
    distances.cruise2 = results.target.range2 - climb_distance_second_leg - descent_distance_second_leg
    distances.reserve_cruise = results.target.reserve - reserve_climb_distance - reserve_descent_distance

    # calculate old distances to calculate convergence
    cruise_distance = (results.segments.cruise.conditions.frames.inertial.position_vector[-1][0] -
                       results.segments.cruise.conditions.frames.inertial.position_vector[0][0])
    cruise_distance_second_leg = (results.segments.cruise_second_leg.conditions.frames.inertial.position_vector[-1][0] -
                                 results.segments.cruise_second_leg.conditions.frames.inertial.position_vector[0][0])
    reserve_cruise_distance = (results.segments.cruise_reserve.conditions.frames.inertial.position_vector[-1][0] -
                               results.segments.cruise_reserve.conditions.frames.inertial.position_vector[0][0])

    deltas = np.array([cruise_distance - distances.cruise, cruise_distance_second_leg - distances.cruise2,
                       reserve_cruise_distance - distances.reserve_cruise])
    converged = all(deltas < 0.1)

    return distances, converged, deltas


# ---------------------------------------------------------------------------------------------------------------------
# helper functions for off-design performance calculations (converged aircraft on off-design mission)
# ---------------------------------------------------------------------------------------------------------------------

def update_mission_off_design_performance_single_leg(results):
    distances, range_converged, range_deltas = update_mission_ranges_single_leg(results)
    delta_fuel_mass = 0

    # normal cruise segment delta fuel mass
    cruise_fuel = results.segments.cruise.conditions.weights.total_mass[0] - \
                  results.segments.cruise.conditions.weights.total_mass[-1]
    cruise_distance = (results.segments.cruise.conditions.frames.inertial.position_vector[-1][0] -
                       results.segments.cruise.conditions.frames.inertial.position_vector[0][0])

    delta_fuel_mass += (distances.cruise - cruise_distance) * (cruise_fuel / cruise_distance)

    # reserve delta fuel mass
    reserve_cruise_fuel = results.segments.reserve_cruise.conditions.weights.total_mass[0] - \
                          results.segments.reserve_cruise.conditions.weights.total_mass[-1]
    reserve_cruise_distance = (results.segments.reserve_cruise.conditions.frames.inertial.position_vector[-1][0] -
                               results.segments.reserve_cruise.conditions.frames.inertial.position_vector[0][0])

    delta_fuel_mass += (distances.reserve_cruise - reserve_cruise_distance) * \
                       (reserve_cruise_fuel / reserve_cruise_distance)

    # TODO: add battery energy

    return distances, delta_fuel_mass, range_converged, range_deltas


def update_mission_off_design_performance_twin_leg(results):
    distances, range_converged, range_deltas = update_mission_ranges_twin_leg(results)
    delta_fuel_mass = 0

    # normal cruise segment delta fuel mass
    cruise_fuel = results.segments.cruise.conditions.weights.total_mass[0] - \
                  results.segments.cruise.conditions.weights.total_mass[-1]
    cruise_distance = (results.segments.cruise.conditions.frames.inertial.position_vector[-1][0] -
                       results.segments.cruise.conditions.frames.inertial.position_vector[0][0])

    delta_fuel_mass += (distances.cruise - cruise_distance) * (cruise_fuel / cruise_distance)

    # second cruise segment delta fuel mass
    cruise_fuel_second_leg = results.segments.cruise_second_leg.conditions.weights.total_mass[0] - \
                             results.segments.cruise_second_leg.conditions.weights.total_mass[-1]
    cruise_distance_second_leg = (results.segments.cruise_second_leg.conditions.frames.inertial.position_vector[-1][0] -
                                  results.segments.cruise_second_leg.conditions.frames.inertial.position_vector[0][0])

    delta_fuel_mass += (distances.cruise2 - cruise_distance_second_leg) * (
                cruise_fuel_second_leg / cruise_distance_second_leg)

    # reserve delta fuel mass
    reserve_cruise_fuel = results.segments.reserve_cruise.conditions.weights.total_mass[0] - \
                          results.segments.reserve_cruise.conditions.weights.total_mass[-1]
    reserve_cruise_distance = (results.segments.reserve_cruise.conditions.frames.inertial.position_vector[-1][0] -
                               results.segments.reserve_cruise.conditions.frames.inertial.position_vector[0][0])

    delta_fuel_mass += (distances.reserve_cruise - reserve_cruise_distance) * \
                       (reserve_cruise_fuel / reserve_cruise_distance)

    # TODO: add battery energy

    return distances, delta_fuel_mass, range_converged, range_deltas

# ---------------------------------------------------------------------------------------------------------------------
# the definitions of the actual mission values start here, these are the functions to call from aircraft analysis
# ---------------------------------------------------------------------------------------------------------------------


def mission_design_range(distances, analyses):

    # these are the parameters that are dependant on the missions defined in FutPrInt50

    mission_tag = 'design_range'

    departure = SUAVE.Attributes.Airports.Airport()
    departure.altitude  =  0 * Units.ft
    departure.delta_isa = 0 * Units.K

    destination = SUAVE.Attributes.Airports.Airport()
    destination.altitude  = 0 * Units.ft
    destination.delta_isa = 0 * Units.K

    alternate = SUAVE.Attributes.Airports.Airport()
    alternate.altitude  = 0 * Units.ft
    alternate.delta_isa = 0 * Units.K

    altitudes = Data()
    altitudes.agl = Data()
    altitudes.asl = Data()

    altitudes.agl.climbout = 1500 * Units.ft
    altitudes.agl.final_approach_fix = 1500 * Units.ft

    altitudes.asl.transition = 10000 * Units.ft
    altitudes.asl.cruise = 21000 * Units.ft
    altitudes.asl.cruise_alternate = 21000 * Units.ft

    holding_duration = 30 * Units.min

    target = Data()
    target.range = 400 * Units.km
    target.reserve = 185 * Units.km

    base_mission_single_leg(mission_tag, departure, destination, alternate, altitudes, distances, holding_duration,
                            target, analyses)


def mission_max_range(distances, analyses):

    mission_tag = 'maximum_range'

    departure = SUAVE.Attributes.Airports.Airport()
    departure.altitude  =  0 * Units.ft
    departure.delta_isa = 0 * Units.K

    destination = SUAVE.Attributes.Airports.Airport()
    destination.altitude  = 0 * Units.ft
    destination.delta_isa = 0 * Units.K

    alternate = SUAVE.Attributes.Airports.Airport()
    alternate.altitude  = 0 * Units.ft
    alternate.delta_isa = 0 * Units.K

    altitudes = Data()
    altitudes.agl = Data()
    altitudes.asl = Data()

    altitudes.agl.climbout = 1500 * Units.ft
    altitudes.agl.final_approach_fix = 1500 * Units.ft

    altitudes.asl.transition = 10000 * Units.ft
    altitudes.asl.cruise = 25000 * Units.ft
    altitudes.asl.cruise_alternate = 21000 * Units.ft

    holding_duration = 30 * Units.min

    target = Data()
    target.range = 800 * Units.km
    target.reserve = 185 * Units.km

    base_mission_single_leg(mission_tag, departure, destination, alternate, altitudes, distances, holding_duration,
                            target, analyses)


def mission_cold_ops(distances, analyses):

    mission_tag = 'cold_ops'

    departure = SUAVE.Attributes.Airports.Airport()
    departure.altitude  =  0 * Units.ft
    departure.delta_isa = -30 * Units.K

    destination = SUAVE.Attributes.Airports.Airport()
    destination.altitude  = 0 * Units.ft
    destination.delta_isa = -30 * Units.K

    alternate = SUAVE.Attributes.Airports.Airport()
    alternate.altitude  = 0 * Units.ft
    alternate.delta_isa = -30 * Units.K

    altitudes = Data()
    altitudes.agl = Data()
    altitudes.asl = Data()

    altitudes.agl.climbout = 1500 * Units.ft
    altitudes.agl.final_approach_fix = 1500 * Units.ft

    altitudes.asl.transition = 10000 * Units.ft
    altitudes.asl.cruise = 21000 * Units.ft
    altitudes.asl.cruise_alternate = 21000 * Units.ft

    holding_duration = 30 * Units.min

    target = Data()
    target.range = 450 * Units.km
    target.reserve = 185 * Units.km

    base_mission_single_leg(mission_tag, departure, destination, alternate, altitudes, distances, holding_duration,
                            target, analyses)


def mission_extreme_cold(distances, analyses):

    mission_tag = 'extreme_cold'

    departure = SUAVE.Attributes.Airports.Airport()
    departure.altitude  =  0 * Units.ft
    departure.delta_isa = -55 * Units.K

    destination = SUAVE.Attributes.Airports.Airport()
    destination.altitude  = 0 * Units.ft
    destination.delta_isa = -55 * Units.K

    alternate = SUAVE.Attributes.Airports.Airport()
    alternate.altitude  = 0 * Units.ft
    alternate.delta_isa = -55 * Units.K

    altitudes = Data()
    altitudes.agl = Data()
    altitudes.asl = Data()

    altitudes.agl.climbout = 1500 * Units.ft
    altitudes.agl.final_approach_fix = 1500 * Units.ft

    altitudes.asl.transition = 10000 * Units.ft
    altitudes.asl.cruise = 21000 * Units.ft
    altitudes.asl.cruise_alternate = 21000 * Units.ft

    holding_duration = 30 * Units.min

    target = Data()
    target.range = 450 * Units.km
    target.reserve = 185 * Units.km

    base_mission_single_leg(mission_tag, departure, destination, alternate, altitudes, distances, holding_duration,
                            target, analyses)


def mission_hot_and_high(distances, analyses):

    mission_tag = 'hot_and_high'

    departure = SUAVE.Attributes.Airports.Airport()
    departure.altitude  = 2230 * Units.m
    departure.delta_isa = 28 * Units.K

    destination = SUAVE.Attributes.Airports.Airport()
    destination.altitude  = 0 * Units.ft
    destination.delta_isa = 28 * Units.K

    alternate = SUAVE.Attributes.Airports.Airport()
    alternate.altitude  = 0 * Units.ft
    alternate.delta_isa = 28 * Units.K

    altitudes = Data()
    altitudes.agl = Data()
    altitudes.asl = Data()

    altitudes.agl.climbout = 1500 * Units.ft
    altitudes.agl.final_approach_fix = 1500 * Units.ft

    altitudes.asl.transition = 10000 * Units.ft
    altitudes.asl.cruise = 21000 * Units.ft
    altitudes.asl.cruise_alternate = 21000 * Units.ft

    holding_duration = 30 * Units.min

    target = Data()
    target.range = 450 * Units.km
    target.reserve = 185 * Units.km

    base_mission_single_leg(mission_tag, departure, destination, alternate, altitudes, distances, holding_duration,
                            target, analyses)


def mission_mountain_ops(distances, analyses):
    mission_tag = 'mountain_ops'

    departure = SUAVE.Attributes.Airports.Airport()
    departure.altitude = 0 * Units.m
    departure.delta_isa = 0 * Units.K

    destination = SUAVE.Attributes.Airports.Airport()
    destination.altitude = 2718 * Units.m
    destination.delta_isa = 0 * Units.K

    alternate = SUAVE.Attributes.Airports.Airport()
    alternate.altitude = 2718 * Units.m
    alternate.delta_isa = 0 * Units.K

    altitudes = Data()
    altitudes.agl = Data()
    altitudes.asl = Data()

    altitudes.agl.climbout = 1500 * Units.ft
    altitudes.agl.final_approach_fix = 1500 * Units.ft

    altitudes.asl.transition = 10000 * Units.ft
    altitudes.asl.cruise = 25000 * Units.ft
    altitudes.asl.cruise_alternate = 25000 * Units.ft

    holding_duration = 30 * Units.min

    target = Data()
    target.range = 450 * Units.km
    target.reserve = 185 * Units.km

    base_mission_single_leg(mission_tag, departure, destination, alternate, altitudes, distances, holding_duration,
                            target, analyses)


def mission_island_ops(distances, analyses):
    mission_tag = 'island_ops'

    departure = SUAVE.Attributes.Airports.Airport()
    departure.altitude = 0 * Units.m
    departure.delta_isa = 0 * Units.K

    intermediate = SUAVE.Attributes.Airports.Airport()
    intermediate.altitude = 0 * Units.m
    intermediate.delta_isa = 0 * Units.K

    destination = SUAVE.Attributes.Airports.Airport()
    destination.altitude = 0 * Units.m
    destination.delta_isa = 0 * Units.K

    alternate = SUAVE.Attributes.Airports.Airport()
    alternate.altitude = 0 * Units.m
    alternate.delta_isa = 0 * Units.K

    altitudes = Data()
    altitudes.agl = Data()
    altitudes.asl = Data()

    altitudes.agl.climbout = 1500 * Units.ft
    altitudes.agl.final_approach_fix = 1500 * Units.ft

    altitudes.asl.transition = 10000 * Units.ft
    altitudes.asl.cruise = 21000 * Units.ft
    altitudes.asl.cruise_alternate = 25000 * Units.ft

    holding_duration = 30 * Units.min

    target = Data()
    target.range = 300 * Units.km
    target.range2 = 300 * Units.km
    target.reserve = 185 * Units.km

    base_mission_single_leg(mission_tag, departure, intermediate, destination, alternate, altitudes, distances,
                            holding_duration, target, analyses)


def mission_stol_ops(distances, analyses):
    mission_tag = 'stol_ops'

    departure = SUAVE.Attributes.Airports.Airport()
    departure.altitude = 0 * Units.m
    departure.delta_isa = 0 * Units.K

    destination = SUAVE.Attributes.Airports.Airport()
    destination.altitude = 0 * Units.m
    destination.delta_isa = 0 * Units.K

    alternate = SUAVE.Attributes.Airports.Airport()
    alternate.altitude = 0 * Units.m
    alternate.delta_isa = 0 * Units.K

    altitudes = Data()
    altitudes.agl = Data()
    altitudes.asl = Data()

    altitudes.agl.climbout = 1500 * Units.ft
    altitudes.agl.final_approach_fix = 1500 * Units.ft

    altitudes.asl.transition = 10000 * Units.ft
    altitudes.asl.cruise = 25000 * Units.ft
    altitudes.asl.cruise_alternate = 25000 * Units.ft

    holding_duration = 30 * Units.min

    target = Data()
    target.range = 450 * Units.km
    target.reserve = 185 * Units.km

    base_mission_single_leg(mission_tag, departure, destination, alternate, altitudes, distances, holding_duration,
                            target, analyses)
