## @ingroup Analyses-Mission-Segments-Climb
# constant_throttle_constant_cas_wrapper.py
#
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Dominik Eisenhut, eisenhut@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Jul 2022, D. Eisenhut
# Modified:
#
# required inputs:
#    settings.calibrated_air_speed
#    settings.throttle
#    settings.tag
#    settings.altitude_end
#    settings.h_inc or settings.number_segments
#
# optional inputs:
#    settings.number_control_points
#    settings.altitude start
#    settings.delta_isa
#
# output:
#    mission

import numpy as np
from SUAVE.Analyses.Mission.Segments.Climb import Constant_Throttle_Constant_Speed
from SUAVE.Core import Units


def constant_throttle_constant_cas_wrapper(mission, analyses, config, base_segment, settings):
    # create several climb segments to "simulate" a constant CAS climb based on constant TAS scheme.

    atmosphere = analyses.base.atmosphere

    try:
        cas = settings.calibrated_air_speed
    except AttributeError:
        raise AttributeError("You have to specify calibrated_air_speed in settings.")

    try:
        throttle = settings.throttle
    except AttributeError:
        raise AttributeError("You need to specify throttle in settings.")

    try:
        tag = settings.tag
    except AttributeError:
        raise AttributeError("You need to specify a tag in settings.")

    try:
        altitude_end = settings.altitude_end
    except AttributeError:
        raise AttributeError("You need to specify altitude_end in settings.")

    if hasattr(settings, "number_control_points"):
        number_control_points = settings.number_control_points
    else:
        number_control_points = 4

    if hasattr(settings, "altitude_start"):
        altitude_start = settings.altitude_start
    else:
        # set altitude_start to altitude at the end of the previous mission segment
        try:
            altitude_start = mission.segments[-1].altitude_end
        except AttributeError:
            raise AttributeError("You have to specify altitude_start if mission is empty.")

    if hasattr(settings, "delta_isa"):
        delta_isa = settings.delta_isa
    else:
        delta_isa = 0

    delta_h = altitude_end - altitude_start

    # calculate number of segments or delta_h
    if hasattr(settings, "h_inc"):
        # calculate number of climb segments
        h_inc = settings.h_inc
        number_segments = round(delta_h / settings.h_inc)
    elif hasattr(settings, "number_segments"):
        # calculate height increment per climb segment
        number_segments = settings.number_segments
        h_inc = delta_h / number_segments
    else:
        raise AttributeError("You need to specify either h_inc or number_segments in settings.")

    # set altitudes for the first segment
    next_altitude_start = altitude_start
    next_altitude_end = altitude_start + h_inc

    for i in range(1, number_segments):
        segment = Constant_Throttle_Constant_Speed(base_segment)
        segment.analyses.extend(config)
        segment.state.numerics.number_control_points = number_control_points
        segment.tag = tag + "_" + str(i)
        segment.altitude_start = next_altitude_start
        segment.altitude_end = next_altitude_end
        segment.temperature_deviation = delta_isa

        # calculate TAS with given CAS in intermediate altitude
        altitude = (next_altitude_start + next_altitude_end) / 2

        MSL_data = atmosphere.compute_values(0.0, 0.0)
        pressure0 = MSL_data.pressure[0]

        ALT_data = atmosphere.compute_values(altitude, delta_isa)
        pressure = ALT_data.pressure[0]

        delta = pressure / pressure0

        kcas = cas / Units.knots

        mach = 2.236 * ((((1 + 4.575e-7 * kcas ** 2) ** 3.5 - 1) / delta + 1) ** 0.2857 - 1) ** 0.5

        qc = pressure * ((1 + 0.2 * mach ** 2) ** 3.5 - 1)
        eas = cas * (pressure / pressure0) ** 0.5 * (
                ((qc / pressure + 1) ** 0.286 - 1) / ((qc / pressure0 + 1) ** 0.286 - 1)) ** 0.5

        tas = eas / np.sqrt(ALT_data.density[0] / MSL_data.density[0])

        segment.air_speed = tas

        segment.throttle = throttle
        mission.append_segment(segment)

        # setup altitudes for next for-loop
        next_altitude_start += h_inc
        next_altitude_end += h_inc

    return mission
