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



def mission_setup_Sizing_Chart(analyses,iteration_setup):
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

    # add to misison
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    #   2ng Segment OEI
    # ------------------------------------------------------------------
    
    segment = Segments.Climb.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "second_segment_oei" #just small letters

    # connect vehicle configuration
    segment.analyses.extend(analyses.second_segment_oei)

    segment.altitude_start = 0 * Units.m
    segment.altitude_end   = 400. * Units.m
    segment.climb_angle    = 0.024  * Units.rad
    segment.analyses.aerodynamics.settings.oswald_efficiency_factor = 0.75

    v_stall = 45
    v2 = 1.2 * v_stall

    segment.air_speed      = v2

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
    segment.altitude = 17000 * Units.ft

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

    segment.state.conditions.weights.total_mass[-1,0] = 18100 * Units.kg

    segment.air_speed = cruise_speed
    segment.distance = cruise_distance
    segment.altitude =16000 * Units.ft #17000 * Units.ft
    segment.temperature_deviation = 0

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
    segment.altitude_end   = 25000 * Units.ft
    segment.climb_rate     = 100 * Units.ft / Units.min
    segment.air_speed      = 267 * Units.knots

    # add to mission
    mission.append_segment(segment)


    # ------------------------------------------------------------------
    # Initial Cruise Altitude - Top of Climb - 300 ft/min with climb speed
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Speed_Constant_Rate(base_segment)
    segment.tag = "initial_cruise_altitude"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)

    segment.altitude_start = 22500 * Units.ft
    segment.altitude_end   = 23500 * Units.ft
    segment.climb_rate     = 300 * Units.ft / Units.min
    segment.air_speed      = 275  * Units.knots #300 236.196

    # add to mission
    mission.append_segment(segment)





    return mission