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



def mission_setup(analyses,iteration_setup,parameters):
    # ------------------------------------------------------------------
    #   Initialize the Mission
    # ------------------------------------------------------------------

    mission = SUAVE.Analyses.Mission.Sequential_Segments()
    mission.tag = 'the_mission'

    # airport
    airport = SUAVE.Attributes.Airports.Airport()
    airport.altitude = iteration_setup.mission_iter.airport_altitude
    airport.delta_isa = iteration_setup.mission_iter.temperature_deviation
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
    idle_throttle = 0.15
    idle_throttle_descent = 0.15



    ########################################################################
    # Start Stuff for Battery_Propeller
    vehicle = analyses.vehicle
    ones_row = base_segment.state.ones_row
    base_segment.process.iterate.unknowns.network        = vehicle.propulsors.network.unpack_unknowns
    base_segment.process.iterate.residuals.network       = vehicle.propulsors.network.residuals
    base_segment.state.residuals.network = 0 * ones_row(6)  # 1 #2

    base_segment.process.iterate.initials.initialize_battery = SUAVE.Methods.Missions.Segments.Common.Energy.initialize_battery
    base_segment.state.unknowns.battery_voltage_under_load = vehicle.propulsors.network.battery.max_voltage * ones_row(1)
    #base_segment.battery_energy = vehicle.propulsors.network.battery.max_energy * 0.80
    base_segment.battery_energy = vehicle.propulsors.network.battery.max_energy * parameters.battery_soc_max

    #hybird
    base_segment.state.unknowns.rpm                             = 1200 * ones_row(1) #nominal rpm propeller --> no, fixed value is better
    base_segment.state.unknowns.rpm_wtp                         = 4300 * ones_row(1) #nominal rpm propellerWTP --> no, fixed value is better

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
    #   First Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "climb_1"

    # connect vehicle configuration
    segment.analyses.extend(analyses.takeoff)
    segment.state.numerics.number_control_points = number_control_climb

    segment.altitude_start = 0.0 * Units.ft
    segment.altitude_end = 1500.0 * Units.ft
    segment.air_speed = 160 * Units.knots  # 160.0 * Units.knots 146
    segment.throttle = climb_throttle


    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * segment.altitude_start/Units.ft + ems[0][0]
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    else:
        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Second Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------


    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "climb_2"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)
    segment.state.numerics.number_control_points = number_control_climb

    segment.altitude_start = 1500.0 * Units.ft
    segment.altitude_end = 4000.0 * Units.ft
    segment.air_speed = 167 * Units.knots#167.0 * Units.knots 152
    segment.throttle = climb_throttle
    # segment.climb_rate     = 915.   * Units['ft/min']

    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    else:
        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Third Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "climb_3"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)
    segment.state.numerics.number_control_points = number_control_climb

    segment.altitude_start = 4000.0 * Units.ft
    segment.altitude_end = 7000.0 * Units.ft
    segment.air_speed = 173 * Units.knots  # 173.0 * Units.knots 158
    segment.throttle = climb_throttle


    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    else:
        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Fourth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "climb_4"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)
    segment.state.numerics.number_control_points = number_control_climb

    segment.altitude_start = 7000.0 * Units.ft
    segment.altitude_end = 9000.0 * Units.ft
    segment.air_speed = 180 * Units.knots  # 180.0 * Units.knots 164
    segment.throttle = climb_throttle

    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    else:
        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

    # add to misison
    mission.append_segment(segment)
    # ------------------------------------------------------------------
    #   Fifth Climb Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    segment.tag = "climb_5"

    # connect vehicle configuration
    segment.analyses.extend(analyses.climb)
    segment.state.numerics.number_control_points = number_control_climb

    segment.altitude_start = 9000.0 * Units.ft
    segment.altitude_end = 11000.0 * Units.ft
    segment.air_speed = 185.6 * Units.knots#185.6 * Units.knots 170
    segment.throttle = climb_throttle

    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
    else:
        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

    # add to misison
    mission.append_segment(segment)

    cruise_altitude = iteration_setup.mission_iter.cruise_altitude

    if cruise_altitude > 11000:

        # ------------------------------------------------------------------
        #   Sixth Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        segment.tag = "climb_6"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)
        segment.state.numerics.number_control_points = number_control_climb

        segment.altitude_start = 11000.0 * Units.ft
        segment.altitude_end = 13000.0 * Units.ft
        segment.air_speed = 191.4 * Units.knots#191.4 * Units.knots 175
        segment.throttle = climb_throttle

        if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
            segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
            segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
        else:
            segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
            segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

        # add to misison
        mission.append_segment(segment)

        if cruise_altitude > 13000:

            # ------------------------------------------------------------------
            #   Seventh Climb Segment: Constant Speed, Constant Rate
            # ------------------------------------------------------------------

            segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
            segment.tag = "climb_7"

            # connect vehicle configuration
            segment.analyses.extend(analyses.climb)
            segment.state.numerics.number_control_points = number_control_climb

            segment.altitude_start = 13000.0 * Units.ft
            segment.altitude_end = 15000.0 * Units.ft
            segment.air_speed = 197.5 * Units.knots#197.5 * Units.knots 180
            segment.throttle = climb_throttle

            if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
                segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
                segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
            else:
                segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
                segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

            # add to misison
            mission.append_segment(segment)

            if cruise_altitude > 15000:

                # ------------------------------------------------------------------
                #   Eighth Climb Segment: Constant Speed, Constant Rate
                # ------------------------------------------------------------------

                segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
                segment.tag = "climb_8"

                # connect vehicle configuration
                segment.analyses.extend(analyses.climb)
                segment.state.numerics.number_control_points = number_control_climb

                segment.altitude_start = 15000.0 * Units.ft
                segment.altitude_end = 17000.0 * Units.ft
                segment.air_speed = 203.8 * Units.knots#203.8 * Units.knots 186
                segment.throttle = climb_throttle

                if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
                    segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
                    segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
                else:
                    segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
                    segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

                # add to misison
                mission.append_segment(segment)

                if cruise_altitude > 17000:

                    # ------------------------------------------------------------------
                    #   Ninth Climb Segment: Constant Speed, Constant Rate
                    # ------------------------------------------------------------------

                    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
                    segment.tag = "climb_9"

                    # connect vehicle configuration
                    segment.analyses.extend(analyses.climb)
                    segment.state.numerics.number_control_points = number_control_climb

                    segment.altitude_start = 17000.0 * Units.ft
                    #segment.altitude_end = 19000.0 * Units.ft
                    segment.altitude_end = 18000.0 * Units.ft
                    #segment.air_speed = 210.4 * Units.knots
                    segment.air_speed = 208.76 * Units.knots#208.76 * Units.knots 191
                    segment.throttle = climb_throttle

                    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
                        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
                        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
                    else:
                        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
                        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

                    # add to misison
                    mission.append_segment(segment)

                    # ------------------------------------------------------------------
                    #   Ninth Point 5 Climb Segment: Constant Speed, Constant Rate
                    # ------------------------------------------------------------------

                    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
                    segment.tag = "climb_9_5"

                    # connect vehicle configuration
                    segment.analyses.extend(analyses.climb)
                    segment.state.numerics.number_control_points = number_control_climb
                    #analyses.climb.energy.network.turboprop.propeller.propulsive_efficiency = 2.

                    #segment.altitude_start = 17000.0 * Units.ft
                    segment.altitude_start = 18000.0 * Units.ft
                    segment.altitude_end = 19000.0 * Units.ft
                    # segment.air_speed = 210.4 * Units.knots
                    segment.air_speed = 212.151 * Units.knots#212.151 * Units.knots 194
                    segment.throttle = climb_throttle

                    if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
                        segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
                        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
                    else:
                        segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
                        segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

                    # add to misison
                    mission.append_segment(segment)

                    if cruise_altitude > 19000:
                        # ------------------------------------------------------------------
                        #   Tenth Climb Segment: Constant Speed, Constant Rate
                        # ------------------------------------------------------------------

                        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
                        segment.tag = "climb_10"

                        # connect vehicle configuration
                        segment.analyses.extend(analyses.climb)
                        segment.state.numerics.number_control_points = number_control_climb

                        segment.altitude_start = 19000.0 * Units.ft
                        segment.altitude_end = 21000.0 * Units.ft
                        segment.air_speed = 217.4 * Units.knots#217.4 * Units.knots 199
                        segment.throttle = climb_throttle
                        if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
                            segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
                            segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
                        else:
                            segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
                            segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

                        # add to misison
                        mission.append_segment(segment)

                        if cruise_altitude > 21000:
                            # ------------------------------------------------------------------
                            #   Eleventh Climb Segment: Constant Speed, Constant Rate
                            # ------------------------------------------------------------------

                            segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
                            segment.tag = "climb_11"

                            # connect vehicle configuration
                            segment.analyses.extend(analyses.climb)
                            segment.state.numerics.number_control_points = number_control_climb

                            segment.altitude_start = 21000.0 * Units.ft
                            segment.altitude_end = 23000.0 * Units.ft
                            segment.air_speed = 224.6 * Units.knots#224.6 * Units.knots 205
                            segment.throttle = climb_throttle

                            if parameters.ems == 7 or parameters.ems == 8 or parameters.ems == 9:
                                segment.state.conditions.electric_throttle = (ems[1][0] - ems[0][0])/ iteration_setup.mission_iter.cruise_altitude * (segment.altitude_end + segment.altitude_start)/2/Units.ft + ems[0][0]
                                segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000
                            else:
                                segment.state.conditions.electric_throttle = climb_electric_throttle_lower11000
                                segment.state.conditions.electric_throttle_WTP = climb_electric_throttle_WTP_lower11000

                            # add to misison
                            mission.append_segment(segment)

                            if cruise_altitude > 23000:
                                # ------------------------------------------------------------------
                                #   Twelveth Climb Segment: Constant Speed, Constant Rate
                                # ------------------------------------------------------------------

                                segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
                                segment.tag = "climb_12"

                                # connect vehicle configuration
                                segment.analyses.extend(analyses.climb)
                                segment.state.numerics.number_control_points = number_control_climb

                                segment.altitude_start = 23000.0 * Units.ft
                                segment.altitude_end = 25000.0 * Units.ft
                                segment.air_speed = 232.3 * Units.knots#232.3 * Units.knots 212
                                segment.throttle = climb_throttle

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
    cruise_distance = iteration_setup.mission_iter.cruise_distance

    segment.air_speed = cruise_speed
    segment.distance = cruise_distance
    segment.state.conditions.electric_throttle = cruise_electric_throttle#-1.#-1.#-1.
    segment.state.conditions.electric_throttle_WTP = cruise_electric_throttle_WTP#1#1.#1

    # add to mission
    mission.append_segment(segment)

    if cruise_altitude > 23000:
        # ------------------------------------------------------------------
        #   First Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        # segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "descent_1"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 25000.0 * Units.ft
        segment.altitude_end = 23000.0 * Units.ft
        segment.air_speed = 232.3 * Units.knots
        # segment.air_speed = cruise_speed
        #segment.throttle = 0.25
        segment.throttle = idle_throttle
        # segment.descent_rate = 915 * Units.ft / Units.min
        # segment.descent_angle = 3. * Units.deg

        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP =descent_electric_throttle_WTP


        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 21000:
        # ------------------------------------------------------------------
        #   Second Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        # segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "descent_2"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 23000.0 * Units.ft
        segment.altitude_end = 21000.0 * Units.ft
        segment.air_speed = 224.6 * Units.knots
        # segment.air_speed = (224.6 * Units.knots + cruise_speed) /2
        # segment.air_speed = cruise_speed
        #segment.throttle = 0.25
        segment.throttle = idle_throttle
        # segment.descent_rate = 1300 * Units.ft / Units.min
        # segment.descent_angle = 3. * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 19000:
        # ------------------------------------------------------------------
        #   Third Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "descent_3"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 21000.0 * Units.ft
        segment.altitude_end = 19000.0 * Units.ft
        segment.air_speed = 217.4 * Units.knots
        #segment.air_speed = (217.4 * Units.knots + cruise_speed) /2
        segment.throttle = idle_throttle
        #segment.descent_rate = 1200 * Units.ft / Units.min
        #segment.descent_angle = 3. * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 17000:
        # ------------------------------------------------------------------
        #   Fourth Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "descent_4"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 19000.0 * Units.ft
        segment.altitude_end = 17000.0 * Units.ft
        segment.air_speed = 210.4 * Units.knots
        segment.throttle = idle_throttle
        # segment.descent_rate = 915 * Units.ft / Units.min
        #segment.descent_angle = 3. * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 15000:
        # ------------------------------------------------------------------
        #   Fifth Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "descent_5"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 17000.0 * Units.ft
        segment.altitude_end = 15000.0 * Units.ft
        segment.air_speed = 203.8 * Units.knots
        segment.throttle = idle_throttle
        # segment.descent_rate = 915 * Units.ft / Units.min
        #segment.descent_angle = 3. * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 13000:
        # ------------------------------------------------------------------
        #   Sixth Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "descent_6"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 15000.0 * Units.ft
        segment.altitude_end = 13000.0 * Units.ft
        segment.air_speed = 197.5 * Units.knots
        segment.throttle = idle_throttle
        # segment.descent_rate = 915 * Units.ft / Units.min
        #segment.descent_angle = 3. * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

    if cruise_altitude > 11000:
        # ------------------------------------------------------------------
        #   Seventh Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "descent_7"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 13000.0 * Units.ft
        segment.altitude_end = 11000.0 * Units.ft
        segment.air_speed = 191.4 * Units.knots
        segment.throttle = idle_throttle
        # segment.descent_rate = 915 * Units.ft / Units.min
        #segment.descent_angle = 3. * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Eighth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "descent_8"

    # connect vehicle configuration
    segment.analyses.extend(analyses.descent)

    segment.altitude_start = 11000.0 * Units.ft
    segment.altitude_end = 9000.0 * Units.ft
    segment.air_speed = 185.6 * Units.knots

    segment.throttle = idle_throttle
    # segment.descent_rate = 915 * Units.ft / Units.min
    #segment.descent_angle = 3. * Units.deg
    segment.state.conditions.electric_throttle = descent_electric_throttle
    segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Ninth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "descent_9"

    # connect vehicle configuration
    segment.analyses.extend(analyses.descent)

    segment.altitude_start = 9000.0 * Units.ft
    segment.altitude_end = 7000.0 * Units.ft
    segment.air_speed = 180.0 * Units.knots

    segment.throttle = idle_throttle
    # segment.descent_rate = 915 * Units.ft / Units.min
    #segment.descent_angle = 3. * Units.deg
    segment.state.conditions.electric_throttle = descent_electric_throttle
    segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Tenth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    # segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "descent_10"

    # connect vehicle configuration
    segment.analyses.extend(analyses.descent)

    segment.altitude_start = 7000.0 * Units.ft
    segment.altitude_end = 4000.0 * Units.ft
    segment.air_speed = 173.0 * Units.knots

    segment.throttle = idle_throttle
    # segment.descent_rate = 915 * Units.ft / Units.min
    # segment.descent_angle = 3. * Units.deg
    segment.state.conditions.electric_throttle = descent_electric_throttle
    segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Eleventh Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "descent_11"

    # connect vehicle configuration
    segment.analyses.extend(analyses.descent)

    segment.altitude_start = 4000.0 * Units.ft
    segment.altitude_end = 1500.0 * Units.ft
    segment.air_speed = 167.0 * Units.knots

    #segment.throttle = 1.
    # segment.descent_rate = 915 * Units.ft / Units.min
    segment.throttle = idle_throttle

    # segment.throttle = 1.
    #segment.descent_rate = 915 * Units.ft / Units.min
    segment.state.conditions.electric_throttle = descent_electric_throttle
    segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

    # segment.throttle = 1.
    #segment.descent_rate = 915 * Units.ft / Units.min



    # add to misison
    mission.append_segment(segment)

    # ------------------------------------------------------------------
    #   Twelveth Descent Segment: Constant Speed, Constant Rate
    # ------------------------------------------------------------------

    #segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
    # segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
    segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
    segment.tag = "descent_12"

    # connect vehicle configuration
    segment.analyses.extend(analyses.landing)

    segment.altitude_start = 1500.0 * Units.ft
    segment.altitude_end = 0.0 * Units.ft
    segment.air_speed = iteration_setup.mission_iter.approach_speed#113.7 * Units.knots #100 120


    #segment.throttle = 1.
    # segment.descent_rate = 915 * Units.ft / Units.min
    segment.descent_angle = 3. * Units.deg
    segment.state.conditions.electric_throttle = descent_electric_throttle
    segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

    # add to misison
    mission.append_segment(segment)

    Mission_name = iteration_setup.mission_iter.Mission_name


    reserve_flag = iteration_setup.mission_iter.reserve_flag

    if reserve_flag:
        # ------------------------------------------------------------------
        #   First Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        segment.tag = "reserve_climb_1"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 0.0 * Units.ft
        segment.altitude_end = 1500.0 * Units.ft
        segment.air_speed = 160.0 * Units.knots
        segment.throttle = climb_throttle
        segment.state.conditions.electric_throttle = reserve_climb_electric_throttle
        segment.state.conditions.electric_throttle_WTP = reserve_climb_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Second Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        segment.tag = "reserve_climb_2"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 1500.0 * Units.ft
        segment.altitude_end = 4000.0 * Units.ft
        segment.air_speed = 167.0 * Units.knots
        segment.throttle = climb_throttle
        # segment.climb_rate     = 915.   * Units['ft/min']
        segment.state.conditions.electric_throttle = reserve_climb_electric_throttle
        segment.state.conditions.electric_throttle_WTP = reserve_climb_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Third Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        segment.tag = "reserve_climb_3"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 4000.0 * Units.ft
        segment.altitude_end = 7000.0 * Units.ft
        segment.air_speed = 173.0 * Units.knots
        segment.throttle = climb_throttle
        segment.state.conditions.electric_throttle = reserve_climb_electric_throttle
        segment.state.conditions.electric_throttle_WTP = reserve_climb_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   Fourth Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        segment.tag = "reserve_climb_4"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 7000.0 * Units.ft
        segment.altitude_end = 9000.0 * Units.ft
        segment.air_speed = 180.0 * Units.knots
        segment.throttle = climb_throttle
        segment.state.conditions.electric_throttle = reserve_climb_electric_throttle
        segment.state.conditions.electric_throttle_WTP = reserve_climb_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)
        # ------------------------------------------------------------------
        #   Fifth Reserve Climb Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------
        #
        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        segment.tag = "reserve_climb_5"

        # connect vehicle configuration
        segment.analyses.extend(analyses.climb)

        segment.altitude_start = 9000.0 * Units.ft
        segment.altitude_end = 11000.0 * Units.ft
        segment.air_speed = 185.6 * Units.knots
        segment.throttle = climb_throttle
        segment.state.conditions.electric_throttle = reserve_climb_electric_throttle
        segment.state.conditions.electric_throttle_WTP = reserve_climb_electric_throttle_WTP

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
        segment.state.conditions.electric_throttle = reserve_electric_throttle
        segment.state.conditions.electric_throttle_WTP = reserve_electric_throttle_WTP

        # add to mission
        mission.append_segment(segment)


        # ------------------------------------------------------------------
        #   Third Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "reserve_descent_3"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 11000.0 * Units.ft
        segment.altitude_end = 9000.0 * Units.ft
        segment.air_speed = 185.6 * Units.knots
        segment.throttle = idle_throttle_descent
        #segment.descent_rate = 915. * Units['ft/min']
        #segment.descent_angle = 3 * Units.deg

        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   4# Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "reserve_descent_4"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 9000.0 * Units.ft
        segment.altitude_end = 7000.0 * Units.ft
        segment.air_speed = 180.0 * Units.knots
        segment.throttle = idle_throttle_descent
        #segment.descent_rate = 915. * Units['ft/min']
        #segment.descent_angle = 3 * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   5# Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "reserve_descent_5"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 7000.0 * Units.ft
        segment.altitude_end = 4000.0 * Units.ft
        segment.air_speed = 173.0 * Units.knots
        segment.throttle = idle_throttle_descent
        #segment.descent_rate = 915. * Units['ft/min']
        #segment.descent_angle = 3 * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   6# Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "reserve_descent_6"

        # connect vehicle configuration
        segment.analyses.extend(analyses.descent)

        segment.altitude_start = 4000.0 * Units.ft
        segment.altitude_end = 1500.0 * Units.ft
        segment.air_speed = 167.0 * Units.knots
        #segment.throttle = 1.
        #segment.descent_rate = 915. * Units['ft/min']
        segment.throttle = idle_throttle_descent
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

        # add to misison
        mission.append_segment(segment)

        # ------------------------------------------------------------------
        #   7# Reserve Descent Segment: Constant Speed, Constant Rate
        # ------------------------------------------------------------------

        #segment = Segments.Climb.Constant_Throttle_Constant_Speed(base_segment)
        #segment = Segments.Descent.Constant_Speed_Constant_Rate(base_segment)
        segment = Segments.Descent.Constant_Speed_Constant_Angle(base_segment)
        segment.tag = "reserve_descent_7"

        # connect vehicle configuration
        segment.analyses.extend(analyses.landing)

        segment.altitude_start = 1500.0 * Units.ft
        segment.altitude_end = 0.0 * Units.ft
        segment.air_speed = iteration_setup.mission_iter.approach_speed_reserve
        #segment.throttle = 1.
        #segment.descent_rate = 915. * Units['ft/min']
        segment.descent_angle = 3 * Units.deg
        segment.state.conditions.electric_throttle = descent_electric_throttle
        segment.state.conditions.electric_throttle_WTP = descent_electric_throttle_WTP

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
        segment.altitude = reserve_hold_altitude

        segment.air_speed = reserve_hold_speed
        segment.distance = reserve_hold_distance
        segment.state.conditions.electric_throttle = reserve_electric_throttle
        segment.state.conditions.electric_throttle_WTP = reserve_electric_throttle_WTP

        # add to mission
        mission.append_segment(segment)

    return mission