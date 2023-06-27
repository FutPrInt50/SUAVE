## @ingroup Methods-Performance
# off_design_mission_calculation_variable_reserve.py
#
# Created:  Apr 2014, T. Orra
# Modified: Jan 2016, E. Botero
#           Jun 2022, J. Mangold

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Units, Data
import time
import numpy as np
import SUAVE
from SUAVE.Plots.Mission_Plots import *
# from Plots import plot_mission

# ----------------------------------------------------------------------
#  Calculate vehicle off_design_mission_calculation_variable_reserve
# ----------------------------------------------------------------------

## @ingroup Methods-Performance
def off_design_mission_calculation_variable_reserve(vehicle,mission,cruise_segment_tag,iteration_setup,payload,range):
    """Calculates a vehicle's off design mission. Includes plotting.

    Assumptions:
    Constant altitude cruise

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
    # elapsed time start
    start_time = time.time()

    # Flags for printing results in command line
    iprint = 1      # Flag for print output data in the prompt line


    # for different propulsion architectures
    Nets  = SUAVE.Components.Energy.Networks
    prop = vehicle.propulsors.network


    #unpack
    masses = vehicle.mass_properties
    if not masses.operating_empty:
        print("Error calculating Payload Range Diagram: Vehicle Operating Empty not defined")
        return True
    else:
        OEW = masses.operating_empty

    if not masses.max_zero_fuel:
        print("Error calculating Payload Range Diagram: Vehicle MZFW not defined")
        return True
    else:
        MZFW = vehicle.mass_properties.max_zero_fuel

    if not masses.max_takeoff:
        print("Error calculating Payload Range Diagram: Vehicle MTOW not defined")
        return True
    else:
        MTOW = vehicle.mass_properties.max_takeoff

    if not masses.max_payload:
        MaxPLD = MZFW - OEW  # If payload max not defined, calculate based in design weights
    else:
        MaxPLD = vehicle.mass_properties.max_payload
        MaxPLD = min(MaxPLD , MZFW - OEW) #limit in structural capability

    if not masses.max_fuel:
        MaxFuel = MTOW - OEW # If not defined, calculate based in design weights
    else:
        MaxFuel = vehicle.mass_properties.max_fuel  # If max fuel capacity not defined
        MaxFuel = min(MaxFuel, MTOW - OEW)

    if payload > MaxPLD:
        payload = MaxPLD
        print('Payload higher than Max Payload')

    # Define payload range points
    #Point  = [ RANGE WITH MAX. PLD   , RANGE WITH MAX. FUEL , FERRY RANGE   ]
    TOW     = MTOW
    FUEL    = min(TOW - OEW - MaxPLD,MaxFuel)
    PLD     = payload

    # allocating Range array
    R       = 0

    # allocating SoC array
    if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
        state_of_charge = 0

    # evaluate the mission
    if iprint:
        print('\n\n\n .......... OFF-DESIGN PERFORMANCE CALCULATION ..........\n')

    # loop for each point of Payload Range Diagram
    #for i in range(len(TOW)):
        ##    for i in [2]:
    if iprint:
        print(('   EVALUATING POINT : ' ))

    # Define takeoff weight
    mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff = TOW
    
    # Define Payload
    mission.segments[0].analyses.weights.mass_properties.payload = PLD

    # Evaluate mission with current TOW
    results = mission.evaluate()
    segment = results.segments[cruise_segment_tag]

    FUEL  = TOW - results.segments[-1].conditions.weights.total_mass[-1,0]

    # Distance convergency in order to have total fuel equal to target fuel
    #
    # User don't have the option of run a mission for a given fuel. So, we
    # have to iterate distance in order to have total fuel equal to target fuel
    #

    maxIter = 25 # maximum iteration limit
    tol = 1.     # fuel convergency tolerance
    error_cruise = 9999.  # error to be minimized
    iter = 0     # iteration count


    #New
    error_reserve = 9999. # error reserve to be minimized
    error_weight = 9999. # error weight to be minimized
    reserves_pct = 0
    errer_soc = -0.1

    while (abs(error_cruise) > 200 or abs(error_reserve) > 200 or abs(error_weight) > 0.5 or errer_soc > 0.0) and iter < maxIter:
        iter = iter + 1

        # Current total fuel burned in mission
        #FUEL  = TOW - results.segments[-1].conditions.weights.total_mass[-1,0]

        # Difference between burned fuel and target fuel
        #missingFuel = FUEL[i] - TotalFuel - reserves

        mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff = OEW + FUEL + payload
        # if mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff > MTOW:
        #     mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff = MTOW
        #     print('TOW > MTOM')
        TOW = mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff

        # Current distance and fuel consuption in the cruise segment
        #CruiseDist = np.diff( segment.conditions.frames.inertial.position_vector[[0,-1],0] )[0]        # Distance [m]
        CruiseFuel = segment.conditions.weights.total_mass[0,0] - segment.conditions.weights.total_mass[-1,0]    # [kg]
        # Current specific range (m/kg)
        #CruiseSR    = CruiseDist / CruiseFuel        # [m/kg]

        # Estimated distance that will result in total fuel burn = target fuel
        #DeltaDist  =  CruiseSR *  missingFuel

        CruiseDist = np.diff( segment.conditions.frames.inertial.position_vector[[0,-1],0] )[0]        # Distance [m]

        climb_segments = [key for key in results.segments.keys() if (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key))]
        first_climb_segment = climb_segments[0]
        last_climb_segment = climb_segments[-1]
        descent_segments = [key for key in results.segments.keys() if (('descent' in key) and ('reserve' not in key) and ('second_leg' not in key))]
        first_descent_segment = descent_segments[0]
        last_descent_segment = descent_segments[-1]


        block_distance = results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1,0]  * Units.m

        DeltaDist = range - block_distance
        error_cruise = DeltaDist

        if DeltaDist < 0:
            mission.segments[cruise_segment_tag].distance = (CruiseDist + DeltaDist * 3/2) #try to make it safer
        elif DeltaDist >= 0:
            mission.segments[cruise_segment_tag].distance = (CruiseDist + DeltaDist* 2/3) #try to make it safer


        if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
            state_of_charge = results.segments[-1].conditions.propulsion.state_of_charge[-1][0]
            SoC_min = vehicle.propulsors.network.battery.soc_min

            errer_soc = SoC_min - state_of_charge
            #print('errer_soc %.4f ' % errer_soc)
            #neue elif falls nichts mehr geändert wird, dann cruise throttle wtp kleiner werden! bzw battery drwww

            for misson_segment_tag in climb_segments:
                if state_of_charge < 0.0:
                    mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.7
                    if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                elif state_of_charge < (SoC_min - 0.03):
                    mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.50
                    if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                elif state_of_charge < (SoC_min - 0.01):
                    mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.97
                    if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                elif state_of_charge < (SoC_min):
                    mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.99
                    if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                elif state_of_charge > (SoC_min) and mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP <1.0:
                    mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 1.005
                    if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                elif state_of_charge > (SoC_min + 0.01) and mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP <1.0:
                    mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 1.03
                    if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                elif error_cruise > 500000: #
                    mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP *= 0.7
                    if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                elif state_of_charge < (SoC_min) and error_weight < 1 and error_cruise < 10:
                    if min(mission.segments[cruise_segment_tag].conditions.propulsion.battery_draw) < 0:
                        mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP * 0.99

                if state_of_charge < (SoC_min) and abs(error_weight) < 1 and abs(error_cruise) < 200 and abs(error_reserve) < 200: #cruise
                    iter = 1
                    if np.any(mission.segments[cruise_segment_tag].conditions.propulsion.battery_draw[:, 0, None] < 0):
                        mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP * 0.9995


        # running mission with new distance
        results = mission.evaluate()
        segment = results.segments[cruise_segment_tag]

        # Difference between burned fuel and target fuel
        #err = ( TOW[i] - results.segments[-1].conditions.weights.total_mass[-1,0] ) - FUEL[i] + reserves
        error_weight = (TOW - results.segments[-1].conditions.weights.total_mass[-1,0]) - FUEL + reserves_pct

        # # for each iter plot
        # plot_mission(results)
        # plot_flight_conditions(results, 'bo-', False)
        # plot_electronic_conditions(results, 'bo-')
        # import pylab as plt
        # plt.show()

        climb_segments = [key for key in results.segments.keys() if (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key))]
        first_climb_segment = climb_segments[0]
        last_climb_segment = climb_segments[-1]
        descent_segments = [key for key in results.segments.keys() if (('descent' in key) and ('reserve' not in key) and ('second_leg' not in key))]
        first_descent_segment = descent_segments[0]
        last_descent_segment = descent_segments[-1]

        block_fuel = results.segments[first_climb_segment].conditions.weights.total_mass[0] - \
                     results.segments[last_descent_segment].conditions.weights.total_mass[-1]

        #New Difference Reserve
        reserve_climb_segments = [key for key in results.segments.keys() if (('climb' in key) and ('reserve' in key) and ('second_leg' not in key))]
        first_reserve_climb_segment = reserve_climb_segments[0]
        last_reserve_climb_segment = reserve_climb_segments[-1]

        reserve_descent_segments = [key for key in results.segments.keys() if (('descent' in key) and ('reserve' in key) and ('second_leg' not in key))]
        first_reserve_descent_segment = reserve_descent_segments[0]
        last_reserve_descent_segment = reserve_descent_segments[-1]

        reserve_climb_distance = (results.segments[last_reserve_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                                  results.segments[first_reserve_climb_segment].conditions.frames.inertial.position_vector[0][0])

        reserve_cruise_distance = (results.segments.reserve_cruise.conditions.frames.inertial.position_vector[-1][0] - \
                                   results.segments.reserve_cruise.conditions.frames.inertial.position_vector[0][0])

        reserve_descent_distance = (results.segments[last_reserve_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                                    results.segments[first_reserve_descent_segment].conditions.frames.inertial.position_vector[0][0])

        error_reserve = iteration_setup.mission_iter.reserve_distance - (reserve_climb_distance + reserve_cruise_distance + reserve_descent_distance)



        mission.segments.reserve_cruise.distance = mission.segments.reserve_cruise.distance + error_reserve

        reserves_pct = block_fuel * iteration_setup.mission_iter.reserve_trip_pct

        alternate_fuel = results.segments[first_reserve_climb_segment].conditions.weights.total_mass[0] - \
                         results.segments[last_reserve_descent_segment].conditions.weights.total_mass[-1]

        hold_fuel = results.segments['hold'].conditions.weights.total_mass[0] - \
                    results.segments['hold'].conditions.weights.total_mass[-1]

        reserve_fuel = reserves_pct + alternate_fuel + hold_fuel

        FUEL = block_fuel + reserve_fuel

        # if FUEL > MaxFuel:
        #     FUEL = MaxFuel
        #     print('FUEL > MaxFuel')



        if iprint:
            if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
                state_of_charge = results.segments[-1].conditions.propulsion.state_of_charge[-1][0]
                print(('     iter: ' +str('%2g' % iter) + ' | Estimated Fuel: ' \
                       + str('%8.0F' % FUEL) + ' (kg) | Current Fuel: ' \
                       + str('%8.0F' % (error_weight+FUEL))+' (kg) | Residual : '+str('%8.2F' % error_weight)+str('%8.0F' % error_cruise)+str('%8.0F' % error_reserve) \
                       + ' | SoC End Mission: ' + str('%8.3F' % state_of_charge)))
            else:
                print(('     iter: ' +str('%2g' % iter) + ' | Estimated Fuel: ' \
                       + str('%8.0F' % FUEL) + ' (kg) | Current Fuel: ' \
                       + str('%8.0F' % (error_weight+FUEL))+' (kg) | Residual : '+str('%8.0F' % error_weight)+str('%8.0F' % error_cruise)+str('%8.0F' % error_reserve)))

    # Allocating resulting range in ouput array.
    R = ( results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1,0] ) * Units.m / Units.nautical_mile      #Distance [nm]

    # # Inserting point (0,0) in output arrays
    # R.insert(0,0)
    # PLD.insert(0,MaxPLD)
    # FUEL.insert(0,0)
    # TOW.insert(0,0)

    # packing results
    off_design_mission_calculation_variable_reserve.range     = np.multiply(R,1.0*Units.nautical_mile / Units.m) # [m]
    off_design_mission_calculation_variable_reserve.payload   = PLD
    off_design_mission_calculation_variable_reserve.fuel      = FUEL
    off_design_mission_calculation_variable_reserve.takeoff_weight = TOW
    off_design_mission_calculation_variable_reserve.trip_fuel = block_fuel
    if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
        off_design_mission_calculation_variable_reserve.state_of_charge_end = state_of_charge


    # Print data in command line
    if iprint:
        print( '\n\n                        RESULTS\n')
        if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
            print( '    RANGE    |   PAYLOAD   |   FUEL      |    TOW      |  TRIP FUEL  |   SoC_End   |')
            print( '     nm      |     kg      |    kg       |     kg      |      kg     |      -      |')

            print(( str('%10.0f' % R) + '   |' + str('%10.0f' % PLD) + '   |' + str('%10.0f' % FUEL) + '   |' + ('%10.0f' % TOW) + '   |' + ('%10.0f' % block_fuel) + '   |' + ('%10.2f' % state_of_charge) + '   |'))

        else:
            print( '    RANGE    |   PAYLOAD   |   FUEL      |    TOW      |  TRIP FUEL  |')
            print( '     nm      |     kg      |    kg       |     kg      |      kg     |')

            print(( str('%10.0f' % R) + '   |' + str('%10.0f' % PLD) + '   |' + str('%10.0f' % FUEL) + '   |' + ('%10.0f' % TOW) + '   |' + ('%10.0f' % block_fuel) + '   |'))
        print(('\n\n   Elapsed time: ' + str('%6.2f' % (time.time() - start_time)) + 's'))



    return off_design_mission_calculation_variable_reserve, results
