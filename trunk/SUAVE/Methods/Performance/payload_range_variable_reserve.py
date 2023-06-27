## @ingroup Methods-Performance
# payload_range_variable_reserve.py
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
import copy
from SUAVE.Plots.Mission_Plots import *
#from Plots import plot_mission

# ----------------------------------------------------------------------
#  Calculate vehicle Payload Range Diagram
# ----------------------------------------------------------------------

## @ingroup Methods-Performance
def payload_range_variable_reserve(vehicle,mission,cruise_segment_tag,iteration_setup):
    """Calculates a vehicle's payload range diagram. Includes plotting.

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

    # Flags for printing results in command line, write output file, and plot
    iprint = 1      # Flag for print output data in the prompt line
    iwrite = 1      # Flag for write an output file
    iplot  = 1      # Flag for plot payload range diagram
    ### could be an user input.
    ##      output_type: 1: Print only              (light)
    ##      output_type: 2: Print + Write           (medium)
    ##      output_type: 3: Print + Write + Plot    (complete)

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


    # Define payload range points
    #Point  = [ RANGE WITH MAX. PLD   , RANGE WITH MAX. FUEL , FERRY RANGE   ]
    TOW     = [ MTOW                               , MTOW                   , OEW + MaxFuel ]
    FUEL    = [ min(TOW[1] - OEW - MaxPLD,MaxFuel) , MaxFuel                , MaxFuel       ]
    PLD     = [ MaxPLD                             , MTOW - MaxFuel - OEW   , 0.            ]

    # allocating Range array
    R       = [0,0,0]

    # safe mission results of 3 payload range points
    results_mission = Data()
    results_mission.max_payload_mtom = Data()
    results_mission.max_fuel_mtom = Data()
    results_mission.ferry_range = Data()

    # allocating SoC array
    if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
        state_of_charge = [0,0,0]

    # evaluate the mission
    if iprint:
        print('\n\n\n .......... PAYLOAD RANGE DIAGRAM CALCULATION ..........\n')

    # loop for each point of Payload Range Diagram
    for i in range(len(TOW)):
        ##    for i in [2]:
        if iprint:
            print(('   EVALUATING POINT : ' + str(i+1)))

        # Define takeoff weight
        mission.segments[0].analyses.weights.vehicle.mass_properties.takeoff = TOW[i]

        # Define Payload
        mission.segments[0].analyses.weights.mass_properties.payload = PLD[i]

        if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
            if i == 2:
                mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP = np.ones_like(mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP)



        # Evaluate mission with current TOW
        results = mission.evaluate()
        segment = results.segments[cruise_segment_tag]

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
        reserves_pct = 0 #default
        errer_soc = -0.1

        while (abs(error_cruise) > 200 or abs(error_reserve) > 200 or abs(error_weight) > 0.5 or errer_soc > 0.0) and iter < maxIter:
            iter = iter + 1

            # Current total fuel burned in mission
            TotalFuel  = TOW[i] - results.segments[-1].conditions.weights.total_mass[-1,0] + reserves_pct

            # Difference between burned fuel and target fuel
            missingFuel = FUEL[i] - TotalFuel

            # Current distance and fuel consuption in the cruise segment
            CruiseDist = np.diff( segment.conditions.frames.inertial.position_vector[[0,-1],0] )[0]        # Distance [m]
            CruiseFuel = segment.conditions.weights.total_mass[0,0] - segment.conditions.weights.total_mass[-1,0]    # [kg]
            # Current specific range (m/kg)
            CruiseSR    = CruiseDist / CruiseFuel        # [m/kg]
            #print('CruiseSR PR %.2f m/kg' % CruiseSR)

            # Estimated distance that will result in total fuel burn = target fuel
            DeltaDist  =  CruiseSR *  missingFuel

            if DeltaDist < 0:
                mission.segments[cruise_segment_tag].distance = (CruiseDist + DeltaDist * 3/2) #try to make it safer
            elif DeltaDist >= 0:
                mission.segments[cruise_segment_tag].distance = (CruiseDist + DeltaDist* 2/3) #try to make it safer

            error_cruise = DeltaDist

            climb_segments = [key for key in results.segments.keys() if (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key))]



            if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
                state_of_charge[i] = results.segments[-1].conditions.propulsion.state_of_charge[-1][0]
                SoC_min = vehicle.propulsors.network.battery.soc_min

                errer_soc = SoC_min - state_of_charge[i]
                #print('errer_soc %.4f ' % errer_soc)

                for misson_segment_tag in climb_segments:

                    if state_of_charge[i] < 0.0 and abs(error_weight) < 100:
                        if np.any(mission.segments[misson_segment_tag].conditions.propulsion.battery_draw[:, 0, None] < 0):
                            mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.995
                    elif state_of_charge[i] < 0.0:
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.7
                        if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                            mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                    elif state_of_charge[i] < (SoC_min - 0.03):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.50
                        if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                            mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                    elif state_of_charge[i] < (SoC_min - 0.01):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.97
                        if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                            mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                    elif state_of_charge[i] < (SoC_min):
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 0.99
                        if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                            mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                    elif state_of_charge[i] > (SoC_min) and mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP <1.0:
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 1.005
                        if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                            mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                    elif state_of_charge[i] > (SoC_min + 0.01) and mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP <1.0:
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP * 1.03
                        if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                            mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)
                    elif i > 0 and error_cruise > 500000: #
                        mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP *= 0.7
                        if mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP < abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle):
                            mission.segments[misson_segment_tag].state.conditions.electric_throttle_WTP = abs(mission.segments[misson_segment_tag].state.conditions.electric_throttle)

                if state_of_charge[i] < (SoC_min) and abs(error_weight) < 100 and abs(error_cruise) < 50000 and abs(error_reserve) < 2000: #cruise
                    iter = 1
                    if np.any(mission.segments[cruise_segment_tag].conditions.propulsion.battery_draw[:, 0, None] < 0):
                        mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP * 0.995

                if state_of_charge[i] < -0.1: #cruise
                    iter = 1
                    if np.any(mission.segments[cruise_segment_tag].conditions.propulsion.battery_draw[:, 0, None] < 0):
                        mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP = mission.segments[cruise_segment_tag].state.conditions.electric_throttle_WTP * 0.995

            # running mission with new distance
            results = mission.evaluate()
            segment = results.segments[cruise_segment_tag]


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
            error_weight = (TOW[i] - results.segments[-1].conditions.weights.total_mass[-1,0]) - FUEL[i] + reserves_pct

            # for each iter plot
            # plot_mission(results)
            # plot_flight_conditions(results, 'bo-', False)
            # plot_electronic_conditions(results, 'bo-')
            # import pylab as plt
            # plt.show()

            # if i == 2:
            #     plot_mission(results)
            #     plot_flight_conditions(results, 'bo-', False)
            #     plot_electronic_conditions(results, 'bo-')
            #     import pylab as plt
            #     plt.show()



            if iprint:
                if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
                    print(('     iter: ' +str('%2g' % iter) + ' | Target Fuel: ' \
                           + str('%8.0F' % FUEL[i]) + ' (kg) | Current Fuel: ' \
                           + str('%8.0F' % (error_weight+FUEL[i]))+' (kg) | Residual : '+str('%8.2F' % error_weight)+str('%8.0F' % error_cruise)+str('%8.0F' % error_reserve) \
                           + ' | SoC End Mission: ' + str('%8.3F' % state_of_charge[i])))
                else:
                    print(('     iter: ' +str('%2g' % iter) + ' | Target Fuel: ' \
                           + str('%8.0F' % FUEL[i]) + ' (kg) | Current Fuel: ' \
                           + str('%8.0F' % (error_weight+FUEL[i]))+' (kg) | Residual : '+str('%8.0F' % error_weight)+str('%8.0F' % error_cruise)+str('%8.0F' % error_reserve) ))

        # Allocating resulting range in ouput array.
        R[i] = ( results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1,0] ) * Units.m / Units.nautical_mile      #Distance [nm]
        if i == 0:
            results_mission.max_payload_mtom = copy.deepcopy(results)
            # plot_mission(results_mission.max_payload_mtom)
            # plot_flight_conditions(results_mission.max_payload_mtom, 'bo-', False)
            # plot_electronic_conditions(results_mission.max_payload_mtom, 'bo-')
            # import pylab as plt
            # plt.show()
        elif i == 1:
            results_mission.max_fuel_mtom = copy.deepcopy(results)
            # plot_mission(results_mission.max_fuel_mtom)
            # plot_flight_conditions(results_mission.max_fuel_mtom, 'bo-', False)
            # plot_electronic_conditions(results_mission.max_fuel_mtom, 'bo-')
            # import pylab as plt
            # plt.show()
        elif i == 2:
            results_mission.ferry_range = copy.deepcopy(results)
            # plot_mission(results_mission.ferry_range)
            # plot_flight_conditions(results_mission.ferry_range, 'bo-', False)
            # plot_electronic_conditions(results_mission.ferry_range, 'bo-')
            # import pylab as plt
            # plt.show()


    # Inserting point (0,0) in output arrays
    R.insert(0,0)
    PLD.insert(0,MaxPLD)
    FUEL.insert(0,0)
    TOW.insert(0,0)
    if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
        state_of_charge.insert(0,results.segments[0].conditions.propulsion.state_of_charge[0][0])

    # packing results
    payload_range_variable_reserve.range     = np.multiply(R,1.0*Units.nautical_mile / Units.m) # [m]
    payload_range_variable_reserve.payload   = PLD
    payload_range_variable_reserve.fuel      = FUEL
    payload_range_variable_reserve.takeoff_weight = TOW
    if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
        payload_range_variable_reserve.state_of_charge_end = state_of_charge
    payload_range_variable_reserve.results_mission = results_mission

    # Write output file
    if iwrite:
        import datetime                 # importing library

        fid = open('PayloadRangeDiagram.dat','w')   # Open output file
        fid.write('Output file with Payload Range Diagram details\n\n') #Start output printing

        fid.write( ' Maximum Takeoff Weight ...........( MTOW ).....: ' + str( '%8.0F'   %   MTOW   ) + ' kg\n' )
        fid.write( ' Operational Empty Weight .........( OEW  ).....: ' + str( '%8.0F'   %   OEW    ) + ' kg\n' )
        fid.write( ' Maximum Zero Fuel Weight .........( MZFW ).....: ' + str( '%8.0F'   %   MZFW   ) + ' kg\n' )
        fid.write( ' Maximum Payload Weight ...........( PLDMX  )...: ' + str( '%8.0F'   %   MaxPLD ) + ' kg\n' )
        fid.write( ' Maximum Fuel Weight ..............( FUELMX )...: ' + str( '%8.0F'   %   MaxFuel) + ' kg\n' )
        fid.write( ' Reserve PCT Fuel  .............................: ' + str( '%8.0F'   %   reserves_pct)+ ' kg\n\n' )

        if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
            fid.write( '    RANGE    |   PAYLOAD   |   FUEL      |    TOW      |   SoC_End   |  \n')
            fid.write( '     nm      |     kg      |    kg       |     kg      |      -      |  \n')

            for i in range(len(TOW)):
                fid.write( str('%10.0f' % R[i]) + '   |' + str('%10.0f' % PLD[i]) + '   |' + str('%10.0f' % FUEL[i]) + '   |' + ('%10.0f' % TOW[i]) + '   |' + ('%10.2f' % state_of_charge[i]) + '   |\n')

        else:
            fid.write( '    RANGE    |   PAYLOAD   |   FUEL      |    TOW      |  \n')
            fid.write( '     nm      |     kg      |    kg       |     kg      |  \n')

            for i in range(len(TOW)):
                fid.write( str('%10.0f' % R[i]) + '   |' + str('%10.0f' % PLD[i]) + '   |' + str('%10.0f' % FUEL[i]) + '   |' + ('%10.0f' % TOW[i]) + '   |\n')

        # Print timestamp
        fid.write(2*'\n'+ 43*'-'+ '\n' + datetime.datetime.now().strftime(" %A, %d. %B %Y %I:%M:%S %p"))
        fid.close

    # Print data in command line
    if iprint:
        if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):
            print( '\n\n                        RESULTS\n')
            print( '    RANGE    |   PAYLOAD   |   FUEL      |    TOW      |   SoC_End   |')
            print( '     nm      |     kg      |    kg       |     kg      |      -      |')
            for i in range(len(TOW)):
                print(( str('%10.0f' % R[i]) + '   |' + str('%10.0f' % PLD[i]) + '   |' + str('%10.0f' % FUEL[i]) + '   |' + ('%10.0f' % TOW[i]) + '   |' + ('%10.2f' % state_of_charge[i]) + '   |'))
            print(('\n\n   Elapsed time: ' + str('%6.2f' % (time.time() - start_time)) + 's'))
        else:
            print( '\n\n                        RESULTS\n')
            print( '    RANGE    |   PAYLOAD   |   FUEL      |    TOW      |')
            print( '     nm      |     kg      |    kg       |     kg      |')
            for i in range(len(TOW)):
                print(( str('%10.0f' % R[i]) + '   |' + str('%10.0f' % PLD[i]) + '   |' + str('%10.0f' % FUEL[i]) + '   |' + ('%10.0f' % TOW[i]) + '   |'))
            print(('\n\n   Elapsed time: ' + str('%6.2f' % (time.time() - start_time)) + 's'))

    #   Plot Payload Range
    # if iplot:
    #
    #     #import pylab
    #     import pylab as plt
    #
    #     title = "Payload Range Diagram"
    #     plt.figure(0)
    #     plt.plot(R,PLD,'r')
    #     plt.xlabel('Range (nm)'); plt.ylabel('Payload (kg)'); plt.title(title)
    #     plt.grid(True)
    #   #  plt.show()

    return payload_range_variable_reserve
