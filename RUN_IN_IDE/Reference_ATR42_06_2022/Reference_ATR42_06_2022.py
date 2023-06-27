## Reference_ATR42_06_2022.py
# 
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Jonas Mangold, mangold@ifb.uni-stuttgart.de, University of Stuttgart
#   Dominik Eisenhut, eisenhut@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Sep 2021, J. Mangold
# Modified: Oct 2021, D. Eisenhut
#           Jun 2022, J. Mangold
#           Jun 2022, F. Brenner
#           Jul 2022, D. Eisenhut
#           Jul 2022, F. Brenner

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

#Python Imports
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
from Plots import plot_mission
from SUAVE.Methods.Performance  import payload_range_variable_reserve, off_design_mission_calculation_variable_reserve
from Mission_Setup_angle import mission_setup # or Mission_Setup_angle
from Mission_Setup_FutPrInt50 import mission_setup_FutPrInt50
from Mission_Setup_Sizing_Chart import mission_setup_Sizing_Chart
from Results_Show import results_show
from Vehicle_Setup import vehicle_setup
from SUAVE.Methods.Performance.sizing_chart_mission_evaluate import sizing_chart_mission_evaluate
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity_variable_CG import compute_component_centers_of_gravity_variable_CG
from SUAVE.Methods.Figures_of_Merit.Figure_of_Merit import Figure_of_Merit
from SUAVE.Methods.Propulsion import gas_turbine_calculate_emission

from pathlib import Path
import os
from SUAVE.Input_Output.Results.write_output import write_output_files


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------

def main(iteration_setup):
    # ---------------------------------------------------------------------------------------
    # INITIALIZING AIRCRAFT

    configs, analyses, vehicle = full_setup(iteration_setup)

    print('full setup OK')
    #simple_sizing(configs, analyses)

    # Determine the vehicle weight breakdown (independent of mission fuel usage)
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate("Raymer")


    deltacg = 2
    while abs(deltacg) > 1e-5:
        compute_component_centers_of_gravity_variable_CG(configs.base)
        oldcg = configs.base.mass_properties.center_of_gravity[0][0]
        configs.base.center_of_gravity()    # CG @ TOM
        configs.base.store_diff()

        newcg = configs.base.mass_properties.center_of_gravity[0][0]
        deltacg = newcg - oldcg



    configs.finalize()
    analyses.finalize()


    #simple_sizing(configs, analyses)



    if generate_surrogates:
        if save_surrogates:
            np.save('CL_surrogate_sub.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.lift_coefficient_sub)
            np.save('CL_surrogate_sup.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.lift_coefficient_sup)
            np.save('CL_surrogate_trans.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.lift_coefficient_trans)
            np.save('CL_w_surrogates_sub.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_lift_coefficient_sub)
            np.save('CL_w_surrogates_sup.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_lift_coefficient_sup)
            np.save('CL_w_surrogates_trans.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_lift_coefficient_trans)
            np.save('CDi_surrogate_sub.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.drag_coefficient_sub)
            np.save('CDi_surrogate_sup.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.drag_coefficient_sup)
            np.save('CDi_surrogate_trans.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.drag_coefficient_trans)
            np.save('CDi_w_surrogates_sub.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_drag_coefficient_sub)
            np.save('CDi_w_surrogates_sup.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_drag_coefficient_sup)
            np.save('CDi_w_surrogates_trans.npy', analyses.configs.base.aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_drag_coefficient_trans)
    else:
        for config in analyses.configs.keys():
            try:
                analyses.configs[config].aerodynamics.process.compute.lift.inviscid_wings.evaluate = analyses.configs[config].aerodynamics.process.compute.lift.inviscid_wings.evaluate_surrogate
            except:
                pass



    # ---------------------------------------------------------------------------------------
    # MISSION ANALYSIS

    mission = analyses.missions.base
    results = mission.evaluate()
    print('MISSION OK')

    #simple_sizing(configs, analyses)


    return mission, results, configs, analyses

# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup(iteration_setup):
    # vehicle data
    vehicle = vehicle_setup(iteration_setup)
    configs = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)
    configs_analyses.vehicle = Data()
    configs_analyses.vehicle = vehicle

    # mission analyses
    #mission = mission_setup_FutPrInt50(configs_analyses,iteration_setup)
    mission = mission_setup(configs_analyses,iteration_setup)
    missions_analyses = missions_setup(mission)

    analyses = SUAVE.Analyses.Analysis.Container()
    analyses.configs = configs_analyses
    analyses.missions = missions_analyses

    return configs, analyses, vehicle


# ----------------------------------------------------------------------
#   Define the Vehicle Analyses
# ----------------------------------------------------------------------

def analyses_setup(configs):
    analyses = SUAVE.Analyses.Analysis.Container()

    # build a base analysis for each config
    for tag, config in configs.items():
        analysis = base_analysis(config)
        analyses[tag] = analysis

    return analyses


def base_analysis(vehicle):
    # ------------------------------------------------------------------
    #   Initialize the Analyses
    # ------------------------------------------------------------------     
    analyses = SUAVE.Analyses.Vehicle()

    # ------------------------------------------------------------------
    #  Basic Geometry Relations
    sizing = SUAVE.Analyses.Sizing.Sizing()
    sizing.features.vehicle = vehicle
    analyses.append(sizing)

    # ------------------------------------------------------------------
    #  Weights

    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    weights.settings.weight_reduction_factors.main_wing = - 0.312
    weights.settings.weight_reduction_factors.empennage = -0.54
    weights.settings.weight_reduction_factors.fuselage = 0.0389
    weights.settings.weight_reduction_factors.structural = 0
    weights.settings.weight_reduction_factors.systems = 0.05405
    weights.settings.weight_reduction_factors.operating_items = -0.4351
    weights.settings.weight_reduction_factors.landing_gear = -0.0615
    weights.settings.weight_reduction_factors.propulsion = 0.135
    weights.settings.payload = Data()
    weights.settings.payload.pax = 95 * Units.kg     # FUTPRINT50: 106 kg = pax + carry-on
    weights.settings.payload.baggage = 0 * Units.kg  # FUTPRINT50: 0 kg


    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = Data()
    aerodynamics.settings.drag_coefficient_increment.base    =  0
    aerodynamics.settings.drag_coefficient_increment.takeoff =  0.01 #0.00206 # to account for average flaps and L/G drag
    aerodynamics.settings.drag_coefficient_increment.climb   =0#0.0001#0.005# 0.018# 0.03
    aerodynamics.settings.drag_coefficient_increment.cruise  =0.00005#0.002#  0.0036#0.003#0.006
    aerodynamics.settings.drag_coefficient_increment.descent =0#0.0001#  0.002
    aerodynamics.settings.drag_coefficient_increment.landing =  0.0190 # to account for average flaps and L/G drag
    aerodynamics.settings.drag_coefficient_increment.second_segment = 0
    aerodynamics.settings.drag_coefficient_increment.second_segment_oei = 0
    aerodynamics.settings.drag_coefficient_increment.ceiling_oei = 0

    aerodynamics.settings.drag_coefficient_due_to_prop = 0.023 #0.015 # 0.023 #Embraer #0.044  # 0.042 0.043 0.046 475 (475) [600] 0.060 0.028
    aerodynamics.settings.oswald_efficiency_factor =0.715#0.71#0.82 #0.83#0.755#0.5 #0.92 0.93 0.96 0.83 0.82 (0.79) [0.83] 0.83

    if not generate_surrogates:
        aerodynamics.settings.use_surrogate = False
        aerodynamics.process.compute.lift.inviscid_wings.evaluate = aerodynamics.process.compute.lift.inviscid_wings.evaluate_surrogate

        aerodynamics.process.compute.lift.inviscid_wings.surrogates.lift_coefficient_sub = np.load('CL_surrogate_sub.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.lift_coefficient_sup = np.load('CL_surrogate_sup.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.lift_coefficient_trans = np.load('CL_surrogate_trans.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_lift_coefficient_sub = np.load('CL_w_surrogates_sub.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_lift_coefficient_sup = np.load('CL_w_surrogates_sup.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_lift_coefficient_trans = np.load('CL_w_surrogates_trans.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.drag_coefficient_sub = np.load('CDi_surrogate_sub.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.drag_coefficient_sup = np.load('CDi_surrogate_sup.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.drag_coefficient_trans = np.load('CDi_surrogate_trans.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_drag_coefficient_sub = np.load('CDi_w_surrogates_sub.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_drag_coefficient_sup = np.load('CDi_w_surrogates_sup.npy', allow_pickle=True)[()]
        aerodynamics.process.compute.lift.inviscid_wings.surrogates.wing_drag_coefficient_trans = np.load('CDi_w_surrogates_trans.npy', allow_pickle=True)[()]
    analyses.append(aerodynamics)

    # ------------------------------------------------------------------
    #  Stability Analysis
    stability = SUAVE.Analyses.Stability.Fidelity_Zero()
    stability.geometry = vehicle
    analyses.append(stability)

    # ------------------------------------------------------------------
    #  Energy
    energy = SUAVE.Analyses.Energy.Energy()
    energy.network = vehicle.propulsors
    analyses.append(energy)

    # ------------------------------------------------------------------
    #  Planet Analysis
    planet = SUAVE.Analyses.Planets.Planet()
    analyses.append(planet)

    # ------------------------------------------------------------------
    #  Atmosphere Analysis
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    atmosphere.features.planet = planet.features
    analyses.append(atmosphere)

    return analyses



# ----------------------------------------------------------------------
#   Define the Configurations
# ---------------------------------------------------------------------

def configs_setup(vehicle):
    # ------------------------------------------------------------------
    ##   Initialize Configurations
    # ------------------------------------------------------------------
    configs = SUAVE.Components.Configs.Config.Container()

    base_config = SUAVE.Components.Configs.Config(vehicle)
    base_config.tag = 'base'
    base_config.landing_gear.landing_gear_extracted = False
    base_config.wings.main_wing.control_surfaces.flap.angle = 0. * Units.deg
    base_config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    configs.append(base_config)

    # ------------------------------------------------------------------
    #   Takeoff Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'takeoff'

    config.wings.main_wing.control_surfaces.flap.angle = 15. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.landing_gear.landing_gear_extracted = True
    configs.append(config)
    config.propulsors.network.propeller.propulsive_efficiency = 0.875
    # ------------------------------------------------------------------
    #   2ng_segment Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'second_segment'

    config.wings.main_wing.control_surfaces.flap.angle = 15. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.landing_gear.landing_gear_extracted = True
    configs.append(config)


    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'climb'

    config.propulsors.network.propeller.propulsive_efficiency = 0.875#0.875 #0.82 0.85 0.875 (0.875) 0.86 0.835 0.825
    config.landing_gear.landing_gear_extracted = False

    configs.append(config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'

    config.propulsors.network.propeller.propulsive_efficiency = 0.83#0.82#0.835 #0.885
    config.landing_gear.landing_gear_extracted = False

    configs.append(config)

    # ------------------------------------------------------------------
    #   Descent Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'descent'

    config.propulsors.network.propeller.propulsive_efficiency = 0.40 #0.85 #0.4 180 0.6
    config.landing_gear.landing_gear_extracted = False
    configs.append(config)

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------

    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'landing'

    config.wings.main_wing.control_surfaces.flap.angle = 30. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.propulsors.network.propeller.propulsive_efficiency = 0.81
    config.landing_gear.landing_gear_extracted = True
    configs.append(config)

    # ------------------------------------------------------------------
    #   Ceiling OEI 18.000 ft +10ISA
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'ceiling_oei'

    config.wings.main_wing.control_surfaces.flap.angle = 0. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.landing_gear.landing_gear_extracted = False
    config.propulsors.network.one_engine_propeller_inoperative = True
    configs.append(config)

    # ------------------------------------------------------------------
    #  second_segment_oei
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'second_segment_oei'

    config.wings.main_wing.control_surfaces.flap.angle = 15. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.landing_gear.landing_gear_extracted = True
    config.propulsors.network.one_engine_propeller_inoperative = True
    configs.append(config)

    return configs


def simple_sizing(configs, analyses):
    base = configs.base
    base.pull_base()

    # wing areas
    for wing in base.wings:
        wing.areas.wetted = 2.0 * wing.areas.reference
        wing.areas.exposed = 0.8 * wing.areas.wetted
        wing.areas.affected = 0.6 * wing.areas.wetted

    # fuselage seats
    base.fuselages['fuselage'].number_coach_seats = base.passengers

    # diff the new data
    base.store_diff()

    # ------------------------------------------------------------------
    #   Landing Configuration
    # ------------------------------------------------------------------
    landing = configs.landing

    # make sure base data is current
    landing.pull_base()

    # landing weight
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff #why?

    # diff the new data
    landing.store_diff()

    return


def missions_setup(base_mission):
    # the mission container
    missions = SUAVE.Analyses.Mission.Mission.Container()

    # ------------------------------------------------------------------
    #   Base Mission
    # ------------------------------------------------------------------

    missions.base = base_mission

    return missions

def reserves_setup(iteration_setup):

    iteration_setup.mission_iter.reserve_flag = True
    if iteration_setup.mission_iter.reserve_mission == 1:
        iteration_setup.mission_iter.reserve_hold_time = 45. * Units.min
        iteration_setup.mission_iter.reserve_hold_altitude = 17000. * Units.ft
        iteration_setup.mission_iter.reserve_hold_speed = 289. * Units.knots  # 290
        iteration_setup.mission_iter.reserve_trip_pct = 0.05
        iteration_setup.mission_iter.reserve_distance = 87 * Units.nautical_mile
    elif iteration_setup.mission_iter.reserve_mission == 2:
        iteration_setup.mission_iter.reserve_hold_time = 30. * Units.min
        iteration_setup.mission_iter.reserve_hold_altitude = 1500. * Units.ft
        iteration_setup.mission_iter.reserve_hold_speed = 164. * Units.knots
        iteration_setup.mission_iter.reserve_trip_pct = 0.03
        iteration_setup.mission_iter.reserve_distance = 185 * Units.km
    elif iteration_setup.mission_iter.reserve_mission == 3:
        iteration_setup.mission_iter.reserve_hold_time = 30. * Units.min
        iteration_setup.mission_iter.reserve_hold_altitude = 1500. * Units.ft
        iteration_setup.mission_iter.reserve_hold_speed = 164. * Units.knots
        iteration_setup.mission_iter.reserve_trip_pct = 0.03
        iteration_setup.mission_iter.reserve_distance = 100 * Units.nautical_mile

    return iteration_setup

if __name__ == '__main__':
    global cruise_altitude, cruise_speed, BOW, TOW, second_leg_cruise_altitude, second_leg_cruise_speed
    global reserve_flag, reserve_cruise_distance, reserve_hold_time, reserve_hold_altitude, reserve_hold_speed
    global generate_surrogates, save_surrogates
    global Mission_name


    generate_surrogates = True
    save_surrogates = False

    iteration_setup = Data()
    iteration_setup.mission_iter = Data()
    iteration_setup.weight_iter = Data()
    iteration_setup.energy_iter = Data()
    iteration_setup.sizing_iter = Data()



    iteration_setup.weight_iter.MTOW = 18600. * Units.kg
    iteration_setup.weight_iter.BOW = 11250. * Units.kg

    # TODO: alles ab hier (weight_iter) wird doch nicht iteriert?

    iteration_setup.weight_iter.Design_Payload = 4560 * Units.kg # 4560 5450 2850 0
    iteration_setup.weight_iter.Max_Payload = 5460. * Units.kg
    #iteration_setup.weight_iter.Max_Fuel = 4500. * Units.kg
    iteration_setup.weight_iter.Max_PAX = 48 * 95 * Units.kg

    iteration_setup.sizing_iter.wing_origin = [[9.0, 0, 1.7]] #9.3
    iteration_setup.sizing_iter.factor_engine_power = 1 #1790/1899.400



    iteration_setup.mission_iter.Mission_name = 'Design range mission'
    iteration_setup.mission_iter.mission_distance = 840 * Units.nautical_miles # 840 457 1603 1759 200
    iteration_setup.mission_iter.cruise_altitude = 21000#17000#25000 #25000
    iteration_setup.mission_iter.cruise_distance = 1225765.9#iteration_setup.mission_iter.mission_distance * 2/3
    iteration_setup.mission_iter.cruise_speed    = 283 * Units.knots#267
    iteration_setup.mission_iter.reserve_cruise_distance = 50766.12
    iteration_setup.mission_iter.reserve_mission = 1 #2
    iteration_setup.mission_iter.mission_type = 'given_range'


    reserves_setup(iteration_setup)



    iteration_setup.weight_iter.FUEL = iteration_setup.weight_iter.MTOW - iteration_setup.weight_iter.BOW - iteration_setup.weight_iter.Design_Payload

    landing_weight = 0.0
    block_distance = 0.0
    # TOW = BOW + PAX + FUEL
    iteration_setup.weight_iter.TOW = iteration_setup.weight_iter.BOW + iteration_setup.weight_iter.Design_Payload + iteration_setup.weight_iter.FUEL
    error = 2
    error_reserve = 2
    delta_percent_mac = 2 # new14102021
    delta_throttle = 2

    while (error > 1.0) or (abs(landing_weight - iteration_setup.weight_iter.BOW - iteration_setup.weight_iter.Design_Payload) > 1.0) or (error_reserve > 1.0) or abs(delta_percent_mac) > 1.0 or delta_throttle > 0.001:
    #while (error > 1.0) or (abs(landing_weight - iteration_setup.weight_iter.BOW - iteration_setup.weight_iter.Design_Payload) > 1.0) or (error_reserve > 1.0):
    ## while (abs(block_distance*Units['nautical_mile']-mission_distance) > 1.0) and (abs(landing_weight-BOW-PAX) > 1.0):
        if iteration_setup.mission_iter.mission_type == 'given_range':
            #TOW = BOW + PAX + FUEL
            iteration_setup.weight_iter.TOW = iteration_setup.weight_iter.BOW + iteration_setup.weight_iter.Design_Payload + iteration_setup.weight_iter.FUEL

        mission, results, configs, analyses = main(iteration_setup)

      #  print('OEW: %.1f' % iteration_setup.weight_iter.BOW)
      #  print('OEW_Weights: %.1f' % configs.base.mass_properties.operating_empty)




        climb_segments = [key for key in results.segments.keys() if (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key))]
        n_climb_segments = len(climb_segments)
        first_climb_segment = climb_segments[0]
        last_climb_segment = climb_segments[-1]
        descent_segments = [key for key in results.segments.keys() if (('descent' in key) and ('reserve' not in key) and ('second_leg' not in key))]
        n_descent_segments = len(descent_segments)
        first_descent_segment = descent_segments[0]
        last_descent_segment = descent_segments[-1]

        reserve_climb_segments = [key for key in results.segments.keys() if (('climb' in key) and ('reserve' in key) and ('second_leg' not in key))]
        n_reserve_climb_segments = len(reserve_climb_segments)
        first_reserve_climb_segment = reserve_climb_segments[0]
        last_reserve_climb_segment = reserve_climb_segments[-1]
        reserve_descent_segments = [key for key in results.segments.keys() if (('descent' in key) and ('reserve' in key) and ('second_leg' not in key))]
        n_reserve_descent_segments = len(reserve_descent_segments)
        first_reserve_descent_segment = reserve_descent_segments[0]
        last_reserve_descent_segment = reserve_descent_segments[-1]

        block_fuel = results.segments[first_climb_segment].conditions.weights.total_mass[0][0] - \
                     results.segments[last_descent_segment].conditions.weights.total_mass[-1][0]

        cruise_fuel = results.segments.cruise.conditions.weights.total_mass[0][0] - \
                      results.segments.cruise.conditions.weights.total_mass[-1][0]

        alternate_fuel = results.segments[first_reserve_climb_segment].conditions.weights.total_mass[0][0] - \
                         results.segments[last_reserve_descent_segment].conditions.weights.total_mass[-1][0]

        reserve_cruise_fuel = results.segments.reserve_cruise.conditions.weights.total_mass[0][0] - \
                              results.segments.reserve_cruise.conditions.weights.total_mass[-1][0]

        hold_fuel = results.segments['hold'].conditions.weights.total_mass[0][0] - \
                    results.segments['hold'].conditions.weights.total_mass[-1][0]

        reserve_fuel_pct = block_fuel * iteration_setup.mission_iter.reserve_trip_pct

        reserve_fuel = reserve_fuel_pct + alternate_fuel + hold_fuel

        block_distance = (results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                          results.segments[first_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        climb_distance = (results.segments[last_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                          results.segments[first_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        cruise_distance = (results.segments.cruise.conditions.frames.inertial.position_vector[-1][0] - \
                           results.segments.cruise.conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        descent_distance = (results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                            results.segments[first_descent_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        reserve_climb_distance = (results.segments[last_reserve_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                                  results.segments[first_reserve_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        reserve_cruise_distance = (results.segments.reserve_cruise.conditions.frames.inertial.position_vector[-1][0] - \
                                   results.segments.reserve_cruise.conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        reserve_descent_distance = (results.segments[last_reserve_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                                    results.segments[first_reserve_descent_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        landing_weight = results.segments['hold'].conditions.weights.total_mass[-1][0] - reserve_fuel_pct

       # print('Landing Weight: %.1f' % landing_weight)

        if iteration_setup.mission_iter.mission_type == 'given_range':
            iteration_setup.weight_iter.FUEL = block_fuel + reserve_fuel
            # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
            #     iteration_setup.weight_iter.FUEL += second_leg_block_fuel
          #  print('Fuel Weight: %.1f' % iteration_setup.weight_iter.FUEL)

            # error = abs(block_distance*Units['nautical_mile'] - mission_distance) / Units['nautical_mile']iteration_setup.mission_iter.mission_distance
            error = abs(block_distance * Units['nautical_mile'] - iteration_setup.mission_iter.mission_distance) / Units['nautical_mile']

            # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
            #     error += abs(second_leg_block_distance*Units['nautical_mile'] - second_leg_mission_distance) / Units['nautical_mile']

        elif iteration_setup.mission_iter.mission_type == 'given_TOW':
            delta_W = landing_weight - BOW - iteration_setup.weight_iter.Design_Payload
            cruise_SR = cruise_distance / cruise_fuel
            mission_distance = mission_distance + (cruise_SR * delta_W) * Units['nautical_mile']

            # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
            #     second_leg_mission_distance = second_leg_mission_distance + (cruise_SR * delta_W) * Units['nautical_mile']
            error = abs(delta_W)

        # reserve_cruise_SR = reserve_cruise_distance / reserve_cruise_fuel
        # reserve_distance = reserve_distance + (reserve_cruise_SR * delta_W) * Units['nautical_mile']
        error_reserve = abs(iteration_setup.mission_iter.reserve_distance - (reserve_climb_distance + reserve_cruise_distance + reserve_descent_distance) * Units['nautical_mile']) / Units['nautical_mile']

        # cruise_distance = mission_distance - (climb_distance + descent_distance) * Units['nautical_mile']
        iteration_setup.mission_iter.cruise_distance = iteration_setup.mission_iter.mission_distance - (climb_distance + descent_distance) * Units['nautical_mile']

        iteration_setup.mission_iter.reserve_cruise_distance = iteration_setup.mission_iter.reserve_distance - (reserve_climb_distance + reserve_descent_distance) * Units['nautical_mile']

        iteration_setup.weight_iter.BOW = configs.base.mass_properties.operating_empty# + 1650

        deltaweight = landing_weight - iteration_setup.weight_iter.BOW - iteration_setup.weight_iter.Design_Payload
        print('Error: %.1f' % error)
        print('Error alternate: %.1f' % error_reserve)

       # print('Cruise distance: %.1f' % iteration_setup.mission_iter.reserve_cruise_distance)
        print('delta weight: %.1f' % deltaweight)


        # Center of Gravity Sizing - Stability
        percent_mac = np.array([])
        for segment in results.segments.values():
            percent_mac = np.append(percent_mac,segment.conditions.stability.static.percent_mac[:,0])

        percent_mac_min = np.min(percent_mac)
        percent_mac_max = np.max(percent_mac)
        reference_mac = percent_mac_min # or min!!!

        percent_mac_iter = 15 # 14-27 acc. Strohmayer # acc. ATR flight manual from 15 to 34
        iteration_setup.sizing_iter.wing_origin[0][0] = iteration_setup.sizing_iter.wing_origin[0][0] - (percent_mac_iter-reference_mac)/100 * configs.base.wings.main_wing.chords.mean_aerodynamic
      #  print('WingOrigin: %.3f' % iteration_setup.sizing_iter.wing_origin[0][0])
         #print('x WingMAC t/4: %.3f' % ((configs.base.wings.main_wing.origin[0][0] +
            #                   configs.base.wings.main_wing.aerodynamic_center[0]))) #+ 0.25 * configs.base.wings.main_wing.chords.mean_aerodynamic))

        delta_percent_mac = (percent_mac_iter - reference_mac)*5
        #delta_percent_mac = (15 - percent_mac_max)*5
        print('deltamac: %.1f' % delta_percent_mac)


        # Power Loading - Sizing Power of Engines

        throttle_old = iteration_setup.sizing_iter.factor_engine_power
        throttle_mission = np.array([])
        for segment in results.segments.values():
            throttle_mission = np.append(throttle_mission, segment.conditions.propulsion.throttle[:,0])

        throttle_mission_max = np.max(throttle_mission)


        if throttle_mission_max > 1.:
            iteration_setup.sizing_iter.factor_engine_power = throttle_old*throttle_mission_max**0.5 #*1.1
        # else:
        #     iteration_setup.sizing_iter.factor_engine_power = throttle_old*throttle_mission_max

        #delta_throttle = abs(throttle_old - iteration_setup.sizing_iter.factor_engine_power)*10
        delta_throttle = 0
       #  print('DeltaThrottle: %.3f' % delta_throttle)
        print('MaxThrottle: %.3f' % throttle_mission_max)
        print('ThrottleFactor: %.3f' % iteration_setup.sizing_iter.factor_engine_power)
        results_off_design=results
        results_design = results
    #run payload diagram
         # results_show(results, configs.base, iteration_setup, first_climb_segment, last_climb_segment, last_descent_segment,
        #          first_descent_segment, first_reserve_climb_segment, last_reserve_descent_segment,
        #          first_reserve_descent_segment, last_reserve_climb_segment, climb_segments)


    ####################################################################################################################
    main_missions = Data()
    ####################################################################################################################
    # Design Mission
    main_missions.design_mission = results

    #results_show(main_missions.design_mission,configs.base,iteration_setup,first_climb_segment,last_climb_segment,last_descent_segment,first_descent_segment,first_reserve_climb_segment,last_reserve_descent_segment,first_reserve_descent_segment,last_reserve_climb_segment,climb_segments)

    ####################################################################################################################
    # payload range
    ####################################################################################################################

    payload_range_run = 1

    if payload_range_run == True:
        payload_range_results = payload_range_variable_reserve(configs.base, mission, "cruise", iteration_setup)

        main_missions.payload_range_max_payload_mtom = payload_range_results.results_mission.max_payload_mtom
        main_missions.payload_range_max_fuel_mtom = payload_range_results.results_mission.max_fuel_mtom
        main_missions.payload_range_ferry_range = payload_range_results.results_mission.ferry_range

    ####################################################################################################################
    # Off design missions --> 5300 kg 400 km
    ####################################################################################################################
    # off design missions
    ####################################################################################################################

    simple_off_design_run = 1

    if simple_off_design_run == True:
        # Pre-Processing

        ####Low Altitude Range Mission###########################################################################################

        configs_analyses = analyses.configs
        iteration_setup.mission_iter.temperature_deviation = 00
        # define new mission parameters
        iteration_setup.mission_iter.cruise_altitude = 21000  # 25000  # 25000 #25000

        # load new mission
        analyses.missions = missions_setup(mission_setup(configs_analyses, iteration_setup))
        analyses.finalize()
        mission = analyses.missions.base

        # run off design point
        off_design_results_5300kg_400km_results, results_5300kg_400km = off_design_mission_calculation_variable_reserve(configs.base, mission, "cruise", iteration_setup, 5300, 400 * Units.km)

        # plot_mission(results_off_design_example)
        # plot_flight_conditions(results_off_design_example, 'bo-', False)
        # plot_electronic_conditions(results_off_design_example, 'bo-')
        # plt.show()
        main_missions.off_design_5300kg_400km = results_5300kg_400km
        main_missions.off_design_evaluation_5300kg_400km = results_5300kg_400km
    # off_design_mission_calculation_variable_reserve_5300kg_400km, results_5300kg_400km = off_design_mission_calculation_variable_reserve(configs.base,mission,"cruise",iteration_setup,5300, 400 * Units.km)
    #
    # main_missions.off_design_evaluation_5300kg_400km = results_5300kg_400km

    results_show(main_missions.off_design_evaluation_5300kg_400km,configs.base,iteration_setup,first_climb_segment,last_climb_segment,last_descent_segment,first_descent_segment,first_reserve_climb_segment,last_reserve_descent_segment,first_reserve_descent_segment,last_reserve_climb_segment,climb_segments)


    ####################################################################################################################

    # configs_analyses = analyses.configs
    #
    # iteration_setup.mission_iter.cruise_altitude = 17000#25000  # 25000 #25000
    #
    # analyses.missions = missions_setup(mission_setup(configs_analyses,iteration_setup))
    #
    # analyses.finalize()
    #
    # mission = analyses.missions.base
    #
    # off_design_mission_calculation_variable_reserve, results = off_design_mission_calculation_variable_reserve(configs.base,mission,"cruise",iteration_setup,4560, 840 * Units.nautical_miles)
    # print('Cruise distance: %.1f' % iteration_setup.mission_iter.cruise_distance)

    # Figure of Merit
    #fom = Figure_of_Merit(results, configs)
    #fom()


    #payload_range_variable_reserve(configs.base, mission, "cruise", iteration_setup)

    # #todo please check other missions like HEA
    #results_show(main_missions.off_design_evaluation_5300kg_400km,configs.base,iteration_setup,first_climb_segment,last_climb_segment,last_descent_segment,first_descent_segment,first_reserve_climb_segment,last_reserve_descent_segment,first_reserve_descent_segment,last_reserve_climb_segment,climb_segments)

    # create results folder path and create absolute path
    Path("../_results").mkdir(exist_ok=True)
    absolute_results_folder_path = os.path.abspath("../_results")

    if configs.base.propulsors.network.turboshaft.p3t3_method:
        mission_list = list(main_missions.keys())
        for mission in mission_list:
            main_missions[mission] = gas_turbine_calculate_emission(main_missions[mission], configs.base)


    # write_output_files(results, configs, absolute_results_folder_path, show_figures=False,
    #                    iteration_setup=iteration_setup)

    fom = SUAVE.Methods.Figures_of_Merit.Figure_of_Merit()
    fom.settings.scaling_minus1_to_one = True
    fom.settings.print_flag = True
    fom.settings.print_new_minmax_values = True

    # New noise values
    fom.settings.correction_sideline = -4.37
    fom.settings.correction_flyover = -9.08
    fom.settings.correction_approach = +0.9

    # ATR42 from 2023-05-17 12:49:40 (Deviation=1.00)
    fom.settings.minmax_co2 =        [ 0.00000e+00, 1.63483e-01]
    fom.settings.minmax_nox =        [ 0.00000e+00, 5.94667e-03]
    fom.settings.minmax_noise_sone = [ 0.00000e+00, 4.46999e+01]
    fom.settings.minmax_doc =        [ 0.00000e+00, 4.35911e+01]
    fom.settings.minmax_dev =        [ -7.88271e+08, 7.88271e+08]
    fom.settings.minmax_cer =        [ -4.19109e+07, 4.19109e+07]
    fom.settings.minmax_prod =       [ 0.00000e+00, 4.59522e+08]

    fom.settings.weighting_factors_co2_nox_noise = [1.05, 1.05, 0.9]
    fom.settings.weighting_factors_dev_cer_prod = [1., 1., 1.]
    fom.settings.weighting_factors_EI_ADI_HEAI = [1., 1., 0.]
    fom.settings.weighting_factors_EI_ADI_HEAI_conv = [.4, 2., 1.]  # Weighting Setting for FutPrInt50
    fom.settings.weighting_factors_EI_ADI_HEAI_env = [2., 2., 1.]  # Weighting Setting for FutPrInt50
    fom.settings.weighting_factors_sideline_flyover_approach = [1., 1., 1.]
    fom.settings.DOC_or_COC = 'COC'

    fom.settings.number_of_microfones_sideline = 200
    fom.settings.correction_sideline = -4.37
    fom.settings.correction_flyover = -9.08
    fom.settings.correction_approach = +0.9


    if iteration_setup.mission_iter.mission_distance != 400 * Units.km:
        fom.calculate(main_missions.off_design_evaluation_5300kg_400km, configs)
    else:
        fom.calculate(main_missions.design_mission, configs) #todo please check other missions like HEA

    # all_converged = True
    # for segment in results.segments:
    #     if not segment.converged:
    #         all_converged = False
    #         break
    #
    # if all_converged:
    #     write_output_files(results, configs, absolute_results_folder_path, show_figures=False,
    #                    iteration_setup=iteration_setup, fom=fom)


    #
    # iteration_setup.mission_iter.reserve_mission = 2  # 2
    #
    # reserves_setup(iteration_setup)
    #
    # configs_analyses = analyses.configs
    #
    # analyses.missions = missions_setup(mission_setup_FutPrInt50(configs_analyses,iteration_setup))
    #
    # analyses.finalize()
    #
    # mission = analyses.missions.base
    #
    #



    ####################################################################################################################
    # Sizing Chart
    ####################################################################################################################

    # defining speed oei ceiling

    cD0 = 275e-4
    aspectratio = 11
    oswaldfactor = 0.833
    k = 1/ (np.pi * aspectratio * oswaldfactor)
    cL = (3 * cD0 / k)**0.5

    v_ceiling_OEI = ((18600 * 9.81) / (0.67155/2 * cL * 54.5))**0.5 #156 kts
    #0.67155

    configs_analyses = analyses.configs

    analyses.missions = missions_setup(mission_setup_Sizing_Chart(configs_analyses,iteration_setup))

    analyses.finalize()

    mission = analyses.missions.base

    sizing_chart_mission_evaluate = sizing_chart_mission_evaluate(configs.base,mission,iteration_setup)

    #results = sizing_chart_mission_evaluate.results

    plot_mission(sizing_chart_mission_evaluate.results,configs.base)
    plot_flight_conditions(sizing_chart_mission_evaluate.results, 'bo-', False)
    plot_aerodynamic_forces(sizing_chart_mission_evaluate.results, 'bo-', False)
    plot_stability_coefficients(sizing_chart_mission_evaluate.results, 'bo-', False)

    plt.show()

    ############################################# WRITE OUTPUT FILES ###################################################

    output_results = Data()
    output_results.main_missions = main_missions
    write_output_files(output_results, configs, absolute_results_folder_path, show_figures=False,
                       iteration_setup=iteration_setup, fom=fom, parameters=Data(), settings=fom.settings)

    ####################################################################################################################
    ####################################################################################################################
    ####################################################################################################################