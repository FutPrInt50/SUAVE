## Config_1_parallel_hybrid.py
# 
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Jonas Mangold, mangold@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Sep 2021, J. Mangold
# Modified: Jul 2022, J. Mangold

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# Python Imports
import numpy as np
import pylab as plt
from matplotlib import ticker
from copy import deepcopy
import math
import gc

try:
    import cPickle as pickle
except ImportError:
    import pickle

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Plots.Mission_Plots import *
from Mission_Setup import mission_setup
from Results_Show import results_show
from Vehicle_Setup import vehicle_setup
from SUAVE.Methods.Center_of_Gravity.compute_component_centers_of_gravity_variable_CG import compute_component_centers_of_gravity_variable_CG
from SUAVE.Methods.Propulsion import gas_turbine_calculate_emission
from SUAVE.Methods.Performance.sizing_chart_mission_evaluate import sizing_chart_mission_evaluate
from Mission_Setup_Sizing_Chart import mission_setup_Sizing_Chart
from SUAVE.Plots.Mission_Plots import *
from Plots import plot_mission
from SUAVE.Methods.Performance import payload_range_variable_reserve
from SUAVE.Methods.Performance import off_design_mission_calculation_variable_reserve
import SUAVE.Methods.Power.Battery.Degradation.battery_sizing_tool as battery_sizing_tool

from pathlib import Path
import os
from SUAVE.Input_Output.Results.write_output import write_output_files
import time


# ----------------------------------------------------------------------
#   Main
# ----------------------------------------------------------------------


def main(iteration_setup, parameters):
    # ---------------------------------------------------------------------------------------
    # INITIALIZING AIRCRAFT

    configs, analyses, vehicle = full_setup(iteration_setup, parameters)

    # print('full setup OK')

    # simple_sizing(configs, analyses) is not necessary anymore

    # Has to be after full_setup !!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!
    # Determine the vehicle weight breakdown (independent of mission fuel usage)
    weights = analyses.configs.base.weights
    breakdown = weights.evaluate("Raymer")

    deltacg = 2
    while abs(deltacg) > 1e-5:
        compute_component_centers_of_gravity_variable_CG(configs.base)
        oldcg = configs.base.mass_properties.center_of_gravity[0][0]
        configs.base.center_of_gravity()  # CG @ TOM
        configs.base.store_diff()

        newcg = configs.base.mass_properties.center_of_gravity[0][0]
        deltacg = newcg - oldcg

    configs.finalize()
    analyses.finalize()


    # # Determine the vehicle weight breakdown (independent of mission fuel usage)
    # weights = analyses.configs.base.weights
    # breakdown = weights.evaluate("Raymer")

    # configs.finalize()
    # analyses.finalize()

    # ---------------------------------------------------------------------------------------
    # MISSION ANALYSIS

    mission = analyses.missions.base
    results = mission.evaluate()
    # print('MISSION OK')

    return mission, results, configs, analyses


# ----------------------------------------------------------------------
#   Analysis Setup
# ----------------------------------------------------------------------

def full_setup(iteration_setup, parameters):
    # vehicle data
    vehicle = vehicle_setup(iteration_setup, parameters)
    configs = configs_setup(vehicle)

    # vehicle analyses
    configs_analyses = analyses_setup(configs)
    configs_analyses.vehicle = Data()
    configs_analyses.vehicle = vehicle

    # mission analyses
    mission = mission_setup(configs_analyses, iteration_setup, parameters)
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
    mk = 0.15
    weights = SUAVE.Analyses.Weights.Weights_Transport()
    weights.vehicle = vehicle
    weights.settings.weight_reduction_factors.main_wing = - 0.312 + mk
    weights.settings.weight_reduction_factors.empennage = -0.54 + mk
    weights.settings.weight_reduction_factors.fuselage = 0.0389 + mk - (vehicle.fuselages.fuselage.mass_factor - 1)
    weights.settings.weight_reduction_factors.structural = 0
    weights.settings.weight_reduction_factors.systems = 0.05405
    weights.settings.weight_reduction_factors.operating_items = -0.4351 + mk
    weights.settings.weight_reduction_factors.landing_gear = -0.0615
    weights.settings.weight_reduction_factors.propulsion = 0.135
    weights.settings.payload = Data()
    weights.settings.payload.pax = 106 * Units.kg  # FUTPRINT50: 106 kg = pax + carry-on
    weights.settings.payload.baggage = 0 * Units.kg  # FUTPRINT50: 0 kg

    analyses.append(weights)

    # ------------------------------------------------------------------
    #  Aerodynamics Analysis
    aerodynamics = SUAVE.Analyses.Aerodynamics.Fidelity_Zero()
    aerodynamics.geometry = vehicle
    aerodynamics.settings.drag_coefficient_increment = Data()
    aerodynamics.settings.drag_coefficient_increment.base = 0  # -0.001
    aerodynamics.settings.drag_coefficient_increment.takeoff = 0.01  # 0.00206 # to account for average flaps and L/G drag
    aerodynamics.settings.drag_coefficient_increment.climb = 0.0001  # 0.005# 0.018# 0.03
    aerodynamics.settings.drag_coefficient_increment.cruise = 0.0001  # 0.002#  0.0036#0.003#0.006
    aerodynamics.settings.drag_coefficient_increment.descent = 0.0001  # 0.002
    aerodynamics.settings.drag_coefficient_increment.landing = 0.0190
    aerodynamics.settings.drag_coefficient_increment.second_segment = 0
    aerodynamics.settings.drag_coefficient_increment.second_segment_oei = 0
    aerodynamics.settings.drag_coefficient_increment.ceiling_oei = 0
    aerodynamics.settings.drag_coefficient_increment.second_segment_oei_wtp = 0

    aerodynamics.settings.drag_coefficient_due_to_prop = 0.020
    aerodynamics.settings.oswald_efficiency_factor = 0.715

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
    #   Initialize Configurations
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
    # config.propulsors.turboprop.gas_turbine.power_scaling_factor = 1.0
    # config.propulsors.turboprop.gas_turbine.gas_turbine_rating = 'NTO'
    # config.propulsors.turboprop.propeller_rpm = 1000. * Units.rpm
    # config.propulsors.turboprop.propeller.propulsive_efficiency = 0.70
    config.wings.main_wing.control_surfaces.flap.angle = 15. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.propulsors.network.propeller_wing_interaction = False
    config.landing_gear.landing_gear_extracted = True
    configs.append(config)

    # ------------------------------------------------------------------
    #   Climb Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'climb'
    config.wings.main_wing.control_surfaces.flap.angle = 0. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.propulsors.network.propeller.propulsive_efficiency = 0.875
    config.propulsors.network.propellerWTP.propulsive_efficiency = 0.875
    config.propulsors.network.propeller_wing_interaction = True
    config.landing_gear.landing_gear_extracted = False
    configs.append(config)

    # ------------------------------------------------------------------
    #   Cruise Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'cruise'
    config.wings.main_wing.control_surfaces.flap.angle = 0. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.propulsors.network.propeller.propulsive_efficiency = 0.83
    config.propulsors.network.propellerWTP.propulsive_efficiency = 0.83
    config.propulsors.network.propeller_wing_interaction = True
    config.landing_gear.landing_gear_extracted = False
    configs.append(config)

    # ------------------------------------------------------------------
    #   Descent Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'descent'
    config.wings.main_wing.control_surfaces.flap.angle = 0. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.propulsors.network.propeller.propulsive_efficiency = 0.85  # 0.4 180
    config.propulsors.network.propellerWTP.propulsive_efficiency = 0.85  # 0.4 180
    config.propulsors.network.propeller_wing_interaction = False
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
    config.propulsors.network.propellerWTP.propulsive_efficiency = 0.81
    config.propulsors.network.propeller_wing_interaction = False
    config.landing_gear.landing_gear_extracted = True
    configs.append(config)

    # ------------------------------------------------------------------
    #   2ng_segment Configuration
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'second_segment'

    config.wings.main_wing.control_surfaces.flap.angle = 15. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.landing_gear.landing_gear_extracted = True
    config.propulsors.network.propeller_wing_interaction = False
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
    config.propulsors.network.propeller_wing_interaction = False
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
    config.propulsors.network.propeller_wing_interaction = False
    configs.append(config)

    # ------------------------------------------------------------------
    #  second_segment_oei for WTP
    # ------------------------------------------------------------------
    config = SUAVE.Components.Configs.Config(base_config)
    config.tag = 'second_segment_oei_wtp'

    config.wings.main_wing.control_surfaces.flap.angle = 15. * Units.deg
    config.wings.main_wing.control_surfaces.slat.angle = 0. * Units.deg
    config.landing_gear.landing_gear_extracted = True
    config.propulsors.network.one_engine_propeller_inoperative = False
    config.propulsors.network.one_engine_propeller_inoperative_WTP = True
    config.propulsors.network.propeller_wing_interaction = False
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
    landing.mass_properties.landing = 0.85 * base.mass_properties.takeoff  # why?

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


def Config_1_Parallel_Hybrid(parameters, settings):
    # if __name__ == '__main__':

    iteration_setup = Data()
    iteration_setup.mission_iter = Data()
    iteration_setup.weight_iter = Data()
    iteration_setup.energy_iter = Data()
    iteration_setup.sizing_iter = Data()

    iteration_setup.weight_iter.MTOW = 18600 * Units.kg  # 18600 15810 2500
    iteration_setup.weight_iter.BOW = iteration_setup.weight_iter.MTOW - 5300 - 1500

    # TODO: alles ab hier (weight_iter) wird doch nicht iteriert?

    iteration_setup.weight_iter.Design_Payload = 5300 * Units.kg  # 4560 5450 2850 0
    iteration_setup.weight_iter.Max_Payload = 5800. * Units.kg
    iteration_setup.weight_iter.Max_Fuel = 4500. * Units.kg
    iteration_setup.weight_iter.Max_PAX = 50 * 106 * Units.kg

    if parameters.ems == 1:
        iteration_setup.energy_iter.battery_energy_used = 5000 * Units.kWh / (1 - parameters.hybridization_power ** 0.5)
    elif parameters.ems == 2:
        iteration_setup.energy_iter.battery_energy_used = 700 * Units.kWh
    elif parameters.ems == 3:
        iteration_setup.energy_iter.battery_energy_used = 150 * Units.kWh #250
    else:
        iteration_setup.energy_iter.battery_energy_used = 200 * Units.kWh #200

    iteration_setup.sizing_iter.wing_origin = [[9, 0, 1.7]]  # [[8.7, 0, 1.7]]

    iteration_setup.sizing_iter.factor_engine_power = 1.  # TODO: hier auch?
    iteration_setup.sizing_iter.factor_power_loading_sizing_chart = 1.
    iteration_setup.sizing_iter.factor_power_loading_sizing_chart_nan = 1.

    iteration_setup.sizing_iter.max_throttle_engine_sizing_mission = 0.90
    iteration_setup.sizing_iter.max_throttle_engine_sizing_chart = 0.97

    iteration_setup.sizing_iter.heat_load_max_vcs = 20 * Units.kW
    iteration_setup.sizing_iter.heat_load_max_liquid = 20 * Units.kW

    iteration_setup.energy_iter.battery_power_mission_max = 400 * Units.kW
    iteration_setup.energy_iter.battery_mass_factor_cea = 1.4

    iteration_setup.sizing_iter.battery_resistance = 0.05# not used (400/1200 * parameters.bus_voltage - 250)/1000 #500 * Units.mohm #default 50?

    iteration_setup.sizing_iter.battery_soh_interval_setting = np.array([100, 95, 90, 85, 80, 75, 70, 65, 60, 55, 50, 45, 40, 35, 30, 25, 20, 15, 10, 5])
    iteration_setup.sizing_iter.battery_stop_after_durability_reached = True

    iteration_setup.sizing_iter.battery_durability_in_months = 0 #months

    iteration_setup.sizing_iter.battery_cell_mass = None
    iteration_setup.sizing_iter.battery_cell_max_voltage = None
    iteration_setup.sizing_iter.battery_cell_min_voltage = None
    iteration_setup.sizing_iter.battery_cell_nominal_voltage = None
    iteration_setup.sizing_iter.battery_cell_nominal_energy = None
    iteration_setup.sizing_iter.battery_cell_initial_temperature = None

    iteration_setup.sizing_iter.battery_pack_initial_soc = None
    iteration_setup.sizing_iter.battery_pack_initial_soh = None
    iteration_setup.sizing_iter.battery_pack_nominal_voltage = None
    iteration_setup.sizing_iter.battery_pack_min_soc = None
    iteration_setup.sizing_iter.battery_pack_max_soc = None

    iteration_setup.sizing_iter.battery_eol_soh = None
    iteration_setup.sizing_iter.battery_number_branches = None
    iteration_setup.sizing_iter.battery_number_cells_series = None

    # if parameters.ems == 3:
    #     iteration_setup.sizing_iter.power_loading = 185.6 * (1 + parameters.hybridization_power)**2.0 * (1 + parameters.generator_oversizing)**3
    # elif parameters.ems == 2:
    #     iteration_setup.sizing_iter.power_loading = 185.6 * (1 + parameters.hybridization_power)**2.0 * (1 + parameters.generator_oversizing)**2 * (parameters.cruise_speed / 525)**0.5 * (21000 / parameters.cruise_altitude)**0.5
    # elif parameters.cruise_speed > 500:
    #     iteration_setup.sizing_iter.power_loading = 185.6 * (1 + parameters.hybridization_power) ** 1.8 * parameters.cruise_speed / 450 * 1.3
    # else:
    #     iteration_setup.sizing_iter.power_loading = 185.6 * (1 + parameters.hybridization_power) ** 2

    #iteration_setup.sizing_iter.power_loading = 185.6 * (1 + parameters.hybridization_power + 0.1) ** 2 # HP 0.1

    if parameters.ems == 2:
        iteration_setup.sizing_iter.power_loading = 185.6 * (1 + parameters.hybridization_power) ** 1.5
    else:
        iteration_setup.sizing_iter.power_loading = 185.6 * (1 + parameters.hybridization_power) ** 1.8

    iteration_setup.mission_iter.temperature_deviation = 0
    iteration_setup.mission_iter.airport_altitude = 0

    iteration_setup.mission_iter.Mission_name = 'Design range mission'
    iteration_setup.mission_iter.mission_distance = parameters.mission_distance
    iteration_setup.mission_iter.cruise_altitude = parameters.cruise_altitude  # 25000 2100
    iteration_setup.mission_iter.cruise_distance = iteration_setup.mission_iter.mission_distance * 2 / 3
    iteration_setup.mission_iter.cruise_speed = parameters.cruise_speed * Units.km / Units.h  # 300 * Units.knots
    iteration_setup.mission_iter.reserve_cruise_distance = 54. * Units.nautical_mile  # 54
    iteration_setup.mission_iter.reserve_mission = 2  # 1
    iteration_setup.mission_iter.mission_type = 'given_range'
    iteration_setup.mission_iter.approach_speed = 110 * Units.knots
    iteration_setup.mission_iter.approach_speed_reserve = 100 * Units.knots

    iteration_setup.mission_iter.reserve_flag = True
    if iteration_setup.mission_iter.reserve_mission == 1:  # extended cruise
        iteration_setup.mission_iter.reserve_hold_time = 45. * Units.min
        iteration_setup.mission_iter.reserve_hold_altitude = 17000 * Units.ft
        iteration_setup.mission_iter.reserve_hold_speed = 289. * Units.knots
        iteration_setup.mission_iter.reserve_trip_pct = 0.0
        iteration_setup.mission_iter.reserve_distance = 87 * Units.nautical_mile
    elif iteration_setup.mission_iter.reserve_mission == 2: # FutPrint50 Reserves
        iteration_setup.mission_iter.reserve_hold_time = 30. * Units.min
        iteration_setup.mission_iter.reserve_hold_altitude = 1500. * Units.ft
        iteration_setup.mission_iter.reserve_hold_speed = 220. * Units.knots #164 #200
        iteration_setup.mission_iter.reserve_trip_pct = 0.03 #see DDT
        iteration_setup.mission_iter.reserve_distance = 100 * Units.nautical_mile
    elif iteration_setup.mission_iter.reserve_mission == 3:
        iteration_setup.mission_iter.reserve_hold_time = 30. * Units.min
        iteration_setup.mission_iter.reserve_hold_altitude = 1500. * Units.ft
        iteration_setup.mission_iter.reserve_hold_speed = 164. * Units.knots
        iteration_setup.mission_iter.reserve_trip_pct = 0.03
        iteration_setup.mission_iter.reserve_distance = 100 * Units.nautical_mile

    iteration_setup.weight_iter.FUEL = iteration_setup.weight_iter.MTOW - iteration_setup.weight_iter.BOW - iteration_setup.weight_iter.Design_Payload

    landing_weight = 0.0
    block_distance = 0.0
    # TOW = BOW + PAX + FUEL
    iteration_setup.weight_iter.TOW = iteration_setup.weight_iter.BOW + iteration_setup.weight_iter.Design_Payload + iteration_setup.weight_iter.FUEL
    error = 2
    error_reserve = 2
    # deltaenergy = 2 wird hier wahr gar nicht benötigt
    error_energy = 2
    delta_percent_mac = 2
    delta_throttle = 2
    delta_battery_mass = 10

    i_battery_runs = 0
    i_sizing_chart_runs = 0

    mission = None
    results = None
    configs = None
    analyses = None

    while (error > 1.0) or (abs(landing_weight - iteration_setup.weight_iter.BOW - iteration_setup.weight_iter.Design_Payload) > 1.0) or (error_reserve > 1.0) or abs(error_energy/iteration_setup.energy_iter.battery_energy_used) > 0.01 or abs(delta_percent_mac) > 1.0 or delta_throttle > 0.001 or delta_battery_mass > 1 * Units.kg:
    ## while (abs(block_distance*Units['nautical_mile']-mission_distance) > 1.0) and (abs(landing_weight-BOW-PAX) > 1.0):

        del mission, results, configs, analyses
        gc.collect()

        if iteration_setup.mission_iter.mission_type == 'given_range':
            # TOW = BOW + PAX + FUEL
            iteration_setup.weight_iter.TOW = iteration_setup.weight_iter.BOW + iteration_setup.weight_iter.Design_Payload + iteration_setup.weight_iter.FUEL

        # ('MTOMvor: %.1f' % iteration_setup.weight_iter.TOW)
        mission, results, configs, analyses = main(iteration_setup, parameters)

        # print('OEW: %.1f' % iteration_setup.weight_iter.BOW)
        # print('OEW_Weights: %.1f' % configs.base.mass_properties.operating_empty)
        # print('MTOM: %.1f' % configs.base.mass_properties.max_takeoff)

        climb_segments = [key for key in results.segments.keys() if (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key))]
        n_climb_segments = len(climb_segments)
        first_climb_segment = climb_segments[0]
        last_climb_segment = climb_segments[-1]
        descent_segments = [key for key in results.segments.keys() if (('descent' in key) and ('reserve' not in key) and ('second_leg' not in key))]
        n_descent_segments = len(descent_segments)
        first_descent_segment = descent_segments[0]
        last_descent_segment = descent_segments[-1]

        # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        #     second_leg_climb_segments = [key for key in results.segments.keys() if (('climb' in key) and ('reserve' not in key) and ('second_leg' in key))]
        #     n_second_leg_climb_segments = len(second_leg_climb_segments)
        #     first_second_leg_climb_segment = second_leg_climb_segments[0]
        #     last_second_leg_climb_segment = second_leg_climb_segments[-1]
        #     second_leg_descent_segments = [key for key in results.segments.keys() if (('descent' in key) and ('reserve' not in key) and ('second_leg' in key))]
        #     n_second_leg_descent_segments = len(second_leg_descent_segments)
        #     first_second_leg_descent_segment = second_leg_descent_segments[0]
        #     last_second_leg_descent_segment = second_leg_descent_segments[-1]

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

        if block_fuel < 0:
            block_fuel = 0

        cruise_fuel = results.segments.cruise.conditions.weights.total_mass[0][0] - \
                      results.segments.cruise.conditions.weights.total_mass[-1][0]

        # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        #     second_leg_block_fuel = results.segments[first_second_leg_climb_segment].conditions.weights.total_mass[0] - \
        #                  results.segments[last_second_leg_descent_segment].conditions.weights.total_mass[-1]
        #
        #     second_leg_cruise_fuel = results.segments.second_leg_cruise.conditions.weights.total_mass[0] - \
        #                   results.segments.second_leg_cruise.conditions.weights.total_mass[-1]

        alternate_fuel = results.segments[first_reserve_climb_segment].conditions.weights.total_mass[0][0] - \
                         results.segments[last_reserve_descent_segment].conditions.weights.total_mass[-1][0]

        reserve_cruise_fuel = results.segments.reserve_cruise.conditions.weights.total_mass[0][0] - \
                              results.segments.reserve_cruise.conditions.weights.total_mass[-1][0]

        hold_fuel = results.segments['hold'].conditions.weights.total_mass[0][0] - \
                    results.segments['hold'].conditions.weights.total_mass[-1][0]

        reserve_fuel = block_fuel * iteration_setup.mission_iter.reserve_trip_pct + alternate_fuel + hold_fuel

        # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        #     reserve_fuel += second_leg_block_fuel * iteration_setup.mission_iter.reserve_trip_pct

        block_distance = (results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                          results.segments[first_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        climb_distance = (results.segments[last_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                          results.segments[first_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        cruise_distance = (results.segments.cruise.conditions.frames.inertial.position_vector[-1][0] - \
                           results.segments.cruise.conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        descent_distance = (results.segments[last_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                            results.segments[first_descent_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        #     second_leg_block_distance = (results.segments[last_second_leg_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
        #                       results.segments[first_second_leg_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']
        #
        #     second_leg_climb_distance = (results.segments[last_second_leg_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
        #                       results.segments[first_second_leg_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']
        #
        #     second_leg_cruise_distance = (results.segments.second_leg_cruise.conditions.frames.inertial.position_vector[-1][0] - \
        #                        results.segments.second_leg_cruise.conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']
        #
        #     second_leg_descent_distance = (results.segments[last_second_leg_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
        #                         results.segments[first_second_leg_descent_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']


        reserve_climb_distance = (results.segments[last_reserve_climb_segment].conditions.frames.inertial.position_vector[-1][0] - \
                                  results.segments[first_reserve_climb_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        reserve_cruise_distance = (results.segments.reserve_cruise.conditions.frames.inertial.position_vector[-1][0] - \
                                   results.segments.reserve_cruise.conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        reserve_descent_distance = (results.segments[last_reserve_descent_segment].conditions.frames.inertial.position_vector[-1][0] - \
                                    results.segments[first_reserve_descent_segment].conditions.frames.inertial.position_vector[0][0]) / Units['nautical_mile']

        landing_weight = results.segments['hold'].conditions.weights.total_mass[-1][0] - block_fuel * iteration_setup.mission_iter.reserve_trip_pct

        # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        #     landing_weight -= second_leg_block_fuel * iteration_setup.mission_iter.reserve_trip_pct

        # print('Landing Weight: %.1f' % landing_weight)

        if iteration_setup.mission_iter.mission_type == 'given_range':
            iteration_setup.weight_iter.FUEL = block_fuel + reserve_fuel
            # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
            #     iteration_setup.weight_iter.FUEL += second_leg_block_fuel
            # print('Fuel Weight: %.1f' % iteration_setup.weight_iter.FUEL)

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

        # if iteration_setup.mission_iter.Mission_name == 'Refuelling island mission':
        #     second_leg_cruise_distance = second_leg_mission_distance - (second_leg_climb_distance + second_leg_descent_distance) * Units['nautical_mile']


        iteration_setup.mission_iter.reserve_cruise_distance = iteration_setup.mission_iter.reserve_distance - (reserve_climb_distance + reserve_descent_distance) * Units['nautical_mile']

        iteration_setup.weight_iter.BOW = configs.base.mass_properties.operating_empty  # + 1650

        deltaweight = abs(landing_weight - iteration_setup.weight_iter.BOW - iteration_setup.weight_iter.Design_Payload)
        # print('Error: %.1f' % error)
        # print('Error alternate: %.1f' % error_reserve)
        # print('Cruise distance: %.1f' % iteration_setup.mission_iter.cruise_distance)
        # print('delta weight: %.1f' % deltaweight)

        # Center of Gravity Sizing - Stability
        percent_mac = np.array([])
        for segment in results.segments.values():
            percent_mac = np.append(percent_mac, segment.conditions.stability.static.percent_mac[:, 0])

        percent_mac_min = np.min(percent_mac)
        percent_mac_max = np.max(percent_mac)

        percent_mac_iter = 19 # 14-27 acc. Strohmayer
        iteration_setup.sizing_iter.wing_origin[0][0] = iteration_setup.sizing_iter.wing_origin[0][0] - (percent_mac_iter-percent_mac_min)/100 * configs.base.wings.main_wing.chords.mean_aerodynamic
        #print('WingOrigin: %.3f' % iteration_setup.sizing_iter.wing_origin[0][0])
        #print('x WingMAC t/4: %.3f' % ((configs.base.wings.main_wing.origin[0][0] +
        #                                configs.base.wings.main_wing.aerodynamic_center[0]))) #+ 0.25 * configs.base.wings.main_wing.chords.mean_aerodynamic))
        delta_percent_mac = (percent_mac_iter - percent_mac_min) * 5
        # delta_percent_mac = (15 - percent_mac_max)*5
        # print('deltamac: %.1f' % delta_percent_mac)

        # Power Loading - Sizing Power of Engines
        throttle_old = iteration_setup.sizing_iter.factor_engine_power
        throttle_mission = np.array([])
        for segment in results.segments.values():
            throttle_mission = np.append(throttle_mission, segment.conditions.propulsion.throttle[:, 0])

        throttle_mission_max = np.max(throttle_mission)

        if throttle_mission_max > iteration_setup.sizing_iter.max_throttle_engine_sizing_mission:
            iteration_setup.sizing_iter.factor_engine_power = throttle_old*(throttle_mission_max + (1 - iteration_setup.sizing_iter.max_throttle_engine_sizing_mission))#**0.5

        numbers_not_converged_throttle = 0
        for segment in results.segments:
            if not segment.converged:
                numbers_not_converged_throttle += 1
                print('Info: Mission not converged')
        if numbers_not_converged_throttle > 0:
            iteration_setup.sizing_iter.factor_power_loading_sizing_chart = iteration_setup.sizing_iter.factor_power_loading_sizing_chart * 1.01

        delta_throttle = abs(iteration_setup.sizing_iter.factor_engine_power - throttle_old)*1
        #delta_throttle = 0 # power loading sizing OUT

        # print('DeltaThrottle: %.3f' % delta_throttle)
        # print('MaxThrottle: %.3f' % throttle_mission_max)
        # print('ThrottleFactor: %.3f' % iteration_setup.sizing_iter.factor_engine_power)

        # Battery Sizing
        # change to min and max
        energy0 = results.segments[first_climb_segment].conditions.propulsion.battery_energy[0][0]
        # energy1 = results.segments[last_reserve_descent_segment].conditions.propulsion.battery_energy[-1]
        energy1 = results.segments['hold'].conditions.propulsion.battery_energy[-1][0]

        # print('Energy0: %.1f' % (energy0 / Units.kWh))
        # print('Energy1: %.1f' % (energy1 / Units.kWh))

        if energy1 < 0:
            usedenergy = (energy0 - energy1*1.0)
        elif results.segments['hold'].conditions.propulsion.state_of_charge[-1][0] > (parameters.battery_soc_min + 0.05):
            usedenergy = (energy0 - energy1*1.1) #1.2
        else:
            usedenergy = (energy0 - energy1 / 1.0)

        # reserve_energy = energy0 * 0.2#usedenergy * 0.2
        old_battery_energy_used = iteration_setup.energy_iter.battery_energy_used
        reserve_energy = parameters.battery_soc_min * iteration_setup.energy_iter.battery_energy_used
        error_energy = energy1 - reserve_energy
        #error_energy = 0
        #deltaenergy = usedenergy + reserve_energy +energy0 * 0.25#- error_energy
        deltaenergy = usedenergy + reserve_energy + (1 - parameters.battery_soc_max) * iteration_setup.energy_iter.battery_energy_used

        # print('delta energy: %.1f' % (deltaenergy / Units.kWh))
        # print('error_energy: %.1f' % (error_energy / Units.kWh))

        if np.isnan(iteration_setup.energy_iter.battery_energy_used):
            iteration_setup.energy_iter.battery_energy_used = old_battery_energy_used
            #iteration_setup.sizing_iter.factor_power_loading_sizing_chart = iteration_setup.sizing_iter.factor_power_loading_sizing_chart * 1.1
        elif deltaweight > 500 and results.segments['hold'].conditions.propulsion.state_of_charge[-1][0] > parameters.battery_soc_min:
            iteration_setup.energy_iter.battery_energy_used = old_battery_energy_used
        else:
            iteration_setup.energy_iter.battery_energy_used = deltaenergy

        state_of_charge = results.segments['hold'].conditions.propulsion.state_of_charge[-1][0]
        print('SoC: %.4f == %.2f' % (state_of_charge, parameters.battery_soc_min))

        # Specific Power
        battery_power_mission = []
        for segment in results.segments.values():
            battery_power_mission.append(-segment.conditions.propulsion.battery_draw[:, 0])
        iteration_setup.energy_iter.battery_power_mission_max = np.max(battery_power_mission)
        specific_power_mission_max = iteration_setup.energy_iter.battery_power_mission_max / configs.base.systems.battery.mass_properties.mass
        # print('Max_specific_power_mission: %.3f' % specific_power_mission_max)
        # print('Battery power_mission: %.3f' % iteration_setup.energy_iter.battery_power_mission_max)

        throttle_mission = []
        for segment in results.segments.values():
            throttle_mission.append(segment.conditions.propulsion.throttle[:, 0])
        throttle_mission_max = np.max(throttle_mission)
        # print('MaxThrottle: %.3f' % throttle_mission_max)

        # TMS Sizing
        heat_load_mission_vcs = []
        heat_load_mission_liquid = []
        for segment in results.segments.values():
            heat_load_mission_vcs.append(segment.conditions.propulsion.heat_load_vcs[:, 0])
            heat_load_mission_liquid.append(segment.conditions.propulsion.heat_load_liquid[:, 0])
        heat_load_max_vcs = np.max(heat_load_mission_vcs)
        heat_load_max_liquid = np.max(heat_load_mission_liquid)
        # print('Heat Load Max: %.3f' % heat_load_max)
        iteration_setup.sizing_iter.heat_load_max_vcs = heat_load_max_vcs
        iteration_setup.sizing_iter.heat_load_max_liquid = heat_load_max_liquid

        # print('Heat Load Max VCS: %.3f' % heat_load_max_vcs)
        # print('Heat Load Max Liquid: %.3f' % heat_load_max_liquid)


        # Approch Landing Speed - CS 25.125

        m_landing = results.segments[last_descent_segment].conditions.weights.total_mass[0][0]
        rho_landing = results.segments[last_descent_segment].conditions.freestream.density[0][0]
        cL_max_landing = 2.7
        ref_area= configs.base.wings.main_wing.areas.reference

        v_stall_landing = (m_landing * 9.81 / ( rho_landing / 2 * ref_area * cL_max_landing))**0.5
        v_approach = 1.3 * v_stall_landing
        iteration_setup.mission_iter.approach_speed = v_approach

        # Approch Landing Speed - CS 25.125

        m_landing_reserve = results.segments[last_reserve_descent_segment].conditions.weights.total_mass[0][0]
        rho_reserve = results.segments[last_reserve_descent_segment].conditions.freestream.density[0][0]
        cL_max_landing = 2.7
        ref_area= configs.base.wings.main_wing.areas.reference

        v_stall_reserve = (m_landing_reserve * 9.81 / ( rho_reserve / 2 * ref_area * cL_max_landing))**0.5
        v_approach_reserve = 1.3 * v_stall_reserve
        iteration_setup.mission_iter.approach_speed_reserve = v_approach_reserve



        battery_degradation_run = 1

        #if abs(error_energy) < 10 * Units.kWh and battery_degradation_run == True and i_battery_runs < 3:
        #if (abs(error_energy/iteration_setup.energy_iter.battery_energy_used) < 0.005) and battery_degradation_run == True and i_battery_runs < 3:
        #if (abs(error_energy/iteration_setup.energy_iter.battery_energy_used) < 0.01) and battery_degradation_run == True and i_battery_runs < 5 and i_sizing_chart_runs > 2:
        if (abs(error_energy/iteration_setup.energy_iter.battery_energy_used) < 0.03) and deltaweight < 50 and battery_degradation_run == True and i_battery_runs < 5:
            i_battery_runs += 1

            # file_name = '500WhperKg_surfaceResponse.mat'
            file_name = f'{parameters.battery_specific_energy}WhperKg_surfaceResponse.mat'
            input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/Battery/', file_name)

            test_cell_parameters = {"cell_model_path": input_file,  # Path of the cell model
                                    "cell_mass": parameters.battery_cell_mass,#0.45,  # kg
                                    "cell_max_voltage": 4.2,  # V
                                    "cell_min_voltage": 2.5,  # V
                                    "cell_nominal_voltage": 3.7,  # V
                                    #"cell_nominal_energy": 22.5 * 3600,  # J
                                    "cell_nominal_energy": parameters.battery_specific_energy * parameters.battery_cell_mass * 3600,  # J
                                    "cell_initial_temperature": 273.15 + 25  # K
                                    }

            test_pack_parameters = {
                "initial_soc": parameters.battery_soc_max * 100,  # %
                "initial_soh": 100,  # %
                "pack_nominal_voltage": configs.base.propulsors.network.battery.max_voltage,  # 800,  # V
                "min_soc": parameters.battery_soc_min * 100,  # %
                "max_soc": 100  # %
            }

            mean_battery_power_mission = []
            battery_time_mission = []

            # Climb
            climb_segments = [key for key in results.segments.keys() if
                              (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key) and (
                                      'climbout' not in key))]

            mean_battery_power_mission_climb = []
            battery_time_mission_climb = []

            ############################################################################################################
            # Start - all climb segments into one

            # for segment in climb_segments:
            #     mean_battery_power_mission_climb.append(np.mean(-results.segments[segment].conditions.propulsion.battery_draw))
            #     climb_time = results.segments[segment].conditions.frames.inertial.time[-1] - results.segments[segment].conditions.frames.inertial.time[0]
            #     battery_time_mission_climb.append(climb_time[0])
            #
            # power_time_climb_sum = sum(np.array(mean_battery_power_mission_climb) * np.array(battery_time_mission_climb)) / sum(np.array(battery_time_mission_climb))
            #
            # mean_battery_power_mission.append(power_time_climb_sum)
            # battery_time_mission.append(sum(np.array(battery_time_mission_climb)))

            # End - all climb segments into one
            ############################################################################################################
            # Start - each climb segment in battery run

            for segment in climb_segments:
                mean_battery_power_mission_climb.append(np.mean(-results.segments[segment].conditions.propulsion.battery_draw))
                climb_time = results.segments[segment].conditions.frames.inertial.time[-1] - results.segments[segment].conditions.frames.inertial.time[0]
                battery_time_mission_climb.append(climb_time[0])

                power_time_climb_sum = sum(np.array(mean_battery_power_mission_climb) * np.array(battery_time_mission_climb)) / sum(np.array(battery_time_mission_climb))

                mean_battery_power_mission.append(power_time_climb_sum)
                battery_time_mission.append(sum(np.array(battery_time_mission_climb)))

            # End - each climb segment in battery run
            ############################################################################################################

            # Cruise

            mean_battery_power_mission.append(np.mean(-results.segments.cruise.conditions.propulsion.battery_draw))
            cruise_time = results.segments.cruise.conditions.frames.inertial.time[-1] - results.segments.cruise.conditions.frames.inertial.time[0]
            battery_time_mission.append(cruise_time[0])

            # Descent

            descent_segments = [key for key in results.segments.keys() if
                                (('descent' in key) and ('reserve' not in key) and ('second_leg' not in key))]

            mean_battery_power_mission_descent = []
            battery_time_mission_descent = []

            for segment in descent_segments:
                mean_battery_power_mission_descent.append(np.mean(-results.segments[segment].conditions.propulsion.battery_draw))
                descent_time = results.segments[segment].conditions.frames.inertial.time[-1] - results.segments[segment].conditions.frames.inertial.time[0]
                battery_time_mission_descent.append(descent_time[0])

            power_time_descent_sum = sum(np.array(mean_battery_power_mission_descent) * np.array(battery_time_mission_descent)) / sum(np.array(battery_time_mission_descent))

            mean_battery_power_mission.append(power_time_descent_sum)
            battery_time_mission.append(sum(np.array(battery_time_mission_descent)))

            # Reserve Climb

            reserve_climb_segments = [key for key in results.segments.keys() if
                                      (('climb' in key) and ('reserve' in key) and ('second_leg' not in key) and (
                                              'climbout' not in key))]

            mean_battery_power_mission_reserve_climb = []
            battery_time_mission_reserve_climb = []

            for segment in reserve_climb_segments:
                mean_battery_power_mission_reserve_climb.append(np.mean(-results.segments[segment].conditions.propulsion.battery_draw))
                reserve_climb_time = results.segments[segment].conditions.frames.inertial.time[-1] - results.segments[segment].conditions.frames.inertial.time[0]
                battery_time_mission_reserve_climb.append(reserve_climb_time[0])

            power_time_reserve_climb_sum = sum(np.array(mean_battery_power_mission_reserve_climb) * np.array(battery_time_mission_reserve_climb)) / sum(np.array(battery_time_mission_reserve_climb))

            mean_battery_power_mission.append(power_time_reserve_climb_sum)
            battery_time_mission.append(sum(np.array(battery_time_mission_reserve_climb)))

            # Reserve Cruise

            mean_battery_power_mission.append(np.mean(-results.segments.reserve_cruise.conditions.propulsion.battery_draw))
            reserve_cruise_time = results.segments.reserve_cruise.conditions.frames.inertial.time[-1] - results.segments.reserve_cruise.conditions.frames.inertial.time[0]
            battery_time_mission.append(reserve_cruise_time[0])

            # Reserve Descent

            reserve_descent_segments = [key for key in results.segments.keys() if
                                        (('descent' in key) and ('reserve' in key) and ('second_leg' not in key))]

            mean_battery_power_mission_reserve_descent = []
            battery_time_mission_reserve_descent = []

            for segment in reserve_descent_segments:
                mean_battery_power_mission_reserve_descent.append(np.mean(-results.segments[segment].conditions.propulsion.battery_draw))
                reserve_descent_time = results.segments[segment].conditions.frames.inertial.time[-1] - results.segments[segment].conditions.frames.inertial.time[0]
                battery_time_mission_reserve_descent.append(reserve_descent_time[0])

            power_time_reserve_descent_sum = sum(np.array(mean_battery_power_mission_reserve_descent) * np.array(battery_time_mission_reserve_descent)) / sum(np.array(battery_time_mission_reserve_descent))

            mean_battery_power_mission.append(power_time_reserve_descent_sum)
            battery_time_mission.append(sum(np.array(battery_time_mission_reserve_descent)))

            # Holding

            mean_battery_power_mission.append(np.mean(-results.segments.hold.conditions.propulsion.battery_draw))
            holding_time = results.segments.hold.conditions.frames.inertial.time[-1] - results.segments.hold.conditions.frames.inertial.time[0]
            battery_time_mission.append(holding_time[0])

            # battery_time_mission.append(15*60) #charging?
            # mean_battery_power_mission.append(-2000) #charging?

            battery_time_mission = np.array(battery_time_mission)

            test_study_parameters = {
                "durability_target": parameters.battery_durability_target,  # in months
                "pack_power": mean_battery_power_mission,
                # W here we specify the power profile : power values ex: np.array([100_000, 10_000, 25_000])
                "time_data": battery_time_mission,
                # s here we specify the power profile : duration values ex: np.array([500, 2000, 1000])
                "pack_checkup_power": np.array([100_000]),  # not is use for now
                "time_data_checkup": np.array([500]),  # not is use for now
                "maximum_branches_number": 1000,
                "soh_interval": iteration_setup.sizing_iter.battery_soh_interval_setting # np.array([90,80,70,65])
            }

            # instantiation
            battery_degradation = battery_sizing_tool.BatteryPack(test_cell_parameters, test_pack_parameters, test_study_parameters, stop_after_durability_reached = iteration_setup.sizing_iter.battery_stop_after_durability_reached)

            # Battery sizing
            battery_degradation.run_sizing()

            if battery_degradation.get_sizing_results()[0, 4] > battery_degradation.get_durability_target() and battery_degradation.get_sizing_results()[0, 0] == 100:
                battery_mass_cea = battery_degradation.get_sizing_results()[0, 1] * parameters.battery_cell_to_pack_factor
                iteration_setup.sizing_iter.battery_durability_in_months = battery_degradation.get_sizing_results()[0, 4]
                configs.base.propulsors.network.battery.durability_in_months = battery_degradation.get_sizing_results()[0, 4]
                iteration_setup.sizing_iter.battery_eol_soh = battery_degradation.get_sizing_results()[0, 0]
                configs.base.propulsors.network.battery.eol_soh = battery_degradation.get_sizing_results()[0, 0]
                iteration_setup.sizing_iter.battery_number_branches = battery_degradation.get_sizing_results()[0, 3]
                configs.base.propulsors.network.battery.number_branches = battery_degradation.get_sizing_results()[0, 3]
                iteration_setup.sizing_iter.battery_number_cells_series = battery_degradation.get_sizing_results()[0, 2]
                configs.base.propulsors.network.battery.number_cells_series = battery_degradation.get_sizing_results()[0, 2]

            else:
                battery_mass_cea = battery_degradation.get_mass()[0] * parameters.battery_cell_to_pack_factor
                iteration_setup.sizing_iter.battery_durability_in_months = battery_degradation.get_durability()
                configs.base.propulsors.network.battery.durability_in_months = battery_degradation.get_durability()
                iteration_setup.sizing_iter.battery_eol_soh = battery_degradation.get_soh()
                configs.base.propulsors.network.battery.eol_soh = battery_degradation.get_soh()
                iteration_setup.sizing_iter.battery_number_branches = battery_degradation.get_number_branches()
                configs.base.propulsors.network.battery.number_branches = battery_degradation.get_number_branches()
                iteration_setup.sizing_iter.battery_number_cells_series = battery_degradation.get_number_cells_series()
                configs.base.propulsors.network.battery.number_cells_series = battery_degradation.get_number_cells_series()

            configs.base.propulsors.network.battery.cell_mass = test_cell_parameters['cell_mass']
            configs.base.propulsors.network.battery.cell_max_voltage = test_cell_parameters['cell_max_voltage']
            configs.base.propulsors.network.battery.cell_min_voltage = test_cell_parameters['cell_min_voltage']
            configs.base.propulsors.network.battery.cell_nominal_voltage = test_cell_parameters['cell_nominal_voltage']
            configs.base.propulsors.network.battery.cell_nominal_energy = test_cell_parameters['cell_nominal_energy']
            configs.base.propulsors.network.battery.cell_initial_temperature = test_cell_parameters['cell_initial_temperature']

            configs.base.propulsors.network.battery.pack_initial_soc = test_pack_parameters['initial_soc']
            configs.base.propulsors.network.battery.pack_initial_soh = test_pack_parameters['initial_soh']
            configs.base.propulsors.network.battery.pack_nominal_voltage = test_pack_parameters['pack_nominal_voltage']
            configs.base.propulsors.network.battery.pack_min_soc = test_pack_parameters['min_soc']
            configs.base.propulsors.network.battery.pack_max_soc =  test_pack_parameters['max_soc']

            iteration_setup.sizing_iter.battery_cell_mass = test_cell_parameters['cell_mass']
            iteration_setup.sizing_iter.battery_cell_max_voltage = test_cell_parameters['cell_max_voltage']
            iteration_setup.sizing_iter.battery_cell_min_voltage = test_cell_parameters['cell_min_voltage']
            iteration_setup.sizing_iter.battery_cell_nominal_voltage = test_cell_parameters['cell_nominal_voltage']
            iteration_setup.sizing_iter.battery_cell_nominal_energy = test_cell_parameters['cell_nominal_energy']
            iteration_setup.sizing_iter.battery_cell_initial_temperature = test_cell_parameters['cell_initial_temperature']

            iteration_setup.sizing_iter.battery_pack_initial_soc = test_pack_parameters['initial_soc']
            iteration_setup.sizing_iter.battery_pack_initial_soh = test_pack_parameters['initial_soh']
            iteration_setup.sizing_iter.battery_pack_nominal_voltage = test_pack_parameters['pack_nominal_voltage']
            iteration_setup.sizing_iter.battery_pack_min_soc = test_pack_parameters['min_soc']
            iteration_setup.sizing_iter.battery_pack_max_soc = test_pack_parameters['max_soc']



            print(f"Battery Mass CEA {battery_mass_cea:.2f} kg")
            # print(f"End of life SoH = {battery_degradation.end_of_life_soh[0]:,.2f} %")
            print('Battery Mass SUAVE %.2f kg' % configs.base.propulsors.network.battery.mass_properties.mass)

            iteration_setup.energy_iter.battery_mass_factor_cea =  iteration_setup.energy_iter.battery_mass_factor_cea * configs.base.propulsors.network.battery.mass_properties.mass / battery_mass_cea

            print('Battery Factor CEA: %.3f' % iteration_setup.energy_iter.battery_mass_factor_cea)
            if np.isnan(iteration_setup.energy_iter.battery_mass_factor_cea):
                iteration_setup.energy_iter.battery_mass_factor_cea = 1

            delta_battery_mass = abs(configs.base.propulsors.network.battery.mass_properties.mass - battery_mass_cea)

            ############################################################################################################
            # not used --> not working
            IR = configs.base.propulsors.network.battery.internal_resistance_cell
            IR_series = IR * battery_degradation.get_number_cells_series()
            IR_total = 1 / (1/IR_series * battery_degradation.get_number_branches())
            iteration_setup.sizing_iter.battery_resistance = IR_total
            #print('Battery IR: %.3f Ohm' % IR_total)
            ############################################################################################################


        elif (abs(error_energy/iteration_setup.energy_iter.battery_energy_used) > 0.1) and battery_degradation_run == True:
            delta_battery_mass = 10
        else:
            delta_battery_mass = 0

        # results_show(results,configs.base,iteration_setup,first_climb_segment,last_climb_segment,last_descent_segment,first_descent_segment,first_reserve_climb_segment,last_reserve_descent_segment,first_reserve_descent_segment,last_reserve_climb_segment,climb_segments)
        # print(configs.base.propulsors.network.battery.max_energy)

        sizing_chart_run = 1

        if sizing_chart_run == 0:
            i_sizing_chart_runs = 5  # see battery degredation run

        #if abs(error_energy) < 50 * Units.kWh and sizing_chart_run == True and i_sizing_chart_runs < 50:
        if (abs(error_energy/iteration_setup.energy_iter.battery_energy_used) < 0.1) and deltaweight < 100 and sizing_chart_run == True and i_sizing_chart_runs < 7:

            i_sizing_chart_runs += 1
            ####################################################################################################################
            # Sizing Chart
            ####################################################################################################################

            configs_analyses = analyses.configs

            analyses.missions = missions_setup(mission_setup_Sizing_Chart(configs_analyses,iteration_setup,parameters))
            #analyses.missions = missions_setup(mission_setup(configs_analyses,iteration_setup,parameters))

            analyses.finalize()

            mission_sizing_chart = analyses.missions.base

            sizing_chart_evaluation = sizing_chart_mission_evaluate(configs.base, mission_sizing_chart, iteration_setup)

            # results = sizing_chart_mission_evaluate.results

            # plot_mission(sizing_chart_evaluation.results)
            # plot_flight_conditions(sizing_chart_evaluation.results, 'bo-', False)
            # plot_aerodynamic_forces(sizing_chart_evaluation.results, 'bo-', False)
            # plot_stability_coefficients(sizing_chart_evaluation.results, 'bo-', False)
            #
            # plt.show()

            throttle_old = iteration_setup.sizing_iter.factor_power_loading_sizing_chart
            throttle_mission = np.array([])
            for segment in sizing_chart_evaluation.results.segments.values():
                throttle_mission = np.append(throttle_mission, segment.conditions.propulsion.throttle[:, 0])

            throttle_mission_max = np.max(throttle_mission)

            if throttle_mission_max > 1.02:
                #throttle_mission_max *= 1.04
                print('Info: SC throttle bigger 1.02')


            if throttle_mission_max > 1.0 and throttle_mission_max < 1.1:
                iteration_setup.sizing_iter.factor_power_loading_sizing_chart = throttle_old*(throttle_mission_max + (1 - iteration_setup.sizing_iter.max_throttle_engine_sizing_chart))**2
            elif throttle_mission_max > 1.1:
                iteration_setup.sizing_iter.factor_power_loading_sizing_chart = throttle_old * (throttle_mission_max + (1 - iteration_setup.sizing_iter.max_throttle_engine_sizing_chart))
            else:
                iteration_setup.sizing_iter.factor_power_loading_sizing_chart = throttle_old*(throttle_mission_max + (1 - iteration_setup.sizing_iter.max_throttle_engine_sizing_chart))#**0.1


            if np.isnan(iteration_setup.sizing_iter.factor_power_loading_sizing_chart):
                iteration_setup.sizing_iter.factor_power_loading_sizing_chart = iteration_setup.sizing_iter.factor_power_loading_sizing_chart_nan * 1.05
                iteration_setup.sizing_iter.factor_power_loading_sizing_chart_nan = iteration_setup.sizing_iter.factor_power_loading_sizing_chart
                i_sizing_chart_runs = 0
                print('Info: Factor SC is NAN')

            else:
                numbers_not_converged = 0
                for segment in sizing_chart_evaluation.results.segments:
                    if not segment.converged:
                        numbers_not_converged += 1
                        print('Info: SC not converged')
                if numbers_not_converged > 0:
                    iteration_setup.sizing_iter.factor_power_loading_sizing_chart = iteration_setup.sizing_iter.factor_power_loading_sizing_chart_nan * 1.05
                    iteration_setup.sizing_iter.factor_power_loading_sizing_chart_nan = iteration_setup.sizing_iter.factor_power_loading_sizing_chart
                    i_sizing_chart_runs -= 2

            # print('Factor Sizing Chart: %.3f' % (iteration_setup.sizing_iter.factor_power_loading_sizing_chart))
            if iteration_setup.sizing_iter.factor_power_loading_sizing_chart < 1 and (delta_throttle > 0. or iteration_setup.sizing_iter.factor_engine_power > 1.):
                iteration_setup.sizing_iter.factor_power_loading_sizing_chart = throttle_old
                #print('Factor Sizing Chart: %.3f' % (iteration_setup.sizing_iter.factor_power_loading_sizing_chart))

            print('Factor Sizing Chart: %.3f' % (iteration_setup.sizing_iter.factor_power_loading_sizing_chart))
            
            # plot_mission(sizing_chart_evaluation.results)
            # plt.show()

    #results_show(results,configs.base,iteration_setup,first_climb_segment,last_climb_segment,last_descent_segment,first_descent_segment,first_reserve_climb_segment,last_reserve_descent_segment,first_reserve_descent_segment,last_reserve_climb_segment,climb_segments)


    ####################################################################################################################
    main_missions = Data()

    ####################################################################################################################

    main_missions.design_mission = results

    ####################################################################################################################
    ####################################################################################################################

    if parameters.mission_distance != 400 * Units.km:
        # Pre-Processing

        ####Evaluation Mission###########################################################################################

        configs_analyses = analyses.configs
        iteration_setup.mission_iter.temperature_deviation = 0
        # define new mission parameters
        #iteration_setup.mission_iter.cruise_altitude = 17000#25000  # 25000 #25000

        # load new mission
        analyses.missions = missions_setup(mission_setup(configs_analyses,iteration_setup,parameters))
        analyses.finalize()
        mission = analyses.missions.base

        # run off design point
        off_design_mission_calculation_variable_reserve_5300kg_400km, results_5300kg_400km= off_design_mission_calculation_variable_reserve(configs.base,mission,"cruise",iteration_setup,5300, 400* Units.km)

        # plot_mission(results_off_design_example)
        # plot_flight_conditions(results_off_design_example, 'bo-', False)
        # plot_electronic_conditions(results_off_design_example, 'bo-')
        # plt.show()

        # print('MTOM %.2f kg' % configs.base.mass_properties.max_takeoff)
        main_missions.off_design_evaluation_5300kg_400km = results_5300kg_400km
        #results_show(results_5300kg_400km, configs.base, iteration_setup, first_climb_segment, last_climb_segment, last_descent_segment,first_descent_segment, first_reserve_climb_segment, last_reserve_descent_segment,first_reserve_descent_segment, last_reserve_climb_segment, climb_segments)

    ####################################################################################################################
    # payload range
    ####################################################################################################################

    payload_range_run = 0

    if payload_range_run == True:
        payload_range_results = payload_range_variable_reserve(configs.base,mission,"cruise",iteration_setup)

        main_missions.payload_range_max_payload_mtom = payload_range_results.results_mission.max_payload_mtom
        main_missions.payload_range_max_fuel_mtom = payload_range_results.results_mission.max_fuel_mtom
        main_missions.payload_range_ferry_range = payload_range_results.results_mission.ferry_range

        plot_mission(main_missions.payload_range_max_payload_mtom)
        #plot_flight_conditions(payload_range_results.results_mission.max_fuel_mtom, 'bo-', False)
        #plot_electronic_conditions(payload_range_results.results_mission.max_fuel_mtom, 'bo-')
        plt.show()

        plot_mission(main_missions.payload_range_max_fuel_mtom)
        plt.show()

        plot_mission(main_missions.payload_range_ferry_range)
        plt.show()


    ####################################################################################################################
    # off design missions
    ####################################################################################################################

    simple_off_design_run = 0

    if simple_off_design_run == True:
        # Pre-Processing

        ####Max Range Mission###########################################################################################

        configs_analyses = analyses.configs
        iteration_setup.mission_iter.temperature_deviation = 00
        # define new mission parameters
        #iteration_setup.mission_iter.cruise_altitude = 17000#25000  # 25000 #25000

        # load new mission
        analyses.missions = missions_setup(mission_setup(configs_analyses,iteration_setup,parameters))
        analyses.finalize()
        mission = analyses.missions.base

        # run off design point
        off_design_Max_Range_results, results_off_design_Max_Range= off_design_mission_calculation_variable_reserve(configs.base,mission,"cruise",iteration_setup,0, 800* Units.nautical_mile)

        # plot_mission(results_off_design_example)
        # plot_flight_conditions(results_off_design_example, 'bo-', False)
        # plot_electronic_conditions(results_off_design_example, 'bo-')
        # plt.show()

        # print('MTOM %.2f kg' % configs.base.mass_properties.max_takeoff)

        main_missions.off_design_Max_Range = results_off_design_Max_Range


    if simple_off_design_run == True:
        # Pre-Processing

        ####ISA +10 Mission###########################################################################################

        configs_analyses = analyses.configs
        iteration_setup.mission_iter.temperature_deviation = 10
        # define new mission parameters
        # iteration_setup.mission_iter.cruise_altitude = 17000#25000  # 25000 #25000

        # load new mission
        analyses.missions = missions_setup(mission_setup(configs_analyses, iteration_setup, parameters))
        analyses.finalize()
        mission = analyses.missions.base

        # run off design point
        off_design_ISAP10_results, results_off_design_ISAP10 = off_design_mission_calculation_variable_reserve(configs.base, mission, "cruise", iteration_setup, 5300, 250 * Units.nautical_mile)

        # plot_mission(results_off_design_example)
        # plot_flight_conditions(results_off_design_example, 'bo-', False)
        # plot_electronic_conditions(results_off_design_example, 'bo-')
        # plt.show()

        # print('MTOM %.2f kg' % configs.base.mass_properties.max_takeoff)

        main_missions.off_design_ISAP10 = results_off_design_ISAP10

    if simple_off_design_run == True:
        # Pre-Processing

        ####ISA -10 Mission###########################################################################################

        configs_analyses = analyses.configs
        iteration_setup.mission_iter.temperature_deviation = -10
        # define new mission parameters
        # iteration_setup.mission_iter.cruise_altitude = 17000#25000  # 25000 #25000

        # load new mission
        analyses.missions = missions_setup(mission_setup(configs_analyses, iteration_setup, parameters))
        analyses.finalize()
        mission = analyses.missions.base

        # run off design point
        off_design_ISAM10_results, results_off_design_ISAM10 = off_design_mission_calculation_variable_reserve(configs.base, mission, "cruise", iteration_setup, 5300, 300 * Units.nautical_mile)

        # plot_mission(results_off_design_example)
        # plot_flight_conditions(results_off_design_example, 'bo-', False)
        # plot_electronic_conditions(results_off_design_example, 'bo-')
        # plt.show()

        # print('MTOM %.2f kg' % configs.base.mass_properties.max_takeoff)

        main_missions.off_design_ISAM10 = results_off_design_ISAM10

    if simple_off_design_run == True:
        # Pre-Processing

        ####Island Mission###########################################################################################

        configs_analyses = analyses.configs
        iteration_setup.mission_iter.temperature_deviation = 0
        # define new mission parameters
        # iteration_setup.mission_iter.cruise_altitude = 17000#25000  # 25000 #25000

        # load new mission
        analyses.missions = missions_setup(mission_setup(configs_analyses, iteration_setup, parameters))
        analyses.finalize()
        mission = analyses.missions.base

        # run off design point
        off_design_island_results, results_off_design_island = off_design_mission_calculation_variable_reserve(configs.base, mission, "cruise", iteration_setup, 5300, 200 * Units.nautical_mile)

        # plot_mission(results_off_design_example)
        # plot_flight_conditions(results_off_design_example, 'bo-', False)
        # plot_electronic_conditions(results_off_design_example, 'bo-')
        # plt.show()

        # print('MTOM %.2f kg' % configs.base.mass_properties.max_takeoff)

        main_missions.off_design_island = results_off_design_island
    ####################################################################################################################
    # for fixed payload

    fix_payload_off_design = 0

    if fix_payload_off_design == True:

        # Pre-Processing
        configs_analyses = analyses.configs

        # define new mission parameters
        #iteration_setup.mission_iter.cruise_altitude = 17000#25000  # 25000 #25000
        #iteration_setup.mission_iter.temperature_deviation=10
        # iteration_setup.mission_iter.temperature_deviation= -10
        iteration_setup.mission_iter.temperature_deviation= 00

        # load new mission
        analyses.missions = missions_setup(mission_setup(configs_analyses,iteration_setup,parameters))
        analyses.finalize()
        mission = analyses.missions.base

        MTOM = configs.base.mass_properties.max_takeoff
        max_fuel = configs.base.mass_properties.max_fuel

        print('MTOM %.2f kg' % configs.base.mass_properties.max_takeoff)
        print('max_fuel %.2f kg' % configs.base.mass_properties.max_fuel)


        error_range = 9999 # defaults
       # payload = 5300 * Units.kg #fixed payload
        #range = iteration_setup.mission_iter.mission_distance
        range = 800 * Units.km
        #range = 200 * Units.km



        while abs(error_range) > 500:

            # run off design point
            off_design_results_ISA, results_off_design_ISA = off_design_mission_calculation_variable_reserve(configs.base,mission,"cruise",iteration_setup, payload, range)

            # plot_mission(results_off_design_ISA)
            # plot_flight_conditions(results_off_design_ISA, 'bo-', False)
            # plot_electronic_conditions(results_off_design_ISA, 'bo-')
            # plt.show()

            new_range = off_design_results_ISA.range
            new_payload = off_design_results_ISA.payload
            new_fuel = off_design_results_ISA.fuel[0]
            new_TOM = off_design_results_ISA.takeoff_weight[0]



            # Current distance and fuel consuption in the cruise segment
            segment = results_off_design_ISA.segments.cruise
            CruiseDist = np.diff( segment.conditions.frames.inertial.position_vector[[0,-1],0] )[0]        # Distance [m]
            CruiseFuel = segment.conditions.weights.total_mass[0,0] - segment.conditions.weights.total_mass[-1,0]    # [kg]

            # Current specific range (m/kg)
            CruiseSR    = CruiseDist / CruiseFuel        # [m/kg]
            print('CruiseSR %.2f m/kg' % CruiseSR)

            #point 1 --> TOM == MTOM and fuel < fuel_max
            #point 2 --> TOM < MTOM and fuel == fuel_max

            if new_fuel <= max_fuel:
                missingFuel = MTOM - new_TOM
            elif new_fuel > max_fuel:
                missingFuel = max_fuel - new_fuel
                if new_TOM > MTOM:
                    missingFuel = missingFuel + MTOM - new_TOM
            else:
                missingFuel = max((max_fuel - new_fuel),(MTOM - new_TOM))

            if (missingFuel + new_fuel) > max_fuel:
                missingFuel = max_fuel - new_fuel
                if new_TOM > MTOM:
                    missingFuel = missingFuel + MTOM - new_TOM


            if new_TOM > MTOM:
                print('new_TOM > MTOM')
            elif new_fuel > max_fuel:
                print('new_fuel > max_fuel')

            if new_payload != payload: #check if payload > max payload
                payload = new_payload

            # Estimated distance that will result in total fuel burn = target fuel
            DeltaDist  =  CruiseSR *  missingFuel

            if DeltaDist < 0:
                range = (new_range + DeltaDist*3/2) # *2/3 todo muss eig wieder raus --> check 4000 kg
            elif DeltaDist > 0:
                range = (new_range + DeltaDist*2/3)
            error_range = DeltaDist
            print('TOM %.2f kg' % new_TOM)
            print('range %.1f km' % (range/Units.km))
            print('error_range %.1f km' % (error_range/Units.km))
            print('fuel %.2f kg' % new_fuel)

        main_missions.results_off_design_ISA = results_off_design_ISA

        print('Final Results for fix payload')
        print('TOM %.2f kg' % new_TOM)
        print('Payload %.1f kg' % new_payload)
        print('range %.1f km' % (range/Units.km))

    ####################################################################################################################
    # for more than one mission
    ####################################################################################################################
    #todo

    # results = Data()
    # results.off_design = Data()
    # results.payload_range = Data()
    # results.sizing_mission = Data()
    #
    # results.sizing_mission.main_mission = results
    # results.payload_range.max_payload_mtom = payload_range_results.results_mission.max_payload_mtom
    # results.payload_range.max_fuel_mtom = payload_range_results.results_mission.max_fuel_mtom
    # results.payload_range.ferry_range = payload_range_results.results_mission.ferry_range


    ####################################################################################################################
    # create results folder path and create absolute path
    ####################################################################################################################

    Path("../_results").mkdir(exist_ok=True)
    absolute_results_folder_path = os.path.abspath("../_results")

    if configs.base.propulsors.network.turboshaft.p3t3_method:
        mission_list = list(main_missions.keys())
        for mission in mission_list:
            main_missions[mission] = gas_turbine_calculate_emission(main_missions[mission], configs.base)

    fom = SUAVE.Methods.Figures_of_Merit.Figure_of_Merit()
    for setting in settings.keys():
        fom.settings[setting] = settings[setting]

    if parameters.mission_distance != 400 * Units.km:
        fom.calculate(main_missions.off_design_evaluation_5300kg_400km, configs)
    else:
        fom.calculate(main_missions.design_mission, configs) #todo please check other missions like HEA

    all_converged = True
    for segment in results.segments:
        if not segment.converged:
            all_converged = False
            break

    if all_converged:
        output_results = Data()
        #output_results.main_missions = Data()
        #output_results.main_missions.design_mission = results
        output_results.main_missions = main_missions
        write_output_files(output_results, configs, absolute_results_folder_path, show_figures=False,
                           iteration_setup=iteration_setup, fom=fom, parameters=parameters, settings=settings)

    return all_converged
