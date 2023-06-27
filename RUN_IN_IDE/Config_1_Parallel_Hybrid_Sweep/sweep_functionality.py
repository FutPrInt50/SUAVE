## sweep_functionality.py
#
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Dominik Eisenhut, eisenhut@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Jun 2022, D. Eisenhut
# Modified: Oct 2022, J. Frank

import itertools
import SUAVE
import os
from SUAVE.Core import Data, Units
from datetime import datetime
from Config_1_Parallel_Hybrid import Config_1_Parallel_Hybrid

# TODO parameters are only functionality examples, need to replace by meaningful ones


def sweep(parameters, variations, settings):
    combinations = list(itertools.product(*list(variations.values())))

    total_iterations = len(combinations)

    print("Simulation started: {}".format(datetime.now().strftime("%y-%m-%d %H:%M:%S")))
    print("Running a total of {} sweeps.".format(total_iterations))
    print("Variables swept:", end=" ")

    for variation in list(variations)[:-1]:
        print(variation, end=", ")

    print(list(variations)[-1])
    print("\nThe following sweeps are used:")
    i = 0
    for combination in combinations:
        i += 1
        print('{0:3d}.\t{1}'.format(i, combination))

    print("\n")

    i = 0
    for combination in combinations:
        i += 1
        print('\n\n\n\n*****************************\n+++++++++++++++++++++++++++++')
        print("\n-----------------------------\nRunning sweep {0:3d} of {1:3d}\n-----------------------------".format(i, total_iterations))
        print('\n*****************************\n+++++++++++++++++++++++++++++\n\n\n\n')
        print(datetime.now().strftime("%y-%m-%d %H:%M:%S"))
        n = 0
        for variation in list(variations):
            parameters[variation] = combination[n]
            n += 1

        # print('wl: %f' % parameters.wing_loading)
        # print('hyb: %f' % parameters.hybridization_power)
        # print('ems: %f' % parameters.ems)
        # print('bat_tech: %f' % parameters.battery_technology_factor)
        try:
            all_converged = Config_1_Parallel_Hybrid(parameters, settings)

            if all_converged:
                print("\n-----------------------------")
                print("\nSweep {0:3d} of {1:3d} ran successfully without errors\n".format(i, total_iterations))
                print("-----------------------------")
            else:
                print("\n-----------------------------")
                print("\nSweep {0:3d} of {1:3d} did not converge\n".format(i, total_iterations))
                print("-----------------------------")

        except Exception as e:
            print("\n-----------------------------")
            print("\nSweep {0:3d} of {1:3d} had an error".format(i, total_iterations))
            print('Error: {}'.format(e))
            print("\n-----------------------------")
            pass


    print("\nSimulation ended: {}".format(datetime.now().strftime("%y-%m-%d %H:%M:%S")))


def base_values():

    # TODO define parameters that should be sweepable

    parameters = Data()

    # Sweep 1 - Aircraft parameters
    parameters.hybridization_power = 0.15
    parameters.cruise_speed = 525. # in km/h
    parameters.cruise_altitude = 21000 # in feet
    parameters.ems = 2
    parameters.wing_loading = 271.5 #341.1
    parameters.generator_oversizing = 0.0 #0.01 #0.01
    parameters.mission_distance = 800 * Units.km
    parameters.wing_aspect_ratio = 13.5

    # Sweep 2 - Component parameters
    parameters.battery_specific_energy = 350 #250 350 500
    parameters.battery_durability_target = 12 #month
    parameters.battery_cell_to_pack_factor = 1.5
    parameters.battery_soc_max = 0.9
    parameters.battery_soc_min = 0.1 # < 0.08 leads to problems
    parameters.battery_cell_mass = 0.045 #kg
    parameters.bus_voltage = 2000
    parameters.electric_motor_nominal_rpm = 25000 # rpm
    parameters.propeller_power_loading = 147563 #W/m^2
    parameters.gas_turbine_TET = 1400
    parameters.gas_turbine_OPR = 16
    parameters.gas_turbine_use_pickle = False

    # Sweep 3 - TMS
    parameters.tms_vcs_case_version = 'CASE1_v2' #CASE1 CASE1_v2 CASE2
    parameters.tms_liquid_case_version = 'CASE4_v2' #CASE4 CASE4_v2 CASE6
    parameters.tms_vcs_case_2_shx_area = 2 #just for CASE 2
    parameters.tms_liquid_case6_shx_area = 2 #just for CASE 6

    # Sweep 4 - Technology factors
    parameters.electric_motor_specific_power_factor = 1.
    parameters.tms_specific_power_factor = 1.
    parameters.tms_drag_factor = 1.
    parameters.turboshaft_emissions_index_factor = 1.
    parameters.fuselage_mass_factor = 1.
    parameters.number_of_engines_for_oei = 2.
    parameters.battery_technology_factor = 1.0 # for very high values

    return parameters


def fom_settings():

    settings = Data()
    settings.trl = Data()

    settings.print_flag = True
    settings.plot_flag = False
    settings.hybrid = True

    settings.scaling_minus1_to_one = True
    # ATR42 from 2023-05-17 12:49:40 (Deviation=1.00)
    settings.minmax_co2 =        [ 0.00000e+00, 1.63483e-01]
    settings.minmax_nox =        [ 0.00000e+00, 5.94667e-03]
    settings.minmax_noise_sone = [ 0.00000e+00, 4.46999e+01]
    settings.minmax_doc =        [ 0.00000e+00, 4.35911e+01]
    settings.minmax_dev =        [ -7.88271e+08, 7.88271e+08]
    settings.minmax_cer =        [ -4.19109e+07, 4.19109e+07]
    settings.minmax_prod =       [ 0.00000e+00, 4.59522e+08]

    settings.weighting_factors_co2_nox_noise = [1.05, 1.05, 0.9]
    settings.weighting_factors_dev_cer_prod = [1., 1., 1.]
    settings.weighting_factors_EI_ADI_HEAI = [1., 1., 0.]
    settings.weighting_factors_EI_ADI_HEAI_conv = [.4, 2., 1.]  # Weighting Setting for FutPrInt50
    settings.weighting_factors_EI_ADI_HEAI_env  = [2., 2., 1.]  # Weighting Setting for FutPrInt50
    settings.weighting_factors_sideline_flyover_approach = [1., 1., 1.]
    settings.DOC_or_COC = 'COC'

    settings.trl.electric_motors = 9.
    settings.trl.wing_tip_propulsion = 6.
    settings.trl.batteries = 9.

    settings.number_of_microfones_sideline = 200
    settings.correction_sideline = -4.37
    settings.correction_flyover = -9.08
    settings.correction_approach = +0.9

    settings.main_wing_comp = True
    settings.fuselage_comp = False
    settings.empennage_comp = True

    return settings


if __name__ == "__main__":

    use_sweep = True

    parameters = base_values()
    settings = fom_settings()

    # variations = {
    #     "hybridization_power": [0.1, 0.2, 0.3],
    #     "cruise_speed": [450, 500, 550],
    #     "cruise_altitude": [17000, 21000, 25000],
    #     "ems": [2],
    #     "generator_oversizing": [0.005],
    #     "mission_distance": [400 * Units.km, 800 * Units.km]
    # }
    variations = {
        "hybridization_power": [0.2],
        "cruise_speed": [450],
        "cruise_altitude": [17000],
        "ems": [3],
        "wing_loading": [341.5],
        "generator_oversizing": [0.005],
        "mission_distance": [400 * Units.km],
        "wing_aspect_ratio": [13.5],
        "battery_specific_energy": [500],
        "battery_durability_target": [12],
        "battery_cell_to_pack_factor": [1.5],
        "battery_soc_max": [0.95],
        "battery_soc_min": [0.1],
        "battery_cell_mass": [0.045],
        "bus_voltage": [800],
        "electric_motor_nominal_rpm": [20000],
        "propeller_power_loading": [147563],
        "gas_turbine_TET": [1600], #not used --> check engine
        "gas_turbine_OPR": [20], #not used --> check engine
    }

    # variations = { #baseline
    #     "hybridization_power": [0.15],
    #     "cruise_speed": [525],
    #     "cruise_altitude": [21000],
    #     "ems": [7,8,9],
    #     "wing_loading": [271.5],
    #     "generator_oversizing": [0.00],
    #     "mission_distance": [800 * Units.km],
    #     "wing_aspect_ratio": [13.5],
    #     "battery_specific_energy": [350],
    #     "battery_durability_target": [6],
    #     "battery_cell_to_pack_factor": [1.5],
    #     "battery_soc_max": [0.80],
    #     "battery_soc_min": [0.20],
    #     "battery_cell_mass": [0.045],
    #     "bus_voltage": [2000],
    #     "electric_motor_nominal_rpm": [25000],
    #     "propeller_power_loading": [147563],
    #     "gas_turbine_TET": [1400], #not used --> check engine
    #     "gas_turbine_OPR": [16], #not used --> check engine
    # }

    if use_sweep:
        sweep(parameters, variations, settings)
    else:
        # print('wl: %f' % parameters.wing_loading)
        # print('hyb: %f' % parameters.hybridization_power)
        # print('ems: %f' % parameters.ems)
        # print('bat_tech: %f' % parameters.battery_technology_factor)
        Config_1_Parallel_Hybrid(parameters, settings)
