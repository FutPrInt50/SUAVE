## write_output.py
#
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Dominik Eisenhut, eisenhut@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Jun 2022, D. Eisenhut
# Modified: Jul 2022, D. Eisenhut
#           Sep 2022, D. Eisenhut

from datetime import datetime
from .write_fact_sheet import write_fact_sheet
from .write_mission import write_mission_csv
from SUAVE.Plots.Mission_Plots import *
from SUAVE.Input_Output.SUAVE import archive
from SUAVE.Core import Data


def output_raw_data(file_id, results, configs, iteration_setup, fom):
    raw = Data()
    raw.results = results
    raw.configs = configs
    if fom is not None:
        raw.fom = fom

    archive(raw, file_id + "_raw_data")
    if iteration_setup is not None:
        archive(iteration_setup, file_id + "_iteration_setup_raw")


def create_figures(tag, results, vehicle, file_id, show_figures):
    plot_flight_conditions(results, line_color='bo-', save_figure=True, save_filename=file_id + "_Flight_Conditions")
    plot_aerodynamic_forces(results, line_color='bo-', save_figure=True, save_filename=file_id + "_Aerodynamic_Forces")
    plot_stability_coefficients(results, line_color='bo-', save_figure=True, save_filename=file_id + "_Stability_Coefficients")
    if 'rpm_propeller' in results.segments.cruise.conditions.propulsion.keys():
        plot_delft_propeller_conditions(results, 'propeller', line_color='bo-', save_figure=True, save_filename=file_id + "_Propeller_Conditions")
    else:
        plot_propeller_conditions(results, line_color='bo-', save_figure=True, save_filename=file_id + "_Propeller_Conditions")
    plot_aerodynamic_coefficients_custom(results, save_figure=True, save_filename=file_id + "_Aerodynamic_Coefficients")
    plot_drag_components_custom(results, save_figure=True, save_filename=file_id + "_Drag_Components")
    plot_detailed_parasite_drag(results, save_figure=True, save_filename=file_id + "_Detailed_Parasite_Drag")
    plot_throttle(results, save_figure=True, save_filename=file_id + "_Throttle")
    plot_mission_profile(results, save_figure=True, save_filename=file_id + "_Mission_Profile")
    plot_altitude_sfc_weight_custom(results, save_figure=True, save_filename=file_id + "_Altitude_SFC_weight")
    plot_altitude_power(results, vehicle, save_figure=True, save_filename=file_id + "_Altitude_Power")
    plot_stability(results, save_figure=True, save_filename=file_id + "_Stability")

    if show_figures:
        plt.show()

    plt.close('all')


def write_parameters(file_id, parameters):
    output = ''
    for key in parameters.keys():
        output = f'{output}{key: <30}{parameters[key]}\n'

    f = open(file_id + "_parameters.txt", "w")
    f.write(output)
    f.close()

def write_settings(file_id, settings):
    output = ''
    for key in settings.keys():
        output = f'{output}{key: <60}{settings[key]}\n'

    f = open(file_id + "_settings.txt", "w")
    f.write(output)
    f.close()


def write_output_files(results, configs, absolute_results_folder_path, save_figures=False, show_figures=False, iteration_setup=None,
                       fom=None, parameters=None, settings=None):
    print("Writing output files ...")

    architecture_tag = configs.base._base.tag
    file_id = absolute_results_folder_path + "/" + architecture_tag + "_" + datetime.now().strftime("%Y%m%d_%H%M%S")

    output_raw_data(file_id, results, configs, iteration_setup, fom)

    # write all mission .csv files
    group_list = list(results.keys())
    for group in group_list:
        mission_list = list(results[group].keys())
        for mission in mission_list:
            write_mission_csv(file_id, mission, configs.base, results[group][mission].segments)

    write_fact_sheet(file_id, configs.base)

    if parameters is not None:
        write_parameters(file_id, parameters)

    if settings is not None:
        write_settings(file_id, settings)

    if save_figures:
        create_figures(architecture_tag, results.main_missions.design_mission, configs.base, file_id, show_figures)

    print("Writing output files completed.")
