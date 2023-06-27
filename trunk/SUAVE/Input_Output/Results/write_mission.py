## write_mission.py
#
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Dominik Eisenhut, eisenhut@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Jul 2022, D. Eisenhut
# Modified: Sep 2022, D. Eisenhut


import numpy as np
from SUAVE.Core import Units
import pandas as pd


def write_mission_csv(file_name, mission_name, vehicle, segments):
    # standard headlines
    headlines = ['segment', 'time', 'altitude_m', 'mach_number', 'velocity_m_s', 'pressure_Pa', 'density_kg_m3',
                 'temperature_C', 'lift_coefficient', 'drag_coefficient', 'angle_of_attack_rad',
                 'flight_path_angle_rad', 'mass_kg', 'mass_flow_kg_s', 'CG_m', 'CG_percent']

    # add conditions.propulsion automatically
    keys = segments[0].conditions.propulsion.keys()
    headlines.extend(keys)

    output_bool = {}

    for headline in headlines:
        output_bool[headline] = True

    # check if all propulsion keys are ndarrays
    for key in keys:
        if type(segments[0].conditions.propulsion[key]) is not np.ndarray:
            output_bool[key] = False


    # grab all data and put it directly into pd.DataFrame
    all_data = []
    for segment in segments:
        new_data = []
        for headline in headlines:
            if output_bool[headline]:
                if headline in keys:
                    new_data.append(segment.conditions.propulsion[headline][:, 0].tolist())
                else:
                    if headline == 'segment':
                        new_data.append([segment.tag] * len(segment.conditions.frames.inertial.time[:, 0]))
                    elif headline == 'time':
                        new_data.append(segment.conditions.frames.inertial.time[:, 0].tolist())
                    elif headline == 'altitude_m':
                        new_data.append(segment.conditions.freestream.altitude[:, 0].tolist())
                    elif headline == 'mach_number':
                        new_data.append(segment.conditions.freestream.mach_number[:, 0].tolist())
                    elif headline == 'velocity_m_s':
                        new_data.append(segment.conditions.freestream.velocity[:, 0].tolist())
                    elif headline == 'pressure_Pa':
                        new_data.append(segment.conditions.freestream.pressure[:, 0].tolist())
                    elif headline == 'density_kg_m3':
                        new_data.append(segment.conditions.freestream.density[:, 0].tolist())
                    elif headline == 'temperature_C':
                        new_data.append(segment.conditions.freestream.temperature[:, 0].tolist())
                    elif headline == 'lift_coefficient':
                        new_data.append(segment.conditions.aerodynamics.lift_coefficient[:, 0].tolist())
                    elif headline == 'drag_coefficient':
                        new_data.append(segment.conditions.aerodynamics.drag_coefficient[:, 0].tolist())
                    elif headline == 'angle_of_attack_rad':
                        new_data.append(segment.conditions.aerodynamics.angle_of_attack[:, 0].tolist())
                    elif headline == 'flight_path_angle_rad':
                        new_data.append(segment.conditions.frames.body.inertial_rotations[:, 1].tolist())
                    elif headline == 'mass_kg':
                        new_data.append(segment.conditions.weights.total_mass[:, 0].tolist())
                    elif headline == 'mass_flow_kg_s':
                        new_data.append(segment.conditions.weights.vehicle_mass_rate[:, 0].tolist())
                    elif headline == 'CG_m':
                        new_data.append(segment.conditions.stability.static.center_of_gravity_mission[:, 0].tolist())
                    elif headline == 'CG_percent':
                        new_data.append(segment.conditions.stability.static.percent_mac[:, 0].tolist())
                    else:
                        print('Warning: write_mission headline specified in list but no if-clause to add value added! ', headline)

            else:
                new_data.append([0] * len(segment.conditions.frames.inertial.time[:, 0]))

        if all_data:
            for i in range(len(new_data)):
                all_data[i].extend(new_data[i])
        else:
            all_data = new_data

    my_dict = {}
    for i, headline in enumerate(headlines):
        my_dict[headline] = all_data[i]

    dataframe = pd.DataFrame(my_dict)
    # calculate additional data within DataFrame
    dataframe = dataframe.assign(l_over_d=lambda x: x.lift_coefficient / x.drag_coefficient)
    dataframe = dataframe.assign(weight=lambda x: x.mass_kg * 9.81)
    dataframe = dataframe.assign(lift=lambda x: x.density_kg_m3 / 2 * x.velocity_m_s**2 * x.lift_coefficient
                                                * vehicle.reference_area)
    dataframe = dataframe.assign(drag=lambda x: x.density_kg_m3 / 2 * x.velocity_m_s ** 2 * x.drag_coefficient
                                                * vehicle.reference_area)

    # remove all the columns for non-existent components
    for key, value in output_bool.items():
        if not value:
            dataframe = dataframe.drop(columns=key)

    dataframe.to_csv(file_name + '_' + mission_name + '.csv')
