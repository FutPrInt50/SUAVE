from SUAVE.Input_Output.SUAVE import load
from SUAVE.Core import Units
from SUAVE.Core.DataOrdered import DataOrdered
import pandas as pd
import numpy as np
import matplotlib.pyplot as plt
import os
from datetime import datetime

jet_a1_specific_energy = 43.15 * 10**6

folder_path = '_results'
# get all files in the specified folder
all_files = [f for f in os.listdir(folder_path) if os.path.isfile(os.path.join(folder_path, f))]

raw_files = []
extraction_files = []
for file in all_files:
    # get all raw_data files within the folder
    if 'raw_data' in file:
        raw_files.append(file)
    # check if any existing extraction files are present in the specified folder
    if 'data_extracted' in file:
        extraction_files.append(file)

# handle if already existing extraction files are present in the specified folder
extract_data = False
if extraction_files:
    # display all the options the user has, old files and to select to create a new extraction file
    print('Already existing extraction files detected in folder:')
    print('(1) Create new extraction file')
    print('(2) Skip process')
    # let the user decide and ask him for an input
    input_incorrect = True
    while input_incorrect:
        # first use try ... except to check if the input is convertible to integer if not the ValueError is captured
        try:
            user_input = int(input('Select one option: '))
            # in this case the user has given a value convertible to integer
            # now check if it is in the correct range of options
            if user_input < 1 or user_input > 2:
                print('Your input must be one of the two options')
            else:
                # the user has given an integer in the correct range now set the condition to leave the while loop
                input_incorrect = False
                # process the input
                if user_input == 1:
                    extract_data = True
                elif user_input == 2:
                    extract_data = False
        except ValueError:
            print('Error: You need to input a string convertable to integer!')
else:
    extract_data = True

print('\n')

# create an extraction file unless the user bypasses this by loading an already existing file
if extract_data:
    total_files = len(raw_files)

    # specify data that can/will be used for plotting
    mtom = np.zeros(total_files)
    ome = np.zeros(total_files)
    max_zero_fuel_mass = np.zeros(total_files)
    max_fuel_mass = np.zeros(total_files)
    operationals_mass = np.zeros(total_files)
    propulsion_mass = np.zeros(total_files)
    structures_mass = np.zeros(total_files)
    systems_mass = np.zeros(total_files)
    payload_design_mission = np.zeros(total_files)
    max_payload = np.zeros(total_files)
    wing_area = np.zeros(total_files)
    wing_span = np.zeros(total_files)
    wing_aspect_ratio = np.zeros(total_files)
    wing_loading = np.zeros(total_files)
    power_loading = np.zeros(total_files)
    hybridization_power = np.zeros(total_files)
    generator_oversizing = np.zeros(total_files)
    design_cruise_speed = np.zeros(total_files)
    design_cruise_altitude = np.zeros(total_files)
    design_range = np.zeros(total_files)
    EMS = np.zeros(total_files)      # energy management strategy int for type
    bat_mass = np.zeros(total_files)  # total battery mass
    bat_max_energy_bol = np.zeros(total_files)
    bat_max_energy_eol = np.zeros(total_files)
    bat_max_power = np.zeros(total_files)
    bat_e_s_cell = np.zeros(total_files)  # battery specific energy
    bat_e_s_pack = np.zeros(total_files)  # battery specific energy
    bat_p_s_pack = np.zeros(total_files)  # battery specific power
    bat_cell_to_pack_factor = np.zeros(total_files)
    bat_durability_target_in_months = np.zeros(total_files) # battery durability in months
    bat_durability_in_months = np.zeros(total_files)
    bat_technology_factor = np.zeros(total_files)

    bat_cell_mass = np.zeros(total_files)
    bat_cell_max_voltage = np.zeros(total_files)
    bat_cell_min_voltage = np.zeros(total_files)
    bat_cell_nominal_voltage = np.zeros(total_files)
    bat_cell_nominal_energy = np.zeros(total_files)
    bat_cell_initial_temperature = np.zeros(total_files)

    bat_initial_soc = np.zeros(total_files)
    bat_initial_soh = np.zeros(total_files)
    bat_pack_nominal_voltage = np.zeros(total_files)
    bat_min_soc = np.zeros(total_files)
    bat_max_soc = np.zeros(total_files)

    bat_eol_soh = np.zeros(total_files)
    bat_number_branches = np.zeros(total_files)
    bat_number_cells_series = np.zeros(total_files)

    emotor_wtp_specific_power_factor = np.zeros(total_files)
    emotor_specific_power_factor = np.zeros(total_files)

    emotor_rpm = np.zeros(total_files)
    bus_voltage = np.zeros(total_files)

    propeller_wtp_power_loading = np.zeros(total_files)
    propeller_power_loading = np.zeros(total_files)

    tms_vcs_tag = np.chararray(total_files, itemsize=30)
    tms_vcs_design_point = np.zeros(total_files)
    tms_vcs_specific_power_factor = np.zeros(total_files)
    tms_vcs_drag_factor = np.zeros(total_files)
    tms_vcs_cross_flow_area = np.zeros(total_files)
    tms_vcs_inlet_area = np.zeros(total_files)
    tms_vcs_exhaust_area = np.zeros(total_files)
    tms_vcs_shx_area = np.zeros(total_files)

    tms_liquid_tag = np.chararray(total_files, itemsize=30)
    tms_liquid_design_point = np.zeros(total_files)
    tms_liquid_specific_power_factor = np.zeros(total_files)
    tms_liquid_drag_factor = np.zeros(total_files)
    tms_liquid_cross_flow_area = np.zeros(total_files)
    tms_liquid_inlet_area = np.zeros(total_files)
    tms_liquid_exhaust_area = np.zeros(total_files)
    tms_liquid_shx_area = np.zeros(total_files)
    tms_liquid_naca_area = np.zeros(total_files)

    turboshaft_emissions_index_factor = np.zeros(total_files)
    turboshaft_surrogate = np.chararray(total_files, itemsize=30)

    number_of_engines_for_oei = np.zeros(total_files)
    fuselage_mass_factor = np.zeros(total_files)

    sfom_nox = np.zeros(total_files)
    sfom_co2 = np.zeros(total_files)
    sfom_noise = np.zeros(total_files)
    sfom_doc = np.zeros(total_files)
    sfom_dev = np.zeros(total_files)
    sfom_cer = np.zeros(total_files)
    sfom_prod = np.zeros(total_files)
    fom_ei = np.zeros(total_files)
    fom_adi = np.zeros(total_files)
    fom_heai = np.zeros(total_files)
    fom = np.zeros(total_files)
    fom_conv = np.zeros(total_files)
    fom_env = np.zeros(total_files)
    weighting_factors_EI = np.zeros(total_files)
    weighting_factors_ADI = np.zeros(total_files)
    weighting_factors_HEAI = np.zeros(total_files)
    weighting_factors_EI_conv = np.zeros(total_files)
    weighting_factors_ADI_conv = np.zeros(total_files)
    weighting_factors_HEAI_conv = np.zeros(total_files)
    weighting_factors_EI_env = np.zeros(total_files)
    weighting_factors_ADI_env = np.zeros(total_files)
    weighting_factors_HEAI_env = np.zeros(total_files)
    weighting_factors_co2 = np.zeros(total_files)
    weighting_factors_nox = np.zeros(total_files)
    weighting_factors_noise = np.zeros(total_files)
    weighting_factors_dev = np.zeros(total_files)
    weighting_factors_cer = np.zeros(total_files)
    weighting_factors_prod = np.zeros(total_files)
    weighting_factors_sideline = np.zeros(total_files)
    weighting_factors_flyover = np.zeros(total_files)
    weighting_factors_approach = np.zeros(total_files)

    # special mission specific variables
    time_2_FL170 = np.zeros(total_files)
    ISA_SL_RoC_m_s = np.zeros(total_files)
    ISA_SL_RoC_ft_min = np.zeros(total_files)

    # go through all raw_data files in the specified folder
    for i, file in enumerate(raw_files):
        # give the user some feedback on the process, loading the files and grabbing all can take some seconds
        print('loading file {} of {}'.format(i+1, total_files))
        print(datetime.now().strftime("%y-%m-%d %H:%M:%S"))
        # open the file
        raw = load(folder_path + '/' + file)

        # Do in first iteration

        if i == 0:

            components = []

            systems = raw.configs.base.systems

            for key in systems.keys():
                item = systems[key]
                if isinstance(item, DataOrdered) and key != 'mass_properties':
                    if 'mass_properties' in systems[key].keys():
                        if 'mass' in systems[key].mass_properties.keys():
                            components.append(key)
                        else:
                            pass
                    else:
                        pass
                else:
                    pass

            components_masses = np.zeros([total_files, len(components)])

            main_missions_keys = list(raw.results.main_missions.keys())

            total_missions = len(main_missions_keys)

            # normal mission specific variables
            fuel_total = np.zeros([total_files, total_missions])  # total fuel consumption during complete flight incl. reserve
            fuel_main_mission = np.zeros([total_files, total_missions])
            fuel_cruise = np.zeros([total_files, total_missions])

            co2_total = np.zeros([total_files, total_missions])  # total CO2 emissions during complete flight incl. reserve
            nox_total = np.zeros([total_files, total_missions])  # total NOx emissions during complete flight incl. reserve
            co_total = np.zeros([total_files, total_missions])  # total CO emissions during complete flight incl. reserve

            co2_main_mission = np.zeros([total_files, total_missions])  # total CO2 emissions during complete flight incl. reserve
            nox_main_mission = np.zeros([total_files, total_missions])  # total NOx emissions during complete flight incl. reserve
            co_main_mission = np.zeros([total_files, total_missions])  # total CO emissions during complete flight incl. reserve

            co2_cruise = np.zeros([total_files, total_missions])
            nox_cruise = np.zeros([total_files, total_missions])
            co_cruise = np.zeros([total_files, total_missions])

            cruise_speed = np.zeros([total_files, total_missions])
            cruise_altitude = np.zeros([total_files, total_missions])
            cruise_mean_l_over_d = np.zeros([total_files, total_missions])
            mean_delta_cl_climb = np.zeros([total_files, total_missions])
            mean_delta_cl_cruise = np.zeros([total_files, total_missions])
            mean_delta_cd_climb = np.zeros([total_files, total_missions])
            mean_delta_cd_cruise = np.zeros([total_files, total_missions])
            tom = np.zeros([total_files, total_missions])
            payload = np.zeros([total_files, total_missions])
            range_total = np.zeros([total_files, total_missions])
            range_main_mission = np.zeros([total_files, total_missions])
            range_cruise = np.zeros([total_files, total_missions])
            soc_end_of_mission = np.zeros([total_files, total_missions])
            battery_total = np.zeros([total_files, total_missions])
            battery_main_mission = np.zeros([total_files, total_missions])
            battery_cruise = np.zeros([total_files, total_missions])

        # grab all data and assign it to the numpy arrays
        mtom[i] = raw.configs.base.mass_properties.max_takeoff
        ome[i] = raw.configs.base.mass_properties.operating_empty
        max_zero_fuel_mass[i] = raw.configs.base.mass_properties.max_zero_fuel
        max_fuel_mass[i] = raw.configs.base.mass_properties.max_fuel
        max_payload[i] = raw.configs.base.mass_properties.max_payload
        operationals_mass[i] = raw.configs.base.mass_properties.operationals
        propulsion_mass[i] = raw.configs.base.mass_properties.propulsion
        structures_mass[i] = raw.configs.base.mass_properties.structures
        systems_mass[i] = raw.configs.base.mass_properties.systems
        payload_design_mission[i] = raw.configs.base.mass_properties.payload
        wing_area[i] = raw.configs.base.reference_area
        wing_span[i] = raw.configs.base.wings.main_wing.spans.projected
        wing_aspect_ratio[i] = raw.configs.base.wings.main_wing.aspect_ratio
        wing_loading[i] = raw.configs.base.wing_loading
        power_loading[i] = raw.configs.base.propulsors.network.power_loading_calculated
        if 'hybridization_power' in raw.configs.base.propulsors.network.keys():
            hybridization_power[i] = raw.configs.base.propulsors.network.hybridization_power
        else:
            hybridization_power[i] = 0
        if 'generator_oversizing' in raw.configs.base.propulsors.network.keys():
            generator_oversizing[i] = raw.configs.base.propulsors.network.generator_oversizing
        else:
            generator_oversizing[i] = 0
        if 'design_cruise_speed' in raw.configs.base.keys():
            design_cruise_speed[i] = raw.configs.base.design_cruise_speed
        else:
            design_cruise_speed[i] = 0
        if 'design_cruise_altitude' in raw.configs.base.keys():
            design_cruise_altitude[i] = raw.configs.base.design_cruise_altitude
        else:
            design_cruise_altitude[i] = 0
        if 'design_range' in raw.configs.base.keys():
            design_range[i] = raw.configs.base.design_range
        else:
            design_range[i] = 0
        if 'ems' in raw.configs.base.propulsors.network.keys():
            EMS[i] = raw.configs.base.propulsors.network.ems
        else:
            EMS[i] = 0
        if 'battery' in raw.configs.base.systems.keys():
            bat_mass[i] = raw.configs.base.systems.battery.mass_properties.mass
            bat_max_energy_eol[i] = raw.configs.base.propulsors.network.battery.max_energy  # this is at eol
            bat_max_power[i] = raw.configs.base.propulsors.network.battery.max_power
            bat_e_s_cell[i] = raw.configs.base.propulsors.network.battery.specific_energy_cell
            bat_e_s_pack[i] = raw.configs.base.propulsors.network.battery.specific_energy
            bat_p_s_pack[i] = raw.configs.base.propulsors.network.battery.specific_power

            bat_cell_to_pack_factor[i] = raw.configs.base.propulsors.network.battery.cell_to_pack_factor
            bat_durability_target_in_months[i] = raw.configs.base.propulsors.network.battery.durability_target_in_months
            bat_durability_in_months[i] = raw.configs.base.propulsors.network.battery.durability_in_months
            bat_technology_factor[i] = raw.configs.base.propulsors.network.battery.battery_technology_factor

            bat_cell_mass[i] = raw.configs.base.propulsors.network.battery.cell_mass
            bat_cell_max_voltage[i] = raw.configs.base.propulsors.network.battery.cell_max_voltage
            bat_cell_min_voltage[i] = raw.configs.base.propulsors.network.battery.cell_min_voltage
            bat_cell_nominal_voltage[i] = raw.configs.base.propulsors.network.battery.cell_nominal_voltage
            bat_cell_nominal_energy[i] = raw.configs.base.propulsors.network.battery.cell_nominal_energy
            bat_cell_initial_temperature[i] = raw.configs.base.propulsors.network.battery.cell_initial_temperature

            bat_initial_soc[i] = raw.configs.base.propulsors.network.battery.pack_initial_soc
            bat_initial_soh[i] = raw.configs.base.propulsors.network.battery.pack_initial_soh
            bat_pack_nominal_voltage[i] = raw.configs.base.propulsors.network.battery.pack_nominal_voltage
            bat_min_soc[i] = raw.configs.base.propulsors.network.battery.pack_min_soc
            bat_max_soc[i] = raw.configs.base.propulsors.network.battery.pack_max_soc

            bat_eol_soh[i] = raw.configs.base.propulsors.network.battery.eol_soh
            bat_number_branches[i] = raw.configs.base.propulsors.network.battery.number_branches
            bat_number_cells_series[i] = raw.configs.base.propulsors.network.battery.number_cells_series

            if None in [raw.configs.base.propulsors.network.battery.cell_nominal_energy,
                        raw.configs.base.propulsors.network.battery.number_branches,
                        raw.configs.base.propulsors.network.battery.number_cells_series]:
                pass
            else:
                bat_max_energy_bol[i] = raw.configs.base.propulsors.network.battery.cell_nominal_energy * \
                                        raw.configs.base.propulsors.network.battery.number_branches * \
                                        raw.configs.base.propulsors.network.battery.number_cells_series

            emotor_wtp_specific_power_factor[i] = raw.configs.base.propulsors.network.emotorWTP.specific_power_factor
            emotor_specific_power_factor[i] = raw.configs.base.propulsors.network.emotor.specific_power_factor

            emotor_rpm[i] = raw.configs.base.propulsors.network.emotor.nominal_rpm
            bus_voltage[i] = raw.configs.base.propulsors.network.dcdc.bus_voltage

            propeller_wtp_power_loading[i] = raw.configs.base.propulsors.network.propellerWTP.power_loading
            propeller_power_loading[i] = raw.configs.base.propulsors.network.propeller.power_loading

            tms_vcs = raw.configs.base.propulsors.network.tms_vcs
            tms_vcs_tag[i] = tms_vcs.tag
            tms_vcs_design_point[i] = tms_vcs.design_point
            tms_vcs_specific_power_factor[i] = tms_vcs.specific_power_factor
            tms_vcs_drag_factor[i] = tms_vcs.drag_factor

            if tms_vcs.tag == 'CASE1' or tms_vcs.tag == 'CASE1_v2':
                tms_vcs_cross_flow_area[i] = tms_vcs.cross_flow_area

            if tms_vcs.tag == 'CASE1_v2':
                tms_vcs_inlet_area[i] = tms_vcs.inlet_area
                tms_vcs_exhaust_area[i] = tms_vcs.exhaust_area

            if tms_vcs.tag == 'CASE2':
                tms_vcs_shx_area[i] = tms_vcs.shin_hx_area

            tms_liquid = raw.configs.base.propulsors.network.tms_liquid
            tms_liquid_tag[i] = tms_liquid.tag
            tms_liquid_design_point[i] = tms_liquid.design_point
            tms_liquid_specific_power_factor[i] = tms_liquid.specific_power_factor
            tms_liquid_drag_factor[i] = tms_liquid.drag_factor

            if tms_liquid.tag == 'CASE4' or tms_liquid.tag == 'CASE4_v2':
                tms_liquid_cross_flow_area[i] = tms_liquid.cross_flow_area

            if tms_liquid.tag == 'CASE4_v2':
                tms_liquid_inlet_area[i] = tms_liquid.inlet_area
                tms_liquid_exhaust_area[i] = tms_liquid.exhaust_area

            if tms_liquid.tag == 'CASE6':
                tms_liquid_shx_area[i] = tms_liquid.shin_hx_area
                tms_liquid_naca_area[i] = tms_liquid.naca_area
        else:
            bat_mass[i] = 0
            bat_max_energy_eol[i] = 0
            bat_max_power[i] = 0
            bat_e_s_cell[i] = 0
            bat_e_s_pack[i] = 0
            bat_p_s_pack[i] = 0

            bat_cell_to_pack_factor[i] = 0
            bat_durability_target_in_months[i] = 0
            bat_durability_in_months[i] = 0
            bat_technology_factor[i] = 0

            bat_cell_mass[i] = 0
            bat_cell_max_voltage[i] = 0
            bat_cell_min_voltage[i] = 0
            bat_cell_min_voltage[i] = 0
            bat_cell_nominal_voltage[i] = 0
            bat_cell_nominal_energy[i] = 0
            bat_cell_initial_temperature[i] = 0

            bat_initial_soc[i] = 0
            bat_initial_soh[i] = 0
            bat_pack_nominal_voltage[i] = 0
            bat_min_soc[i] = 0
            bat_max_soc[i] = 0

            bat_eol_soh[i] = 0
            bat_number_branches[i] = 0
            bat_number_cells_series[i] = 0

            bat_max_energy_bol[i] = 0

            emotor_wtp_specific_power_factor[i] = 0
            emotor_specific_power_factor[i] = 0

            emotor_rpm[i] = 0
            bus_voltage[i] = 0

            propeller_wtp_power_loading[i] = 0
            propeller_power_loading[i] = 0

            tms_vcs_tag[i] = 0
            tms_vcs_design_point[i] = 0
            tms_vcs_specific_power_factor[i] = 0
            tms_vcs_drag_factor[i] = 0
            tms_vcs_cross_flow_area[i] = 0
            tms_vcs_inlet_area[i] = 0
            tms_vcs_exhaust_area[i] = 0
            tms_vcs_shx_area[i] = 0

            tms_liquid_tag[i] = 0
            tms_liquid_design_point[i] = 0
            tms_liquid_specific_power_factor[i] = 0
            tms_liquid_drag_factor[i] = 0
            tms_liquid_cross_flow_area[i] = 0
            tms_liquid_inlet_area[i] = 0
            tms_liquid_exhaust_area[i] = 0
            tms_liquid_shx_area[i] = 0
            tms_liquid_naca_area[i] = 0


        if 'emissions_index_factor' in raw.configs.base.propulsors.network.turboshaft.keys():
            turboshaft_emissions_index_factor[i] = raw.configs.base.propulsors.network.turboshaft.emissions_index_factor
        else:
            turboshaft_emissions_index_factor[i] = 1

        if raw.configs.base.propulsors.network.turboshaft.input_file[-1] == 'p': #pickle
            turboshaft_surrogate[i] = raw.configs.base.propulsors.network.turboshaft.input_file[-11:-2]
        elif raw.configs.base.propulsors.network.turboshaft.input_file[-1] == 'v': #csv
            turboshaft_surrogate[i] = raw.configs.base.propulsors.network.turboshaft.input_file[-13:-4]

        if 'number_of_engines_for_oei' in raw.configs.base.propulsors.network.keys():
            number_of_engines_for_oei[i] = raw.configs.base.propulsors.network.number_of_engines_for_oei
        else:
            number_of_engines_for_oei[i] = 2

        if 'fuselage_mass_factor' in raw.configs.base.fuselages.fuselage.keys():
            fuselage_mass_factor[i] = raw.configs.base.fuselages.fuselage.mass_factor
        else:
            fuselage_mass_factor[i] = 1

        for j, key in enumerate(components):
            components_masses[i, j] = raw.configs.base.systems[key].mass_properties.mass

        sfom_nox[i] = raw.fom.sfom_nox
        sfom_co2[i] = raw.fom.sfom_co2
        sfom_noise[i] = raw.fom.sfom_noise
        sfom_doc[i] = raw.fom.sfom_doc
        sfom_dev[i] = raw.fom.sfom_dev
        sfom_cer[i] = raw.fom.sfom_cer
        sfom_prod[i] = raw.fom.sfom_prod
        fom_ei[i] = raw.fom.fom_ei
        fom_adi[i] = raw.fom.fom_adi
        fom_heai[i] = raw.fom.fom_heai
        fom[i] = raw.fom.fom
        fom_conv[i] = raw.fom.fom_conv
        fom_env[i] = raw.fom.fom_env

        weighting_factors_EI[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI[0]
        weighting_factors_ADI[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI[1]
        weighting_factors_HEAI[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI[2]
        weighting_factors_EI_conv[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI_conv[0]
        weighting_factors_ADI_conv[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI_conv[1]
        weighting_factors_HEAI_conv[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI_conv[2]
        weighting_factors_EI_env[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI_env[0]
        weighting_factors_ADI_env[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI_env[1]
        weighting_factors_HEAI_env[i] = raw.fom.settings.weighting_factors_EI_ADI_HEAI_env[2]
        weighting_factors_co2[i] = raw.fom.settings.weighting_factors_co2_nox_noise[0]
        weighting_factors_nox[i] = raw.fom.settings.weighting_factors_co2_nox_noise[1]
        weighting_factors_noise[i] = raw.fom.settings.weighting_factors_co2_nox_noise[2]
        weighting_factors_dev[i] = raw.fom.settings.weighting_factors_dev_cer_prod[0]
        weighting_factors_cer[i] = raw.fom.settings.weighting_factors_dev_cer_prod[1]
        weighting_factors_prod[i] = raw.fom.settings.weighting_factors_dev_cer_prod[2]
        weighting_factors_sideline[i] = raw.fom.settings.weighting_factors_sideline_flyover_approach[0]
        weighting_factors_flyover[i] = raw.fom.settings.weighting_factors_sideline_flyover_approach[1]
        weighting_factors_approach[i] = raw.fom.settings.weighting_factors_sideline_flyover_approach[2]

        #########################################
        # here come the special missions
        #########################################
        if 'special_missions' in raw.results:
            if 'climb_2_FL170' in raw.results.special_missions.keys():
                time_2_FL170[i] = raw.results.special_missions.climb_2_FL170.segments[-1].conditions.frames.inertial.time[-1] - \
                                  raw.results.special_missions.climb_2_FL170.segments[0].conditions.frames.inertial.time[0]
            else:
                time_2_FL170[i] = np.nan

            if 'ISA_SL_RoC' in raw.results.special_missions.keys():
                ISA_SL_RoC_m_s[i] = (raw.results.special_missions.ISA_SL_RoC.segments[0].altitude_end -
                                     raw.results.special_missions.ISA_SL_RoC.segments[0].altitude_start) / \
                                    (raw.results.special_missions.ISA_SL_RoC.segments[0].conditions.frames.inertial.time[-1] -
                                     raw.results.special_missions.ISA_SL_RoC.segments[0].conditions.frames.inertial.time[0])
                ISA_SL_RoC_ft_min[i] = ISA_SL_RoC_m_s[i] * 60 / Units.ft
            else:
                ISA_SL_RoC_m_s[i] = np.nan
                ISA_SL_RoC_ft_min[i] = np.nan
        else:
            pass

        ##########################################
        # here come the main missions
        ##########################################

        for mission in raw.results.main_missions.keys():
            if mission not in main_missions_keys:
                main_missions_keys.append(mission)

                fuel_total = np.hstack((fuel_total, np.zeros((total_files, 1))))
                fuel_main_mission = np.hstack((fuel_main_mission, np.zeros((total_files, 1))))
                fuel_cruise = np.hstack((fuel_cruise, np.zeros((total_files, 1))))

                co2_total = np.hstack((co2_total, np.zeros((total_files, 1))))
                nox_total = np.hstack((nox_total, np.zeros((total_files, 1))))
                co_total = np.hstack((co_total, np.zeros((total_files, 1))))

                co2_main_mission = np.hstack((co2_main_mission, np.zeros((total_files, 1))))
                nox_main_mission = np.hstack((nox_main_mission, np.zeros((total_files, 1))))
                co_main_mission = np.hstack((co_main_mission, np.zeros((total_files, 1))))

                co2_cruise = np.hstack((co2_cruise, np.zeros((total_files, 1))))
                nox_cruise = np.hstack((nox_cruise, np.zeros((total_files, 1))))
                co_cruise = np.hstack((co_cruise, np.zeros((total_files, 1))))

                cruise_speed = np.hstack((cruise_speed, np.zeros((total_files, 1))))
                cruise_altitude = np.hstack((cruise_altitude, np.zeros((total_files, 1))))
                cruise_mean_l_over_d = np.hstack((cruise_mean_l_over_d, np.zeros((total_files, 1))))

                mean_delta_cl_climb = np.hstack((mean_delta_cl_climb, np.zeros((total_files, 1))))
                mean_delta_cl_cruise = np.hstack((mean_delta_cl_cruise, np.zeros((total_files, 1))))
                mean_delta_cd_climb = np.hstack((mean_delta_cd_climb, np.zeros((total_files, 1))))
                mean_delta_cd_cruise = np.hstack((mean_delta_cd_cruise, np.zeros((total_files, 1))))

                tom = np.hstack((tom, np.zeros((total_files, 1))))
                payload = np.hstack((payload, np.zeros((total_files, 1))))

                range_total = np.hstack((range_total, np.zeros((total_files, 1))))
                range_main_mission = np.hstack((range_main_mission, np.zeros((total_files, 1))))
                range_cruise = np.hstack((range_cruise, np.zeros((total_files, 1))))

                soc_end_of_mission = np.hstack((soc_end_of_mission, np.zeros((total_files, 1))))
                battery_total = np.hstack((battery_total, np.zeros((total_files, 1))))
                battery_main_mission = np.hstack((battery_main_mission, np.zeros((total_files, 1))))
                battery_cruise = np.hstack((battery_cruise, np.zeros((total_files, 1))))

            j = main_missions_keys.index(mission)

            results = raw.results.main_missions[mission]
            # get last segment tag from main mission
            segments_keys = results.segments.keys()
            for n in range(len(segments_keys)):
                segment_key = segments_keys[-1 - n]
                if 'reserve' not in segment_key and segment_key != 'hold':
                    last_segment_main_mission = segment_key
                    break

            climb_segments = [key for key in results.segments.keys() if
                              (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key))]

            fuel_total[i, j] = results.segments[0].conditions.weights.total_mass[0][0] - \
                               results.segments[-1].conditions.weights.total_mass[-1][0]
            fuel_main_mission[i, j] = results.segments[0].conditions.weights.total_mass[0][0] - \
                                      results.segments[last_segment_main_mission].conditions.weights.total_mass[-1][0]
            fuel_cruise[i, j] = results.segments['cruise'].conditions.weights.total_mass[0][0] - \
                          results.segments['cruise'].conditions.weights.total_mass[-1][0]

            if raw.configs.base.propulsors.network.turboshaft.p3t3_method:
                co2_total[i, j] = results.segments[-1].conditions.propulsion.co2_emissions_total[-1, 0]
                nox_total[i, j] = results.segments[-1].conditions.propulsion.nox_emissions_total[-1, 0]
                co_total[i, j] = results.segments[-1].conditions.propulsion.co_emissions_total[-1, 0]
                co2_main_mission[i, j] = results.segments[last_segment_main_mission].conditions.propulsion.co2_emissions_total[-1, 0]
                nox_main_mission[i, j] = results.segments[last_segment_main_mission].conditions.propulsion.nox_emissions_total[-1, 0]
                co_main_mission[i, j] = results.segments[last_segment_main_mission].conditions.propulsion.co_emissions_total[-1, 0]
                co2_cruise[i, j] = results.segments.cruise.conditions.propulsion.co2_emissions_total[-1, 0] - \
                                   results.segments.cruise.conditions.propulsion.co2_emissions_total[0, 0]
                nox_cruise[i, j] = results.segments.cruise.conditions.propulsion.nox_emissions_total[-1, 0] - \
                                   results.segments.cruise.conditions.propulsion.nox_emissions_total[0, 0]
                co_cruise[i, j] = results.segments.cruise.conditions.propulsion.co_emissions_total[-1, 0] - \
                                   results.segments.cruise.conditions.propulsion.co_emissions_total[0, 0]

            cruise_mean_l_over_d[i, j] = (np.mean(results.segments.cruise.conditions.aerodynamics.lift_coefficient) /
                                       np.mean(results.segments.cruise.conditions.aerodynamics.drag_coefficient))

            delta_cl = np.array([])
            delta_cd = np.array([])
            for segment in climb_segments:
                delta_cl = np.append(delta_cl, results.segments[segment].conditions.aerodynamics.propeller_wing_interaction_delta_lift)
                delta_cd = np.append(delta_cd, results.segments[segment].conditions.aerodynamics.drag_breakdown.propeller_wing_interaction_delta_drag)

            mean_delta_cl_climb[i, j] = np.mean(delta_cl)
            mean_delta_cd_climb[i, j] = np.mean(delta_cd)
            mean_delta_cl_cruise[i, j] = np.mean(results.segments.cruise.conditions.aerodynamics.propeller_wing_interaction_delta_lift)
            mean_delta_cd_cruise[i, j] = np.mean(results.segments.cruise.conditions.aerodynamics.drag_breakdown.propeller_wing_interaction_delta_drag)
            cruise_speed[i, j] = results.segments.cruise.air_speed
            cruise_altitude[i, j] = results.segments.cruise.conditions.freestream.altitude[0][0]
            tom[i, j] = results.segments[0].analyses.weights.mass_properties.takeoff
            payload[i, j] = results.segments[0].analyses.weights.mass_properties.payload
            range_total[i, j] = results.segments[-1].conditions.frames.inertial.position_vector[-1][0] - \
                                results.segments[0].conditions.frames.inertial.position_vector[0][0]
            range_main_mission[i, j] = results.segments[last_segment_main_mission].conditions.frames.inertial.position_vector[-1][0] - \
                                       results.segments[0].conditions.frames.inertial.position_vector[0][0]
            range_cruise[i, j] = results.segments['cruise'].conditions.frames.inertial.position_vector[-1][0] - \
                                 results.segments['cruise'].conditions.frames.inertial.position_vector[0][0]
            if 'state_of_charge' in results.segments[-1].conditions.propulsion.keys():
                soc_end_of_mission[i, j] = results.segments[-1].conditions.propulsion.state_of_charge[-1][0]

                battery_total[i, j] = results.segments[0].conditions.propulsion.battery_energy[0][0] - \
                                      results.segments[-1].conditions.propulsion.battery_energy[-1][0]
                battery_main_mission[i, j] = results.segments[0].conditions.propulsion.battery_energy[0][0] - \
                                             results.segments[last_segment_main_mission].conditions.propulsion.battery_energy[-1][0]
                battery_cruise[i, j] = results.segments['cruise'].conditions.propulsion.battery_energy[0][0] - \
                                       results.segments['cruise'].conditions.propulsion.battery_energy[-1][0]
            else:
                soc_end_of_mission[i, j] = 0
                battery_total[i, j] = 0
                battery_main_mission[i, j] = 0
                battery_cruise[i, j] = 0

    # create a dictionary to create a pandas.DataFrame from the data grabbed
    all_data = {}
    all_data['file_name'] = raw_files
    all_data['mtom'] = mtom
    all_data['ome'] = ome
    all_data['max_zero_fuel_mass'] = max_zero_fuel_mass
    all_data['max_fuel_mass'] = max_fuel_mass
    all_data['max_payload'] = max_payload
    all_data['operationals_mass'] = operationals_mass
    all_data['propulsion_mass'] = propulsion_mass
    all_data['systems_mass'] = systems_mass
    all_data['payload_design_mission'] = payload_design_mission
    all_data['max_payload'] = max_payload

    for j, key in enumerate(components):
        all_data[f'{key}_mass'] = components_masses[:, j]

    all_data['wing_area'] = wing_area
    all_data['wing_span'] = wing_span
    all_data['wing_aspect_ratio'] = wing_aspect_ratio
    all_data['wing_loading'] = wing_loading
    all_data['power_loading'] = power_loading
    all_data['hybridization_power'] = hybridization_power
    all_data['generator_oversizing'] = generator_oversizing
    all_data['design_cruise_speed'] = design_cruise_speed
    all_data['design_cruise_altitude'] = design_cruise_altitude
    all_data['design_range'] = design_range
    all_data['EMS'] = EMS
    all_data['bat_mass'] = bat_mass
    all_data['bat_max_energy_bol'] = bat_max_energy_bol
    all_data['bat_max_energy_eol'] = bat_max_energy_eol
    all_data['bat_max_power'] = bat_max_power
    all_data['bat_e_s_cell'] = bat_e_s_cell
    all_data['bat_e_s_pack'] = bat_e_s_pack
    all_data['bat_p_s_pack'] = bat_p_s_pack
    all_data['bat_cell_to_pack_factor'] = bat_cell_to_pack_factor
    all_data['bat_durability_target_in_months'] = bat_durability_target_in_months
    all_data['bat_durability_in_months'] = bat_durability_in_months
    all_data['bat_technology_factor'] = bat_technology_factor
    all_data['bat_cell_mass'] = bat_cell_mass
    all_data['bat_cell_max_voltage'] = bat_cell_max_voltage
    all_data['bat_cell_min_voltage '] = bat_cell_min_voltage
    all_data['bat_cell_nominal_voltage'] = bat_cell_nominal_voltage
    all_data['bat_cell_nominal_energy'] = bat_cell_nominal_energy
    all_data['bat_cell_initial_temperature'] = bat_cell_initial_temperature
    all_data['bat_initial_soc'] = bat_initial_soc
    all_data['bat_initial_soh'] = bat_initial_soh
    all_data['bat_pack_nominal_voltage'] = bat_pack_nominal_voltage
    all_data['bat_min_soc'] = bat_min_soc
    all_data['bat_max_soc'] = bat_max_soc
    all_data['bat_eol_soh'] = bat_eol_soh
    all_data['bat_number_branches'] = bat_number_branches
    all_data['bat_number_cells_series'] = bat_number_cells_series
    all_data['emotor_wtp_specific_power_factor'] = emotor_wtp_specific_power_factor
    all_data['emotor_specific_power_factor'] = emotor_specific_power_factor
    all_data['emotor_rpm'] = emotor_rpm
    all_data['bus_voltage'] = bus_voltage
    all_data['propeller_wtp_power_loading'] = propeller_wtp_power_loading
    all_data['propeller_power_loading'] = propeller_power_loading
    all_data['tms_vcs_tag'] = tms_vcs_tag
    all_data['tms_vcs_design_point'] = tms_vcs_design_point
    all_data['tms_vcs_specific_power_factor'] = tms_vcs_specific_power_factor
    all_data['tms_vcs_drag_factor'] = tms_vcs_drag_factor
    all_data['tms_vcs_cross_flow_area'] = tms_vcs_cross_flow_area
    all_data['tms_vcs_inlet_area'] = tms_vcs_inlet_area
    all_data['tms_vcs_exhaust_area'] = tms_vcs_exhaust_area
    all_data['tms_vcs_shx_area'] = tms_vcs_shx_area
    all_data['tms_liquid_tag'] = tms_liquid_tag
    all_data['tms_liquid_design_point'] = tms_liquid_design_point
    all_data['tms_liquid_specific_power_factor'] = tms_liquid_specific_power_factor
    all_data['tms_liquid_drag_factor'] = tms_liquid_drag_factor
    all_data['tms_liquid_cross_flow_area'] = tms_liquid_cross_flow_area
    all_data['tms_liquid_inlet_area'] = tms_liquid_inlet_area
    all_data['tms_liquid_exhaust_area'] = tms_liquid_exhaust_area
    all_data['tms_liquid_shx_area'] = tms_liquid_shx_area
    all_data['tms_liquid_naca_area'] = tms_liquid_naca_area
    all_data['turboshaft_emissions_index_factor'] = turboshaft_emissions_index_factor

    all_data['turboshaft_surrogate'] = turboshaft_surrogate

    all_data['number_of_engines_for_oei'] = number_of_engines_for_oei
    all_data['fuselage_mass_factor'] = fuselage_mass_factor
    all_data['sfom_nox'] = sfom_nox
    all_data['sfom_co2'] = sfom_co2
    all_data['sfom_noise'] = sfom_noise
    all_data['sfom_doc'] = sfom_doc
    all_data['sfom_dev'] = sfom_dev
    all_data['sfom_cer'] = sfom_cer
    all_data['sfom_prod'] = sfom_prod
    all_data['fom_ei'] = fom_ei
    all_data['fom_adi'] = fom_adi
    all_data['fom_heai'] = fom_heai
    all_data['fom'] = fom
    all_data['fom_conv'] = fom_conv
    all_data['fom_env'] = fom_env
    all_data['weighting_factors_EI'] = weighting_factors_EI
    all_data['weighting_factors_ADI'] = weighting_factors_ADI
    all_data['weighting_factors_HEAI'] = weighting_factors_HEAI
    all_data['weighting_factors_EI_conv'] = weighting_factors_EI_conv
    all_data['weighting_factors_ADI_conv'] = weighting_factors_ADI_conv
    all_data['weighting_factors_HEAI_conv'] = weighting_factors_HEAI_conv
    all_data['weighting_factors_EI_env'] = weighting_factors_EI_env
    all_data['weighting_factors_ADI_env'] = weighting_factors_ADI_env
    all_data['weighting_factors_HEAI_env'] = weighting_factors_HEAI_env
    all_data['weighting_factors_co2'] = weighting_factors_co2
    all_data['weighting_factors_nox'] = weighting_factors_nox
    all_data['weighting_factors_noise'] = weighting_factors_noise
    all_data['weighting_factors_dev'] = weighting_factors_dev
    all_data['weighting_factors_cer'] = weighting_factors_cer
    all_data['weighting_factors_prod'] = weighting_factors_prod
    all_data['weighting_factors_sideline'] = weighting_factors_sideline
    all_data['weighting_factors_flyover'] = weighting_factors_flyover
    all_data['weighting_factors_approach'] = weighting_factors_approach
    all_data['time_2_FL170'] = time_2_FL170
    all_data['ISA_SL_RoC_m_s'] = ISA_SL_RoC_m_s
    all_data['ISA_SL_RoC_ft_min'] = ISA_SL_RoC_ft_min

    for i, key in enumerate(main_missions_keys):
        all_data[f'{key}-fuel_total'] = fuel_total[:, i]
        all_data[f'{key}-fuel_main_mission'] = fuel_main_mission[:, i]
        all_data[f'{key}-fuel_cruise'] = fuel_cruise[:, i]
        all_data[f'{key}-co2_total'] = co2_total[:, i]
        all_data[f'{key}-nox_total'] = nox_total[:, i]
        all_data[f'{key}-co_total'] = co_total[:, i]
        all_data[f'{key}-co2_main_mission'] = co2_main_mission[:, i]
        all_data[f'{key}-nox_main_mission'] = nox_main_mission[:, i]
        all_data[f'{key}-co_main_mission'] = co_main_mission[:, i]
        all_data[f'{key}-co2_cruise'] = co2_cruise[:, i]
        all_data[f'{key}-nox_cruise'] = nox_cruise[:, i]
        all_data[f'{key}-co_cruise'] = co_cruise[:, i]
        all_data[f'{key}-cruise_speed'] = cruise_speed[:, i]
        all_data[f'{key}-cruise_altitude'] = cruise_altitude[:, i]
        all_data[f'{key}-tom'] = tom[:, i]
        all_data[f'{key}-payload'] = payload[:, i]
        all_data[f'{key}-range_total'] = range_total[:, i]
        all_data[f'{key}-range_main_mission'] = range_main_mission[:, i]
        all_data[f'{key}-range_cruise'] = range_cruise[:, i]
        all_data[f'{key}-soc_end_of_mission'] = soc_end_of_mission[:, i]
        all_data[f'{key}-battery_total'] = battery_total[:, i]
        all_data[f'{key}-battery_main_mission'] = battery_main_mission[:, i]
        all_data[f'{key}-battery_cruise'] = battery_cruise[:, i]
        all_data[f'{key}-mean_l_over_d_cruise'] = cruise_mean_l_over_d[:, i]
        all_data[f'{key}-mean_delta_cl_climb'] = mean_delta_cl_climb[:, i]
        all_data[f'{key}-mean_delta_cd_climb'] = mean_delta_cd_climb[:, i]
        all_data[f'{key}-mean_delta_cl_cruise'] = mean_delta_cl_cruise[:, i]
        all_data[f'{key}-mean_delta_cd_cruise'] = mean_delta_cd_cruise[:, i]
        all_data[f'{key}-esar_total'] = (battery_total[:, i] + fuel_total[:, i] * jet_a1_specific_energy) / range_total[:, i]
        all_data[f'{key}-esar_main_mission'] = (battery_main_mission[:, i] + fuel_main_mission[:, i] * jet_a1_specific_energy) / range_main_mission[:, i]
        all_data[f'{key}-esar_cruise'] = (battery_cruise[:, i] + fuel_cruise[:, i] * jet_a1_specific_energy) / range_cruise[:, i]

    dataframe = pd.DataFrame(all_data)
    # specify file name and save the .csv-file afterwards
    filename = folder_path + '/' + '_data_extracted_' + datetime.now().strftime("%Y%m%d_%H%M%S") + '.csv'
    dataframe.to_csv(filename)
