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
import os
import scipy.io as sio
try:
    import cPickle as pickle
except ImportError:
    import pickle

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_segmented_planform, create_tapered_wing, horizontal_tail_planform_raymer, vertical_tail_planform_raymer, fuselage_planform
from SUAVE.Methods.Power.Battery.Ragone.find_ragone_optimum import find_ragone_optimum
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_fuel_volume

# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup(iteration_setup, parameters):
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------


    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'Hybrid CoG'

    vehicle.systems.accessories = "None"
    vehicle.systems.control = "fully powered"
    vehicle.systems.apu = "None"

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff = iteration_setup.weight_iter.TOW #18600. * Units.kg
    vehicle.mass_properties.takeoff = iteration_setup.weight_iter.TOW
    vehicle.mass_properties.operating_empty = iteration_setup.weight_iter.BOW
    #vehicle.mass_properties.max_zero_fuel = 16700. * Units.kg
    vehicle.mass_properties.cargo = 0.0 * Units.kg
    vehicle.mass_properties.max_payload = iteration_setup.weight_iter.Max_Payload
    #vehicle.mass_properties.max_fuel = iteration_setup.weight_iter.Max_Fuel
    vehicle.mass_properties.payload = iteration_setup.weight_iter.Design_Payload
    vehicle.mass_properties.max_zero_fuel = iteration_setup.weight_iter.BOW + vehicle.mass_properties.max_payload

    # envelope properties
    vehicle.envelope.ultimate_load = 2.5 * 1.5
    vehicle.envelope.limit_load = 2.5
    vehicle.maximum_mach_operational = 0.55

    # basic parameters
    vehicle.wing_loading = parameters.wing_loading#341.096644#342.9836 # airbus#boeing341.284404
    #vehicle.reference_area = 54.5 * Units['meters**2']
    vehicle.reference_area =  vehicle.mass_properties.max_takeoff / vehicle.wing_loading#54.5 * Units['meters**2']
    vehicle.passengers = 50
    # vehicle.systems.control = "aerodynamic powered"
    # vehicle.systems.accessories = "short-range"

    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    altitude = iteration_setup.mission_iter.cruise_altitude * Units.ft
    temperature_deviation = 0
    atmo_data = atmosphere.compute_values(altitude, temperature_deviation)

    vehicle.design_mach_number = iteration_setup.mission_iter.cruise_speed / atmo_data.speed_of_sound[0][0]
    vehicle.design_range = iteration_setup.mission_iter.mission_distance
    vehicle.design_cruise_altitude = altitude
    vehicle.design_cruise_speed = iteration_setup.mission_iter.cruise_speed

    landing_gear = SUAVE.Components.Landing_Gear.Landing_Gear()

    landing_gear.main_strut_length = 0.64 * Units.m
    landing_gear.main_wheels = 4.
    landing_gear.nose_strut_length = 0.64
    landing_gear.nose_wheels = 2.
    landing_gear.main_tire_diameter = 0.8 * Units.meter
    landing_gear.nose_tire_diameter = 0.45 * Units.meter
    landing_gear.main_units = 2.

    vehicle.landing_gear = landing_gear

    # ------------------------------------------------------------------
    #   Main Wing
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Main_Wing()
    #wing.tag = 'tapered_main_wing'
    wing.tag = 'main_wing'

    #wing.origin = [[13.61, 0, -0.93]]
    #wing.origin = [[11.0946, 0, 0]]
    #wing.origin = [[10, 0, 0]]
    wing.origin = iteration_setup.sizing_iter.wing_origin

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'section_1'
    segment.percent_span_location = 0.0
    segment.twist = 3 * Units.deg
    segment.root_chord_percent = 1.
    segment.dihedral_outboard = 0. * Units.degrees
    segment.sweeps.quarter_chord = 0.0 * Units.degrees
    segment.thickness_to_chord = 0.18
    wing.Segments.append(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'section_2'
    segment.percent_span_location = 0.39476#0.32
    segment.twist = 2 * Units.deg
    segment.root_chord_percent = 1
    segment.dihedral_outboard = 0. * Units.degrees
    segment.sweeps.quarter_chord = 2 * Units.degrees #0.519
    segment.thickness_to_chord = 0.155
    wing.Segments.append(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'section_3'
    segment.percent_span_location = 1
    segment.twist = 0 * Units.deg
    segment.root_chord_percent = 0.54863 #0.72
    segment.dihedral_outboard = 0.0 * Units.degrees
    segment.sweeps.quarter_chord = 0.0 * Units.degrees
    segment.thickness_to_chord = 0.13
    wing.Segments.append(segment)

    del segment

    create_tapered_wing(wing, parameters.wing_aspect_ratio, vehicle.reference_area * Units['m**2'], span_limit=36 * Units.meter)
    wing_segmented_planform(wing,True)

    if wing.spans.projected > 35.99:
        print('Span: %.1f m' % wing.spans.projected)

    # Settings
    wing.vertical = False
    wing.symmetric = True
    wing.high_lift = True

    # High Lift Devices
    wing_flap = Data()
    wing_flap.tag = 'flap'
    wing_flap.type = "single_slotted"
    wing_flap.chord_fraction = 0.32
    wing_flap.span_fraction_start = 0.10
    wing_flap.span_fraction_end = 0.68
    wing.append_control_surface(wing_flap)
    wing_slat = Data()
    wing_slat.tag = 'slat'
    wing.append_control_surface(wing_slat)
    wing.dynamic_pressure_ratio = 1.0
    wing.flap_ratio = 0.32 * 0.58

    wing_fuel_volume(wing, eta_tank = 0.31863*0.5)

    wing.max_camber = 0.02806
    # add to vehicle
    vehicle.append_component(wing)

    vehicle.mass_properties.max_fuel = vehicle.wings.main_wing.fuel_volume * 780
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'

    fuselage.mass_factor = parameters.fuselage_mass_factor

    # Geometry Dependencies
    fuselage.number_coach_seats = vehicle.passengers
    fuselage.seat_pitch = 0.762
    fuselage.seats_abreast = 4
    fuselage.fineness.nose = 1.23 #2
    fuselage.fineness.tail = 2.93
    fuselage.lengths.fore_space = 2.0 * Units.meter
    fuselage.lengths.aft_space = 0. * Units.meter
    fuselage.width = 2.77 * Units.meter
    fuselage.heights.maximum = 3.020 * Units.meter
    fuselage.lengths.total = 0 #22.67 * Units.meter

    # Stability analysis
    fuselage.areas.side_projected = 46.6 * Units['meters**2']
    fuselage.heights.at_quarter_length = 3.02 * Units.meter
    fuselage.heights.at_three_quarters_length = 3.02 * Units.meter
    fuselage.heights.at_wing_root_quarter_chord = 3.02 * Units.meter

    #fuselage.cm_alpha_method = "k2-k1" second method for cm_alpha(fuselage)

    # Use?
    fuselage.differential_pressure = 6.00 * Units.psi

    fuselage_planform(fuselage)

    #belly fairing
    fuselage.n_belly = 1  # Aircraft has an additional belly fairing (1). Aircraft has no additional belly fairing (0)
    fuselage.size_belly = 0.15 # percentage of belly fairing wetted area to fuselage wetted area

    # add to vehicle
    vehicle.append_component(fuselage)


    # ------------------------------------------------------------------
    #  Horizontal Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Horizontal_Tail()
    wing.tag = 'horizontal_stabilizer'

    # Geometry Dependencies
    wing.taper = 0.6
    wing.sweeps.quarter_chord = 7 * Units.deg #11.31
    wing.aspect_ratio = 4.56
    wing.thickness_to_chord = 0.13
    wing.dihedral = 0 * Units.deg
    wing.vertical = False
    wing.symmetric = True

    wing.dynamic_pressure_ratio = 1#0.9
    wing.twists.root = 2.0 * Units.degrees
    wing.twists.tip = 2.0 * Units.degrees

    # Sizing
    wing.c_ht = 1.12#0.9 # twin turboprop
    wing.origin = [[0, 0, 5.]]

    wing.origin[0][0] = vehicle.fuselages.fuselage.lengths.nose + vehicle.fuselages.fuselage.lengths.cabin + 0.601 * vehicle.fuselages.fuselage.lengths.tail #0.64

    #wing.aerodynamic_center[0] = 1
    deltaaerocenterhtp= 2

    while abs(deltaaerocenterhtp) > 1e-10:
        oldaerocenter = wing.aerodynamic_center[0]

        wing.l_ht = wing.origin[0][0] + wing.aerodynamic_center[0]  - (vehicle.wings.main_wing.origin[0][0] +
                                                                       vehicle.wings.main_wing.aerodynamic_center[0])

        horizontal_tail_planform_raymer(wing, vehicle.wings.main_wing, wing.l_ht, wing.c_ht)

        newaerocenter = wing.aerodynamic_center[0]
        deltaaerocenterhtp = oldaerocenter - newaerocenter

    #if wing.origin[0][0] + wing.chords.root > vehicle.fuselages.fuselage.lengths.total*0.999:
        #print('HTP Root longer than Fuselage')

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'

    wing.taper = 0.475
    wing.sweeps.quarter_chord = 35 * Units.deg #35
    wing.aspect_ratio = 1.325 #1.3
    wing.thickness_to_chord = 0.12
    wing.dihedral = 0 * Units.deg
    wing.vertical = True
    wing.symmetric = False
    wing.t_tail = True

    wing.dynamic_pressure_ratio = 1.0
    wing.twists.root = 0.0 * Units.degrees
    wing.twists.tip = 0.0 * Units.degrees

    wing.c_vt = 0.122#0.08 # twin turboprop

    wing.origin = [[0, 0, 1.1]]
    wing.origin[0][0] = vehicle.fuselages.fuselage.lengths.nose + vehicle.fuselages.fuselage.lengths.cabin + 0.3 * vehicle.fuselages.fuselage.lengths.tail #0.48 #0.16 0.40

    deltaaerocentervtp= 2

    while abs(deltaaerocentervtp) > 1e-10:
        oldaerocenter = wing.aerodynamic_center[0]

        wing.l_vt = wing.origin[0][0] + wing.aerodynamic_center[0]  - (vehicle.wings.main_wing.origin[0][0] +
                                                                       vehicle.wings.main_wing.aerodynamic_center[0])
        vertical_tail_planform_raymer(wing, vehicle.wings.main_wing, wing.l_vt, wing.c_vt)
        newaerocenter = wing.aerodynamic_center[0]
        deltaaerocentervtp = oldaerocenter - newaerocenter

    #if wing.origin[0][0] + wing.chords.root > vehicle.fuselages.fuselage.lengths.total*0.98:
        #print('VTP Root longer than Fuselage')

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    # Propulsor --> Battery Propeller try
    # ------------------------------------------------------------------

    # Constant Values
    power_loading = iteration_setup.sizing_iter.power_loading * iteration_setup.sizing_iter.factor_engine_power * iteration_setup.sizing_iter.factor_power_loading_sizing_chart#190
    hybridization_power = parameters.hybridization_power#0.1
    voltage = parameters.bus_voltage

    # Network
    net = SUAVE.Components.Energy.Networks.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus()
    net.tag = 'network'

    net.hybridization_power = hybridization_power
    net.generator_oversizing = parameters.generator_oversizing
    net.ems = parameters.ems
    net.power_loading_calculated = power_loading

    net.number_of_engines_for_oei = parameters.number_of_engines_for_oei

    net.number_of_engines = 2.
    net.nacelle_diameter = 1.15 * Units.meters #network - do not use as physical component
    net.engine_length = 3.5 * Units.meters #network - do not use as physical component
    net.areas = Data() #network - do not use as physical component
    net.areas.wetted = 2.0 * np.pi * net.nacelle_diameter * net.engine_length #network - do not use as physical component

    net.number_of_WTP = 2

    net.electric_throttle = 1.0 #why is it here

    # Engine Surrogate
    #net.off_takes = 0.0425
    #net.sfc_scaling_factor = 1
    #net.power_scaling_factor = iteration_setup.sizing_iter.factor_engine_power




    # ------------------------------------------------------------------
    #  Engine Turboshaft Surrogate
    # ------------------------------------------------------------------

    turboshaft = SUAVE.Components.Energy.Converters.Turboshaft_Surrogate()

    turboshaft.specific_power = (1790 / 412 * 1000) / 1.2 * Units.W / Units.kg
    turboshaft.origin = [[7.0, 4.05, 0], [7.0, -4.05, 0]]

    turboshaft.nominal_rpm_low_pressure_turbine = 28000 #rpm

    #turboshaft.power_scaling_factor = iteration_setup.sizing_iter.factor_engine_power

    if net.ems == 10:
        turboshaft_sea_level_power = power_loading * (1 - hybridization_power/2) / net.number_of_engines * vehicle.mass_properties.max_takeoff
    elif net.ems == 10:
        turboshaft_sea_level_power = power_loading * (1 - 0) / net.number_of_engines * vehicle.mass_properties.max_takeoff
    else:
        turboshaft_sea_level_power = power_loading * (1 - hybridization_power) / net.number_of_engines * vehicle.mass_properties.max_takeoff

    turboshaft.sea_level_power = turboshaft_sea_level_power
    turboshaft.sfc_scaling_factor = 1 #1.35 #1.557145 #ATR 2005 mission 1.35 means a ~ 10 % improvement in cruise SFC for the FutPrInt mission
    turboshaft.off_takes = 0.08

    turboshaft.emissions_index_factor = parameters.turboshaft_emissions_index_factor

    turboshaft.emissions_icao = Data()
    turboshaft.emissions_icao.throttle = np.array([0.07, 0.30, 0.78, 0.80, 0.90, 1])
    turboshaft.emissions_icao.EI_nox_sea_level = np.array([6.9, 9.8, 15.6, 16.2, 16.5, 17.7]) * turboshaft.emissions_index_factor
    turboshaft.emissions_icao.EI_co_sea_level = np.array([9.2, 3.7, 2.2, 2, 2, 2]) * turboshaft.emissions_index_factor

#    file_name = 'HEA141610.csv' #TET = 1400 OPR = 16 1MW
#    file_name = 'HEA141615.csv' #TET = 1400 OPR = 16 1.5MW
#    file_name = 'HEA141620.csv' #TET = 1400 OPR = 16 2MW
#    file_name = 'HEA141625.csv' #TET = 1400 OPR = 16 2.5MW
#    file_name = 'HEA161610.csv' #TET = 1600 OPR = 16 1MW
#    file_name = 'HEA161615.csv' #TET = 1600 OPR = 16 1.5MW
#    file_name = 'HEA161620.csv' #TET = 1600 OPR = 16 2MW
#    file_name = 'HEA161625.csv' #TET = 1600 OPR = 16 2.5MW
#    file_name = 'HEA162010.csv' #TET = 1600 OPR = 20 1MW
#    file_name = 'HEA162015.csv' #TET = 1600 OPR = 20 1.5MW
#    file_name = 'HEA162020.csv' #TET = 1600 OPR = 20 2MW
#    file_name = 'HEA162025.csv' #TET = 1600 OPR = 20 2.5MW

    turboshaft.p3t3_method = True
    turboshaft.deltaisa_method = True
    turboshaft.interpolate_nominal_power = False
    turboshaft.surrogate_type = 'LinearNDI'  # LinearNDI #linear
    turboshaft.gas_turbine_use_pickle = parameters.gas_turbine_use_pickle

    if turboshaft.gas_turbine_use_pickle and turboshaft.surrogate_type == 'LinearNDI':
        # the pickled files use LinearNDI; p3t3, interpolate_nominal_power and deltaisa are turned on (True)
        # if settings prior differ from that, these settings will be overwritten by build_surrogate
        file_name = f'HEA{parameters.gas_turbine_TET}{parameters.gas_turbine_OPR}.p'
    else:
        # if the csv files are used, surrogate_type p3t3, deltaisa and interpolate might be changed
        file_name = f'HEA{parameters.gas_turbine_TET}{parameters.gas_turbine_OPR}.csv'

    # file_name = 'HEA141620.csv'
    file_name = 'HEA162020.csv'

    turboshaft.input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/Gas_Turbine', file_name)
    turboshaft.use_extended_surrogate = True
    turboshaft.build_surrogate()

    if turboshaft.p3t3_method == True:
        throttles = turboshaft.emissions_icao.throttle
        if turboshaft.interpolate_nominal_power:
            cons = np.zeros([len(throttles), 5])
            power = turboshaft.sea_level_power / turboshaft.nominal_power_input_scale
            if power > turboshaft.nominal_power_min_max[1]:
                power = turboshaft.nominal_power_min_max[1]
            elif power < turboshaft.nominal_power_min_max[0]:
                power = turboshaft.nominal_power_min_max[0]
            cons[:, 4] = power
        elif turboshaft.deltaisa_method:
            cons = np.zeros([len(throttles), 4])
        else:
            cons = np.zeros([len(throttles), 3])

        cons[:, 2] = throttles

        if turboshaft.surrogate_type == 'LinearNDI':
            turboshaft.emissions_icao.p3_sea_level_icao = turboshaft.p3_surrogate(cons).flatten() * turboshaft.p3_input_scale
            turboshaft.emissions_icao.t3_sea_level_icao = turboshaft.t3_surrogate(cons).flatten() * turboshaft.t3_input_scale
            turboshaft.emissions_icao.far_sl_icao = turboshaft.far_surrogate(cons).flatten() * turboshaft.far_input_scale

        else:
            turboshaft.emissions_icao.p3_sea_level_icao = turboshaft.p3_surrogate.predict(cons).flatten() * turboshaft.p3_input_scale
            turboshaft.emissions_icao.t3_sea_level_icao = turboshaft.t3_surrogate.predict(cons).flatten() * turboshaft.t3_input_scale
            turboshaft.emissions_icao.far_sl_icao = turboshaft.far_surrogate.predict(cons).flatten() * turboshaft.far_input_scale

    # add to network
    net.turboshaft = turboshaft
    net.sea_level_power = net.turboshaft.sea_level_power
    # print(f'Sea Level Power: {net.turboshaft.sea_level_power:.2f} W')

    # ------------------------------------------------------------------
    #  Gear Box
    # ------------------------------------------------------------------
    # instantiate
    gearbox = SUAVE.Components.Energy.Converters.Gearbox()
    gearbox.tag = 'gear_box'
    gearbox.origin = [[7.2, 4.05, 0],[7.2, -4.05, 0]]
    gearbox.efficiency = 0.985
    # setup
    net.gearbox = gearbox


    # ------------------------------------------------------------------
    #  Gear Box WTP
    # ------------------------------------------------------------------
    # instantiate
    gearboxWTP = SUAVE.Components.Energy.Converters.Gearbox()
    gearboxWTP.tag = 'gear_box_WTP'
    gearboxWTP.origin = [[7.0, vehicle.wings.main_wing.spans.projected/2, 0],[7.0, -vehicle.wings.main_wing.spans.projected/2, 0]]
    gearboxWTP.efficiency = 0.985
    # setup
    net.gearboxWTP = gearboxWTP


    # Electric Motor - Mid Fid
    emotor = SUAVE.Components.Energy.Converters.Motor_Mid_Fid_eta()
    emotor.rated_power = power_loading * (hybridization_power + parameters.generator_oversizing) / 4 * vehicle.mass_properties.max_takeoff
    emotor.rated_voltage = voltage
    emotor.nominal_rpm = parameters.electric_motor_nominal_rpm #rpm
    emotor.origin = [[7.0, 4.05, 0], [7.0, -4.05, 0]]

    if emotor.nominal_rpm >= 5000:
        file_name = 'electric_motor_high_11.p'
    elif emotor.nominal_rpm <= 1200:
        file_name = 'electric_motor_low_11.p'
    else:
        print('Motor rpm parameter fail')
    emotor.input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/Electric_Motor', file_name)
    emotor.build_surrogate()
    emotor.specific_power_factor = parameters.electric_motor_specific_power_factor
    emotor.specific_power = emotor.specific_power_factor * emotor.specific_power_surrogate([emotor.rated_power/emotor.power_nom_input_scale, emotor.rated_voltage/emotor.voltage_nom_input_scale, emotor.nominal_rpm/emotor.rpm_nom_input_scale])[0][0]

    # add to network
    net.emotor = emotor

    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()

    # Propeller
    propeller = SUAVE.Components.Energy.Converters.Delft_Propeller_Lo_Fid()
    propeller.tag = 'propeller'
    propeller.power_loading = parameters.propeller_power_loading
    propeller.tip_radius = ((net.turboshaft.sea_level_power + net.emotor.rated_power)/(np.pi * propeller.power_loading)) ** 0.5
    #propeller.propulsive_efficiency = 0.85
    propeller.number_of_blades = 6
    propeller.thrust_calibration = 1.
    propeller.origin = [[7.0, 4.05, 0],[7.0, -4.05, 0]]

    propeller.tip_mach_number = 0.72
    altitude = iteration_setup.mission_iter.cruise_altitude * Units.ft
    temperature_deviation = 0
    atmo_data =  atmosphere.compute_values(altitude, temperature_deviation)
    a = atmo_data.speed_of_sound[0][0]
    v = propeller.tip_mach_number * a

    #propeller.nominal_rpm = 1200 * Units.rpm #TODO# please check if rpm with unit!!!!!!!!!!!!!!!!!t und auch noch wie der zusammenhang zwischen power loading und drehzahl ist
    propeller.nominal_rpm = v / (np.pi * propeller.tip_radius * 2) * 60


    # changed matrix: coeff_new
    file_name = 'coeff_new.csv'
    input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/Aero', file_name)
    propeller.coeff_new = np.genfromtxt(input_file, delimiter=',')

    # add to network
    net.propeller = propeller

    # Electronic_Speed_Controller_eta_Inverter_Mid_Fid
    esc = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller_eta_Inverter_Mid_Fid()
    esc.specific_power = 15 * Units.kW / Units.kg
    esc.origin = net.emotor.origin

    file_name = 'DCAC_Inverter_fixed_eff.csv'
    esc.input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/DCAC_Inverter', file_name)
    esc.build_surrogate()

    # add to network
    net.esc = esc




    # Battery
    bat = SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion()
    bat.max_voltage = voltage
    bat.origin = vehicle.wings.main_wing.origin
    bat.mass_properties.center_of_gravity[0][0] = 0
    bat.ragone.upper_bound = 3000 *Units.Wh/Units.kg
    bat.ragone.const_1     = 88.818  *Units.kW/Units.kg
    bat.ragone.const_2     = -.01533 /(Units.Wh/Units.kg)
    # bat.battery_technology_factor = parameters.battery_technology_factor * iteration_setup.energy_iter.battery_mass_factor_cea
    bat.battery_technology_factor = parameters.battery_technology_factor / parameters.battery_cell_to_pack_factor * iteration_setup.energy_iter.battery_mass_factor_cea
    find_ragone_optimum(bat, iteration_setup.energy_iter.battery_energy_used, iteration_setup.energy_iter.battery_power_mission_max)
    bat.internal_resistance_cell = 30 * Units.mohm
    bat.resistance = 50 * Units.mohm #iteration_setup.sizing_iter.battery_resistance --> not working
    # bat.specific_energy = bat.specific_energy * parameters.battery_technology_factor
    # bat.specific_power = bat.specific_power * parameters.battery_technology_factor
    bat.origin = [[9.0, 0, 0]]
    bat.durability_target_in_months = parameters.battery_durability_target #
    bat.durability_in_months = iteration_setup.sizing_iter.battery_durability_in_months
    bat.soc_max = parameters.battery_soc_max
    bat.soc_min = parameters.battery_soc_min

    bat.cell_mass = iteration_setup.sizing_iter.battery_cell_mass
    bat.cell_max_voltage = iteration_setup.sizing_iter.battery_cell_max_voltage
    bat.cell_min_voltage = iteration_setup.sizing_iter.battery_cell_min_voltage
    bat.cell_nominal_voltage = iteration_setup.sizing_iter.battery_cell_nominal_voltage
    bat.cell_nominal_energy = iteration_setup.sizing_iter.battery_cell_nominal_energy
    bat.cell_initial_temperature = iteration_setup.sizing_iter.battery_cell_initial_temperature

    bat.pack_initial_soc = iteration_setup.sizing_iter.battery_pack_initial_soc
    bat.pack_initial_soh = iteration_setup.sizing_iter.battery_pack_initial_soh
    bat.pack_nominal_voltage = iteration_setup.sizing_iter.battery_pack_nominal_voltage
    bat.pack_min_soc = iteration_setup.sizing_iter.battery_pack_min_soc
    bat.pack_max_soc = iteration_setup.sizing_iter.battery_pack_max_soc

    bat.eol_soh = iteration_setup.sizing_iter.battery_eol_soh
    bat.number_branches = iteration_setup.sizing_iter.battery_number_branches
    bat.number_cells_series = iteration_setup.sizing_iter.battery_number_cells_series

    bat.cell_to_pack_factor = parameters.battery_cell_to_pack_factor

    bat.specific_energy_cell = parameters.battery_specific_energy

    # add to network
    net.battery = bat
    net.voltage = bat.max_voltage

    #print('Battery Mass: %.1f' % bat.mass_properties.mass)
    #print('Battery Spec Energy: %.1f Wh/kg' % (bat.specific_energy / (Units.Wh / Units.kg)))
    #print('Battery Spex Power: %.1f W/kg' % (bat.specific_power / (Units.W / Units.kg)))

    # WTP Electric Motor Mid Fid
    emotorWTP = SUAVE.Components.Energy.Converters.Motor_Mid_Fid_eta()
    emotorWTP.rated_power = power_loading * (hybridization_power - parameters.generator_oversizing) / 4 * vehicle.mass_properties.max_takeoff
    emotorWTP.rated_voltage = voltage
    emotorWTP.origin = [[7.0, vehicle.wings.main_wing.spans.projected / 2, 0], [7.0, -vehicle.wings.main_wing.spans.projected / 2, 0]]
    emotorWTP.nominal_rpm = parameters.electric_motor_nominal_rpm #rpm

    if emotorWTP.nominal_rpm >= 5000:
        file_name = 'electric_motor_high_21.p'
    elif emotorWTP.nominal_rpm <= 1200:
        file_name = 'electric_motor_low_11.p'
    else:
        print('Motor rpm parameter fail')
    emotorWTP.input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/Electric_Motor', file_name)
    emotorWTP.build_surrogate()

    emotorWTP.specific_power_factor = parameters.electric_motor_specific_power_factor
    emotorWTP.specific_power = emotorWTP.specific_power_factor * emotorWTP.specific_power_surrogate([emotorWTP.rated_power/emotorWTP.power_nom_input_scale, emotorWTP.rated_voltage/emotorWTP.voltage_nom_input_scale, emotorWTP.nominal_rpm/emotorWTP.rpm_nom_input_scale])[0][0]

    # add to network
    net.emotorWTP = emotorWTP


    # Propeller WTP
    propellerWTP = SUAVE.Components.Energy.Converters.Delft_Propeller_Lo_Fid()
    propellerWTP.tag = 'propellerWTP'
    propellerWTP.power_loading = parameters.propeller_power_loading
    propellerWTP.tip_radius =(net.emotorWTP.rated_power/(np.pi * propellerWTP.power_loading)) ** 0.5
    #propellerWTP.propulsive_efficiency = 0.85
    propellerWTP.thrust_calibration = 1.
    propellerWTP.number_of_blades = 6

    propellerWTP.tip_mach_number = 0.72
    altitude = iteration_setup.mission_iter.cruise_altitude * Units.ft
    temperature_deviation = 0
    atmo_data =  atmosphere.compute_values(altitude, temperature_deviation)
    a = atmo_data.speed_of_sound[0][0]
    v = propellerWTP.tip_mach_number * a

    #propellerWTP.nominal_rpm = 4500 * Units.rpm
    propellerWTP.nominal_rpm = v / (np.pi * propellerWTP.tip_radius * 2) * 60


    propellerWTP.origin = [[7.0, vehicle.wings.main_wing.spans.projected/2, 0],[7.0, -vehicle.wings.main_wing.spans.projected/2, 0]]
    # add to network
    net.propellerWTP = propellerWTP


    # Electronic_Speed_Controller_eta_Inverter_Mid_Fid WTP
    escWTP = SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller_eta_Inverter_Mid_Fid()
    escWTP.specific_power = 15 * Units.kW / Units.kg
    escWTP.origin = net.emotorWTP.origin

    file_name = 'DCAC_Inverter_fixed_eff.csv'
    escWTP.input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/DCAC_Inverter', file_name)
    escWTP.build_surrogate()

    # add to network
    net.escWTP = escWTP


    # BUS - DCDC Converter - Mid Fid - Bahareh
    dcdc = SUAVE.Components.Energy.Converters.DCDC_Converter_Mid_Fid()
    dcdc.bus_voltage = voltage
    dcdc.specific_power = 15 * Units.kW / Units.kg
    dcdc.origin = [[9.0, 0, 0]]

    file_name = 'DCDC_Converter_fixed_eff.csv'
    dcdc.input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/DCDC_Converter', file_name)
    dcdc.build_surrogate()

    # add to network
    net.dcdc = dcdc


    # Power Cable
    cable = SUAVE.Components.Energy.Distributors.Cable_Power()
    cable.origin = [[9.0, 0, 0]]
    # add to network
    net.cable = cable

    # # Thermal Management System
    # tms = SUAVE.Components.Energy.Thermal_Management_System.Thermal_Management_System()
    # tms.origin = [[9.0, 0, 0]]
    # tms.shx_area = 3 * Units.meters**2
    # tms.max_heat_load = iteration_setup.sizing_iter.heat_load_max #150 *Units.kW
    # # add to network
    # net.tms = tms

    # Thermal Management System - Case 1 or 2 --> Liquid and VCS
    tms_vcs = SUAVE.Components.Energy.Thermal_Management_System.TMS_Surrogates()
    tms_vcs.tag = parameters.tms_vcs_case_version #CASE1 #CASE1_v2 #CASE2
    tms_vcs.origin = [[9.0, 0, 0], [9.0, 0, 0]]
    tms_vcs.number_system = 2 #one in each wing
    tms_vcs.design_point = iteration_setup.sizing_iter.heat_load_max_vcs / Units.kW #150 *Units.kW

    tms_vcs.specific_power_factor = parameters.tms_specific_power_factor
    tms_vcs.drag_factor = parameters.tms_drag_factor

    tms_vcs.input_path = os.path.join(SUAVE.__path__[0], 'Data_Files/TMS')
    tms_vcs.build_surrogate()

    if tms_vcs.tag == 'CASE1' or tms_vcs.tag == 'CASE1_v2':
        tms_vcs.cross_flow_area = np.float(tms_vcs.interp_Area(tms_vcs.design_point))

    if tms_vcs.tag == 'CASE1_v2':
        tms_vcs.inlet_area = np.float(tms_vcs.interp_Area_inlet(tms_vcs.design_point))
        tms_vcs.exhaust_area = np.float(tms_vcs.interp_Area_exhaust(tms_vcs.design_point))

    if tms_vcs.tag == 'CASE2':
        tms_vcs.shin_hx_area = parameters.tms_vcs_case_2_shx_area

    # add to network
    net.tms_vcs = tms_vcs

    # Thermal Management System - Case 4 or 6 --> Liquid
    tms_liquid = SUAVE.Components.Energy.Thermal_Management_System.TMS_Surrogates()
    tms_liquid.tag = parameters.tms_liquid_case_version #CASE4 #CASE4_v2 #CASE6
    tms_liquid.origin = [[9.0, 0, 0], [9.0, 0, 0]]
    tms_liquid.number_system = 2 #one in each wing
    tms_liquid.design_point = iteration_setup.sizing_iter.heat_load_max_liquid / Units.kW #150 *Units.kW

    tms_liquid.specific_power_factor = parameters.tms_specific_power_factor
    tms_liquid.drag_factor = parameters.tms_drag_factor

    tms_liquid.input_path = os.path.join(SUAVE.__path__[0], 'Data_Files/TMS')
    tms_liquid.build_surrogate()

    if tms_liquid.tag == 'CASE4' or tms_liquid.tag == 'CASE4_v2':
        tms_liquid.cross_flow_area = np.float(tms_liquid.interp_Area(tms_liquid.design_point))

    if tms_liquid.tag == 'CASE4_v2':
        tms_liquid.inlet_area = np.float(tms_liquid.interp_Area_inlet(tms_liquid.design_point))
        tms_liquid.exhaust_area = np.float(tms_liquid.interp_Area_exhaust(tms_liquid.design_point))

    if tms_liquid.tag == 'CASE6':
        tms_liquid.shin_hx_area = parameters.tms_liquid_case6_shx_area
        tms_liquid.naca_area = np.float(tms_liquid.interp_Area_NACA((tms_liquid.design_point, tms_liquid.shin_hx_area)))

        #interp_Area_NACA
    # add to network
    net.tms_liquid = tms_liquid


    # final add to vehicle
    vehicle.append_component(net)





    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle

