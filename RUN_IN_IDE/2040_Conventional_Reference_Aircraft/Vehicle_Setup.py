## Conventional_Reference_Aircraft.py
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
#           Jul 2022 F.Brenner

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

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
import os
from SUAVE.Methods.Geometry.Two_Dimensional.Planform.wing_planform import wing_planform
from SUAVE.Methods.Propulsion import propeller_design
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_segmented_planform, create_tapered_wing, horizontal_tail_planform_raymer, vertical_tail_planform_raymer, fuselage_planform
from SUAVE.Methods.Geometry.Two_Dimensional.Planform import wing_fuel_volume


# ----------------------------------------------------------------------
#   Define the Vehicle
# ----------------------------------------------------------------------

def vehicle_setup(iteration_setup):
    # ------------------------------------------------------------------
    #   Initialize the Vehicle
    # ------------------------------------------------------------------

    vehicle = SUAVE.Vehicle()
    vehicle.tag = 'CRA2040_reference_aircraft'

    vehicle.systems.accessories = "None" #"short-range"
    vehicle.systems.control = "fully powered"
    vehicle.systems.apu = "None"

    # ------------------------------------------------------------------
    #   Vehicle-level Properties
    # ------------------------------------------------------------------

    # mass properties
    vehicle.mass_properties.max_takeoff = iteration_setup.weight_iter.TOW #18600. * Units.kg
    vehicle.mass_properties.takeoff = iteration_setup.weight_iter.TOW
    vehicle.mass_properties.operating_empty = iteration_setup.weight_iter.BOW
    vehicle.mass_properties.cargo = 0.0 * Units.kg
    vehicle.mass_properties.max_payload = 5800 * Units.kg #5300.0 * Units.kg
    vehicle.mass_properties.max_fuel = 3700 * Units.kg #4100
    vehicle.mass_properties.payload = iteration_setup.weight_iter.Design_Payload
    vehicle.mass_properties.max_zero_fuel = iteration_setup.weight_iter.BOW + vehicle.mass_properties.max_payload

    # envelope properties
    vehicle.envelope.ultimate_load = 2.5 * 1.5
    vehicle.envelope.limit_load = 2.5
    vehicle.maximum_mach_operational = 0.55

    # basic parameters
    vehicle.wing_loading = 341.3 # 341.096644#342.9836 # airbus#boeing341.284404
    #vehicle.reference_area = 54.5 * Units['meters**2']
    vehicle.reference_area =  float(vehicle.mass_properties.max_takeoff / vehicle.wing_loading)#54.5 * Units['meters**2']
    vehicle.passengers = 50 #48 57
    # vehicle.systems.control = "aerodynamic powered"
    # vehicle.systems.accessories = "short-range"

    vehicle.design_mach_number = 0.48
    vehicle.design_range = iteration_setup.mission_iter.mission_distance # 800 * Units.km # 840 * Units.nautical_miles

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
    wing.tag = 'main_wing'
    wing.origin = iteration_setup.sizing_iter.wing_origin

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'section_1'
    segment.percent_span_location = 0.0
    segment.twist = 3. * Units.deg
    segment.root_chord_percent = 1.
    segment.dihedral_outboard = 0. * Units.degrees
    segment.sweeps.quarter_chord = 0.0 * Units.degrees
    segment.thickness_to_chord = 0.18

    wing.Segments.append(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'section_2'
    segment.percent_span_location = 0.39476#0.32
    segment.twist = 2. * Units.deg
    segment.root_chord_percent = 1
    segment.dihedral_outboard = 0. * Units.degrees
    segment.sweeps.quarter_chord = 3 * Units.degrees #0.519
    segment.thickness_to_chord = 0.155

    wing.Segments.append(segment)

    segment = SUAVE.Components.Wings.Segment()
    segment.tag = 'section_3'
    segment.percent_span_location = 1
    segment.twist = 0. * Units.deg
    segment.root_chord_percent = 0.54863 #0.72
    segment.dihedral_outboard = 0.0 * Units.degrees
    segment.sweeps.quarter_chord = 0.0 * Units.degrees
    segment.thickness_to_chord = 0.13
    wing.Segments.append(segment)

    del segment

    create_tapered_wing(wing, 13.5, vehicle.reference_area * Units['m**2'], span_limit=36 * Units.meter)
    wing_segmented_planform(wing,True)


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
    wing.flap_ratio = wing_flap.chord_fraction * (wing_flap.span_fraction_end - wing_flap.span_fraction_start) # 0.32 * 0.58


    wing_fuel_volume(wing, eta_tank = 0.31863)

    wing.max_camber = 0.02806
     # add to vehicle
    vehicle.append_component(wing)

    vehicle.mass_properties.max_fuel = vehicle.wings.main_wing.fuel_volume * 780 # 3700 * Units.kg #4100
    # ------------------------------------------------------------------
    #  Fuselage
    # ------------------------------------------------------------------

    fuselage = SUAVE.Components.Fuselages.Fuselage()
    fuselage.tag = 'fuselage'


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

    wing.c_ht = 1.065#0.9 # twin turboprop

    wing.origin = [[0, 0, 5.]]

    wing.origin[0][0] = vehicle.fuselages.fuselage.lengths.nose + vehicle.fuselages.fuselage.lengths.cabin + 0.705 * vehicle.fuselages.fuselage.lengths.tail #0.635


    wing.aerodynamic_center[0] = 1
    deltaaerocenterhtp= 2

    while abs(deltaaerocenterhtp) > 1e-10:
        oldaerocenter = wing.aerodynamic_center[0]

        wing.l_ht = wing.origin[0][0] + wing.aerodynamic_center[0]  - (vehicle.wings.main_wing.origin[0][0] +
                                                                       vehicle.wings.main_wing.aerodynamic_center[0])

        horizontal_tail_planform_raymer(wing, vehicle.wings.main_wing, wing.l_ht, wing.c_ht)

        newaerocenter = wing.aerodynamic_center[0]
        deltaaerocenterhtp = oldaerocenter - newaerocenter
    #wing.aerodynamic_center[0] = wing.origin[0][0]

    if wing.origin[0][0] + wing.chords.root > vehicle.fuselages.fuselage.lengths.total*0.999:
        print('HTP Root longer than Fuselage')

    # add to vehicle
    vehicle.append_component(wing)

    # ------------------------------------------------------------------
    #   Vertical Stabilizer
    # ------------------------------------------------------------------

    wing = SUAVE.Components.Wings.Vertical_Tail()
    wing.tag = 'vertical_stabilizer'


    # Geometry Dependencies
    # Calculated from 3-view drawing:
    # Area 15.515, height 4.534, c_root 4.63 c_tip 2.2 AS 1.325 taper 0.475

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

    wing.c_vt = 0.1249#0.08 # twin turboprop

    wing.origin = [[0, 0, 1.1]] #vehicle.wings.main_wing.origin
    wing.origin[0][0] = vehicle.fuselages.fuselage.lengths.nose + vehicle.fuselages.fuselage.lengths.cabin + 0.412 * vehicle.fuselages.fuselage.lengths.tail #0.334

    deltaaerocentervtp= 2

    while abs(deltaaerocentervtp) > 1e-10:
        oldaerocenter = wing.aerodynamic_center[0]

        wing.l_vt = wing.origin[0][0] + wing.aerodynamic_center[0]  - (vehicle.wings.main_wing.origin[0][0] +
                                                                       vehicle.wings.main_wing.aerodynamic_center[0])
        vertical_tail_planform_raymer(wing, vehicle.wings.main_wing, wing.l_vt, wing.c_vt)
        newaerocenter = wing.aerodynamic_center[0]
        deltaaerocentervtp = oldaerocenter - newaerocenter

    if wing.origin[0][0] + wing.chords.root > vehicle.fuselages.fuselage.lengths.total*0.98:
        print('VTP Root longer than Fuselage')

    # add to vehicle
    vehicle.append_component(wing)


    # ------------------------------------------------------------------
    # Prop System
    # ------------------------------------------------------------------

    net = SUAVE.Components.Energy.Networks.Propulsor_Turboshaft_Surrogate()
    net.tag = 'network'
    power_loading = 1790*2*1e3/18600
    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()
    # working fluid
    net.working_fluid = SUAVE.Attributes.Gases.Air()
    net.power_loading_calculated = power_loading

    # ------------------------------------------------------------------
    #  Turboshaft
    # ------------------------------------------------------------------

    # Setup
    net.number_of_engines = 2
    net.nacelle_diameter = 1.15 * Units.meter
    net.engine_length = 3.5 * Units.meter
    net.thrust_angle = 0.0 * Units.deg

    # compute engine areas
    net.areas = Data()
    net.areas.wetted = 2.0 * np.pi * net.nacelle_diameter * net.engine_length

    # ------------------------------------------------------------------
    #  Engine Turboshaft Surrogate
    # ------------------------------------------------------------------

    turboshaft = SUAVE.Components.Energy.Converters.Turboshaft_Surrogate()

    turboshaft.specific_power = (1790/412*1000)/1.2 * Units.W/Units.kg
    turboshaft.turboshaft_length = 1.572 * Units.m  # just turboshaft
    turboshaft.origin = [[7.2, 4.45, 0],[7.2, -4.45, 0]]# cg-2322 mm

    turboshaft.nominal_rpm_low_pressure_turbine = 28000 #rpm

    turboshaft.power_scaling_factor = iteration_setup.sizing_iter.factor_engine_power
    turboshaft.sea_level_power = power_loading/net.number_of_engines * vehicle.mass_properties.max_takeoff*turboshaft.power_scaling_factor

    turboshaft.sfc_scaling_factor = 1 #1.35 #1.557145 #ATR 2005 mission 1.35 means a ~ 10 % improvement in cruise SFC for the FutPrInt mission
    turboshaft.off_takes = 0.08

    turboshaft.emissions_icao = Data()
    turboshaft.emissions_icao.throttle = np.array([0.07, 0.30, 0.78, 0.80, 0.90, 1])
    turboshaft.emissions_icao.EI_nox_sea_level = np.array([6.9, 9.8, 15.6, 16.2, 16.5, 17.7])
    turboshaft.emissions_icao.EI_co_sea_level = np.array([9.2, 3.7, 2.2, 2, 2, 2])

    turboshaft.p3t3_method                 = True
    turboshaft.deltaisa_method             = True
    turboshaft.gas_turbine_use_pickle = False

    file_name = 'CRA-Feb2023.csv' #PW127-MAX_TET_1385K_Delta_ISA0_low.csv #PW127-MAX_TET_1385K.csv
    turboshaft.surrogate_type = 'LinearNDI'#'Linear' #LinearNDI #
    turboshaft.input_file = os.path.join(SUAVE.__path__[0], 'Data_Files/Gas_Turbine/', file_name)
    turboshaft.use_extended_surrogate = False
    turboshaft.interpolate_nominal_power = False
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

    # ------------------------------------------------------------------
    #  Gear Box
    # ------------------------------------------------------------------

    # instantiate
    gearbox = SUAVE.Components.Energy.Converters.Gearbox()
    gearbox.tag = 'gear_box'
    gearbox.origin = [[7.29, 4.45, 0],[7.29, -4.45, 0]]
    gearbox.efficiency = 0.985

    # setup
    net.gearbox = gearbox
    # add  gas turbine network turboprop to the vehicle


    # ------------------------------------------------------------------
    #  Propeller
    # ------------------------------------------------------------------

    #propeller = SUAVE.Components.Energy.Converters.Delft_Propeller_Lo_Fid() #Higher Fid-Level
    propeller = SUAVE.Components.Energy.Converters.Propeller_Lo_Fid() #Low Fid-Level
    propeller.tag = 'propeller'
    propeller.origin = [[7.015, 4.45, 0],[7.015, -4.45, 0]]
    propeller.thrust_calibration = 1
    propeller.tip_radius = 2.2 * Units.meter
    propeller.propulsive_efficiency = 0.85
    propeller.number_of_blades = 6
    #propeller.nominal_rpm = 1000
    propeller.tip_mach_number = 0.8
    altitude = iteration_setup.mission_iter.cruise_altitude * Units.ft
    temperature_deviation = 0
    atmo_data = atmosphere.compute_values(altitude, temperature_deviation)
    a = atmo_data.speed_of_sound[0][0]
    v = propeller.tip_mach_number * a

    # propeller.nominal_rpm = 1200 * Units.rpm #TODO# please check if rpm with unit!!!!!!!!!!!!!!!!!t und auch noch wie der zusammenhang zwischen power loading und drehzahl ist
    propeller.nominal_rpm = v / (np.pi * propeller.tip_radius * 2) * 60

    # add to network
    net.propeller = propeller


    vehicle.append_component(net)

    # ------------------------------------------------------------------
    #   Vehicle Definition Complete
    # ------------------------------------------------------------------

    return vehicle


# if __name__ == '__main__':
#     iteration_setup = Data()
#     iteration_setup.weight_iter = Data()
#     iteration_setup.weight_iter.TOW = 18600. * Units.kg
#     iteration_setup.weight_iter.BOW = 12500. * Units.kg
#    # iteration_setup.weight_iter.Design_Payload = 5300. * Units.kg
#
#     vehicle = vehicle_setup(iteration_setup)
#     a = 'abc'

