## write_fact_sheet.py
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


from SUAVE.Core import Units
from SUAVE.Components import Physical_Component
import SUAVE


def write_fact_sheet(file_name, base):
    output = file_name

    ###################################
    # overall
    ###################################

    output += segment_heading('Overall')
    output += add_data('MTOM', base.mass_properties.max_takeoff, 'kg')
    output += add_data('OME', base.mass_properties.operating_empty, 'kg')
    output += add_data('Payload Design Mission', base.mass_properties.payload, 'kg')
    output += add_data('Fuel mass Design Mission', base.systems.fuel.mass_properties.mass, 'kg')
    output += add_data('Maximum Payload', base.mass_properties.max_payload, 'kg')

    ###################################
    # main wing
    ###################################

    main_wing = base.wings.main_wing

    output += segment_heading("Main Wing")
    output += data_heading("Areas")
    output += add_subdata("Reference Area", main_wing.areas.reference, "m^2")
    output += add_subdata("Wetted Area", main_wing.areas.wetted, "m^2")
    output += data_heading("Spans")
    output += add_subdata("Total", main_wing.spans.total, "m")
    output += add_subdata("Projected", main_wing.spans.total, "m")
    output += add_data("Aspect Ratio", main_wing.aspect_ratio, "-")
    output += add_data("Taper Ratio", main_wing.taper, "-")
    output += add_data("Dihedral", main_wing.dihedral, "rad")
    output += data_heading("Chords")
    output += add_subdata("Root Chord Length", main_wing.chords.root, "m")
    output += add_subdata("Mean Aerodynamic Chord", main_wing.chords.mean_aerodynamic, "m")
    output += add_subdata("Mean Geometric Chord", main_wing.chords.mean_geometric, "m")
    output += add_data("Mass", main_wing.mass_properties.mass, "kg")
    output += data_heading("Origin")
    output += add_subdata("x", main_wing.origin[0][0], "m")
    output += add_subdata("y", main_wing.origin[0][1], "m")
    output += add_subdata("z", main_wing.origin[0][2], "m")
    output += data_heading("Relative Center of Gravity")
    output += add_subdata("x", main_wing.mass_properties.center_of_gravity[0][0], "m")
    output += add_subdata("y", main_wing.mass_properties.center_of_gravity[0][1], "m")
    output += add_subdata("z", main_wing.mass_properties.center_of_gravity[0][2], "m")

    for i in range(len(base.wings.main_wing.Segments) - 1):
        segment = base.wings.main_wing.Segments[i]
        output += data_heading(segment.tag)
        output += add_subdata("Root Chord Percentage", segment.root_chord_percent, "-")
        output += add_subdata("Taper Ratio", segment.taper, "-")
        output += add_subdata("Dihedral", segment.dihedral_outboard, "rad")
        output += add_subdata("Percent Span Location", segment.percent_span_location, "-")
        output += add_subdata("Thickness to Chord", segment.thickness_to_chord, "-")
        output += add_subdata("Sweep Quarter Chord", segment.sweeps.quarter_chord, "rad")
        output += add_subdata("Twist", segment.twist, "rad")

    segment = base.wings.main_wing.Segments[-1]
    output += data_heading(segment.tag)
    output += add_subdata("Root Chord Percentage", segment.root_chord_percent, "-")
    output += add_subdata("Percent Span Location", segment.percent_span_location, "-")
    output += add_subdata("Thickness to Chord", segment.thickness_to_chord, "-")
    output += add_subdata("Sweep Quarter Chord", segment.sweeps.quarter_chord, "rad")
    output += add_subdata("Twist", segment.twist, "rad")

    ###################################
    # fuselage
    ###################################

    fuselage = base.fuselages.fuselage

    output += segment_heading("Fuselage")
    output += add_data("Length", fuselage.lengths.total, "m")
    output += add_data("Width", fuselage.width, "m")
    output += add_data("Height", fuselage.heights.maximum, "m")
    output += data_heading("Areas")
    output += add_subdata("Front Projected", fuselage.areas.front_projected, "m^2")
    output += add_subdata("Side Projected", fuselage.areas.side_projected, "m^2")
    output += add_subdata("Wetted", fuselage.areas.wetted, "m^2")
    output += add_data("Mass", fuselage.mass_properties.mass, "kg")
    output += data_heading("Origin")
    output += add_subdata("x", fuselage.origin[0][0], "m")
    output += add_subdata("y", fuselage.origin[0][1], "m")
    output += add_subdata("z", fuselage.origin[0][2], "m")
    output += data_heading("Relative Center of Gravity")
    output += add_subdata("x", fuselage.mass_properties.center_of_gravity[0][0], "m")
    output += add_subdata("y", fuselage.mass_properties.center_of_gravity[0][1], "m")
    output += add_subdata("z", fuselage.mass_properties.center_of_gravity[0][2], "m")
    output += data_heading("Cabin")
    output += add_subdata("Coach Seats", fuselage.number_coach_seats, "-")
    output += add_subdata("Seat Pitch", fuselage.seat_pitch, "m")
    output += add_subdata("Seat Abreast", fuselage.seats_abreast, "-")

    ###################################
    # horizontal tail
    ###################################

    horizontal_stabilizer = base.wings.horizontal_stabilizer

    output += segment_heading("Horizontal Tail")
    output += data_heading("Areas")
    output += add_subdata("Reference Area", horizontal_stabilizer.areas.reference, "m^2")
    output += add_subdata("Wetted Area", horizontal_stabilizer.areas.wetted, "m^2")
    output += data_heading("Spans")
    output += add_subdata("Total", horizontal_stabilizer.spans.total, "m")
    output += add_subdata("Projected", horizontal_stabilizer.spans.total, "m")
    output += add_data("Aspect Ratio", horizontal_stabilizer.aspect_ratio, "-")
    output += add_data("Taper Ratio", horizontal_stabilizer.taper, "-")
    output += add_data("Dihedral", horizontal_stabilizer.dihedral, "rad")
    output += data_heading("Chords")
    output += add_subdata("Root Chord Length", horizontal_stabilizer.chords.root, "m")
    output += add_subdata("Mean Aerodynamic Chord", horizontal_stabilizer.chords.mean_aerodynamic, "m")
    output += add_subdata("Mean Geometric Chord", horizontal_stabilizer.chords.mean_geometric, "m")
    output += add_data("Mass", horizontal_stabilizer.mass_properties.mass, "kg")
    output += data_heading("Origin")
    output += add_subdata("x", horizontal_stabilizer.origin[0][0], "m")
    output += add_subdata("y", horizontal_stabilizer.origin[0][1], "m")
    output += add_subdata("z", horizontal_stabilizer.origin[0][2], "m")
    output += data_heading("Relative Center of Gravity")
    output += add_subdata("x", horizontal_stabilizer.mass_properties.center_of_gravity[0][0], "m")
    output += add_subdata("y", horizontal_stabilizer.mass_properties.center_of_gravity[0][1], "m")
    output += add_subdata("z", horizontal_stabilizer.mass_properties.center_of_gravity[0][2], "m")

    ###################################
    # vertical tail
    ###################################

    vertical_stabilizer = base.wings.vertical_stabilizer

    output += segment_heading("Vertical Tail")
    output += data_heading("Areas")
    output += add_subdata("Reference Area", vertical_stabilizer.areas.reference, "m^2")
    output += add_subdata("Wetted Area", vertical_stabilizer.areas.wetted, "m^2")
    output += data_heading("Spans")
    output += add_subdata("Total", vertical_stabilizer.spans.total, "m")
    output += add_subdata("Projected", vertical_stabilizer.spans.total, "m")
    output += add_data("Aspect Ratio", vertical_stabilizer.aspect_ratio, "-")
    output += add_data("Taper Ratio", vertical_stabilizer.taper, "-")
    output += add_data("Dihedral", vertical_stabilizer.dihedral, "rad")
    output += data_heading("Chords")
    output += add_subdata("Root Chord Length", vertical_stabilizer.chords.root, "m")
    output += add_subdata("Mean Aerodynamic Chord", vertical_stabilizer.chords.mean_aerodynamic, "m")
    output += add_subdata("Mean Geometric Chord", vertical_stabilizer.chords.mean_geometric, "m")
    output += add_data("Mass", vertical_stabilizer.mass_properties.mass, "kg")
    output += data_heading("Origin")
    output += add_subdata("x", vertical_stabilizer.origin[0][0], "m")
    output += add_subdata("y", vertical_stabilizer.origin[0][1], "m")
    output += add_subdata("z", vertical_stabilizer.origin[0][2], "m")
    output += data_heading("Relative Center of Gravity")
    output += add_subdata("x", vertical_stabilizer.mass_properties.center_of_gravity[0][0], "m")
    output += add_subdata("y", vertical_stabilizer.mass_properties.center_of_gravity[0][1], "m")
    output += add_subdata("z", vertical_stabilizer.mass_properties.center_of_gravity[0][2], "m")

    ###################################
    # landing gear
    ###################################

    gear = base.landing_gear

    output += segment_heading("Landing Gear")

    output += segment_subheading("Main Gear")
    output += add_data("Mass", gear.main.mass_properties.mass, "kg")
    output += data_heading("Origin")
    output += add_subdata("x", gear.main.origin[0][0], "m")
    output += add_subdata("y", gear.main.origin[0][1], "m")
    output += add_subdata("z", gear.main.origin[0][2], "m")
    output += data_heading("Relative Center of Gravity")
    output += add_subdata("x", gear.main.mass_properties.center_of_gravity[0][0], "m")
    output += add_subdata("y", gear.main.mass_properties.center_of_gravity[0][1], "m")
    output += add_subdata("z", gear.main.mass_properties.center_of_gravity[0][2], "m")

    output += segment_subheading("Nose Gear")
    output += add_data("Mass", gear.nose.mass_properties.mass, "kg")
    output += data_heading("Origin")
    output += add_subdata("x", gear.nose.origin[0][0], "m")
    output += add_subdata("y", gear.nose.origin[0][1], "m")
    output += add_subdata("z", gear.nose.origin[0][2], "m")
    output += data_heading("Relative Center of Gravity")
    output += add_subdata("x", gear.nose.mass_properties.center_of_gravity[0][0], "m")
    output += add_subdata("y", gear.nose.mass_properties.center_of_gravity[0][1], "m")
    output += add_subdata("z", gear.nose.mass_properties.center_of_gravity[0][2], "m")

    ###################################
    # Powertrain
    ###################################

    network = base.propulsors.network
    systems = base.systems

    output += segment_heading("Powertrain")

    output += add_data("Max. Shaft power", network.sea_level_power, "W")
    output += add_data("Max. Fuel Storage total", base.mass_properties.max_fuel, "kg")
    output += add_data("Mass Propulsion", base.mass_properties.propulsion, "kg")

    if 'hybridization_power' in network:
        output += add_data('Hybridization of power', network.hybridization_power, '-')
        output += add_data('EMS', network.ems, '-')
    else:
        output += add_data('Hybridization of power', 0, '-')
        output += add_data('EMS', 0, '-')

    output += add_data('Power Loading', network.power_loading_calculated, 'W/kg')

    if 'generator_oversizing' in network:
        output += add_data('Generator oversizing', network.generator_oversizing, '-')

    physical_components = []

    for key in network.keys():
        item = network[key]
        if isinstance(item, Physical_Component):
            physical_components.append(key)

    for component in physical_components:
        if type(network[component]) == SUAVE.Components.Energy.Converters.DCDC_Converter_Mid_Fid:
            output += segment_subheading(component)
            output += add_subdata("Bus voltage", network[component].bus_voltage, "V")
            output += add_subdata("Specific power", network[component].specific_power, "W/kg")

        elif type(network[component]) == SUAVE.Components.Energy.Distributors.Cable_Power:
            output += segment_subheading(component)

        elif type(network[component]) == SUAVE.Components.Energy.Converters.Propeller_Lo_Fid:
            output += segment_subheading(component)
            output += add_subdata("Diameter", network[component].tip_radius * 2, "m")
            output += add_subdata("Design Rotational Speed", network[component].nominal_rpm / Units.rpm, "rpm")
            output += add_subdata("Number of Blades", network[component].number_of_blades, "-")

        elif type(network[component]) == SUAVE.Components.Energy.Converters.Delft_Propeller_Lo_Fid:
            output += segment_subheading(component)
            output += add_subdata("Tip Radius", network[component].tip_radius, 'm')
            output += add_subdata('Number of Blades', network[component].number_of_blades, '-')
            output += add_subdata('Tip mach number', network[component].tip_mach_number, '-')
            output += add_subdata('Nominal rpm', network[component].nominal_rpm, 'rpm')

        elif type(network[component]) == SUAVE.Components.Energy.Converters.Turboshaft_Surrogate:
            output += segment_subheading(component)
            output += add_subdata("Peak Shaft Power", network[component].sea_level_power, "W")

        elif type(network[component]) == SUAVE.Components.Energy.Converters.Gearbox:
            output += segment_subheading(component)
            output += add_subdata('Efficiency', network[component].efficiency, '-')

        elif type(network[component]) == SUAVE.Components.Energy.Converters.Motor_Mid_Fid_eta:
            output += segment_subheading(component)
            output += add_subdata("Rated Power", network[component].rated_power, "W")
            output += add_subdata("Rated Voltage", network[component].rated_voltage, "V")
            output += add_subdata("Nominal RPM", network[component].nominal_rpm, "rpm")
            output += add_subdata("Specific Power", network[component].specific_power, "W/kg")

        elif type(network[component]) == SUAVE.Components.Energy.Distributors.Electronic_Speed_Controller_eta_Inverter_Mid_Fid:
            output += segment_subheading(component)
            output += add_subdata('Specific power', network[component].specific_power, 'W/kg')

        elif type(network[component]) == SUAVE.Components.Energy.Storages.Batteries.Constant_Mass.Lithium_Ion:
            output += segment_subheading(component)
            output += add_subdata("Peak Power", network[component].max_power, "W")
            output += add_subdata("Total Energy stored", network[component].max_energy, "J")
            output += add_subdata('Max. Voltage', network[component].max_voltage, 'V')
            output += add_subdata("Specific Energy", network[component].specific_energy, "J/kg")
            output += add_subdata("Specific Power", network[component].specific_power, "W/kg")
            output += add_subdata("Technology factor", network[component].battery_technology_factor, "-")

        elif type(network[component]) == SUAVE.Components.Energy.Thermal_Management_System.TMS_Surrogates:
            output += segment_subheading(component)
            output += add_subdata("Design Point", network[component].design_point, "W")
            output += add_subdata("Cross flow area", network[component].cross_flow_area, "m^2")

        output += add_subdata("Mass", systems[component].mass_properties.mass, "kg")
        output += data_subheading("Origin")
        output += add_subsubdata("x", systems[component].origin[0][0], "m")
        output += add_subsubdata("y", systems[component].origin[0][1], "m")
        output += add_subsubdata("z", systems[component].origin[0][2], "m")
        output += data_subheading("Relative Center of Gravity")
        output += add_subsubdata("x", systems[component].mass_properties.center_of_gravity[0][0], "m")
        output += add_subsubdata("y", systems[component].mass_properties.center_of_gravity[0][1], "m")
        output += add_subsubdata("z", systems[component].mass_properties.center_of_gravity[0][2], "m")

    f = open(file_name + "_fact_sheet.txt", "w")
    f.write(output)
    f.close()


def segment_heading(heading):
    out = "\n\n"
    out += u"\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022" \
           u"\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022" \
           u"\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022" \
           u"\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022"
    out += "\n " + heading + "\n"
    out += u"\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022" \
           u"\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022" \
           u"\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022" \
           u"\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022\u2022"
    out += "\n"

    return out


def segment_subheading(heading):
    out = "\n\n-----------------------------------------------------------------\n "
    out += heading
    out += "\n-----------------------------------------------------------------\n"

    return out


def data_heading(header):
    out = f"\n... {header:30}"

    return out


def data_subheading(header):
    out = f"\n    ... {header:26}"

    return out


def add_data(header, data, unit):
    out = f"\n... {header:30}\t{data:14.2f} {unit:10}"

    return out


def add_subdata(header, data, unit):
    out = f"\n    ... {header:26}\t{data:14.2f} {unit:10}"

    return out


def add_subsubdata(header, data, unit):
    out = f"\n        ... {header:22}\t{data:14.2f} {unit:10}"

    return out
