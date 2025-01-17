## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Drag
# parasite_drag_total.py
#
# Created:  Jan 2014, T. Orra
# Modified: Jan 2016, E. Botero 
#           Jul 2017, M. Clarke
#           Jun 2022, F.Brenner

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np

# ----------------------------------------------------------------------
#  Total Parasite Drag
# ----------------------------------------------------------------------

## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Drag
def parasite_total(state,settings,geometry):
    """Sums component parasite drag

    Assumptions:
    None

    Source:
    None

    Inputs:
    geometry.reference_area                             [m^2]
    geometry.wings.areas.reference                      [m^2]
    geometry.fuselages.areas.front_projected            [m^2]
    geometry.propulsors.number_of_engines               [Unitless]
    geometry.propulsors.nacelle_diameter                [m]
    conditions.aerodynamics.drag_breakdown.
      parasite[wing.tag].parasite_drag_coefficient      [Unitless]
      parasite[fuselage.tag].parasite_drag_coefficient  [Unitless]
      parasite[propulsor.tag].parasite_drag_coefficient [Unitless]


    Outputs:
    total_parasite_drag                                 [Unitless]

    Properties Used:
    N/A
    """

    # unpack
    conditions             =  state.conditions
    wings                  = geometry.wings
    fuselages              = geometry.fuselages
    propulsors             = geometry.propulsors
    vehicle_reference_area = geometry.reference_area


    #compute parasite drag total
    total_parasite_drag       = 0.0
    total_parasite_drag_dc    = 0.0
    wings_parasite_drag       = 0.0
    fuselage_parasite_drag    = 0.0
    propulsors_parasite_drag  = 0.0
    pylons_parasite_drag      = 0.0
    miscellaneous_parasite_drag = 0.0
    # from wings
    for wing in wings.values():
        parasite_drag = conditions.aerodynamics.drag_breakdown.parasite[wing.tag].parasite_drag_coefficient 
        conditions.aerodynamics.drag_breakdown.parasite[wing.tag].parasite_drag_coefficient = parasite_drag * wing.areas.reference/vehicle_reference_area
        total_parasite_drag += parasite_drag * wing.areas.reference/vehicle_reference_area
        wings_parasite_drag += parasite_drag * wing.areas.reference/vehicle_reference_area * 10000
        total_parasite_drag_dc+= parasite_drag * wing.areas.reference/vehicle_reference_area * 10000
        # from fuselage
    for fuselage in fuselages.values():
        if fuselage.tag == 'fuselage_bwb':
            continue
        parasite_drag = conditions.aerodynamics.drag_breakdown.parasite[fuselage.tag].parasite_drag_coefficient 
        conditions.aerodynamics.drag_breakdown.parasite[fuselage.tag].parasite_drag_coefficient = parasite_drag * fuselage.areas.front_projected/vehicle_reference_area
        total_parasite_drag += parasite_drag * fuselage.areas.front_projected/vehicle_reference_area
        fuselage_parasite_drag = parasite_drag * fuselage.areas.front_projected/vehicle_reference_area * 10000
        total_parasite_drag_dc += parasite_drag * fuselage.areas.front_projected/vehicle_reference_area * 10000

    # from propulsors
    for propulsor in propulsors.values():
        ref_area = (propulsor.nacelle_diameter /  2)**2 * np.pi
        parasite_drag = conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].parasite_drag_coefficient
        conditions.aerodynamics.drag_breakdown.parasite[propulsor.tag].parasite_drag_coefficient  = parasite_drag * ref_area/vehicle_reference_area * propulsor.number_of_engines
        total_parasite_drag += parasite_drag * ref_area/vehicle_reference_area * propulsor.number_of_engines
        propulsors_parasite_drag = parasite_drag * ref_area/vehicle_reference_area * propulsor.number_of_engines * 10000
        total_parasite_drag_dc += parasite_drag * ref_area/vehicle_reference_area * propulsor.number_of_engines * 10000

    # from pylons
    try:
        parasite_drag = conditions.aerodynamics.drag_breakdown.parasite['pylon'].parasite_drag_coefficient
    except:
        parasite_drag = 0. # not currently available for supersonics

    total_parasite_drag += parasite_drag
    total_parasite_drag_dc += parasite_drag * 10000
    pylons_parasite_drag = parasite_drag * 10000

  # from Misc
    misc_drag = conditions.aerodynamics.drag_breakdown.miscellaneous.total
    total_parasite_drag += misc_drag
    total_parasite_drag_dc += misc_drag * 10000
    miscellaneous_parasite_drag = misc_drag * 10000

    # dump to condtitions
    state.conditions.aerodynamics.drag_breakdown.parasite.total = total_parasite_drag
    state.conditions.aerodynamics.drag_breakdown.parasite.totaldc = total_parasite_drag_dc
    state.conditions.aerodynamics.drag_breakdown.parasite.wings = wings_parasite_drag
    state.conditions.aerodynamics.drag_breakdown.parasite.fuselage = fuselage_parasite_drag
    state.conditions.aerodynamics.drag_breakdown.parasite.propulsors = propulsors_parasite_drag
    state.conditions.aerodynamics.drag_breakdown.parasite.pylons = pylons_parasite_drag
    state.conditions.aerodynamics.drag_breakdown.parasite.miscellaneous = miscellaneous_parasite_drag


    return total_parasite_drag