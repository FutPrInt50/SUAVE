## @ingroup Methods-Aerodynamics-Fidelity_Zero-Drag
# total_aircraft.py
# 
# Created:  Dec 2013, A. Variyar
# Modified: Feb 2014, A. Variyar, T. Lukaczyk, T. Orra
#           Jan 2016, E. Botero 
#           Oct 2017, T. MacDonald

# ----------------------------------------------------------------------
#  Total Aircraft
# ----------------------------------------------------------------------

## @ingroup Methods-Aerodynamics-Fidelity_Zero-Drag
def total_aircraft(state,settings,geometry):
    """ This computes the total drag of an aircraft and stores
    that data in the conditions structure.

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    settings.
      drag_coefficient_increment                   [Unitless]
      lift_to_drag_adjustment                      [Unitless] (.1 is 10% increase in L/D)
    state.conditions.aerodynamics.drag_breakdown.
      trim_corrected_drag                          [Unitless]
      spoiler_drag                                 [Unitless]

    Outputs:
    aircraft_total_drag (drag coefficient)         [Unitless]

    Properties Used:
    N/A
    """    

    # Unpack inputs
    conditions    = state.conditions
    configuration = settings

    drag_coefficient_increment   = configuration.drag_coefficient_increment[geometry.tag]
    drag_coefficient_due_to_prop = configuration.drag_coefficient_due_to_prop
    trim_corrected_drag          = conditions.aerodynamics.drag_breakdown.trim_corrected_drag
    spoiler_drag                 = conditions.aerodynamics.drag_breakdown.spoiler_drag
    landing_gear_extracted_drag  = conditions.aerodynamics.drag_breakdown.landing_gear_extracted_drag
    feathered_propeller_drag     = conditions.aerodynamics.drag_breakdown.feathered_propeller_drag
    delta_drag_prop_wing         = conditions.aerodynamics.drag_breakdown.propeller_wing_interaction_delta_drag
    tms_vcs_drag                 = conditions.aerodynamics.drag_breakdown.tms_vcs_drag
    tms_liquid_drag              = conditions.aerodynamics.drag_breakdown.tms_liquid_drag

    #prop_drag = drag_coefficient_due_to_prop * conditions.propulsion.propeller_thrust / (0.5 * conditions.freestream.density * conditions.freestream.velocity ** 2 * 3.1415 * geometry.propulsors.turboprop.propeller.tip_radius ** 2)
    prop_drag = drag_coefficient_due_to_prop * conditions.propulsion.propeller_thrust / (0.5 * conditions.freestream.density * conditions.freestream.velocity ** 2 * 3.1415 * (geometry.propulsors.network.propeller.tip_radius*2) ** 2)
    prop_drag[prop_drag < 0.0] = 0.0

    aircraft_total_drag = 0.0
    # Add drag_coefficient_increment
    aircraft_total_drag += trim_corrected_drag  + spoiler_drag + prop_drag + drag_coefficient_increment + landing_gear_extracted_drag + feathered_propeller_drag + delta_drag_prop_wing + tms_vcs_drag + tms_liquid_drag
    conditions.aerodynamics.drag_breakdown.drag_coefficient_increment = drag_coefficient_increment
    
    # Add L/D correction
    aircraft_total_drag = aircraft_total_drag/(1.+configuration.lift_to_drag_adjustment) 

    # Store to results
    conditions.aerodynamics.drag_breakdown.total = aircraft_total_drag
    conditions.aerodynamics.drag_coefficient     = aircraft_total_drag
    conditions.aerodynamics.drag_breakdown.prop_drag            = prop_drag

    return aircraft_total_drag