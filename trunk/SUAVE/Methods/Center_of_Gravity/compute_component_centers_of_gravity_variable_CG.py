## @ingroup Methods-Center_of_Gravity
# compute_component_centers_of_gravity.py
#
# Created:  Oct 2015, M. Vegh
# Modified: Jan 2016, E. Botero
# Mofified: Jun 2017, M. Clarke
#           Apr 2020, M. Clarke
#           May 2020, E. Botero
#           May 2021, M. Clarke

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
from SUAVE.Methods.Geometry.Three_Dimensional.compute_span_location_from_chord_length import compute_span_location_from_chord_length
from SUAVE.Methods.Geometry.Three_Dimensional.compute_chord_length_from_span_location import compute_chord_length_from_span_location
from SUAVE.Methods.Flight_Dynamics.Static_Stability.Approximations.Supporting_Functions.convert_sweep import convert_sweep
from SUAVE.Components.Energy.Energy_Component import Energy_Component
import SUAVE

# ----------------------------------------------------------------------
#  Computer Aircraft Center of Gravity
# ----------------------------------------------------------------------

## @ingroup Methods-Center_of_Gravity
def compute_component_centers_of_gravity_variable_CG(vehicle, nose_load = 0.06):
    """ computes the CG of all of the vehicle components based on correlations 
    from AA241

    Assumptions:
    None

    Source:
    AA 241 Notes

    Inputs:
    vehicle

    Outputs:
    None

    Properties Used:
    N/A
    """  
    
    C =  SUAVE.Components
    
    # Go through all wings
    for wing in vehicle.wings:
    
        if wing.sweeps.leading_edge == None:
            wing.sweeps.leading_edge = convert_sweep(wing,old_ref_chord_fraction = 0.25 ,new_ref_chord_fraction = 0.0)
        
        if isinstance(wing,C.Wings.Main_Wing):
                wing.mass_properties.center_of_gravity[0][0] = .05*wing.chords.mean_aerodynamic +wing.aerodynamic_center[0]             
                
            
        elif isinstance(wing,C.Wings.Horizontal_Tail):
            chord_length_h_tail_35_percent_semi_span  = compute_chord_length_from_span_location(wing,.35*wing.spans.projected*.5)
            h_tail_35_percent_semi_span_offset        = np.tan(wing.sweeps.quarter_chord)*.35*.5*wing.spans.projected   
            wing.mass_properties.center_of_gravity[0][0] = .3*chord_length_h_tail_35_percent_semi_span + \
                                                                          h_tail_35_percent_semi_span_offset            

        elif isinstance(wing,C.Wings.Vertical_Tail):
            chord_length_v_tail_35_percent_semi_span  = compute_chord_length_from_span_location(wing,.35*wing.spans.projected)
            v_tail_35_percent_semi_span_offset        = np.tan(wing.sweeps.quarter_chord)*.35*.5*wing.spans.projected
            wing.mass_properties.center_of_gravity[0][0] = .3*chord_length_v_tail_35_percent_semi_span + \
                                                                        v_tail_35_percent_semi_span_offset
        else:
            span_location_mac = compute_span_location_from_chord_length(wing, wing.chords.mean_aerodynamic)
            mac_le_offset     = np.tan(wing.sweeps.leading_edge)*span_location_mac
            
            wing.mass_properties.center_of_gravity[0][0] = .3*wing.chords.mean_aerodynamic + mac_le_offset
            
            
    # Go through all the propulsors
    propulsion_moment = 0.
    propulsion_mass   = 0.
    for prop in vehicle.propulsors:

        if isinstance(prop, SUAVE.Components.Energy.Networks.Propulsor_Turboshaft_Surrogate):
            if hasattr(prop.turboshaft, 'specific_power'):
                vehicle.systems.turboshaft.mass_properties.center_of_gravity[0][0] = prop.turboshaft.turboshaft_length * 0.65
                # propeller and gearbox cg are on origin

        else:
            prop.mass_properties.center_of_gravity[0][0] = prop.engine_length*.35 #0.5
            propulsion_mass                              += prop.mass_properties.mass
            propulsion_moment                            += propulsion_mass*(np.sum(np.array(prop.origin),axis=0) +
                                                                             prop.mass_properties.center_of_gravity)

            for key,Comp in prop.items():
                if isinstance(Comp,Energy_Component):
                    propulsion_moment += prop[key].mass_properties.mass*(np.sum(np.array(prop[key].origin),axis=0) +
                                                                         prop[key].mass_properties.center_of_gravity)
                    propulsion_mass   += prop[key].mass_properties.mass*len(prop[key].origin)

    if propulsion_mass!= 0.:
        propulsion_cg = propulsion_moment/propulsion_mass
    else:
        propulsion_cg = np.array([[0.,0.,0.]])

    # Go through all the fuselages
    for fuse in vehicle.fuselages:
        fuse.mass_properties.center_of_gravity[0][0]   = .40*fuse.lengths.total #0.45 turbofan 0.4 turboprop

    #---------------------------------------------------------------------------------
    # All other components
    #---------------------------------------------------------------------------------
     
    # Select a length scale depending on what kind of vehicle this is
    length_scale = 1.
    nose_length  = 0.
     
    # Check if there is a fuselage
    if len(vehicle.fuselages) == 0.:
        for wing in vehicle.wings:
            if isinstance(wing,C.Wings.Main_Wing):
                b = wing.chords.root
                if b>length_scale:
                    length_scale = b
                    nose_length  = 0.25*b
    else:
        for fuse in vehicle.fuselages:
            nose   = fuse.lengths.nose
            length = fuse.lengths.total
            if length > length_scale:
                length_scale = length
                nose_length  = nose
                
    # unpack all components:
    avionics                                                = vehicle.systems.avionics
    furnishings                                             = vehicle.systems.furnishings
    apu                                                     = vehicle.systems.apu
    passengers                                              = vehicle.payload.passengers
    baggage                                                 = vehicle.payload.baggage
    cargo                                                   = vehicle.payload.cargo
    air_conditioner                                         = vehicle.systems.air_conditioner
    optionals                                               = vehicle.systems.optionals
    crew                                                    = vehicle.systems.crew
    fuel                                                    = vehicle.systems.fuel
    control_systems                                         = vehicle.systems.control_systems
    electrical_systems                                      = vehicle.systems.electrical_systems
    main_gear                                               = vehicle.landing_gear.main    
    nose_gear                                               = vehicle.landing_gear.nose 
    hydraulics                                              = vehicle.systems.hydraulics
        
    avionics.origin[0][0]                                      = 0.4 * nose_length
    avionics.mass_properties.center_of_gravity[0][0]           = 0.0
    
    furnishings.origin[0][0]                                   = 0.49 * length_scale #0.51
    furnishings.mass_properties.center_of_gravity[0][0]        = 0.0
    
    #assumption that it's at 90% of fuselage length (not from notes)
    apu.origin[0][0]                                           = 0.9 * length_scale   
    apu.mass_properties.center_of_gravity[0][0]                = 0.0
    
    passengers.origin[0][0]                                    =0.46 * length_scale# 0.49 * #0.472 0.49
    passengers.mass_properties.center_of_gravity[0][0]         = 0.0
    
    baggage.origin[0][0]                                       = 0.49 * length_scale #0.51
    baggage.mass_properties.center_of_gravity[0][0]            = 0.0
    
    cargo.origin[0][0]                                         = 0.51 * length_scale #0.49
    cargo.mass_properties.center_of_gravity[0][0]              = 0.0
    
    air_conditioner.origin[0][0]                               = nose_length
    air_conditioner.mass_properties.center_of_gravity[0][0]    = 0.0
    
    optionals.origin[0][0]                                     = 0.49 * length_scale #0.51
    optionals.mass_properties.center_of_gravity[0][0]          = 0.0

    crew.origin[0][0]                                          = 0.31 * length_scale #0.51
    crew.mass_properties.center_of_gravity[0][0]               = 0.0

    fuel.origin[0][0]                                          = vehicle.wings.main_wing.origin[0][0] + vehicle.wings.main_wing.aerodynamic_center[0]
    fuel.mass_properties.center_of_gravity                     = vehicle.wings.main_wing.mass_properties.center_of_gravity
    
    control_systems.origin[0][0]                               = vehicle.wings.main_wing.origin[0][0] 
    control_systems.mass_properties.center_of_gravity[0][0]    = vehicle.wings.main_wing.mass_properties.center_of_gravity[0][0] + \
        .1*vehicle.wings.main_wing.chords.mean_aerodynamic
    
    
    electrical_systems.origin[0][0]                            = .75*(.5*length_scale) + propulsion_cg[0][0]*.25
    electrical_systems.mass_properties.center_of_gravity[0][0] = 0.0
    
    hydraulics.origin[0][0]                                    = .75*(vehicle.wings.main_wing.origin[0][0] + \
                                                                      wing.mass_properties.center_of_gravity[0][0]) + 0.25* \
                                                                     length_scale*.95
    hydraulics.mass_properties.center_of_gravity[0][0]         = 0.0       
    
    # Now the landing gear
    
    # Nose gear
    nose_gear.origin[0][0]                                     = 0.545*nose_length #0.25
    nose_gear.mass_properties.center_of_gravity[0][0]          = 0.0   
    
    # Main gear
    #moment_sans_main = vehicle.center_of_gravity()[0][0]*(vehicle.sum_mass()-main_gear.mass_properties.mass)
    moment_sans_main = vehicle.mass_properties.center_of_gravity[0][0]*(vehicle.sum_mass()-main_gear.mass_properties.mass)
    
    main_gear_location = moment_sans_main/(vehicle.mass_properties.takeoff-main_gear.mass_properties.mass)/(1-nose_load)
    main_gear.origin[0][0]                                     = main_gear_location
    main_gear.mass_properties.center_of_gravity[0][0]          = 0.0
    