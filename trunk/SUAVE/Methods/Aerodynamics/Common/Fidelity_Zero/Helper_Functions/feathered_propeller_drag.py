## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Helper_Functions
# feathered_propeller_drag.py
# 
# Created:  Jul 2022, J. Mangold
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUAVE Imports
import SUAVE
import numpy as np
from SUAVE.Components import Wings
from SUAVE.Core import Units, Data

# ----------------------------------------------------------------------
#  Compute drag of feathered propeller
# ----------------------------------------------------------------------

## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Helper_Functions
def feathered_propeller_drag(state,settings,geometry):
    """Compute drag of feathered propeller

    Assumptions:
    -

    Source:
    Torenbeek 1982 Eqn: G-63

    Inputs:
    state.conditions.freestream.dynamic_pressure                                                [Pa]
    state.conditions.aerodynamics.drag_breakdown.windmilling_drag.windmilling_drag_coefficient  [Unitless]
    geometry.
      mass_properties.center_of_gravity                                                         [m]
      propulsors. 
        number_of_engines                                                                       [Unitless]
      wings.
        tag                                                                                     
        sref (this function probably doesn't run)                                               [m^2]
	spans.projected                                                                         [m]
      reference_area                                                                            [m^2]

    Outputs:
    asymm_trim_drag_coefficient                                                                 [Unitless]
    (packed in state.conditions.aerodynamics.drag_breakdown.asymmetry_trim_coefficient)

    Properties Used:
    N/A
    """ 
    # ==============================================
	# Unpack
    # ==============================================
    vehicle    = geometry
    propulsors = vehicle.propulsors
    wings      = vehicle.wings
    dyn_press  = state.conditions.freestream.dynamic_pressure

    if vehicle.propulsors.network.one_engine_propeller_inoperative == False:
        feathered_propeller_drag = np.zeros_like(dyn_press)
        
    elif vehicle.propulsors.network.one_engine_propeller_inoperative == True:

        MTOM = vehicle.mass_properties.max_takeoff
        span = vehicle.wings.main_wing.spans.projected
    
         # Defining reference area
        if vehicle.reference_area:
                reference_area = vehicle.reference_area
        else:
            n_wing = 0
            for wing in wings:
                if not isinstance(wing,Wings.Main_Wing): continue
                n_wing = n_wing + 1
                reference_area = wing.sref
            if n_wing > 1:
                print(' More than one Main_Wing in the vehicle. Last one will be considered.')
            elif n_wing == 0:
                print('No Main_Wing defined! Using the 1st wing found')
                for wing in wings:
                    if not isinstance(wing,Wings.Wing): continue
                    reference_area = wing.sref
                    break


        feathered_propeller_drag = 0.00125 * vehicle.propulsors.network.propeller.number_of_blades * (2 * vehicle.propulsors.network.propeller.tip_radius) ** 2 / dyn_press

    elif vehicle.propulsors.network.one_engine_propeller_inoperative_WTP == True:

        MTOM = vehicle.mass_properties.max_takeoff
        span = vehicle.wings.main_wing.spans.projected

        # Defining reference area
        if vehicle.reference_area:
            reference_area = vehicle.reference_area
        else:
            n_wing = 0
            for wing in wings:
                if not isinstance(wing,Wings.Main_Wing): continue
                n_wing = n_wing + 1
                reference_area = wing.sref
            if n_wing > 1:
                print(' More than one Main_Wing in the vehicle. Last one will be considered.')
            elif n_wing == 0:
                print('No Main_Wing defined! Using the 1st wing found')
                for wing in wings:
                    if not isinstance(wing,Wings.Wing): continue
                    reference_area = wing.sref
                    break


        feathered_propeller_drag = vehicle.propulsors.network.number_of_WTP * 0.00125 * vehicle.propulsors.network.propellerWTP.number_of_blades * (2 * vehicle.propulsors.network.propellerWTP.tip_radius) ** 2 / dyn_press

    # dump data to state
    state.conditions.aerodynamics.drag_breakdown.feathered_propeller_drag = feathered_propeller_drag

    return feathered_propeller_drag