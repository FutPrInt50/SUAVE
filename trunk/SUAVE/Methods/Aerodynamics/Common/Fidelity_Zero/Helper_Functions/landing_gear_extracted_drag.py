## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Helper_Functions
# landing_gear_extracted_drag.py
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
#  Compute drag of extracted landing gear
# ----------------------------------------------------------------------

## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Helper_Functions
def landing_gear_extracted_drag(state,settings,geometry):
    """Compute drag of extracted landing gear

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
    c_L        = state.conditions.aerodynamics.lift_coefficient

    if vehicle.landing_gear.landing_gear_extracted == False:
        landing_gear_extracted_drag = np.zeros_like(c_L)
        
    elif vehicle.landing_gear.landing_gear_extracted == True:

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



        delta_f = vehicle.wings.main_wing.control_surfaces.flap.angle
        c_L_0 = vehicle.wings.main_wing.max_camber * 2*np.pi
        reference_area_S_wf = 1/vehicle.wings.main_wing.flap_ratio
        l_uc_mg = vehicle.landing_gear.main_strut_length
        l_uc = l_uc_mg

        c_g = reference_area / span

        F_UC = (1 - 0.04 * ((c_L + delta_f * c_L_0 * (1.5 * reference_area_S_wf -1))/(l_uc/c_g)))**2

        constant = 7*1e-4
        c_D_UC_basic = constant * MTOM**0.785 / reference_area

        landing_gear_extracted_drag = F_UC * c_D_UC_basic

    # dump data to state
    state.conditions.aerodynamics.drag_breakdown.landing_gear_extracted_drag = landing_gear_extracted_drag


    return landing_gear_extracted_drag