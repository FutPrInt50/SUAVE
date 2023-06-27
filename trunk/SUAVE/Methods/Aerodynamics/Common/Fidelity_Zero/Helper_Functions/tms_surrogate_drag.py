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
def tms_surrogate_drag(state,settings,geometry):
    """
    For a given heat load and altitude, determines the power consumption of the system

    Assumptions:


    Inputs:
    altitude: [ft]


    Outputs:

    """


    # Unpack

    vehicle    = geometry
    propulsors = vehicle.propulsors
    wings      = vehicle.wings

    Nets  = SUAVE.Components.Energy.Networks
    TMS = SUAVE.Components.Energy.Thermal_Management_System
    prop= vehicle.propulsors.network

    if isinstance(prop, Nets.Hybrid_Propulsor_Turboshaft_Surrogate_Battery_Propeller_Bus):

        if isinstance(prop.tms_vcs, TMS.TMS_Surrogates) and isinstance(prop.tms_liquid, TMS.TMS_Surrogates):

            tms_vcs = vehicle.propulsors.network.tms_vcs
            tms_vcs_drag_newton = tms_vcs.interp_Drag
            design_point_vcs = vehicle.propulsors.network.tms_vcs.design_point

            tms_liquid = vehicle.propulsors.network.tms_liquid
            tms_liquid_drag_newton = tms_liquid.interp_Drag
            design_point_liquid = vehicle.propulsors.network.tms_liquid.design_point

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


            mach        = state.conditions.freestream.mach_number[:,0]
            altitude    = state.conditions.freestream.altitude[:,0]
            heat_load_vcs   = state.conditions.propulsion.heat_load_vcs[:,0]
            heat_load_liquid   = state.conditions.propulsion.heat_load_liquid[:,0]
            rho         = state.conditions.freestream.density
            v_inf       = state.conditions.freestream.velocity

            # VCS
            Q_ratio_vcs     = heat_load_vcs/(design_point_vcs*Units.kW)

            if any(mach < tms_vcs.mach_min_max[0]) or any(mach > tms_vcs.mach_min_max[1]):
                #print(f'replacing mach out of bounds\n{mach}')
                mach[mach < tms_vcs.mach_min_max[0]] = tms_vcs.mach_min_max[0]
                mach[mach > tms_vcs.mach_min_max[1]] = tms_vcs.mach_min_max[1]

            if any(altitude < tms_vcs.altitude_min_max[0]) or any(altitude > tms_vcs.altitude_min_max[1]):
                #print(f'replacing altitude out of bounds\n{altitude}')
                altitude[altitude < tms_vcs.altitude_min_max[0]] = tms_vcs.altitude_min_max[0]
                altitude[altitude > tms_vcs.altitude_min_max[1]] = tms_vcs.altitude_min_max[1]

            if any(Q_ratio_vcs < tms_vcs.q_ratio_min_max[0]) or any(Q_ratio_vcs > tms_vcs.q_ratio_min_max[1]):
                #print(f'replacing Q_ratio out of bounds\n{Q_ratio}')
                Q_ratio_vcs[Q_ratio_vcs < tms_vcs.q_ratio_min_max[0]] = tms_vcs.q_ratio_min_max[0]
                Q_ratio_vcs[Q_ratio_vcs > tms_vcs.q_ratio_min_max[1]] = tms_vcs.q_ratio_min_max[1]

            # if (design_point_vcs < tms_vcs.design_point_min_max[0]) or (design_point_vcs > tms_vcs.design_point_min_max[1]):
            #     #print(f'replacing design_point out of bounds\n{design_point}')
            #     design_point_vcs[design_point_vcs < tms_vcs.design_point_min_max[0]] = tms_vcs.design_point_min_max[0]
            #     design_point_vcs[design_point_vcs > tms_vcs.design_point_min_max[1]] = tms_vcs.design_point_min_max[1]

            if (design_point_vcs < tms_vcs.design_point_min_max[0]):
                #print(f'replacing design_point out of bounds\n{design_point}')
                design_point_vcs = tms_vcs.design_point_min_max[0]
            elif (design_point_vcs > tms_vcs.design_point_min_max[1]):
                #print(f'replacing design_point out of bounds\n{design_point}')
                design_point_vcs = tms_vcs.design_point_min_max[1]

            if np.isnan(design_point_vcs).any() or np.isnan(Q_ratio_vcs).any() or np.isnan(altitude).any() or np.isnan(mach).any():
                tms_vcs_drag_newton = np.zeros_like(v_inf)
            else:
                if tms_vcs.tag == 'CASE1' or tms_vcs.tag == 'CASE1_v2':
                    tms_vcs_drag_newton = tms_vcs_drag_newton((mach, altitude, Q_ratio_vcs, design_point_vcs)) * tms_vcs.number_system
                elif tms_vcs.tag == 'CASE2':
                    tms_vcs_drag_newton = tms_vcs_drag_newton((mach, altitude, Q_ratio_vcs, design_point_vcs, tms_vcs.shin_hx_area)) * tms_vcs.number_system
                    if np.isnan(tms_vcs_drag_newton).any():
                        tms_vcs_drag_newton = np.zeros_like(tms_vcs_drag_newton)
                        print('TMS case 2 drag vcs nan')

                tms_vcs_drag_newton = tms_vcs_drag_newton.reshape(len(tms_vcs_drag_newton),1)

            tms_vcs_drag = tms_vcs_drag_newton / (rho/2 * v_inf**2 * reference_area) * tms_vcs.drag_factor

            # Liquid
            Q_ratio_liquid     = heat_load_liquid/(design_point_liquid*Units.kW)

            if any(mach < tms_liquid.mach_min_max[0]) or any(mach > tms_liquid.mach_min_max[1]):
                #print(f'replacing mach out of bounds\n{mach}')
                mach[mach < tms_liquid.mach_min_max[0]] = tms_liquid.mach_min_max[0]
                mach[mach > tms_liquid.mach_min_max[1]] = tms_liquid.mach_min_max[1]

            if any(altitude < tms_liquid.altitude_min_max[0]) or any(altitude > tms_liquid.altitude_min_max[1]):
                #print(f'replacing altitude out of bounds\n{altitude}')
                altitude[altitude < tms_liquid.altitude_min_max[0]] = tms_liquid.altitude_min_max[0]
                altitude[altitude > tms_liquid.altitude_min_max[1]] = tms_liquid.altitude_min_max[1]

            if any(Q_ratio_liquid < tms_liquid.q_ratio_min_max[0]) or any(Q_ratio_liquid > tms_liquid.q_ratio_min_max[1]):
                #print(f'replacing Q_ratio out of bounds\n{Q_ratio}')
                Q_ratio_liquid[Q_ratio_liquid < tms_liquid.q_ratio_min_max[0]] = tms_liquid.q_ratio_min_max[0]
                Q_ratio_liquid[Q_ratio_liquid > tms_liquid.q_ratio_min_max[1]] = tms_liquid.q_ratio_min_max[1]

            # if (design_point_liquid < tms_liquid.design_point_min_max[0]) or (design_point_liquid > tms_liquid.design_point_min_max[1]):
            #     #print(f'replacing design_point out of bounds\n{design_point}')
            #     design_point_liquid[design_point_liquid < tms_liquid.design_point_min_max[0]] = tms_liquid.design_point_min_max[0]
            #     design_point_liquid[design_point_liquid > tms_liquid.design_point_min_max[1]] = tms_liquid.design_point_min_max[1]

            if (design_point_liquid < tms_liquid.design_point_min_max[0]):
                #print(f'replacing design_point out of bounds\n{design_point}')
                design_point_liquid = tms_liquid.design_point_min_max[0]
            elif (design_point_liquid > tms_liquid.design_point_min_max[1]):
                #print(f'replacing design_point out of bounds\n{design_point}')
                design_point_liquid = tms_liquid.design_point_min_max[1]

            if np.isnan(design_point_liquid).any() or np.isnan(Q_ratio_liquid).any() or np.isnan(altitude).any() or np.isnan(mach).any():
                tms_liquid_drag_newton = np.zeros_like(v_inf)
            else:
                if tms_liquid.tag == 'CASE4' or tms_liquid.tag == 'CASE4_v2':
                    tms_liquid_drag_newton = tms_liquid_drag_newton((mach, altitude, Q_ratio_liquid, design_point_liquid)) * tms_liquid.number_system
                elif tms_liquid.tag == 'CASE6':
                    tms_liquid_drag_newton = tms_liquid_drag_newton((mach, altitude, Q_ratio_liquid, design_point_liquid, tms_liquid.shin_hx_area)) * tms_liquid.number_system
                    if np.isnan(tms_liquid_drag_newton).any():
                        tms_liquid_drag_newton = np.zeros_like(tms_liquid_drag_newton)
                        print('TMS case 6 drag liquid nan')

                tms_liquid_drag_newton = tms_liquid_drag_newton.reshape(len(tms_liquid_drag_newton),1)

            tms_liquid_drag = tms_liquid_drag_newton / (rho/2 * v_inf**2 * reference_area) * tms_liquid.drag_factor



            
    else:
        tms_vcs_drag = np.zeros_like(state.conditions.freestream.velocity)
        tms_liquid_drag = np.zeros_like(state.conditions.freestream.velocity)

    # dump data to state
    state.conditions.aerodynamics.drag_breakdown.tms_vcs_drag = tms_vcs_drag
    state.conditions.aerodynamics.drag_breakdown.tms_liquid_drag = tms_liquid_drag


    return tms_vcs_drag, tms_liquid_drag