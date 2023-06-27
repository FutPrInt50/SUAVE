## @ingroup Methods-Performance
# sizing_chart.py
#
# Created:  May 2022, J.Mangold
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Data, Units
import time
import numpy as np
import SUAVE

# ----------------------------------------------------------------------
#  Calculate Vehicle Sizing Chart
# ----------------------------------------------------------------------

## @ingroup Methods-Performance
def sizing_chart(vehicle,results):
    """Calculates a vehicle's sizing chart. Includes plotting.

    Assumptions:
    tbd

    Source:
    N/A

    Inputs:
    vehicle.mass_properties.
      operating_empty                     [kg]
      max_zero_fuel                       [kg]
      max_takeoff                         [kg]
      max_payload                         [kg]
      max_fuel                            [kg]
      takeoff                             [kg]
    mission.segments[0].analyses.weights.
      vehicle.mass_properties.takeoff     [kg]
    cruise_segment_tag                    <string>

    Outputs:
    payload_range.
      range                             [m]
      payload                           [kg]
      fuel                              [kg]
      takeoff_weight                    [kg]
    PayloadRangeDiagram.dat (text file)

    Properties Used:
    N/A
    """

    #unpack

    atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()

    OEM =   vehicle.mass_properties.operating_empty
    MZFM =  vehicle.mass_properties.max_zero_fuel
    MTOM =  vehicle.mass_properties.max_takeoff

    ref_area= vehicle.wings.main_wing.areas.reference

    v_stall = 45

    v2 = 1.2 * v_stall


    sizing_chart.power = Data()

    ####################################################################################################################
    # Take off Field Length
    k_TO = 2.45
    s_TOFL = 1165
    eta_TO = 0.75
    cL_max_TO_new = MTOM * 9.81 / (1.225/2 * (51)**2 * ref_area)
    cL_max_TO = 0.8 * 2.7
    rho_airport = 1.225

    sizing_chart.power.take_off = MTOM * 9.81 * v2 / eta_TO * k_TO / (cL_max_TO * rho_airport * s_TOFL) * (MTOM / ref_area) / 2
    sizing_chart.power.take_off_new = MTOM * 9.81 * (51*1.2) / eta_TO * k_TO / (cL_max_TO_new * rho_airport * s_TOFL) * (MTOM / ref_area) / 2


    ####################################################################################################################
    # 2ng Segment OEI

    eta_climb = 0.8
    gamma_min_OEI = 0.024
    cD_cL_SecondSegment = 1/8
    m2_m0 = 0.99

    sizing_chart.power.second_segment_OEI = MTOM * 9.81 * v2 / eta_climb * m2_m0 * (gamma_min_OEI + cD_cL_SecondSegment)


    ####################################################################################################################
    # Cruise

    v_cruise = sum(results.segments.cruise.conditions.freestream.velocity[:, 0])/len(results.segments.cruise.conditions.freestream.velocity)
    eta_cruise = sum(results.segments.cruise.conditions.propulsion.etap[:, 0])/len(results.segments.cruise.conditions.propulsion.etap)
    cD = sum(results.segments.cruise.conditions.aerodynamics.drag_coefficient[:, 0])/len(results.segments.cruise.conditions.aerodynamics.drag_coefficient)
    rho_cruise = sum(results.segments.cruise.conditions.freestream.density[:,0])/len(results.segments.cruise.conditions.freestream.density)

    Drag = rho_cruise / 2 * v_cruise**2 * ref_area * cD
    Force = Drag

    sizing_chart.power.cruise = Force * v_cruise / eta_cruise / 2

    ####################################################################################################################
    # Climb Requirment 1850 ft/min MTOM, SL, ISA

    v_v_1850 = 1850 * Units.ft/Units.min
    m4_m0 = 0.98
    v_climb_1 = sum(results.segments.climb_1.conditions.freestream.velocity[:, 0])/len(results.segments.climb_1.conditions.freestream.velocity)
    cD = sum(results.segments.climb_1.conditions.aerodynamics.drag_coefficient[:, 0])/len(results.segments.climb_1.conditions.aerodynamics.drag_coefficient)
    cL = sum(results.segments.climb_1.conditions.aerodynamics.lift_coefficient[:, 0])/len(results.segments.climb_1.conditions.aerodynamics.lift_coefficient)

    LoD_climb_1 = cL/cD

    sizing_chart.power.climb1850ftmin = MTOM * 9.81 * v_climb_1 / eta_climb * m4_m0 * (v_v_1850 / v_climb_1 + 1/LoD_climb_1) / 2



    ####################################################################################################################
    # Ceiling OEI 18.000 ft +10ISA

    v_v_100 = 100 * Units.ft/Units.min
    m4_m0 = 0.98

    atmo_data_ceilingOEI =  atmosphere.compute_values(18000 * Units.ft,10 * Units.K)

    rho_ceilingOEI = atmo_data_ceilingOEI.density

    settings = Data()
    settings.wing_parasite_drag_form_factor = 1.1
    settings.recalculate_total_wetted_area = True



    state = Data()
    state.conditions = Data()
    state.conditions.freestream = Data()
    state.conditions.freestream.mach_number = np.array([0.5])
    state.conditions.freestream.temperature = np.array([280])
    state.conditions.freestream.reynolds_number = np.array([5e6])

    #wing_parasite_drag = SUAVE.Methods.Aerodynamics.Common.Fidelity_Zero.Drag.parasite_drag_wing(state,settings,vehicle.wings.main_wing)



    cD0 = 275e-4
    aspectratio = 11
    oswaldfactor = 0.833
    k = 1/ (np.pi * aspectratio * oswaldfactor)
    cL = (3 * cD0 / k)**0.5

    delta_cD_OEI = 20e-4
    cD = cD0 + cL**2 * k + delta_cD_OEI

    LoD_ceilingOEI = cL / cD
    v_ceilingOEI = (0.97 * MTOM * 9.81 / (rho_ceilingOEI/2 * ref_area * cL))**0.5


    sizing_chart.power.ceilingOEI = MTOM * 9.81 * v_ceilingOEI / eta_cruise * m4_m0 * (v_v_100 / v_ceilingOEI + 1/LoD_ceilingOEI)

    ####################################################################################################################
    # Service Ceiling 25.000 ft

    m4_m0 = 0.98
    cD = sum(results.segments.cruise.conditions.aerodynamics.drag_coefficient[:, 0])/len(results.segments.cruise.conditions.aerodynamics.drag_coefficient)
    cL = sum(results.segments.cruise.conditions.aerodynamics.lift_coefficient[:, 0])/len(results.segments.cruise.conditions.aerodynamics.lift_coefficient)

    LoD_cruise = cL/cD

    sizing_chart.power.ceilingservice= MTOM * 9.81 * v_cruise / eta_cruise * m4_m0 * (v_v_100 / v_cruise + 1/LoD_cruise) / 2

    ####################################################################################################################
    # Initial Cruise Altitude - Top of Climb - 300 ft/min

    v_v_300 = 300 * Units.ft/Units.min
    eta_TOC = 0.8
    m4_m0 = 0.98
    cD = sum(results.segments.climb_12.conditions.aerodynamics.drag_coefficient[:, 0])/len(results.segments.climb_12.conditions.aerodynamics.drag_coefficient)
    cL = sum(results.segments.climb_12.conditions.aerodynamics.lift_coefficient[:, 0])/len(results.segments.climb_12.conditions.aerodynamics.lift_coefficient)
    v_TOC = sum(results.segments.climb_12.conditions.freestream.velocity[:, 0])/len(results.segments.climb_12.conditions.freestream.velocity)

    LoD_TOC = cL/cD

    sizing_chart.power.topofclimb= MTOM * 9.81 * v_TOC / eta_TOC * m4_m0 * (v_v_300 / v_TOC + 1/LoD_TOC) / 2

    return sizing_chart
