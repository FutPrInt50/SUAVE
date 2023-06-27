## @ingroup Methods-Figures_of_Merit
# hybrid_electric_aircraft_introduction.py
#
# Created:  May 2022, J. Frank


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
import copy
from SUAVE.Core import Units
from SUAVE.Core import Data
from SUAVE.Methods.Figures_of_Merit.Supporting_Functions.material_costs import material_costs


# ----------------------------------------------------------------------
#  Method
# ----------------------------------------------------------------------
## @ingroup Methods-Figures_of_Merit
def hybrid_electric_aircraft_introduction(results, vehicle, settings, hybrid=False):
    """ This method contains methods for evaluating the hybrid electric aircraft introduction index.
    """
    heai = Data()

    # Constants and Inputs
    dollar_to_euro = settings.dollar_to_euro
    CPI = settings.consumer_price_index
    q = settings.q
    n_FTA = settings.n_FTA
    Q_total = settings.Q_total
    percent_learning_curve = settings.percent_learning_curve
    labor_rate_tool = settings.labor_rate_tool
    labor_rate_mfg = settings.labor_rate_mfg
    labor_rate_qc = settings.labor_rate_qc
    labor_rate_engr = settings.labor_rate_engr
    F_comp_engr = settings.F_comp_engr
    F_comp_dev = settings.F_comp_dev
    F_comp_tool = settings.F_comp_tool
    F_comp_mfg = settings.F_comp_mfg
    F_comp_qc = settings.F_comp_qc

    if hybrid == True:
        F_HEA_engr = settings.F_HEA_engr
        F_HEA_dev = settings.F_HEA_dev
        F_HEA_tool = settings.F_HEA_tool
        F_HEA_mfg = settings.F_HEA_mfg
        F_HEA_qc = settings.F_HEA_qc
        F_HEA_ft = settings.F_HEA_ft
    else:
        F_HEA_engr = 1.
        F_HEA_dev = 1.
        F_HEA_tool = 1.
        F_HEA_mfg = 1.
        F_HEA_qc = 1.
        F_HEA_ft = 1.


    # UNPACK VALUES
    w_s = vehicle.mass_properties.operating_empty # Maybe OME should be used instead because structures tend to be to small (or + system_masses?) (CHANGED TO OPERATING EMPTY 2022-5-6)
    v_max = 0
    for segment in results.segments.values():
        if np.ceil(segment.conditions.freestream.velocity[:][0]) > v_max:
            v_max = np.ceil(segment.conditions.freestream.velocity[:][0])

    v_max = v_max * 3.6 # Conversion m/s -> km/h

    f_comp = 0.43 * (settings.main_wing_comp * vehicle.wings.main_wing.mass_properties.mass + settings.fuselage_comp * vehicle.fuselages.fuselage.mass_properties.mass + settings.empennage_comp * (
                vehicle.wings.horizontal_stabilizer.mass_properties.mass + vehicle.wings.vertical_stabilizer.mass_properties.mass)) / (
                         vehicle.mass_properties.structures - vehicle.landing_gear.nose.mass_properties.mass - vehicle.landing_gear.main.mass_properties.mass) + 0.1


    #print('F_COMP: %.3f' % f_comp)
    fudge_hea_engr  =   (1 + f_comp * (F_comp_engr - 1)) * F_HEA_engr  # SB 3.60
    fudge_hea_dev   =   (1 + f_comp * (F_comp_dev - 1)) * F_HEA_dev  # from SB 3.61
    fudge_hea_tool  =   (1 + f_comp * (F_comp_tool - 1)) * F_HEA_tool  # SB 3.70
    fudge_hea_mfg   =   (1 + f_comp * (F_comp_mfg - 1)) * F_HEA_mfg  # SB 3.71
    fudge_hea_qc    =   (1 + f_comp * (F_comp_qc - 1)) * F_HEA_qc  # SB 3.72

    # DEVELOPMENT COSTS
    heai.h_engr = 5.18 * w_s ** 0.777 * v_max ** 0.894 * q ** 0.163  # In hours, according Raymer 18.1
    heai.c_engr = heai.h_engr * labor_rate_engr * fudge_hea_engr * CPI # from SB 3.60
    heai.c_dev_s_conventional = 67.4 * w_s ** 0.63 * v_max ** 1.3 # In $, according Raymer 18.5
    heai.c_dev_s = heai.c_dev_s_conventional * fudge_hea_dev * CPI # from SB 3.61

    heai.c_dev = dollar_to_euro * (heai.c_engr + heai.c_dev_s) # from SB 3.59

    # CERTIFICATION CHALLENGES
    heai.c_ft = 1947 * w_s ** 0.325 * v_max ** 0.822 * n_FTA ** 1.21 * F_HEA_ft * CPI # from SB 3.65, modified Raymer 18.6
    heai.c_cer = dollar_to_euro * heai.c_ft # According SB 3.66

    # PRODUCTION ASPECTS
    x_LearningCurve = np.log2(2*percent_learning_curve/100) # SB 3.69
    F_production_quantity = 1/((1/Q_total)**(x_LearningCurve-1)) # SB 3.69

    heai.h_tool = 7.22 * w_s ** 0.777 * v_max ** 0.696 * q ** 0.263 # Raymer 18.2
    heai.h_mfg = 10.5 * w_s ** 0.82 * v_max ** 0.484 * q ** 0.641  # Raymer 18.3
    heai.h_qc = 0.133 * heai.h_mfg # Raymer 18.4

    heai.m_costs = material_costs(vehicle,hybrid)

    heai.c_tool = heai.h_tool * fudge_hea_tool * labor_rate_tool * CPI # SB 3.70
    heai.c_mfg = heai.h_mfg * fudge_hea_mfg * labor_rate_mfg * CPI # SB 3.71
    heai.c_qc = heai.h_qc * fudge_hea_qc * labor_rate_qc * CPI # SB 3.72
    heai.c_mat = heai.m_costs.total # Calculation according SB 3.73 -3.81

    heai.c_prod = dollar_to_euro * F_production_quantity * (heai.c_tool + heai.c_mfg + heai.c_qc + heai.c_mat) # SB 3.68

    return heai