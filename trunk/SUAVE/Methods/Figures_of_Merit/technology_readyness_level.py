## @ingroup Methods-Figures_of_Merit-Supporting_Functions
# technology_readyness_level.py
#
# Created:  May 2022, J. Frank


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
from scipy.interpolate import interp1d
from SUAVE.Core import Data

# ----------------------------------------------------------------------
#  Method
# ----------------------------------------------------------------------
## @ingroup Methods-Figures_of_Merit-Supporting_Functions
def technology_readyness_level(trl_c):
    """ This method returns a total value for the total technology readyness level of the aircraft between 0 and 1
    """
    ordinal_trl = [1,2,3,4,5,6,7,8,9]
    cardinal_trl = [0.26,0.53,0.71,1.14,1.97,2.74,4.26,6.81,9]
    ordinal_cardinal = interp1d(ordinal_trl,cardinal_trl)

    trl_d = Data()
    trl_d.fuel_cell = 9.
    trl_d.hydrogen_tank = 9.
    trl_d.electric_motors = 9.
    trl_d.distributed_propulsion = 9.
    trl_d.wing_tip_propulsion = 9.
    trl_d.batteries = 9.
    trl_d.electric_taxiing = 9.
    trl_d.landing_gear_drive = 9.
    trl_d.pemfc = 9.
    trl_d.composite_secondary_structures = 9.
    trl_d.spiroid_wingtip = 9.
    trl_d.split_winglets = 9.
    trl_d.hybrid_laminar_flow = 9.
    trl_d.fly_by_light = 9.

    i = 0
    trl_sum = 0.
    for trl in trl_c.keys():
        trl_c_ordinal = trl_c[trl]
        if trl in trl_d.keys():
            trl_d_ordinal = trl_d[trl]
        else:
            trl_d_ordinal = 9.
        trl_c_cardinal = ordinal_cardinal(trl_c_ordinal)
        trl_d_cardinal = ordinal_cardinal(trl_d_ordinal)
        trl_sum += trl_c_cardinal / trl_d_cardinal
        i += 1

    if i > 0:
        trl_total = trl_sum / i
    else:
        trl_total = 1.
    return trl_total