## @ingroup Methods-Weights-Correlations-Propulsion
# gearbox.py
# 
# Created:  Aug 2022, J. Mangold
# Modified:


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Units
import numpy as np

# ----------------------------------------------------------------------
#   Gearbox Mass
#   Calculation of the gearbox mass
# ----------------------------------------------------------------------

## @ingroup Methods-Weights-Correlations-Propulsion
def gear_box(sea_level_power_in, rpm_in, rpm_out):
    """ Calculate the mass of a propeller
    
    Assumptions:
    
    Source: 
            CRAN
            
    Inputs:
            Power_In in kW
            rpm_In in min^(*-1)
            rpm_Out in min^(*-1)
    
    Outputs:
            gerabox mass          [kilograms]
        
    Properties Used:
            N/A
    """     
    # Calculation

    k_factor = 34 # 34 for current technology; 26 for future technology
    power_in = sea_level_power_in / Units.kW

    gearbox    = k_factor * ((power_in)**0.76 * (rpm_in)**0.13) / ((rpm_out)**0.89) * Units.kg

    return gearbox

