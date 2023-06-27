## @ingroup Methods-Weights-Correlations-Propulsion
# propeller.py
# 
# Created:  Feb 2022, J. Mangold
# Modified:


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Units
import numpy as np

# ----------------------------------------------------------------------
#   Propeller Mass LoFid
#   Calculation of the propeller mass
# ----------------------------------------------------------------------

## @ingroup Methods-Weights-Correlations-Propulsion
def propeller_LoFid(propeller, sea_level_power):
    """ Calculate the mass of a propeller
    
    Assumptions:
    
    Source: 
            Torenbeek
            Y.A.P. Teeuwen - Propeller Design for Conceptual Turboprop Aircraft
            
    Inputs:
            tbd
            tbd
    
    Outputs:
            propeller mass          [kilograms]
        
    Properties Used:
            N/A
    """     
    # Calculation

    D_p     = propeller.tip_radius * 2
    B       = propeller.number_of_blades
    P_max   = sea_level_power / Units['kW']
    calibration_ATR = 1.070122


    wt_propeller    = (1.1*(D_p * P_max * np.sqrt(B))**0.52) * Units.kg / calibration_ATR

    return wt_propeller

