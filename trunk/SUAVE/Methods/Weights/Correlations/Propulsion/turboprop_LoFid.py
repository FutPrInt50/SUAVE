## @ingroup Methods-Weights-Correlations-Propulsion
# engine_jet.py
# 
# Created:  Okt 2021, J. Mangold
# Modified:


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Units

# ----------------------------------------------------------------------
#   Turboprop LoFid
#   Calculation of the turboprop mass based on specific power
# ----------------------------------------------------------------------

## @ingroup Methods-Weights-Correlations-Propulsion
def turboprop_LoFid(engine):
    """ Calculate the weight of the dry engine  
    
    Assumptions:
            calculated engine mass
    
    Source: 
            N/A
            
    Inputs:
            specific power
            sea level power
    
    Outputs:
            mass - turboprop                       [kilograms]
        
    Properties Used:
            N/A
    """     
    # Calculation

    mass = engine.sea_level_power / engine.specific_power

    return mass