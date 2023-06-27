## @ingroup Methods-Weights-Correlations-Thermal_Management_System
# Thermal_Management_System_LoFid.py
# 
# Created:  Feb 2022, J. Mangold
# Modified:


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Units

# ----------------------------------------------------------------------
#   Thermal Management System LoFid
#   Calculation of the Thermal_Management_System mass based on ??
# ----------------------------------------------------------------------

## @ingroup Methods-Weights-Correlations-Electric
def TMS_system_LoFid(TMS):
    """ Calculate the weight of the dry engine  
    
    Assumptions:
            calculated electric motor mass
    
    Source: 
            N/A
            
    Inputs:
            specific power      [W/kg]
            power               [W]
    
    Outputs:
            mass                [kg]
        
    Properties Used:
            N/A
    """     
    # Calculation

    if TMS.shx_area == 2:
        mass = 2.681*TMS.max_heat_load / Units.kW + 104.7

    elif TMS.shx_area == 3:
        mass = 2.679*TMS.max_heat_load / Units.kW + 112.9

    return mass

