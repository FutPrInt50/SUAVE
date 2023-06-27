## @ingroup Methods-Weights-Correlations-Electric
# Electric_Components_LoFid.py
# 
# Created:  Feb 2022, J. Mangold
# Modified:


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

from SUAVE.Core import Units

# ----------------------------------------------------------------------
#   Electric Components LoFid
#   Calculation of the electric component mass based on specific power
# ----------------------------------------------------------------------

## @ingroup Methods-Weights-Correlations-Electric
def electric_motor_LoFid(motor):
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

    mass = motor.rated_power / motor.specific_power

    return mass


def electronic_speed_controller_LoFid(esc,motor):
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

    mass = motor.rated_power / esc.specific_power

    return mass

def dcdc_converter_LoFid(dcdc,power_max):
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

    mass = power_max / dcdc.specific_power

    return mass

def cable_LoFid(input):
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

    mass = 100

    return mass