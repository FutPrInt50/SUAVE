## @ingroup Methods-Center_of_Gravity
# compute_mission_center_of_gravity.py
#
# Created:  Nov 2015, M. Vegh
# Modified: Jan 2016, E. Botero
#           Oct 2021, D. Eisenhut

# ----------------------------------------------------------------------
#  Computer Mission Center of Gravity
# ----------------------------------------------------------------------

## @ingroup Methods-Center_of_Gravity
def compute_mission_center_of_gravity(vehicle, mission_fuel_weight):
    """ computes the CG for the vehicle based on the mzfw cg of the
    vehicle, and an assigned fuel

    Assumptions:
    None

    Source:
    N/A

    Inputs:
    vehicle 
    mission_fuel_weight           [Newtons]

    Outputs:
    cg                            [meters]

    Properties Used:
    N/A
    """  

    zf_cg      = vehicle.mass_properties.zero_fuel_center_of_gravity
    zf_weight  = vehicle.mass_properties.zero_fuel_weight
    fuel_origin = vehicle.systems.fuel.origin
    fuel_cg     = vehicle.systems.fuel.mass_properties.center_of_gravity
    
    cg         =((zf_cg)*zf_weight+(fuel_cg+fuel_origin)*mission_fuel_weight)/(mission_fuel_weight+zf_weight)
   
    return cg
