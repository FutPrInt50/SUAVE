# Created:  May 2022, J. Frank


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import copy
from SUAVE.Core import Units
from SUAVE.Core import Data

# ----------------------------------------------------------------------
#  Method
# ----------------------------------------------------------------------
## @ingroup Methods-Figures_of_Merit-Supporting_Functions
def material_costs(vehicle,hybrid = False):
    """ This method computes the material costs
    """

    # Unpack inputs
    mc = Data()
    if hybrid == False:
        w_s = vehicle.mass_properties.operating_empty - vehicle.mass_properties.propulsion
        p_gt = vehicle.propulsors.network.sea_level_power / Units.kW
        n_gt = vehicle.propulsors.network.number_of_engines
        n_p = vehicle.propulsors.network.number_of_engines
        d_p = vehicle.propulsors.network.propeller.tip_radius * 2 / Units.ft
        p_sp = vehicle.propulsors.network.sea_level_power / Units.kW

        mc.airframe = 949.5 * w_s
        mc.gas_turbine = 493.6 * p_gt * n_gt
        mc.propeller = 210 * n_p * (d_p) ** 2 * (p_sp / d_p) ** 0.12
        mc.fuel_cell = 0.
        mc.hydrogen_tank = 0.
        mc.electrical_motors = 0.
        mc.power_management_system = 0.
        mc.battery = 0.

    if hybrid == True:
        w_s = vehicle.mass_properties.operating_empty - vehicle.mass_properties.propulsion - vehicle.propulsors.network.battery.mass_properties.mass
        p_gt = vehicle.propulsors.network.turboshaft.sea_level_power / Units.kW
        n_gt = vehicle.propulsors.network.number_of_engines
        n_em_wtp = 2
        p_em_wtp = vehicle.propulsors.network.emotorWTP.rated_power / Units.kW
        n_p = n_gt + n_em_wtp
        d_p = vehicle.propulsors.network.propeller.tip_radius * 2 / Units.ft
        n_em = vehicle.propulsors.network.number_of_engines
        p_em = vehicle.propulsors.network.emotor.rated_power / Units.kW

        e_bat = (vehicle.propulsors.network.battery.mass_properties.mass * vehicle.propulsors.network.battery.specific_energy) / Units.kWh
        p_sp = (vehicle.propulsors.network.turboshaft.sea_level_power + vehicle.propulsors.network.emotor.rated_power) / Units.kW

        mc.airframe = 949.5 * w_s
        mc.gas_turbine = 493.6 * p_gt * n_gt
        mc.propeller = 210 * n_p * (d_p) ** 2 * (p_sp / d_p) ** 0.12
        mc.fuel_cell = 0.
        mc.hydrogen_tank = 0.
        mc.electrical_motors = 174 * (n_em * p_em + n_em_wtp * p_em_wtp)
        mc.power_management_system = 113.1 * (n_em * p_em + n_em_wtp * p_em_wtp)
        mc.battery = 1 * 150 * e_bat # Only one battery, no swapping considered

    mc.total = mc.airframe + mc.gas_turbine + mc.propeller + mc.fuel_cell + mc.hydrogen_tank + mc.electrical_motors + mc.power_management_system + mc.battery


    return mc