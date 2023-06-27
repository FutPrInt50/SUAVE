## @ingroup Components-Energy-Thermal_Management_System
# Thermal_Managment_System.py
#
# Created:  Nov 2021, F. R. Barbosa
# Modified: Mar 2022, J. Mangold
#
# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
# suave imports
import SUAVE

# package imports
from SUAVE.Core import Units,Data
from SUAVE.Components.Energy.Energy_Component import Energy_Component

import numpy as np

# ----------------------------------------------------------------------
# Thermal_Management_System Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Thermal_Management_System
class Thermal_Management_System(Energy_Component):
    """This is a thermal management system component.

    Assumptions:
    None

    Source:
    None
    """
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
        None

        Source:
        None

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        None
        """
        self.shx_area = 0.0
        self.max_heat_load = 0.0


    def power_consumption(self,conditions):
        """
        For a given heat load and altitude, determines the power consumption of the system

        Assumptions:


        Inputs:
        heat load: [kW]
        altitude: [ft]
        SHX area: [m**2]


        Outputs:
        total_power_consumption
        fan_power_consumption
        pump_power_consumption
        compressor_power_consumption

        """
        # Unpack
        heat_load = self.inputs.heat_load
        altitude_ft = conditions.freestream.altitude / Units.ft


        #shx_area = Thermal_Management_System.shx_area # check this
        if self.shx_area == 2:
            a_total      = -0.0003*(heat_load/1000) - 0.0744
            b_total      = 238.2*(heat_load/1000) - 4932.3
            a_compressor = -0.0696
            b_compressor = 221.29*(heat_load/1000) - 3806.2
            a_fan        = 20.594
            b_fan        = -334.8

            total_power_consumption      = a_total*altitude_ft + b_total
            compressor_power_consumption = a_compressor*altitude_ft + b_compressor


        elif self.shx_area == 3:
            a_total      = -0.0004*(heat_load/1000) - 0.0954
            b_total      = 237.01*(heat_load/1000) - 6385.3
            a_compressor = -0.1045
            b_compressor = 221.09*(heat_load/1000) - 5599.5
            a_fan        = 20.585
            b_fan        = -497.3

            total_power_consumption      = a_total*altitude_ft + b_total
            compressor_power_consumption = a_compressor*altitude_ft + b_compressor


        if altitude_ft.all == 0: #change to dynamic pressure
            fan_power_consumption = a_fan*(heat_load/1000) + b_fan
        elif altitude_ft.any != 0:
            fan_power_consumption = 0

        #todo problem at zero battery draw!!!! Check Felipe
        if min(total_power_consumption) < 2000:
            total_power_consumption = np.ones_like(total_power_consumption)*2000 #default value of 2000 W, if negative power consumption

        self.outputs.compressor_power_consumption = compressor_power_consumption
        self.outputs.fan_power_consumption        = fan_power_consumption
        self.outputs.total_power_consumption      = total_power_consumption

        return total_power_consumption