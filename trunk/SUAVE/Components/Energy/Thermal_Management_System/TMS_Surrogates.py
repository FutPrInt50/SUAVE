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
import os
import pickle
# package imports
from SUAVE.Core import Units,Data
from SUAVE.Components.Energy.Energy_Component import Energy_Component

import numpy as np

# ----------------------------------------------------------------------
# Thermal_Management_System Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Thermal_Management_System
class TMS_Surrogates(Energy_Component):
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
        self.design_point = 0.0
        self.number_system = 1
        
    def build_surrogate(self):
        """ Build a surrogate. Multiple options for models are available including:

         Assumptions:
         None

         Source:
         N/A

         Inputs:
         state [state()]

         Outputs:
         self.sfc_surrogate    [fun()]



         Properties Used:
         Defaulted values
        """

        input_path = self.input_path
        tag = self.tag

        #file_name = 'interp_mdotair_CASE1.pck'
        file_name = f'interp_mdotair_{tag}.pck'
        input_file = os.path.join(input_path, file_name)

        # Load pickle files with interpolation functions
        with open(input_file, 'rb') as file_handle:
            # mdotair = interp_mdotair_CASE1((Mach, Altitude, Q_ratio, Design_point))
            # Inputs:
            #   Mach:         Mach number [-]
            #   Altitude:     Altitude [m]
            #   Q_ratio:      Ratio between operating and design point heat dissipation [-]
            #   Design_point: Design point heat dissipation [kW]
            # Outputs:
            #   mdotair:      Air mass flow [kg/s]
            self.interp_mdotair = pickle.load(file_handle)
            self.interp_mdotair.bounds_error = False
            self.interp_mdotair.fill_value = None #None --> extrapolation


        #file_name = 'interp_Drag_CASE1.pck'
        file_name = f'interp_Drag_{tag}.pck'
        input_file = os.path.join(input_path, file_name)

        with open(input_file, 'rb') as file_handle:
            # drag = interp_Drag_CASE1((Mach, Altitude, Q_ratio, Design_point))
            # Inputs:
            #   Mach:         Mach number [-]
            #   Altitude:     Altitude [m]
            #   Q_ratio:      Ratio between operating and design point heat dissipation [-]
            #   Design_point: Design point heat dissipation [kW]
            # Outputs:
            #   drag:         Drag [N]
            self.interp_Drag = pickle.load(file_handle)
            #self.interp_Drag.bounds_error = False
            self.interp_Drag.fill_value = None #None --> extrapolation


        #file_name = 'interp_Powerconsumption_CASE1.pck'
        file_name = f'interp_Powerconsumption_{tag}.pck'
        input_file = os.path.join(input_path, file_name)

        with open(input_file, 'rb') as file_handle:
            # Powerconsumption = interp_Powerconsumption_CASE1((Mach, Altitude, Q_ratio, Design_point))
            # Inputs:
            #   Mach:         Mach number [-]
            #   Altitude:     Altitude [m]
            #   Q_ratio:      Ratio between operating and design point heat dissipation [-]
            #   Design_point: Design point heat dissipation [kW]
            # Outputs:
            #   Powerconsumption: Total power consumption [W]
            self.interp_Powerconsumption = pickle.load(file_handle)

            self.mach_min_max = np.array([np.amin(self.interp_Powerconsumption.grid[0]), np.amax(self.interp_Powerconsumption.grid[0])])
            self.altitude_min_max = np.array([np.amin(self.interp_Powerconsumption.grid[1]), np.amax(self.interp_Powerconsumption.grid[1])])
            self.q_ratio_min_max = np.array([np.amin(self.interp_Powerconsumption.grid[2]), np.amax(self.interp_Powerconsumption.grid[2])])
            self.design_point_min_max = np.array([np.amin(self.interp_Powerconsumption.grid[3]), np.amax(self.interp_Powerconsumption.grid[3])])
            
            #self.interp_Powerconsumption.bounds_error = False
            #self.interp_Powerconsumption.fill_value = None #None --> extrapolation


        #file_name = 'interp_mTMS_CASE1.pck'
        file_name = f'interp_mTMS_{tag}.pck'
        input_file = os.path.join(input_path, file_name)

        with open(input_file, 'rb') as file_handle:
            # mTMS = interp_mTMS_CASE1(Design_point)
            # Inputs:
            #   Design_point: Design point heat dissipation [kW]
            # Outputs:
            #   mTMS:         TMS mass [kg]
            self.interp_mTMS = pickle.load(file_handle)
            self.interp_mTMS.bounds_error = False
            if tag == 'CASE1' or tag == 'CASE1_v2' or tag == 'CASE4 'or tag == 'CASE4_v2':
                self.interp_mTMS.fill_value = "extrapolate" # --> extrapolation
            if tag == 'CASE2' or tag == 'CASE6':
                self.interp_mTMS.fill_value = None


        if tag == 'CASE1' or tag == 'CASE1_v2' or tag == 'CASE4 'or tag == 'CASE4_v2':

            #file_name = 'interp_Area_CASE1.pck'
            file_name = f'interp_Area_{tag}.pck'
            input_file = os.path.join(input_path, file_name)

            with open(input_file, 'rb') as file_handle:
                # Area = interp_Area_CASE1(Design_point)
                # Inputs:
                #   Design_point: Design point heat dissipation [kW]
                # Outputs:
                #   Area:         Heat exchangers total frontal area [m2]
                self.interp_Area = pickle.load(file_handle)
                self.interp_Area.bounds_error = False
                self.interp_Area.fill_value = "extrapolate" # --> extrapolation

        if tag == 'CASE1_v2' or tag == 'CASE4_v2':

            file_name = f'interp_Area_exhaust_{tag}.pck'
            input_file = os.path.join(input_path, file_name)

            with open(input_file, 'rb') as file_handle:
                # Area = interp_Area_exhaust_CASE1(Design_point)
                # Inputs:
                #   Design_point: Design point heat dissipation [kW]
                # Outputs:
                #   Area:         Heat exchangers total frontal area [m2]
                self.interp_Area_exhaust = pickle.load(file_handle)
                self.interp_Area_exhaust.bounds_error = False
                self.interp_Area_exhaust.fill_value = "extrapolate" # --> extrapolation

            file_name = f'interp_Area_inlet_{tag}.pck'
            input_file = os.path.join(input_path, file_name)

            with open(input_file, 'rb') as file_handle:
                # Area = interp_Area_inlet_CASE1(Design_point)
                # Inputs:
                #   Design_point: Design point heat dissipation [kW]
                # Outputs:
                #   Area:         Heat exchangers total frontal area [m2]
                self.interp_Area_inlet = pickle.load(file_handle)
                self.interp_Area_inlet.bounds_error = False
                self.interp_Area_inlet.fill_value = "extrapolate" # --> extrapolation

        if tag == 'CASE6':

            file_name = f'interp_Area_NACA_{tag}.pck'
            input_file = os.path.join(input_path, file_name)

            with open(input_file, 'rb') as file_handle:
                # Area = interp_Area_NACA_CASE6(Design_point, SHX_A)
                # Inputs:
                #   Design_point: Design point heat dissipation [kW]
                #   SHX_A:        Skin heat exchanger area [m2]
                # Outputs:
                #   Area:         Heat exchangers total frontal area [m2]
                self.interp_Area_NACA = pickle.load(file_handle)
                self.interp_Area_NACA.bounds_error = False
                self.interp_Area_NACA.fill_value = None # --> extrapolation # check RegularGrid or Linear



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
        power_consumption = self.interp_Powerconsumption
        heat_load = self.inputs.heat_load[:,0]
        design_point = self.design_point

        if np.isnan(heat_load).any():
            #print('Info: heat_load is NAN')
            #conditions.propulsion.throttle = np.ones_like(throttle) * 0.5
            heat_load[np.isnan(heat_load)] = design_point*Units.kW * 0.5
        

        mach       = conditions.freestream.mach_number[:,0]
        altitude   = conditions.freestream.altitude[:,0]
        Q_ratio    = heat_load/(design_point*Units.kW)

        if any(mach < self.mach_min_max[0]) or any(mach > self.mach_min_max[1]):
            #print(f'replacing mach out of bounds\n{mach}')
            mach[mach < self.mach_min_max[0]] = self.mach_min_max[0]
            mach[mach > self.mach_min_max[1]] = self.mach_min_max[1]

        if any(altitude < self.altitude_min_max[0]) or any(altitude > self.altitude_min_max[1]):
            #print(f'replacing altitude out of bounds\n{altitude}')
            altitude[altitude < self.altitude_min_max[0]] = self.altitude_min_max[0]
            altitude[altitude > self.altitude_min_max[1]] = self.altitude_min_max[1]

        if any(Q_ratio < self.q_ratio_min_max[0]) or any(Q_ratio > self.q_ratio_min_max[1]):
            #print(f'replacing Q_ratio out of bounds\n{Q_ratio}')
            Q_ratio[Q_ratio < self.q_ratio_min_max[0]] = self.q_ratio_min_max[0]
            Q_ratio[Q_ratio > self.q_ratio_min_max[1]] = self.q_ratio_min_max[1]

        if (design_point < self.design_point_min_max[0]):
            #print(f'replacing design_point out of bounds\n{design_point}')
            design_point = self.design_point_min_max[0]
        elif (design_point > self.design_point_min_max[1]):
            #print(f'replacing design_point out of bounds\n{design_point}')
            design_point = self.design_point_min_max[1]

        if np.isnan(design_point).any() or np.isnan(Q_ratio).any() or np.isnan(altitude).any() or np.isnan(mach).any():
            total_power_consumption = np.ones_like(conditions.freestream.altitude) * 1000
            #print('TMS power is nan')
        else:
            if self.tag == 'CASE1' or self.tag == 'CASE1_v2' or self.tag == 'CASE4 'or self.tag == 'CASE4_v2':
                total_power_consumption = power_consumption((mach, altitude, Q_ratio, design_point))
            elif self.tag == 'CASE2' or self.tag == 'CASE6':
                total_power_consumption = power_consumption((mach, altitude, Q_ratio, design_point, self.shin_hx_area))

            total_power_consumption = total_power_consumption.reshape(len(total_power_consumption),1)

        if np.isnan(total_power_consumption).any():
            total_power_consumption = np.ones_like(total_power_consumption)

        #total_power_consumption = total_power_consumption.reshape(len(total_power_consumption),1)

        
        #todo problem at zero battery draw!!!! Check Felipe
        #if min(total_power_consumption) < 2000:
            #total_power_consumption = np.ones_like(total_power_consumption)*2000 #default value of 2000 W, if negative power consumption

        #total_power_consumption[total_power_consumption<0.] = 0.
        #total_power_consumption[total_power_consumption>50000.] = 50000.
        
        self.outputs.total_power_consumption      = total_power_consumption

        return total_power_consumption


    def tms_air_consumption(self,conditions):
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
        tms_air_consumption = self.interp_mdotair
        heat_load = self.inputs.heat_load[:,0]
        design_point = self.design_point


        mach       = conditions.freestream.mach_number[:,0]
        altitude   = conditions.freestream.altitude[:,0]
        Q_ratio    = heat_load/(design_point*Units.kW)

        if self.tag == 'CASE1' or self.tag == 'CASE1_v2' or self.tag == 'CASE4 'or self.tag == 'CASE4_v2':
            tms_air_consumption = tms_air_consumption((mach, altitude, Q_ratio, design_point))
        elif self.tag == 'CASE2' or self.tag == 'CASE6':
            tms_air_consumption = tms_air_consumption((mach, altitude, Q_ratio, design_point, self.shin_hx_area))

        tms_air_consumption = tms_air_consumption.reshape(len(tms_air_consumption),1)

        if np.isnan(tms_air_consumption).any():
            tms_air_consumption = np.zeros_like(tms_air_consumption)

        self.outputs.tms_air_consumption      = tms_air_consumption

        return tms_air_consumption