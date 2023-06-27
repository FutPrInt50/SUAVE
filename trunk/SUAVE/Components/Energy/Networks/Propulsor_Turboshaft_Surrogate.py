## @ingroup Components-Energy-Networks
# Propulsor_Turboshaft_Surrogate.py
#
# Created:  Mar 2017, E. Botero
# Modified: Jan 2020, T. MacDonald
#           May 2021, E. Botero
#           Sep 2021, J. Mangold

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from copy import deepcopy
from SUAVE.Components.Propulsors.Propulsor import Propulsor
from SUAVE.Methods.Utilities.Cubic_Spline_Blender import Cubic_Spline_Blender

from SUAVE.Core import Data, Units
import sklearn
from sklearn import gaussian_process
from sklearn.gaussian_process.kernels import RationalQuadratic, ConstantKernel, RBF, Matern
from sklearn import neighbors
from sklearn import svm, linear_model

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class Propulsor_Turboshaft_Surrogate(Propulsor):
    """ This is a way for you to load engine data from a source.
        A .csv file is read in, a surrogate made, that surrogate is used during the mission analysis.
        
        You need to use build surrogate first when setting up the vehicle to make this work.
        
        Assumptions:
        The input format for this should be Altitude, Mach, Throttle, Power, SFC(power-related)
        
        Source:
        None
    """        
    def __defaults__(self):
        """ This sets the default values for the network to function
        
            Assumptions:
            None
            
            Source:
            N/A
            
            Inputs:
            None
            
            Outputs:
            None
            
            Properties Used:
            N/A
        """          
        self.nacelle_diameter         = None
        self.engine_length            = None
        self.number_of_engines        = None
        self.tag                      = 'Engine_Deck_Surrogate'
        self.input_file               = None
        self.sfc_surrogate            = None
        self.thrust_surrogate         = None
        self.thrust_angle             = 0.0
        self.areas                    = Data()
        self.surrogate_type           = 'gaussian'
        self.altitude_input_scale     = 1.
        self.power_input_scale        = 1.
        self.power_anchor_scale        = 1.
        self.sfc_anchor               = None
        self.sfc_anchor_scale         = 1.
        self.sfc_anchor_conditions    = np.array([[1.,1.,1.]])
        self.thrust_anchor            = None
        self.thrust_anchor_scale      = 1.
        self.thrust_anchor_conditions = np.array([[1.,1.,1.]])
        self.sfc_rubber_scale         = 1.
        self.use_extended_surrogate   = False
        self.negative_throttle_values = False


   
    # manage process with a driver function
    def evaluate_thrust(self,state):
        """ Calculate thrust given the current state of the vehicle
        
            Assumptions:
            None
            
            Source:
            N/A
            
            Inputs:
            state [state()]
            
            Outputs:
            results.thrust_force_vector [newtons]
            results.vehicle_mass_rate   [kg/s]
            
            Properties Used:
            Defaulted values
        """
        
        # Unpack the conditions
        conditions = state.conditions
        # rescale altitude for proper surrogate performance
        altitude   = conditions.freestream.altitude/self.altitude_input_scale
        mach       = conditions.freestream.mach_number
        throttle   = conditions.propulsion.throttle
        delta_ISA  = conditions.freestream.delta_ISA

        ################################################################################################################
        turboshaft = self.turboshaft
        
        # Run the engine
        turboshaft.power(conditions)
        power = turboshaft.outputs.power     
        sfc          = turboshaft.outputs.power_specific_fuel_consumption
        mdot_one_engine = turboshaft.outputs.mdot_one_engine
        residual_thrust_one_engine = turboshaft.outputs.residual_thrust_one_engine

        if turboshaft.p3t3_method == True:
            p3  = turboshaft.outputs.p3
            t3  = turboshaft.outputs.t3
            far = turboshaft.outputs.far

        ################################################################################################################
        # Propeller and Gearbox
        propeller = self.propeller
        gearbox = self.gearbox

        # link
        propeller.inputs.omega = state.conditions.propulsion.rpm * Units.rpm
        propeller.inputs.torque = power / propeller.inputs.omega * gearbox.efficiency

        # step 4
        F, Q, P, Cp = propeller.spin(conditions)

        ################################################################################################################

        # Results

        # Pack the conditions for outputs
        a = conditions.freestream.speed_of_sound
        R = propeller.tip_radius
        rpm = propeller.inputs.omega / Units.rpm

        num_engines = (self.number_of_engines - self.one_engine_propeller_inoperative)

        conditions.propulsion.rpm = rpm
        conditions.propulsion.propeller_rpm = rpm
        conditions.propulsion.propeller_torque = Q
        conditions.propulsion.propeller_motor_torque = Q
        conditions.propulsion.propeller_power_coefficient = Cp
        conditions.propulsion.power = P * num_engines
        conditions.propulsion.propeller_tip_mach = (R * rpm * Units.rpm) / a
        conditions.propulsion.motor_torque = propeller.inputs.torque

        if turboshaft.p3t3_method == True:
            conditions.propulsion.gas_turbine_p3 = p3
            conditions.propulsion.gas_turbine_t3  = t3
            conditions.propulsion.gas_turbine_far = far

        # noise
        # outputs.number_of_engines                = num_engines
        # conditions.noise.sources.propeller       = outputs

        # Create the outputs
        F = num_engines * (F + residual_thrust_one_engine) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]
        F_mag = np.atleast_2d(np.linalg.norm(F, axis=1))
        conditions.propulsion.disc_loading = (F_mag.T) / (num_engines * np.pi * (R / Units.feet) ** 2)  # N/m^2
        conditions.propulsion.power_loading = (F_mag.T) / (P / num_engines)  # N/W
        conditions.propulsion.propeller_thrust = F_mag.T

        results = Data()
        results.thrust_force_vector = F
        results.vehicle_mass_rate = mdot_one_engine * num_engines

        ################################################################################################################

   
        return results          


    def unpack_unknowns(self, segment):
        """Unpacks the unknowns set in the mission to be available for the mission.

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:
        state.unknowns.rpm                 [RPM]

        Outputs:
        state.conditions.propulsion.rpm    [RPM]


        Properties Used:
        N/A
        """

        segment.state.conditions.propulsion.rpm = segment.state.unknowns.rpm

        return

    def residuals(self, segment):
        """ Calculates a residual based on torques

        Assumptions:

        Inputs:

        Outputs:

        Properties Used:
            N/A

        """

        # Unpack
        # fake propeller rpm
        if 'parameter_rpm' in segment.keys():
            rpm_actual = segment.parameter_rpm * np.ones_like(segment.state.unknowns.rpm)
        else:
            rpm_actual = self.propeller.nominal_rpm  * np.ones_like(segment.state.unknowns.rpm)
        rpm_predict = segment.state.unknowns.rpm
        segment.state.residuals.network[:,0] = (rpm_actual[:,0] - rpm_predict[:,0])/rpm_actual[:,0]

        return