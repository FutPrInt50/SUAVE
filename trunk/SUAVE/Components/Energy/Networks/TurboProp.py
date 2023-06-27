## @ingroup Components-Energy-Networks
# TurboProp.py
# 
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by: 
#   Felipe Issamu Kitadani Odaguil, felipe.odaguil@embraer.com.br, Embraer S.A.
# 
# Created:  Mar 2021, F. Odaguil
# Modified: 

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np

from SUAVE.Core import Data, Units
from SUAVE.Components.Propulsors.Propulsor import Propulsor

# ----------------------------------------------------------------------
#  TurboProp Network
# ----------------------------------------------------------------------

class TurboProp(Propulsor):
    """ This is a turboprop. 
    
        Assumptions:
        None
        
    """      
    
    def __defaults__(self):
        """ This sets the default values for the network to function.
    
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
        
        #setting the default values
        self.tag = 'Turboprop'
        self.number_of_engines = 1.0
        self.nacelle_diameter  = 1.0
        self.engine_length     = 1.0
        self.bypass_ratio      = 1.0
        
        self.propeller         = None
        self.propeller_rpm     = 0.0
        self.thrust_angle      = 0.0
        self.sfc_scaling_factor = 1.0
        
    # linking the different network components
    def evaluate_thrust(self,state):

        #Unpack
        conditions        = state.conditions
        propeller         = self.propeller
        propeller_rpm     = self.propeller_rpm * np.ones_like(conditions.propulsion.throttle)
        neng              = self.number_of_engines
        gas_turbine       = self.gas_turbine

        # Run the gas turbine
        gas_turbine.inputs.throttle = conditions.propulsion.throttle
        gas_turbine.evaluate(conditions)
        gt_power = gas_turbine.outputs.power
        fuel_flow_rate = gas_turbine.outputs.fuel_flow * self.sfc_scaling_factor #/ conditions.propulsion.throttle**(1/2.5) if SFC is in idle to low
        resfn = gas_turbine.outputs.residual_thrust

        gas_turbine_torque = gt_power / propeller_rpm

        # Link
        propeller.inputs.omega = propeller_rpm
        propeller.thrust_angle = self.thrust_angle
        propeller.inputs.torque = gas_turbine_torque * self.gearbox.efficiency

        # Run the propeller
        # F, Q, P, Cp, outputs, etap = propeller.spin(conditions)
        F, Q, P, Cp = propeller.spin(conditions)

        # conditions.propulsion.propeller_efficiency = F*conditions.freestream.velocity/P

        # Pack the conditions for outputs
        rpm = propeller_rpm / Units.rpm
        a                                        = conditions.freestream.speed_of_sound
        R                                        = propeller.tip_radius

        conditions.propulsion.rpm = rpm
        conditions.propulsion.propeller_torque = Q
        conditions.propulsion.propeller_thrust = self.number_of_engines * (F + resfn)
        conditions.propulsion.propeller_power = P
        conditions.propulsion.propeller_rpm      = rpm
        conditions.propulsion.gas_turbine_power = gt_power

        # for plots
        conditions.propulsion.propeller_motor_torque = Q
        conditions.propulsion.propeller_power_coefficient = Cp
        conditions.propulsion.power              = gt_power * self.number_of_engines
        conditions.propulsion.propeller_tip_mach = (R*rpm*Units.rpm)/a
        conditions.propulsion.motor_torque       = Q

        conditions.propulsion.disc_loading       = (conditions.propulsion.propeller_thrust)/ (self.number_of_engines*np.pi*(R/Units.feet)**2)   # N/m^2
        conditions.propulsion.power_loading      = (conditions.propulsion.propeller_thrust)/(P / self.number_of_engines)    # N/W
        # conditions.noise.sources.propellers[prop.tag] = outputs

        # Create the outputs
        
        F = self.number_of_engines * (F + resfn) * [np.cos(self.thrust_angle), 0, -np.sin(self.thrust_angle)]

        mdot = fuel_flow_rate * self.number_of_engines
            
        results = Data()
        results.thrust_force_vector = F
        results.vehicle_mass_rate = mdot

        return results
    
    def unpack_unknowns(self,segment):
        """"""

        # Here we are going to unpack the unknowns (pitch_command) provided for this network
        # segment.state.conditions.propulsion.pitch_command = segment.state.unknowns.pitch_command
        # segment.state.conditions.propulsion.torque = segment.state.unknowns.torque

        return

    def residuals(self,segment):
        """"""

        # Here we are going to pack the residuals (power) from the network

        # Unpack
        # p_gas_turbine = segment.state.conditions.propulsion.gas_turbine_power
        # p_propeller   = segment.state.conditions.propulsion.propeller_power

        # Return the residuals
        # segment.state.residuals.net[:,0] = p_gas_turbine[:,0] - p_propeller[:,0]
        
        return

    __call__ = evaluate_thrust


