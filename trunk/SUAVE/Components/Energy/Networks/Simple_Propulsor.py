## @ingroup Components-Energy-Networks
# Simple_Propulsor.py
# 
# Created:  Ago 2018, M. Gallani


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from SUAVE.Components.Propulsors.Propulsor import Propulsor

from SUAVE.Core import Data, Units

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class Simple_Propulsor(Propulsor):
    """ A simple prpulsor network that just outputs thrust as a function of a Max Thrust and throttle settings.
        
        Unknowns:
            Throttle
    
        Assumptions:
        
        Source:
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

        self.max_thrust        = 0.
        self.fuel_flow         = 0. 
        self.nacelle_diameter  = None
        self.engine_length     = None
        self.number_of_engines = None
        self.tag               = 'network'

    
    # manage process with a driver function
    def evaluate_thrust(self,state):
        """ Calculate thrust given the current state of the vehicle
    
            Assumptions:
                
            Source:

    
            Inputs:

    
            Outputs:
    
    
            Properties Used:
            Defaulted values
        """          
    
        # unpack
        conditions  = state.conditions
        max_thrust  = self.max_thrust
        fuel_flow   = self.fuel_flow
        eta         = conditions.propulsion.throttle[:,0,None]  #beschreibt Stellung des Schubhebels
        
#        eta[eta > 1.0] = 1.0
       
        #Create the outputs
        F    = eta * max_thrust * [1,0,0]  

        
        mdot = eta*fuel_flow
        
             
        conditions.propulsion.thrust = F
        
        results = Data()
        results.thrust_force_vector = F
        results.vehicle_mass_rate   = mdot

        return results
    
    def weight(self):
        """ Calculates the weight of the propulsion system
            Does not include battery weight, payload nor avionics.
            Battery weight should be defined and calculated individually
            Avionics and payload weight should be included on the empty weight script 
        """
        
        # List of which components' weights shall be calculated and included on the propulsive system weight
        items_list = []
        
        # Initialize weight at zero
        weight = 0
        
        # Run the list of items and calculate the weight for each one
        # Add everything to come up with the total system weight
        for i in items_list:
            W = self[i].weight()
            weight += W
        
        # Output the weight
        self.mass_properties.mass = weight
        
        return weight
    
    
    def unpack_unknowns(self,segment,state):
        """ This is an extra set of unknowns which are unpacked from the mission solver and send to the network.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state.unknowns.propeller_power_coefficient [None]
    
            Outputs:
            state.conditions.propulsion.propeller_power_coefficient [None]
    
            Properties Used:
            N/A
        """       
        
        return
    
    def residuals(self,segment,state):
        """ This packs the residuals to be send to the mission solver.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            state.conditions.propulsion:
                motor_torque                          [N-m]
                propeller_torque                      [N-m]
            
            Outputs:
            None
    
            Properties Used:
            None
        """  
        
        # Here we are going to pack the residuals from the network
        # Equation 1: Power balance between motor and propeller
        
        return        
            
    __call__ = evaluate_thrust