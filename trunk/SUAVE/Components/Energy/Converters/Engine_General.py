# gas_turbine_2019.py
#
# Created:  Feb 2020, M. Gallani

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE
import os
from SUAVE.Core import Data, Units

# package imports
import numpy as np
from scipy import interpolate, interp

from matplotlib import pyplot as plt
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from SUAVE.Methods.Propulsion import engine_interp_gasturb

# ----------------------------------------------------------------------
#  Gas Turbine Class
# ----------------------------------------------------------------------

class Engine_General(Energy_Component):

    def __defaults__(self):


        ### General Data
        self.tag                    = 'gas_turbine'
        
        self.rated_power            = 0.0  # reference engine sea-level power
        self.weight_model           = None
        self.performance_model      = engine_interp_gasturb

        ### Interp Method Data
        self.power_scaling_factor   = 1.0  # scaling factor (same for all ratings)
        self.gas_turbine_rating     = 'MCR'
        self.bucket                 = Data()
        self.bucket.RMTR            = []
        self.bucket.RSFC            = []
        self.data_file              = 'engine.out'
        self.tables                 = None
        self.power_extraction       = 0.0
        
        ### Simple Performance Method Data
        self.rated_power = 0.
        self.rated_speed = 0.
        self.torque_limit= 0.
        self.rated_sfc   = 0.
        
        ### Weight Model Data
        self.gasturbine_type        = 'turboprop' # turboprop or turboshaft

    def evaluate(self, conditions):
        """ The gas turbine output shaft power, fuel flow and residual thrust

        Inputs:
            inputs.throttle     [-]
            check method-specific inputs
                
        Outputs:
            Brake power (or Shaft power)
            Power (brake) specific fuel consumption
            Fuel flow
            Residual thrust

        """

        self.outputs = self.performance_model(self, conditions)

        conditions.propulsion.power = self.outputs.power
        conditions.propulsion.BSFC = self.outputs.specific_fuel_consumption
        conditions.propulsion.fuel_flow = self.outputs.fuel_flow
        conditions.propulsion.available_power = self.outputs.available_power

        return self.outputs

    def weight(self):

        W = self.mass_properties.mass = self.weight_model(self)

        return W
    
    def warnings(self,conditions):
        
        warnings = Data()
        
        if self.outputs.warnings.delta_isa_outofbound == True:
            warnings.update({'CAUTION 1':'Gas Turbine delta isa out of bounds. Extrapolated value may not be representative. Check gas_turbine_rating and .out file'})
        if self.outputs.warnings.Hpkft_outofbound == True:
            warnings.update({'CAUTION 2':'Gas Turbine altitude out of bounds. Extrapolated value may not be representative. Check gas_turbine_rating and .out file'})
        if self.outputs.warnings.KTAS_outofbound == True:
            warnings.update({'CAUTION 3':'Gas Turbine flight speed out of bounds. Extrapolated value may not be representative. Check gas_turbine_rating and .out file'})
        if self.outputs.warnings.power_unavailable == True:
            warnings.update({'WARNING 1':'More power required from gas turb than available'})
            
        return warnings
