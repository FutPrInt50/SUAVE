## @ingroup Components-Energy-Converters
# Combustion_Turboprop.py
#
# Created:  Sep, 2021: J. Mangold
# Modified
#

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE
from SUAVE.Core import Data, Units

# package imports
import numpy as np
from SUAVE.Components.Energy.Energy_Component import Energy_Component

# ----------------------------------------------------------------------
#  Internal Combustion Engine Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class Combustion_Turboprop(Energy_Component):
    """This is an internal combustion engine component.
    
    Assumptions:
    None

    Source:
    None
    """           
    def __defaults__(self):

        self.sea_level_power                 = 0.0
        self.flat_rate_altitude              = 0.0
        self.rated_speed                     = 0.0
        self.inputs.speed                    = 0.0
        self.power_specific_fuel_consumption = 0.45 # lb/hr/hp :: see Gudmundsson
        self.max_power                       = 0.0

    def power(self,conditions):
        """ The Combustion_Turboprop output power and specific power consumption
        Inputs:
            Engine:
                sea-level power
                flat rate altitude
                rated_speed (RPM)
                throttle setting
                inputs.speed (RPM)
            Freestream conditions:
                altitude
                delta_isa
        Outputs:
            Brake power (or Shaft power)
            Power (brake) specific fuel consumption
            Fuel flow
            Torque
        """

        # Unpack
        altitude                         = conditions.freestream.altitude
        delta_isa                        = conditions.freestream.delta_ISA
        V                                = conditions.freestream.velocity[:, 0, None]
        throttle                         = conditions.propulsion.combustion_engine_throttle
        PSLS                             = self.sea_level_power
        h_flat                           = self.flat_rate_altitude
        speed                            = self.inputs.speed
        power_specific_fuel_consumption  = self.power_specific_fuel_consumption

        time                             = conditions.frames.inertial.time / Units.min

        if time[-1] < 1.5 and self.max_power != 0: #(5) [1.5] # maximum power under 5 min available
            PSLS = self.max_power


        altitude_virtual = altitude - h_flat       # shift in power lapse due to flat rate
        altitude_virtual[altitude_virtual<0.] = 0. # don't go below sea level
        
        atmo             = SUAVE.Analyses.Atmospheric.US_Standard_1976()
        atmo_values      = atmo.compute_values(altitude_virtual,delta_isa)
        p                = atmo_values.pressure
        T                = atmo_values.temperature
        rho              = atmo_values.density
        a                = atmo_values.speed_of_sound
        mu               = atmo_values.dynamic_viscosity


        # computing the sea-level ISA atmosphere conditions
        atmo_values = atmo.compute_values(0,0)
        p0          = atmo_values.pressure[0,0]
        T0          = atmo_values.temperature[0,0]
        rho0        = atmo_values.density[0,0]
        a0          = atmo_values.speed_of_sound[0,0]
        mu0         = atmo_values.dynamic_viscosity[0,0]

        # calculating the density ratio:
        sigma = rho / rho0

        # calculating Mach-numer:
        Ma = V / a

        # calculating temperature ratio:
        theta = T / T0 * (1 + (1.4 - 1) / 2 * Ma**2)

        # calculating pressure ratio:
        delta = p / p0 * (1 + (1.4 - 1) / 2 * Ma**2) ** (1.4 / (1.4 - 1))

        # F_F_SL = np.zeros_like(Ma)
        # i = 0
        # for V in V:
        #
        #     if Ma[i] < 0.1:
        #         #F = F_SL * delta
        #         F_F_SL[i] = delta[i]
        #     elif Ma[i] > 0.1 and theta[i] < throttle[i]:
        #         F_F_SL[i] = delta[i] * (1 - 0.96 * (Ma[i] - 0.1) ** 0.25)
        #     elif Ma[i] > 0.1 and theta[i] > throttle[i]:
        #         F_F_SL[i] = delta[i] * (1 - 0.96 * (Ma[i] - 0.1) ** 0.25 - 3 * (theta[i] - throttle[i]) / (8.13 * (Ma[i] - 0.1)))
        #     else:
        #         F = 0
        #
        #     i = i+1

        #Pavailable =  PSLS * sigma ** 0.6 * (T0 / T) ** 0.9# * F_F_SL / 0.4
        #Pavailable = PSLS * p / p0 ** 1. * (T0 / T) ** 0.5
        Pavailable = PSLS * (p / p0) ** 1. * (T0 / T) ** 0.5





        # calculating available power based on Gagg and Ferrar model (ref: # - eq. 7-16)
        #Pavailable                    = PSLS * (sigma - 0.117) / 0.883
        #Pavailable                    = PSLS * (1.132 * sigma - 0.132)
        #Pavailable = PSLS
        Pavailable[h_flat > altitude] = PSLS

        # applying throttle setting
        output_power                  = Pavailable * throttle 
        output_power[output_power<0.] = 0.
        SFC                           = power_specific_fuel_consumption * Units['lb/hp/hr'] / throttle**(1/2.5) #1/2
        #SFC                           = power_specific_fuel_consumption * Units['lb/hp/hr'] * (1 / (throttle-0.25) ** (1 / 5) - 0.05)  # 1/(x-0.25)^(1/5)-0.05

        SFC[SFC < power_specific_fuel_consumption * Units['lb/hp/hr']] = power_specific_fuel_consumption * Units['lb/hp/hr'] #* 1.2

        #fuel flow rate
        a               = np.zeros_like(altitude)
        fuel_flow_rate  = np.fmax(output_power*SFC,a)

        #torque
        torque = output_power/speed
        
        # store to outputs
        self.outputs.power                           = output_power
        self.outputs.power_specific_fuel_consumption = power_specific_fuel_consumption
        self.outputs.fuel_flow_rate                  = fuel_flow_rate
        self.outputs.torque                          = torque

        return self.outputs
    
    
    def calculate_throttle(self,conditions):
        """ The internal combustion engine output power and specific power consumption
        
        source: 
        
        Inputs:
            Engine:
                sea-level power
                flat rate altitude
                rated_speed (RPM)
                throttle setting
                inputs.power
            Freestream conditions:
                altitude
                delta_isa
        Outputs:
            Brake power (or Shaft power)
            Power (brake) specific fuel consumption
            Fuel flow
            Torque
            throttle setting
        """

        # Unpack
        altitude                         = conditions.freestream.altitude
        delta_isa                        = conditions.freestream.delta_ISA
        PSLS                             = self.sea_level_power
        h_flat                           = self.flat_rate_altitude
        power_specific_fuel_consumption  = self.power_specific_fuel_consumption
        output_power                     = self.inputs.power*1.0


        altitude_virtual = altitude - h_flat       # shift in power lapse due to flat rate
        altitude_virtual[altitude_virtual<0.] = 0. # don't go below sea level
        
        atmo             = SUAVE.Analyses.Atmospheric.US_Standard_1976()
        atmo_values      = atmo.compute_values(altitude_virtual,delta_isa)
        p                = atmo_values.pressure
        T                = atmo_values.temperature
        rho              = atmo_values.density
        a                = atmo_values.speed_of_sound
        mu               = atmo_values.dynamic_viscosity

        # computing the sea-level ISA atmosphere conditions
        atmo_values = atmo.compute_values(0,0)
        p0          = atmo_values.pressure[0,0]
        T0          = atmo_values.temperature[0,0]
        rho0        = atmo_values.density[0,0]
        a0          = atmo_values.speed_of_sound[0,0]
        mu0         = atmo_values.dynamic_viscosity[0,0]

        # calculating the density ratio:
        sigma = rho / rho0

        # calculating available power based on Gagg and Ferrar model (ref: S. Gudmundsson, 2014 - eq. 7-16)
        Pavailable                    = PSLS * (sigma - 0.117) / 0.883        
        Pavailable[h_flat > altitude] = PSLS


        # applying throttle setting
        throttle = output_power/Pavailable 
        output_power[output_power<0.] = 0. 
        SFC                           = power_specific_fuel_consumption * Units['lb/hp/hr']

        #fuel flow rate
        a               = np.zeros_like(altitude)
        fuel_flow_rate  = np.fmax(output_power*SFC,a)

        
        # store to outputs
        self.outputs.power_specific_fuel_consumption = power_specific_fuel_consumption
        self.outputs.fuel_flow_rate                  = fuel_flow_rate
        self.outputs.throttle                        = throttle

        return self.outputs    

