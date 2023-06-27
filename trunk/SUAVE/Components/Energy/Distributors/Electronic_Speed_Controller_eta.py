## @ingroup Components-Energy-Distributors
# Electronic_Speed_Controller_eta.py
#
# Created:  Jun 2014, E. Botero
# Modified: Jan 2016, T. MacDonald
#           Sep 2021, J. Mangold

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE
import numpy as np

from SUAVE.Components.Energy.Energy_Component import Energy_Component

# ----------------------------------------------------------------------
#  Electronic Speed Controller Class
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Distributors
class Electronic_Speed_Controller_eta(Energy_Component):
    
    def __defaults__(self):
        """ This sets the default values.
    
            Assumptions:
            None
    
            Source:
            N/A
    
            Inputs:
            None
    
            Outputs:
            None
    
            Properties Used:
            None
            """         
        
        self.efficiency = 0.0
    
    def voltageout(self,conditions,eta):
        """ The voltage out of the electronic speed controller
        
            Assumptions:
            The ESC's output voltage is linearly related to throttle setting
    
            Source:
            N/A
    
            Inputs:
            conditions.propulsion.throttle [0-1] 
            self.inputs.voltage            [volts]
    
            Outputs:
            voltsout                       [volts]
            self.outputs.voltageout        [volts]
    
            Properties Used:
            None
           
        """
        # Unpack, don't modify the throttle
        #eta = (conditions.propulsion.throttle[:,0,None])*1.0
        #eta = conditions.electric_throttle_sum[:,0,None]
        
        # Negative throttle is bad
        #eta[eta<=0.0] = 0.0 # maybe not for battery charging
        
        # Cap the throttle
        #eta[eta>=1.0] = 1.0
        
        voltsin  = self.inputs.voltagein
        voltsout = abs(eta)*voltsin
        
        # Pack the output
        self.outputs.voltageout = voltsout # fix for voltage is always postive, that with negativ current battery charging is possible
        
        return voltsout
    
    def currentin(self,conditions,eta):
        """ The current going into the speed controller
        
            Assumptions:
                The ESC draws current.
            
            Inputs:
                self.inputs.currentout [amps]
               
            Outputs:
                outputs.currentin      [amps]
            
            Properties Used:
                self.efficiency - [0-1] efficiency of the ESC
               
        """
        
        # Unpack, don't modify the throttle
        #eta = (conditions.propulsion.throttle[:,0,None])*1.0
        #eta = conditions.electric_throttle_sum[:, 0, None]
        currentout = self.inputs.currentout
        eff        = self.efficiency * np.ones_like(currentout)

        currentin = currentout * abs(eta) * eff**(-np.sign(currentout))
        #currentin[eta>0]  = currentout[eta>0]*abs(eta[eta>0])/eff
        
        # Pack
        self.outputs.currentin = currentin
        self.outputs.power_in  = self.outputs.voltageout*currentin
        
        return currentin