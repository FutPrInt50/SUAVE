## @ingroup Components-Energy-Distributors
# Cable_Power.py
#
# Created:  Feb 2022, J.Mangold
# Modified: 
#

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE
import numpy as np

from SUAVE.Components.Energy.Energy_Component import Energy_Component

# ----------------------------------------------------------------------
#  Cable Power Class
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Distributors
class Cable_Power(Energy_Component):
    
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