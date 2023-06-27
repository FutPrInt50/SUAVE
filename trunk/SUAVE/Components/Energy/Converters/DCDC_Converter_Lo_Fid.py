## @ingroup Components-Energy-Converters
# DCDC_Converter_Lo_Fid.py
#
# Created:  Feb 2022, B. Zaghari and J. Mangold
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np

# suave imports
import SUAVE

# package imports
from SUAVE.Components.Energy.Energy_Component import Energy_Component

# ----------------------------------------------------------------------
#  Converter DC/DC Class
# ----------------------------------------------------------------------
## @ingroup Components-Energy-Converters
class DCDC_Converter_Lo_Fid(Energy_Component):
    """This is a low-fidelity DCDC converter component.
    
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
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        None
        """
        self.bus_voltage        = 0.0
        self.inputs.eta         = 1.0


    def covert_lo(self,conditions):
        """Calculates the coverters's XXX

        Assumptions:
    
        Source:
        N/A
    
        Inputs: 
        self.
            inputs.currentOut       [A]
            inputs.voltageIn       [V]
    
        Outputs:
        self.outputs.
            outputs.powerIn         [W]
            outputs.currentIn       [A]
           
        Properties Used:
        self.
            bus_voltage             [V]

        """          

        dcdc_voltageOut = self.bus_voltage

        dcdc_currentOut = self.inputs.currentOut
        dcdc_voltageIn = self.inputs.voltageIn

        dcdc_powerOut = dcdc_currentOut * dcdc_voltageOut

        dcdc_efficiency = 1 - 0.9* np.abs(np.log10(dcdc_voltageOut / dcdc_voltageIn))
        # this is based on a basic relation between voltage of the battery
        # and voltage of the bus. The efficiency is defined based in the bus voltage
        # dcdc_VoltageOut and the voltage comming from the battery (dcdc_voltageIn)
        # later this can be changed to an efficiency map based on the component and
        # topology of the converter. bus voltage will be studied depends on the
        # simulation.

        dcdc_powerIn =  dcdc_powerOut / dcdc_efficiency**(np.sign(dcdc_currentOut))

        dcdc_currentIn = dcdc_powerIn / dcdc_voltageIn
        
        self.outputs.powerIn = dcdc_powerIn
        self.outputs.currentIn = dcdc_currentIn


        return dcdc_powerIn, dcdc_currentIn