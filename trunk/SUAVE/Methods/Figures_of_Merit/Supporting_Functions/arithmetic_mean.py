## @ingroup Methods-Figures_of_Merit-Supporting_Functions
# arithmetic_mean.py
#
# Created:  May 2022, J. Frank


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np

# ----------------------------------------------------------------------
#  Method
# ----------------------------------------------------------------------
## @ingroup Methods-Figures_of_Merit-Supporting_Functions
def arithmetic_mean(values, weights=[1]):
    """ This method returns a value between 0 and 1, where 0 equals the first array entry and 1 the second entry
    """
    if len(weights) != len(values):
        weights = np.ones_like(values)
        print('Warning: Weights and values have different lengths. Weights are set to 1.')
    values = np.array(values)
    weights = np.array(weights)

    am = np.sum(values * weights)/np.sum(weights)

    return am