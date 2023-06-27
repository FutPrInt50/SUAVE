## @ingroup Methods-Figures_of_Merit-Supporting_Functions
# minmax.py
#
# Created:  May 2022, J. Frank


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------


# ----------------------------------------------------------------------
#  Method
# ----------------------------------------------------------------------
## @ingroup Methods-Figures_of_Merit-Supporting_Functions
def minmax(value, borders):
    """ This method returns a value between 0 and 1, where 0 equals the first array entry and 1 the second entry
    """

    min = borders[0]
    max = borders[1]

    minmax =  1 - (value - min)/(max-min)

    return minmax