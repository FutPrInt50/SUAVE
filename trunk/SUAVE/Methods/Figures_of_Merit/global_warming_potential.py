## @ingroup Methods-Figures_of_Merit-Supporting_Functions
# global_warming_potential.py
#
# Created:  May 2022, J. Frank


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
import copy
from SUAVE.Core import Units
from scipy.interpolate import interp1d
from SUAVE.Core import Data

# ----------------------------------------------------------------------
#  Method
# ----------------------------------------------------------------------
## @ingroup Methods-Figures_of_Merit-Supporting_Functions
def global_warming_potential(results,vehicle,settings,LH2=False):
    """ This method computes the global warming potential of CO2 and NOx Emissions
    """

    # Unpack inputs
    alt_data = [0, 1000, 2000, 3000, 4000, 5000, 6000, 7000, 8000, 9000, 10000, 11000, 12000, 13000, 14000,
                15000] * Units.meter
    gwp_data = [-7.1, -7.1, -7.1, -4.3, -1.5, 6.5, 14.5, 37.5, 60.5, 64.7, 68.9, 57.7, 46.5, 25.6, 4.6, 0.6]
    gwp_interpol = interp1d(alt_data, gwp_data)

    p3t3 = vehicle.propulsors.network.turboshaft.p3t3_method

    weight_payload = results.segments[0].analyses.weights.mass_properties.payload

    gwp = Data()
    m_fuel = 0
    gwp.nox = 0.
    gwp.co2 = 0.
    for segment in results.segments.values():
        if ('reserve' not in segment.tag) and ('hold' not in segment.tag):
            alt = np.mean(segment.conditions.freestream.altitude[:, 0])
            if alt > alt_data[-1]:
                alt = alt_data[-1]
            if alt < alt_data[0]:
                alt = alt_data[0]
            gwp_iter = gwp_interpol(alt)
            if p3t3 == True:
                m_CO2 = segment.conditions.propulsion.co2_emissions_total[-1] - \
                        segment.conditions.propulsion.co2_emissions_total[0]
                m_NOX = segment.conditions.propulsion.nox_emissions_total[-1] - \
                        segment.conditions.propulsion.nox_emissions_total[0]
            else:
                if LH2 == True:
                    FB_A1 = 0.
                    FB_LH2 = segment.conditions.weights.total_mass[0] - \
                             segment.conditions.weights.total_mass[-1]
                else:
                    FB_A1 = segment.conditions.weights.total_mass[0] - \
                            segment.conditions.weights.total_mass[-1]
                    FB_LH2 = 0
                m_NOX = FB_A1 * 0.014 + FB_LH2 * 0.0043
                m_CO2 = FB_A1 * 3.15
            
            gwp.nox += m_NOX * gwp_iter
            gwp.co2 += m_CO2

    segments = [key for key in results.segments.keys() if ('reserve' not in key) and ('hold' not in key)]
    last_segment = segments[-1]

    # gwp.co2 = gwp.co2 / (results.segments[last_segment].conditions.frames.inertial.position_vector[-1][
    #                          0] / Units.km * vehicle.passengers)
    gwp.co2 = gwp.co2 / (results.segments[last_segment].conditions.frames.inertial.position_vector[-1][
                             0] / Units.km * weight_payload / 100)
    # gwp.nox = gwp.nox / (results.segments[last_segment].conditions.frames.inertial.position_vector[-1][
    #                          0] / Units.km * vehicle.passengers)
    gwp.nox = gwp.nox / (results.segments[last_segment].conditions.frames.inertial.position_vector[-1][
                             0] / Units.km * weight_payload / 100)

    print('FoM Payload: %.0f kg' % weight_payload)
    print('FoM Range: %.0f km' % (results.segments[last_segment].conditions.frames.inertial.position_vector[-1][0] / Units.km))
    return gwp