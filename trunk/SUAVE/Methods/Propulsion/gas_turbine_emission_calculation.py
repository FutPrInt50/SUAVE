## gas_turbine_emission_calculation.py
#
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Dominik Eisenhut, eisenhut@ifb.uni-stuttgart.de, University of Stuttgart
#   Hossein B. Enalou, Cranfield University
#
# Created:  Jul 2022, D. Eisenhut (derived from file of H. Enalou)
# Modified:

"""Sets the specified conditions which are given for the segment type.

    Assumptions:


    Source:
    N/A

    Inputs:
    results
    vehicle or configs.base

    Outputs:
    results

    Properties Used:
    N/A
    """

from math import exp
import numpy as np

from SUAVE.Core import Data


def gas_turbine_calculate_emission(results, vehicle, x=1., y=0.4, z=0.):
    # unpack ICAO reference values for turboshaft
    ei_nox_sl_icao = vehicle.propulsors.network.turboshaft.emissions_icao.EI_nox_sea_level
    ei_co_sl_icao = vehicle.propulsors.network.turboshaft.emissions_icao.EI_co_sea_level
    p3_sl_icao = vehicle.propulsors.network.turboshaft.emissions_icao.p3_sea_level_icao
    t3_sl_icao = vehicle.propulsors.network.turboshaft.emissions_icao.t3_sea_level_icao
    far_sl_icao = vehicle.propulsors.network.turboshaft.emissions_icao.far_sl_icao

    prev_co  = 0
    prev_co2 = 0
    prev_nox = 0

    for segment in results.segments:
        # check if mission results have humidity, if use it, otherwise humidity is ignored
        if hasattr(segment.conditions.freestream, "humidity"):
            humidity = segment.conditions.freestream.humidity
        else:
            humidity = 0

        # unpack mission values at altitude
        p3_alt = segment.conditions.propulsion.gas_turbine_p3
        t3_alt = segment.conditions.propulsion.gas_turbine_t3
        far_alt = segment.conditions.propulsion.gas_turbine_far
        fuel_flow = segment.conditions.weights.vehicle_mass_rate

        # calculate sea level CO emission index for actual temperature at mission conditions
        co_sl_func = np.poly1d(np.polyfit(t3_sl_icao, ei_co_sl_icao, 3))
        ei_co_sl = co_sl_func(t3_alt)

        # calculate sea level NOx emission index for actual temperature at mission conditions
        nox_sl_func = np.poly1d(np.polyfit(t3_sl_icao, ei_nox_sl_icao, 3))
        ei_nox_sl = nox_sl_func(t3_alt)

        # calculate actual pressure p3 for actual temperature at mission condition
        p3_sl_func = np.poly1d(np.polyfit(t3_sl_icao, p3_sl_icao, 3))
        p3_sl = p3_sl_func(t3_alt)

        far_sl_func = np.poly1d(np.polyfit(t3_sl_icao, far_sl_icao, 3))
        far_sl = far_sl_func(t3_alt)

        # calculate actual emission index at altitude for given conditions
        segment.conditions.propulsion.co_emissions_index  = ei_co_sl * (p3_alt / p3_sl)**x * (far_alt / far_sl)**z   # g/kg
        segment.conditions.propulsion.co2_emissions_index = 3160 * np.ones(segment.conditions.propulsion.co_emissions_index.shape) # g/kg # ICAO Carbon Calculator https://www.icao.int/environmental-protection/CarbonOffset/Documents/Methodology%20ICAO%20Carbon%20Calculator_v10-2017.pdf [01.09.22]
        segment.conditions.propulsion.nox_emissions_index = ei_nox_sl * (p3_alt / p3_sl) ** y * exp(humidity) * (
                    far_alt / far_sl) ** z  # g/kg

        # calculate emission flow rates
        co_rate  = segment.conditions.propulsion.co_emissions_index * fuel_flow / 1000      # kg/s
        co2_rate = segment.conditions.propulsion.co2_emissions_index * fuel_flow / 1000     # kg/s
        nox_rate = segment.conditions.propulsion.nox_emissions_index * fuel_flow / 1000     # kg/s

        # get total emissions until mission point
        I = segment.state.numerics.time.integrate

        segment.conditions.propulsion.co_emissions_total  = prev_co + np.dot(I, co_rate)    # kg
        segment.conditions.propulsion.co2_emissions_total = prev_co2 + np.dot(I, co2_rate)   # kg
        segment.conditions.propulsion.nox_emissions_total = prev_nox + np.dot(I, nox_rate)  # kg

        # set overall emissions for last point in segment -> carried over in next for loop index/segment
        prev_co  = segment.conditions.propulsion.co_emissions_total[-1, 0]
        prev_co2 = segment.conditions.propulsion.co2_emissions_total[-1, 0]
        prev_nox = segment.conditions.propulsion.nox_emissions_total[-1, 0]

    return results
