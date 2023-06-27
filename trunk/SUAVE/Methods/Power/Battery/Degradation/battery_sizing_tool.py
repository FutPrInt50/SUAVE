# -*- coding: utf-8 -*-
# battery_sizing_tool v1.0
# Copyright CEA, 2022

import math
import warnings

import numpy as np
import scipy.io as sio
import scipy.interpolate as sipint
from typing import Union
import logging
from math import ceil
import time

# Constants
KELVIN_TO_C = 273.15
use_logging = False

# Logging informations
if use_logging:
    logging.basicConfig(filename='journal.log', encoding='utf-8', level=logging.DEBUG, filemode='w',
                        format='%(levelname)s %(asctime)s >> %(message)s', datefmt='%a %d/%m/%Y %H:%M:%S')


class BatteryPack:

    def __init__(self, cell_parameters, pack_parameters, study_parameters, stop_after_durability_reached=True):

        # Attributes defines by user or external data
        self.cell_model_path = cell_parameters["cell_model_path"]
        self.cell_mass = cell_parameters["cell_mass"]
        self.cell_max_voltage = cell_parameters["cell_max_voltage"]
        self.cell_min_voltage = cell_parameters["cell_min_voltage"]
        self.cell_nominal_voltage = cell_parameters["cell_nominal_voltage"]
        self.cell_nominal_energy = cell_parameters["cell_nominal_energy"]
        self.cell_initial_temperature = cell_parameters["cell_initial_temperature"]

        self.soh = pack_parameters["initial_soh"]
        self.soc = pack_parameters["initial_soc"]
        self.__number_cells_series = None
        self.__number_branches = None
        self.initial_soc = pack_parameters["initial_soc"]
        self.initial_soh = pack_parameters["initial_soh"]
        self.pack_nominal_voltage = pack_parameters["pack_nominal_voltage"]
        self.min_soc = pack_parameters["min_soc"]
        self.max_soc = pack_parameters["max_soc"]

        self.durability_target = study_parameters["durability_target"]  # in month
        self.pack_power = study_parameters["pack_power"]
        self.time_data = study_parameters["time_data"]
        self.pack_checkup_power = study_parameters["pack_checkup_power"]
        self.time_data_checkup = study_parameters["time_data_checkup"]
        self.maximum_branches_number = study_parameters["maximum_branches_number"]
        self.soh_interval = study_parameters["soh_interval"]

        # Attributes computed with class methods or read
        self.__cell_temperature = self.cell_initial_temperature
        self.__mass = None
        self.__volume = 0
        self.__soc_map = np.zeros(0)
        self.__voltage_map = np.zeros(0)
        self.__heat_loss_map = np.zeros(0)
        self.__soh_map = np.zeros(0)
        self.__breakpoints_power = np.zeros(0)
        self.__breakpoints_initial_soc = np.zeros(0)
        self.__breakpoints_temperature = np.zeros(0)
        self.__breakpoints_initial_soh = np.zeros(0)
        self.__breakpoints_time = np.zeros(0)
        self.__cell_name = ""
        self.__cell_power = np.zeros(np.size(self.pack_power) + 1)
        self.__cell_checkup_power = np.zeros(np.size(self.pack_checkup_power) + 1)
        self.__cell_voltage = 0
        self.__cell_heat_losses = 0
        self.__durability = 0
        self.__soc_interp = None
        self.__voltage_interp = None
        self.__soh_interp = None
        self.__heat_loss_interp = None
        self.__minimal_pack_energy = np.sum(self.pack_power * self.time_data)
        self.__end_of_life_soh = None
        self.__pack_voltage = None

        self.stop_after_durability_reached = stop_after_durability_reached

        # sizing results [soh, minimum mass, n_cells_series, n_branches, durability]
        self.__sizing_results = np.zeros((len(self.soh_interval), 5))

        # We load the battery model
        self.__load_battery_model()

    def set_cell_model_path(self, value: str):
        value = value.strip()  # we remove blank characters at the end and begining
        if value == "":
            raise ValueError("empty string")
        self.__cell_model_path = value

    def get_cell_model_path(self):
        return self.__cell_model_path

    def set_cell_mass(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("cell_mass must be stricly positive")
        self.__cell_mass = value

    def get_cell_mass(self):
        return self.__cell_mass

    def set_cell_max_voltage(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("cell_max_voltage must be stricly positive")
        self.__cell_max_voltage = value

    def get_cell_max_voltage(self):
        return self.__cell_max_voltage

    def set_cell_min_voltage(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("cell_min_voltage must be stricly positive")
        self.__cell_min_voltage = value

    def get_cell_min_voltage(self):
        return self.__cell_min_voltage

    def set_cell_nominal_voltage(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("cell_nominal_voltage must be stricly positive")
        self.__cell_nominal_voltage = value

    def get_cell_nominal_voltage(self):
        return self.__cell_nominal_voltage

    def set_cell_nominal_energy(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("cell_nominal_energy must be stricly positive")
        self.__cell_nominal_energy = value

    def get_cell_nominal_energy(self):
        return self.__cell_nominal_energy

    def set_cell_initial_temperature(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("cell_nominal_temperature must be stricly positive")
        self.__cell_initial_temperature = value

    def get_cell_initial_temperature(self):
        return self.__cell_initial_temperature

    def set_number_cells_series(self, value: int):
        if value <= 0:
            raise ValueError("number_cells_series must be stricly positive")
        self.__number_cells_series = value

    def get_number_cells_series(self):
        return self.__number_cells_series

    def set_number_branches(self, value: int):
        if value <= 0:
            raise ValueError("number_branches must be stricly positive")
        self.__number_branches = value

    def get_number_branches(self):
        return self.__number_branches

    def set_soh(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("SoH must be strictly positive")
        self.__soh = value

    def get_soh(self):
        return self.__soh

    def set_soc(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("SoC must be strictly positive")
        self.__soc = value

    def get_soc(self):
        return self.__soc

    def set_initial_soc(self, value: Union[int, float]):
        if not (0 <= value <= 100):
            raise ValueError("initial_soc does not respect condition 0 =< initial_soc <= 100")
        self.__initial_soc = value

    def get_initial_soc(self):
        return self.__initial_soc

    def set_initial_soh(self, value: Union[int, float]):
        if not (0 <= value <= 100):
            raise ValueError("initial_soh does not respect condition 0 =< initial_soh <= 100")
        self.__initial_soh = value

    def get_initial_soh(self):
        return self.__initial_soh

    def set_pack_nominal_voltage(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("pack_nominal_voltage must be stricly positive")
        self.__pack_nominal_voltage = value

    def get_pack_nominal_voltage(self):
        return self.__pack_nominal_voltage

    def set_cell_temperature(self, value: Union[int, float]):
        if value <= 0:
            raise ValueError("cell_temperature must be strictly positive")
        self.__cell_temperature = value

    def get_cell_temperature(self):
        return self.__cell_temperature

    def set_min_soc(self, value: Union[int, float]):
        if not 0 <= value < 100:
            raise ValueError("min_soc must be positive and less than 100")
        self.__min_soc = value

    def get_min_soc(self):
        return self.__min_soc

    def set_max_soc(self, value: Union[int, float]):
        if not 0 < value <= 100:
            raise ValueError("max_soc must be positive and less than 100")
        self.__max_soc = value

    def get_max_soc(self):
        return self.__max_soc

    def set_durability_target(self, value: int):
        if value <= 0:
            raise ValueError("durability_target must be stricly positive")
        self.__durability_target = value

    def get_durability_target(self):
        return self.__durability_target

    def set_pack_power(self, value: np.ndarray):
        self.__pack_power = value

    def get_pack_power(self):
        return self.__pack_power

    def set_pack_checkup_power(self, value: np.ndarray):
        self.__pack_checkup_power = value

    def get_pack_checkup_power(self):
        return self.__pack_checkup_power

    def set_time_data(self, value: np.ndarray):
        self.__time_data = value

    def get_time_data(self):
        return self.__time_data

    def set_time_data_checkup(self, value: np.ndarray):
        self.__time_data_checkup = value

    def get_time_data_checkup(self):
        return self.__time_data_checkup

    def set_maximum_branches_number(self, value: int):
        if value <= 0:
            raise ValueError("maximum_branches_number must be stricly positive")
        self.__maximum_branches_number = value

    def get_maximum_branches_number(self):
        return self.__maximum_branches_number

    def set_soh_interval(self, value: np.ndarray):
        self.__soh_interval = value

    def get_soh_interval(self):
        return self.__soh_interval

    def get_mass(self):
        return self.__mass

    def get_volume(self):
        return self.__volume

    def get_soc_map(self):
        return self.__soc_map

    def get_voltage_map(self):
        return self.__voltage_map

    def get_heat_loss_map(self):
        return self.__heat_loss_map

    def get_soh_map(self):
        return self.__soh_map

    def get_breakpoints_power(self):
        return self.__breakpoints_power

    def get_breakpoints_initial_soc(self):
        return self.__breakpoints_initial_soc

    def get_breakpoints_temperature(self):
        return self.__breakpoints_temperature

    def get_breakpoints_initial_soh(self):
        return self.__breakpoints_initial_soh

    def get_breakpoints_time(self):
        return self.__breakpoints_time

    def get_cell_name(self):
        return self.__cell_name

    def get_cell_power(self):
        return self.__cell_power

    def get_cell_checkup_power(self):
        return self.__cell_checkup_power

    def get_cell_voltage(self):
        return self.__cell_voltage

    def get_cell_heat_losses(self):
        return self.__cell_heat_losses

    def get_durability(self):
        return self.__durability

    def get_sizing_results(self):
        return self.__sizing_results

    def get_soc_interp(self):
        return self.__soc_interp

    def get_voltage_interp(self):
        return self.__voltage_interp

    def get_soh_interp(self):
        return self.__soh_interp

    def get_heat_loss_interp(self):
        return self.__heat_loss_interp

    def get_minimal_pack_energy(self):
        return self.__minimal_pack_energy

    def get_end_of_life_soh(self):
        return self.__end_of_life_soh

    def get_pack_voltage(self):
        return self.__pack_voltage

    cell_model_path = property(get_cell_model_path, set_cell_model_path)
    cell_mass = property(get_cell_mass, set_cell_mass)
    cell_max_voltage = property(get_cell_max_voltage, set_cell_max_voltage)
    cell_min_voltage = property(get_cell_min_voltage, set_cell_min_voltage)
    cell_nominal_voltage = property(get_cell_nominal_voltage, set_cell_nominal_voltage)
    cell_nominal_energy = property(get_cell_nominal_energy, set_cell_nominal_energy)
    cell_initial_temperature = property(get_cell_initial_temperature, set_cell_initial_temperature)

    cell_temperature = property(get_cell_temperature, set_cell_temperature)
    number_cells_series = property(get_number_cells_series, set_number_cells_series)
    number_branches = property(get_number_branches, set_number_branches)
    soh = property(get_soh, set_soh)
    soc = property(get_soc, set_soc)
    initial_soc = property(get_initial_soc, set_initial_soc)
    initial_soh = property(get_initial_soh, set_initial_soh)
    pack_nominal_voltage = property(get_pack_nominal_voltage, set_pack_nominal_voltage)
    min_soc = property(get_min_soc, set_min_soc)
    max_soc = property(get_max_soc, set_max_soc)

    durability_target = property(get_durability_target, set_durability_target)
    pack_power = property(get_pack_power, set_pack_power)
    pack_checkup_power = property(get_pack_checkup_power, set_pack_checkup_power)
    time_data = property(get_time_data, set_time_data)
    time_data_checkup = property(get_time_data_checkup, set_time_data_checkup)
    minimal_pack_energy = property(get_minimal_pack_energy)
    maximum_branches_number = property(get_maximum_branches_number, set_maximum_branches_number)
    soh_interval = property(get_soh_interval, set_soh_interval)

    mass = property(get_mass)
    volume = property(get_volume)
    soc_map = property(get_soc_map)
    voltage_map = property(get_voltage_map)
    heat_loss_map = property(get_heat_loss_map)
    soh_map = property(get_soh_map)
    breakpoints_power = property(get_breakpoints_power)
    breakpoints_initial_soc = property(get_breakpoints_initial_soc)
    breakpoints_temperature = property(get_breakpoints_temperature)
    breakpoints_initial_soh = property(get_breakpoints_initial_soh)
    breakpoints_time = property(get_breakpoints_time)
    cell_name = property(get_cell_name)
    cell_power = property(get_cell_power)
    cell_checkup_power = property(get_cell_checkup_power)
    cell_voltage = property(get_cell_voltage)
    cell_heat_losses = property(get_cell_heat_losses)
    durability = property(get_durability)
    sizing_results = property(get_sizing_results)
    soc_interp = property(get_soc_interp)
    voltage_interp = property(get_voltage_interp)
    soh_interp = property(get_soh_interp)
    heat_loss_interp = property(get_heat_loss_interp)
    end_of_life_soh = property(get_end_of_life_soh)
    pack_voltage = property(get_pack_voltage)

    def run_sizing(self):
        print(f"Battery sizing in progress")
        start_time = time.time()
        if use_logging:
            logging.info("Starting battery sizing...")
            logging.info("Presizing for each SoH")
        self.presize()

        if use_logging:
            logging.info("Computing durability for each results of presizing")
        self.__compute_durability()
        if use_logging:
            logging.info("Interpolating battery mass")
        self.__interpolate_battery_mass()

        stop_time = time.time()
        duration = stop_time - start_time
        print(f"Battery sizing completed in {duration:,.2f} s.")
        if use_logging:
            logging.info(f"Battery sizing completed in {duration:,.2f} s.")

    def __load_battery_model(self):
        """
        Load battery model by reading model map and creating linear interpolators
        """
        if use_logging:
            logging.info(f"Loading {self.cell_model_path}")
        model_map = sio.loadmat(self.cell_model_path, squeeze_me=True)
        self.__soc_map = model_map["surfaceResponse"]["SoCMatrix"][()]
        self.__voltage_map = model_map["surfaceResponse"]["voltageMatrix"][()]
        self.__heat_loss_map = model_map["surfaceResponse"]["heatLossMatrix"][()]
        self.__soh_map = model_map["surfaceResponse"]["SoHMatrix"][()]
        self.__breakpoints_power = model_map["surfaceResponse"]["breakpoints"][()]["P"][()]
        self.__breakpoints_initial_soc = model_map["surfaceResponse"]["breakpoints"][()]["SoC_ini"][()]
        self.__breakpoints_temperature = model_map["surfaceResponse"]["breakpoints"][()]["T"][()]
        self.__breakpoints_initial_soh = model_map["surfaceResponse"]["breakpoints"][()]["SoH_ini"][()]
        self.__breakpoints_time = model_map["surfaceResponse"]["breakpoints"][()]["time"][()]
        self.__cell_name = model_map["surfaceResponse"]["cellName"]

        # Interpolator
        # SoC = f(P, SoC ini, T, SoH ini, Time)
        self.__soc_interp = sipint.RegularGridInterpolator((self.breakpoints_power, self.breakpoints_initial_soc,
                                                            self.breakpoints_temperature, self.breakpoints_initial_soh,
                                                            self.breakpoints_time), self.soc_map,
                                                           method='linear', bounds_error=False, fill_value=np.nan)

        # voltage = f(P, SoC ini, T, SoH ini, Time)
        self.__voltage_interp = sipint.RegularGridInterpolator((self.breakpoints_power, self.breakpoints_initial_soc,
                                                                self.breakpoints_temperature,
                                                                self.breakpoints_initial_soh, self.breakpoints_time),
                                                               self.voltage_map,
                                                               method='linear', bounds_error=False,
                                                               fill_value=np.nan)

        # soh = f(P, SoC ini, T, SoH ini, Time)
        self.__soh_interp = sipint.RegularGridInterpolator((self.breakpoints_power, self.breakpoints_initial_soc,
                                                            self.breakpoints_temperature, self.breakpoints_initial_soh,
                                                            self.breakpoints_time), self.soh_map,
                                                           method='linear', bounds_error=False, fill_value=np.nan)

        # heat loss = f(P, SoC ini, T, SoH ini, Time)
        self.__heat_loss_interp = sipint.RegularGridInterpolator((self.breakpoints_power, self.breakpoints_initial_soc,
                                                                  self.breakpoints_temperature,
                                                                  self.breakpoints_initial_soh, self.breakpoints_time),
                                                                 self.heat_loss_map,
                                                                 method='linear', bounds_error=False, fill_value=np.nan)

    def __interpolate_battery_mass(self):
        """
        Interpolation of battery mass based on sizing results and durability calculations for a given durability
        objective

        :return:
        self.__mass : battery mass
        self.__end_of_life_soh : battery end of life soh
        """
        # masses, index = np.unique(self.sizing_results[:, 1], return_index=True)
        # durability = np.unique(self.sizing_results[:, 4])
        # soh = np.unique(self.sizing_results[:, 0])

        durability, index_first = np.unique(self.sizing_results[:, 4], return_inverse=True)
        index = np.zeros(len(durability))
        for i in range(len(index)):
            index[i] = np.where(index_first==i)[0][-1]
        index = index.astype(int)
        masses = self.sizing_results[:, 1]
        masses = masses[index]
        soh = self.sizing_results[:, 0]
        soh_redux = soh[index]

        with np.printoptions(precision=3, suppress=True):
            print(self.sizing_results)

        self.__mass = sipint.interpn(durability.reshape(1, -1), masses, np.array([self.durability_target]),
                                     method='linear', bounds_error=False, fill_value=None)
        #self.__mass = sipint.interpn(soh.reshape(1, -1), self.sizing_results[:, 1], np.array([80]),
                       #method='linear', bounds_error=False, fill_value=None)

        #self.__durability = sipint.interpn(soh.reshape(1, -1), self.sizing_results[:, 4], np.array([80]),
                                     #method='linear', bounds_error=False, fill_value=None)


        # If we have duplicate values for mass we remove the corresponding higher SoH
        #soh_redux = soh[index]
        self.__end_of_life_soh = sipint.interpn(durability.reshape(1, -1), soh_redux, np.array([self.durability_target]),
                                                method='linear', bounds_error=False, fill_value=None)

        #self.__end_of_life_soh = np.array([80])

        if np.isnan(self.__mass) or np.isnan(self.__end_of_life_soh):
            if use_logging:
                logging.warning(f"Durability target ({self.durability_target} months) exceeds max durability computed "
                            f"({max(durability)} months).")
            warnings.warn("Durability target exceeds max durability computed")

    def __set_battery_state(self, soc, temperature, soh):
        """
        Set internal battery parameters
        """
        self.__soc = soc
        self.__cell_temperature = temperature
        self.__soh = soh

    def simulate_pulse(self, power, time):
        # here goes the simulation of a pulse : voltage end of pulse, soc end of pulse, soh end of pulse
        # y = f(P, SoC ini, T, SoH ini, Time)
        is_architecture_valid = False

        # Checking that input data are within breakpoints intervals for power
        if power <= self.breakpoints_power.max(axis=0):

            # Interpolation
            soc = self.soc_interp(np.array([power, self.soc, self.cell_temperature, self.soh, time]))[0]
            #soc = self.soc - (self.soc - self.soc_interp(np.array([abs(power), self.soc, self.cell_temperature, self.soh, time]))[0]) * np.sign(power)
            cell_voltage = self.voltage_interp(np.array([power, self.soc, self.cell_temperature, self.soh, time]))[0]
            soh = self.soh_interp(np.array([power, self.soc, self.cell_temperature, self.soh, time]))[0]

            # Non valid architectures : nan, soc limits, cell voltage limits, soh > 0
            if not (np.isnan(soc) or np.isnan(cell_voltage) or np.isnan(soh)):

                if (self.min_soc <= soc <= self.max_soc
                        and self.cell_min_voltage <= cell_voltage <= self.cell_max_voltage
                        and soh > 0):
                    is_architecture_valid = True
                    self.soh = soh
                    self.soc = soc
                    self.__cell_voltage = cell_voltage
                    self.__pack_voltage = cell_voltage * self.number_cells_series
                    self.__mass = self.number_cells_series * self.number_branches * self.cell_mass

        return is_architecture_valid

    def compute_heat_losses(self):
        """
        Compute heat losses over a power profil
        """
        self.__cell_heat_losses = 0
        for index in range(np.size(self.cell_power)):
            self.__cell_heat_losses += \
                self.heat_loss_interp(np.array([self.cell_power[index], self.soc, self.cell_temperature, self.soh,
                                                self.time_data[index]]))[0]

    def presize(self):
        """
        Presizing of the battery for several SOH, return optimal n branches, for each SOH

        :returns
        self.__sizing_results[i_soh, 0] = soh
        self.__sizing_results[i_soh, 1] = optimal mass
        self.__sizing_results[i_soh, 2] = optimal number_cells_series
        self.__sizing_results[i_soh, 3] = optimal number_branches
        self.__sizing_results[i_soh, 4] = durability
        """
        # Setting number of cells in series
        self.number_cells_series = ceil(self.pack_nominal_voltage / self.cell_nominal_voltage)

        # Minimum number of branches for max number of cells in series
        n_branches_min = math.ceil(self.minimal_pack_energy /
                                   (self.number_cells_series * self.cell_nominal_energy))

        if self.maximum_branches_number <= n_branches_min:
            raise ValueError("Maximum_branches_number is inferior to the number of branches imposed by energy sizing")
        i_soh = -1
        for soh in self.soh_interval:
            i_soh += 1
            mass = np.inf  # Mass is initialized at a very high value

            n_branches_min = math.ceil(self.minimal_pack_energy / (self.number_cells_series * self.cell_nominal_energy))

            if n_branches_min <= self.maximum_branches_number:
                # Dichotomy
                is_architecture_valid = False
                exit_flag_dichotomy = True
                i_branch_up = self.maximum_branches_number
                i_branch_down = n_branches_min
                while exit_flag_dichotomy:
                    i_branch = round((i_branch_up + i_branch_down) / 2)
                    self.__number_branches = i_branch
                    self.__cell_power = np.divide(self.pack_power,
                                                  self.number_cells_series * self.__number_branches)
                    # Resume battery state
                    self.__set_battery_state(self.initial_soc, self.cell_initial_temperature, soh)
                    # Power profile simulation
                    for index in range(np.size(self.cell_power)):
                        is_architecture_valid = self.simulate_pulse(self.cell_power[index], self.time_data[index])
                        if not is_architecture_valid:
                            break

                    if is_architecture_valid:
                        if (i_branch - i_branch_down) > 1:
                            i_branch_up = i_branch
                        else:
                            exit_flag_dichotomy = False
                            mass_temp = self.cell_mass * self.__number_branches * self.number_cells_series
                            if mass_temp < mass:  # saving better configuration
                                mass = mass_temp
                                self.__sizing_results[i_soh, 0] = soh
                                self.__sizing_results[i_soh, 1] = mass
                                self.__sizing_results[i_soh, 2] = self.number_cells_series
                                self.__sizing_results[i_soh, 3] = self.number_branches
                                self.__sizing_results[i_soh, 4] = 0  # durability

                    else:
                        if (i_branch_up - i_branch) > 1:
                            i_branch_down = i_branch
                        else:
                            exit_flag_dichotomy = False
                            self.__number_branches = i_branch_up
                            # Resume battery state
                            self.__set_battery_state(self.initial_soc, self.cell_initial_temperature, soh)
                            # Power profile simulation
                            for index in range(np.size(self.cell_power)):
                                is_architecture_valid = self.simulate_pulse(self.cell_power[index],
                                                                            self.time_data[index])
                                if not is_architecture_valid:
                                    break

                            mass_temp = self.cell_mass * self.__number_branches * self.number_cells_series
                            if mass_temp < mass:  # saving better configuration
                                mass = mass_temp
                                self.__sizing_results[i_soh, 0] = soh
                                self.__sizing_results[i_soh, 1] = mass
                                self.__sizing_results[i_soh, 2] = self.number_cells_series
                                self.__sizing_results[i_soh, 3] = self.number_branches
                                self.__sizing_results[i_soh, 4] = 0  # durability

    def run_checkup(self):
        """
        Checking that battery reaches performances target for checkup_profil
        """
        # y = f(P, SoC ini, T, SoH ini, Time)
        is_checkup_valid = True
        # Resume battery state (except soh)
        self.__set_battery_state(self.initial_soc, self.cell_initial_temperature, self.soh)
        self.__cell_checkup_power = np.divide(self.pack_checkup_power,
                                              self.number_cells_series * self.__number_branches)
        for index in range(np.size(self.cell_power)):

            # Interpolation
            soc = self.soc_interp(np.array([self.__cell_checkup_power[index], self.soc, self.cell_temperature, self.soh,
                                            self.time_data_checkup[index]]))[0]
            cell_voltage = self.voltage_interp(np.array([self.__cell_checkup_power[index], self.soc,
                                                         self.cell_temperature, self.soh,
                                                         self.time_data_checkup[index]]))[0]
            soh = self.soh_interp(np.array([self.__cell_checkup_power[index], self.soc, self.cell_temperature, self.soh,
                                            self.time_data_checkup[index]]))[0]

            # Non valid architectures : nan, soc limits, cell voltage limits
            if not (np.isnan(soc) or np.isnan(cell_voltage) or np.isnan(soh)):
                if not (self.min_soc <= soc <= self.max_soc
                        and self.cell_min_voltage <= cell_voltage <= self.cell_max_voltage):
                    is_checkup_valid = False
                    break

        return is_checkup_valid

    def __compute_durability(self):
        # For each SOH target results we compute the durability
        for i_soh in range(len(self.soh_interval)):
            # Setting the battery parameters to its initial values computed before
            self.__number_cells_series = self.sizing_results[i_soh, 2]
            self.__number_branches = self.sizing_results[i_soh, 3]
            self.__set_battery_state(self.initial_soc, self.cell_initial_temperature, self.initial_soh)

            # Setting the cell power
            self.__cell_power = np.divide(self.pack_power, self.number_cells_series * self.__number_branches)

            # Starting the sizing
            self.__size_battery_pack_raw_temperature()

            self.__sizing_results[i_soh, 4] = self.durability  # durability
            if use_logging:
                #logging.info("Computing durability")
                logging.info(f"Computing: durability = {self.durability} Month")
            #print(f"Durability fin {self.durability:,.2f}")
            #print(f"SoH fin {self.soh:,.2f}")
            if self.durability >= self.durability_target and self.stop_after_durability_reached:
                if use_logging:
                    logging.info(f"Computing: durability target reached, stopping calculation")
                self.__sizing_results = np.delete(self.__sizing_results, slice(i_soh + 1, self.__sizing_results.shape[0]), 0)
                break

    def __size_battery_pack_raw_temperature(self):

        # Improving sizing with ageing simulation
        self.__simulate_aging()

        # power_to_cooling_fluid = {
        #     "first_mission": np.array([42]),
        #     "last_mission": np.array([43])
        # }

        self.compute_heat_losses()

    def __simulate_aging(self):
        self.__soh = self.initial_soh
        # is_checkup_valid = self.run_checkup()
        is_checkup_valid = True
        is_architecture_valid = True
        self.__durability = 0

        while is_architecture_valid and is_checkup_valid:  # and self.durability != self.durability_target
            is_architecture_valid = self.__simulate_month()
            # is_checkup_valid = self.run_checkup()
            self.__durability += 1  # in month
            #print(f"Durability {self.__durability:,.2f}")
            #print(f"SoH {self.soh:,.2f}")
            # if use_logging:
                # logging.debug(f"SOH initial = {self.initial_soh} : durability = {self.durability}, soh = {self.soh}")

    def __simulate_month(self):
        is_architecture_valid = True
        day_number = 0

        while is_architecture_valid and day_number <= 30*4: #attentione *6
            is_architecture_valid = self.__simulate_day()
            day_number += 1

        return is_architecture_valid

    def __simulate_day(self):
        is_architecture_valid = False
        self.__set_battery_state(self.initial_soc, self.cell_initial_temperature, self.soh)
        for index in range(np.size(self.cell_power)):
            is_architecture_valid = self.simulate_pulse(self.cell_power[index],
                                                        self.time_data[index])
            if not is_architecture_valid:
                break

        return is_architecture_valid
