# battery_sizing_tool v1.1 #

## Disclaimer ##

This software has been developed specifically for FUTPRINT50 project. It comes without any warranty and for research purposes only. See license below.

Copyright CEA, 2023  

<https://cea.fr/>

### Purpose ###

This algorithm aims at sizing a battery taken into account a given power profile and the battery aging. It first computes several sizing for several initial states of health then, for each results, computes the associated aging and finally, interpolates the battery mass for a given durability. Battery mass is the sum of cells mass.

## Running ##

1. Start your favorite Python IDE.
2. Create a virtual environment
3. Install numpy and scipy modules.
4. Run script.py

## Configuration ##

Program takes several inputs. They are listed below. Units are given in []

### test_cell_parameters ###

Cell parameters

* ```cell_model_path``` Path of the surrogate cell model (.mat file)
* ```cell_mass``` [kg]
* ```cell_max_voltage``` [V]
* ```cell_min_voltage``` [V]
* ```cell_nominal_voltage``` [V]
* ```cell_nominal_energy``` [J]
* ```cell_initial_temperature``` [K]

### test_pack_parameters ###

Pack parameters: a module is an assembly of cells, a pack is an assembly of modules.

* ```initial_soc``` Initial battery state of charge [%]
* ```initial_soh``` Initial battery state of health [%]
* ```pack_nominal_voltage``` Battery pack nominal voltage [%]
* ```min_soc``` Minimal battery state of charge (constraint) [%]
* ```max_soc``` Maximal battery state of charge (constraint) [%]
* ```module_max_voltage``` Maximal module voltage (constraint) [%]
* ```cooling_fluid_flow``` Not in use
* ```cell_to_fluid_thermal_resistance``` Not in use

### test_study_parameters ###

* ```durability_target``` battery durability target [days]: I want my battery lasts ```durability_target``` days
* ```pack_power``` power profile applied to battery [W] : np.array. Positive is discharging, negative is charging
* ```time_data``` associated time data of the power profile [s]
* ```pack_checkup_power``` not is use
* ```time_data_checkup``` not is use
* ```maximum_branches_number``` maximum number of branches investigated by the algorithm. User should set a high value.
* ```soh_interval``` State of health investigated by the algorithm [%] np.array. First value must be 100. Vector must be monotone. Lowest value must not be lower than the lowest value of the surrogate model.

## Contact ##

boris.berseneff@cea.fr

## License ##

Copyright 2023 CEA

Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the “Software”), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED “AS IS”, WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
