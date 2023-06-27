## sweep_functionality.py
#
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Dominik Eisenhut, eisenhut@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Jun 2022, D. Eisenhut
# Modified:

import itertools
from SUAVE.Core import Data
from datetime import datetime


# TODO parameters are only functionality examples, need to replace by meaningful ones


def sweep(parameters, variations):
    combinations = list(itertools.product(*list(variations.values())))

    total_iterations = len(combinations)

    print("Simulation started: {}".format(datetime.now().strftime("%y-%m-%d %H:%M:%S")))
    print("Running a total of {} sweeps.".format(total_iterations))
    print("Variables swept:", end=" ")

    for variation in list(variations)[:-1]:
        print(variation, end=", ")

    print(list(variations)[-1])
    print("\nThe following sweeps are used:")
    i = 0
    for combination in combinations:
        i += 1
        print('{0:3d}.\t{1}'.format(i, combination))

    print("\n")

    i = 0
    for combination in combinations:
        i += 1
        print("\n-----------------------------\nRunning sweep {0:3d} of {1:3d}\n-----------------------------".format(i, total_iterations))
        print(datetime.now().strftime("%y-%m-%d %H:%M:%S"))
        n = 0
        for variation in list(variations):
            parameters[variation] = combination[n]
            n += 1

        # TODO Call the main function with parameters from "Aircraft.py"

    print("\nSimulation ended: {}".format(datetime.now().strftime("%y-%m-%d %H:%M:%S")))


def base_values():

    # TODO define parameters that should be sweepable

    parameters = Data()

    parameters.wing_loading   = 5000
    parameters.power_loading  = 20
    parameters.energy_density = 300

    return parameters


if __name__ == "__main__":

    use_sweep = True

    parameters = base_values()

    variations = {
        "wing_loading": [4000, 5000, 6000],
        "power_loading": [15, 20, 25],
        "energy_density": [200, 300, 400]
    }

    if use_sweep:
        sweep(parameters, variations)
    else:
        print('not defined so far')
        # TODO Call the main function with parameters from "Aircraft.py"
