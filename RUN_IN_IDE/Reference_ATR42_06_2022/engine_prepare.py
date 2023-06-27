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



import os
import SUAVE
import numpy as np
from sklearn import svm
from scipy.interpolate import griddata
from scipy.interpolate import LinearNDInterpolator
####################################################################################################


file_name = 'PW127-MAX_TET_1385K.csv'
input_file = os.path.join(SUAVE.__path__[0], 'Data_Files\\Gas_Turbine\\', file_name)

# Load the CSV file
my_data = np.genfromtxt(input_file, delimiter=';')

# Remove the header line
my_data = np.delete(my_data,np.s_[0],axis=0)

features  = my_data[:,:3] # Altitude, Mach, Throttle
delta_isa = np.transpose(np.atleast_2d(my_data[:, 11]))
features = np.c_[features, delta_isa]
# sea_level_static_row_index = np.where(np.all(xy == [0, 0, 1], axis=1))
power = np.transpose(np.atleast_2d(my_data[:, 3]))  #Power
sfc = np.transpose(np.atleast_2d(my_data[:, 4])) # SFC
# p3 = np.transpose(np.atleast_2d(my_data[:, 12]))  # P3
# t3 = np.transpose(np.atleast_2d(my_data[:, 13]))  # T3
# residual_thrust = np.transpose(np.atleast_2d(my_data[:, 15]))  # Residual Thrust
# far = np.transpose(np.atleast_2d(my_data[:, 14]))

targets = np.c_[power, sfc]

max_features = features.max(axis=0)
max_targets = targets.max(axis=0)
#
features_normed = features / max_features
targets_normed = targets / max_targets
#
# regr = svm.SVR()
#
# regr.fit(features_normed, power_normed)
#
# power_predicted = regr.predict(features_normed) * max_power
#
# accuracy = power.flatten() / power_predicted.T
#
# print(1)


# second try

request = np.array([[0,	0,	1.0,0],[7620,	0.5,	0.96,0], [0, 0, 1.028472448,0],[7620, 0.5, 0.957270908,0]])

# hallo = griddata(features_normed, power_normed, request/max_vals,method='linear')*max_power

linInter= LinearNDInterpolator(features_normed, targets_normed,fill_value=2)

halllo1 = linInter(request / max_features) * max_targets


print(1)