# sizing_chart_plot.py
#
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Jonas Mangold, mangold@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Oct 2022, J. Mangold
# Modified:


import os
import SUAVE
import numpy as np
from sklearn import svm
import numpy as np
#import pylab as plt
import matplotlib.pyplot as plt
from SUAVE.Core import Data, Units

####################################################################################################################
# Settings

wing_loading = np.linspace(1, 800, 50, endpoint=True)

number_of_engines = 2

startx, endx = 0, 600
starty, endy = 0, 250

plt.axis([startx, endx, starty, endy])

atmosphere = SUAVE.Analyses.Atmospheric.US_Standard_1976()


####################################################################################################################
# Stall Speed

cL_max = 1.707
rho = 1.225
v_stall = 110 * Units.knots

stall_speed = cL_max * rho / (2 * 9.81) * v_stall ** 2

plt.vlines(stall_speed, starty, endy)

####################################################################################################################
# Landing Field Length

m0_mLanding = 1/0.984
s_landing = 1126
s_a = 305
sigma = 1
cL_max_landing = 2.7
A = 0.66 #0.66 mit reverse thrust
calibration = 1.112

landing_field_length = calibration * m0_mLanding * (s_landing / 1.67 - s_a) * sigma * cL_max_landing / (0.51 * A * 9.81)

plt.vlines(landing_field_length, starty, endy)

####################################################################################################################
# Take off Field Length

v_stall = 45
v2 = 1.2 * v_stall

k_TO = 2.45
s_TOFL = 1165
eta_TO = 0.75
#cL_max_TO_new = MTOM * 9.81 / (1.225/2 * (51)**2 * ref_area)
cL_max_TO = 0.8 * 2.7
rho_airport = 1.225

#takeoff_field_length = 9.81 * v2 / eta_TO * k_TO / (cL_max_TO * rho_airport * s_TOFL) * (wing_loading) #/ number_of_engines

K_TO = 2.45 / 0.75 * 0.8977
takeoff_field_length = 9.81 * K_TO * 1.2 / s_TOFL * (2 * 9.81) ** 0.5 * (wing_loading / (rho_airport * cL_max_TO)) ** (3/2)

plt.plot(wing_loading,takeoff_field_length)


####################################################################################################################
# 2ng Segment OEI

eta_climb = 0.875
gamma_min_OEI = 0.024
LoverD = 10.25
m2_m0 = 0.99

secondsegment_oei = 9.81 * v2 / eta_climb * m2_m0 * (gamma_min_OEI + 1/LoverD) * number_of_engines / (number_of_engines - 1)

plt.hlines(secondsegment_oei,startx, endx)


####################################################################################################################
# Climb Requirment 1850 ft/min MTOM, SL, ISA

v_v_1850 = 1850 * Units.ft/Units.min
m4_m0 = 0.98
v_climb = 160 * Units.knots
LoD_climb = 12.8

climb1850ftmin = 9.81 * v_climb / eta_climb * m4_m0 * (v_v_1850 / v_climb + 1/LoD_climb)

plt.hlines(climb1850ftmin,startx, endx)


####################################################################################################################
# Cruise
mcruise_m0 = 0.99

v_cruise = 289 * Units.knots
eta_cruise = 0.83

altitude = 17000 * Units.ft
temperature_deviation = 0

atmo_data =  atmosphere.compute_values(altitude, temperature_deviation)
rho_cruise = atmo_data.density[0][0]
pressure_cruise = atmo_data.pressure[0][0]
T_cruise = atmo_data.temperature[0][0]
q_cruise = rho_cruise / 2 * v_cruise ** 2
cD0 = 258 * 1e-4
aspect_ratio = 11
oswald_factor = 0.71

P_P_cruise1 = 1 / ((rho_cruise / 1.225) * (288.15 / T_cruise) ** 0.5)
P_P_cruise2 = 1 / ((pressure_cruise / 101325 ) * (288.15 / T_cruise) ** 0.5)
P_P_cruise = 1 / (rho_cruise/1.225 )** 0.52

cruise = 9.81 * v_cruise / eta_cruise * P_P_cruise * (cD0 * q_cruise / (wing_loading * 9.81) + wing_loading * 9.81 / (q_cruise * np.pi * aspect_ratio * oswald_factor) * mcruise_m0 ** 2)

plt.plot(wing_loading,cruise)

####################################################################################################################
# Ceiling OEI 16.000 ft +10ISA FCOM page 1159 FL160 18000kg
#passt
v_v_100 = 100 * Units.ft/Units.min # oder auch 0
m4_m0 = 0.97

altitude = 16000 * Units.ft
temperature_deviation = 10

atmo_data_ceilingOEI =  atmosphere.compute_values(altitude, temperature_deviation)
rho_ceilingOEI = atmo_data_ceilingOEI.density[0][0]

eta_cruise = 0.83

LoD_ceilingOEI = 16
v_ceilingOEI = 187 * Units.knots

P_P_ceilingoei = 1 / (rho_ceilingOEI/1.225 )** 0.5

ceilingoei = 9.81 * P_P_ceilingoei * v_ceilingOEI / eta_cruise * m4_m0 * (v_v_100 / v_ceilingOEI + 1/LoD_ceilingOEI) * number_of_engines / (number_of_engines - 1)

plt.hlines(ceilingoei,startx, endx)

####################################################################################################################
# Service Ceiling 25.000 ft
#passt
v_v_service_ceiling = 0.5 * Units.m / Units.s
m4_m0 = 0.9409

altitude = 25000 * Units.ft
temperature_deviation = 0

atmo_data =  atmosphere.compute_values(altitude, temperature_deviation)
rho_service_ceiling = atmo_data.density[0][0]

eta_cruise = 0.83

LoD_service_ceiling = 13.8
v_service_ceiling = 300 * Units.knots

P_P_service_ceiling = 1 / (rho_service_ceiling/1.225 )** 0.5


service_ceiling = 9.81 * P_P_service_ceiling * v_service_ceiling / eta_cruise * m4_m0 * (v_v_service_ceiling / v_service_ceiling + 1/LoD_service_ceiling)

plt.hlines(service_ceiling,startx, endx)




####################################################################################################################
# Initial Cruise Altitude - Top of Climb - 300 ft/min at 24000 ft, see FCOM page 1046 or 1044
# passt

v_v_300 = 300 * Units.ft / Units.min
m4_m0 = 0.97

altitude = 24000 * Units.ft
temperature_deviation = 0

atmo_data =  atmosphere.compute_values(altitude, temperature_deviation)
rho_initial_cruise_altitude = atmo_data.density[0][0]

eta_cruise = 0.83

LoD_initial_cruise_altitude = 13.8
v_initial_cruise_altitude = 232.259 * Units.knots

P_P_initial_cruise_altitude = 1 / (rho_initial_cruise_altitude/1.225 )** 0.5


initial_cruise_altitude = 9.81 * P_P_initial_cruise_altitude * v_initial_cruise_altitude / eta_cruise * m4_m0 * (v_v_300 / v_initial_cruise_altitude + 1/LoD_initial_cruise_altitude)

plt.hlines(initial_cruise_altitude,startx, endx)

####################################################################################################################
# Plot


plt.show()