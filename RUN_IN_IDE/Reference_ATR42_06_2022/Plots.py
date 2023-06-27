## Reference_ATR42_06_2022.py
# 
# This code was contributed under project FUTPRINT50 <www.futprint50.eu>
# that has received funding from the European Union’s Horizon 2020 
# Research and Innovation programme under Grant Agreement No 875551.
#
# Contributed by:
#   Jonas Mangold, mangold@ifb.uni-stuttgart.de, University of Stuttgart
#
# Created:  Sep 2021, J. Mangold
# Modified: Jun 2022, F.Brenner

# ----------------------------------------------------------------------
#   Imports
# ----------------------------------------------------------------------

# Python Imports
import numpy as np
import pylab as plt
from matplotlib import ticker
from copy import deepcopy
try:
    import cPickle as pickle
except ImportError:
    import pickle

# SUAVE Imports
import SUAVE
from SUAVE.Core import Data, Units

def plot_mission(results,vehicle):
    markersize = 3
    # ------------------------------------------------------------------
    #   Aerodynamics
    # ------------------------------------------------------------------
    #f1 = open(Mission_name + '_aero_coef.dat', 'w')
    #f1.write('Distance [nm], CL, CD, L/D, Drag [N], Thrust [N]\n')
    fig = plt.figure("Aerodynamic Coefficients")
    for segment in results.segments.values():
        distance = (segment.conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        CLift = segment.conditions.aerodynamics.lift_coefficient[:, 0]
        CDrag = segment.conditions.aerodynamics.drag_coefficient[:, 0]
        Drag = -segment.conditions.frames.wind.drag_force_vector[:, 0]
        Thrust = segment.conditions.frames.body.thrust_force_vector[:, 0]

        LoverD = CLift/CDrag

        axes = plt.subplot(4, 1, 1)
        axes.plot(distance, CLift, 'bo-', markersize=markersize)
        axes.set_ylabel('CL')
        # axes.get_yaxis().get_major_formatter().set_scientific(False)
        # axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True, which='both')
        axes.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.6f}"))

        axes = plt.subplot(4, 1, 2, sharex = axes)
        axes.plot(distance, CDrag, 'bo-', markersize=markersize)
        axes.set_ylabel('CD')
        # axes.get_yaxis().get_major_formatter().set_scientific(False)
        # axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True, which='both')
        axes.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.6f}"))

        axes = plt.subplot(4, 1, 3, sharex = axes)
        axes.plot(distance, LoverD, 'bo-', markersize=markersize)
        axes.set_ylabel('L/D')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True, which='both')

        axes = plt.subplot(4, 1, 4, sharex = axes)
        axes.plot(distance, Drag, 'bo-', markersize=markersize)
        axes.plot(distance, Thrust, 'ro-', markersize=markersize)
        axes.set_xlabel('Distance (nm)')
        axes.set_ylabel('Drag and Thrust (N)')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True, which='both')

        #for i,d in enumerate(distance):
        #    f1.write('%f,%f,%f,%f,%f,%f\n' %(d, CLift[i], CDrag[i], LoverD[i], Drag[i], Thrust[i]))

    #f1.close()
    fig.suptitle('Aerodynamic Coefficients', fontsize=14)

    #     ------------------------------------------------------------------
    #       Aerodynamics 2
    #     ------------------------------------------------------------------
    #f2 = open(Mission_name + '_drag_breakdown.dat', 'w')
    #f2.write('Distance [nm], Parasite, Induced, Compressibility, Miscellaneous, Drag increment, Propeller drag, Total\n')
    fig = plt.figure("Drag Components")
    axes = plt.gca()
    for i, segment in enumerate(results.segments.values()):
        distance = (segment.conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown
        cdp = drag_breakdown.parasite.total[:, 0]
        cdi = drag_breakdown.induced.total[:, 0]
        cdc = drag_breakdown.compressible.total[:, 0]
        #cdt = drag_breakdown.trim.total[:, 0]
        cd = drag_breakdown.total[:, 0]
        cdinc = drag_breakdown.drag_coefficient_increment * np.ones_like(cd)
        cdprop = drag_breakdown.prop_drag

        axes.plot(distance, cdp, 'ko-', label='Parasite', markersize=markersize)
        axes.plot(distance, cdi, 'bo-', label='Induced', markersize=markersize)
        axes.plot(distance, cdc, 'go-', label='Compressibility', markersize=markersize)
        #axes.plot(distance, cdm, 'yo-', label='Miscellaneous', markersize=markersize)
        axes.plot(distance, cdinc, 'mo-', label='Drag increment', markersize=markersize)
        axes.plot(distance, cdprop, 'co-', label='Propeller Drag', markersize=markersize)
        axes.plot(distance, cd, 'ro-', label='Total', markersize=markersize)

        if i == 0:
            axes.legend(loc='upper center')

        #for i,d in enumerate(distance):
        #    f2.write('%f,%f,%f,%f,%f,%f,%f,%f\n' %(d, cdp[i], cdi[i], cdc[i], cdm[i], cdinc[i], cdprop[i], cd[i]))

    #f2.close()
    axes.set_xlabel('Distance (nm)')
    axes.set_ylabel('CD')
    axes.grid(True)
    fig.suptitle('Drag Components', fontsize=14)
    axes.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.8f}"))

    #     ------------------------------------------------------------------
    #       Aerodynamics 3
    #     ------------------------------------------------------------------
    #f2 = open(Mission_name + '_drag_breakdown.dat', 'w')
    #f2.write('Distance [nm], Parasite, Induced, Compressibility, Miscellaneous, Drag increment, Propeller drag, Total\n')
    fig = plt.figure("Detailed Parasite Drag Components")
    axes = plt.gca()
    for i, segment in enumerate(results.segments.values()):
        distance = (segment.conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        drag_breakdown = segment.conditions.aerodynamics.drag_breakdown

        cdp = drag_breakdown.parasite.totaldc [:, 0]
        cdw = drag_breakdown.parasite.wings[:, 0]
        cdf = drag_breakdown.parasite.fuselage[:, 0]
        cdpr = drag_breakdown.parasite.propulsors[:, 0]
        #cdpy = drag_breakdown.parasite.pylons[:, 0]
        cdm = drag_breakdown.parasite.miscellaneous[:, 0]

        axes.plot(distance, cdp, 'ro-', label='Parasite', markersize=markersize)
        axes.plot(distance, cdw, 'ko-', label='Wing', markersize=markersize)
        axes.plot(distance, cdf, 'yo-', label='Fuselage', markersize=markersize)
        axes.plot(distance, cdpr, 'bo-', label='Nacelles', markersize=markersize)
        #axes.plot(distance, cdpy, 'go-', label='Pylons', markersize=markersize)
        axes.plot(distance, cdm, 'co-', label='Miscellaneous', markersize=markersize)


        if i == 0:
            axes.legend(loc='upper right')

        #for i,d in enumerate(distance):
        #    f2.write('%f,%f,%f,%f,%f,%f,%f,%f\n' %(d, cdp[i], cdi[i], cdc[i], cdm[i], cdinc[i], cdprop[i], cd[i]))

    #f2.close()
    axes.set_xlabel('Distance (nm)')
    axes.set_ylabel('Drag Counts')
    axes.grid(True)
    fig.suptitle('Detailed Component Drag', fontsize=14)
    axes.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.0f}"))

    # ------------------------------------------------------------------
    #   Throttle
    # ------------------------------------------------------------------
    fig = plt.figure("Throttle History")
    axes = plt.subplot2grid((5, 1), (0, 0), rowspan=4)
    for i in range(len(results.segments)):
        distance = (results.segments[i].conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        eta = results.segments[i].conditions.propulsion.throttle[:, 0]
        axes.plot(distance, eta, 'o-', color='blue', linewidth=1.0, markersize=markersize)

    axes.set_xlabel('Distance (nm)')
    axes.set_ylabel('Throttle')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    axes.set_ylim(0,1.1)
    axes.grid(True)

    axes = plt.subplot(5, 1, 5, sharex = axes)
    for i in range(len(results.segments)):
        distance = (results.segments[i].conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
        eta = results.segments[i].conditions.propulsion.throttle[:, 0]
        axes.plot(distance, eta, 'o-', color='blue', linewidth=1.0, markersize=markersize)
    axes.set_xlabel('Distance (nm)')
    axes.set_ylabel('Throttle')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    fig.suptitle('Throttle History', fontsize=14)
    axes.grid(True)

    axes = plt.subplot(5, 1, 5, sharex = axes)
    for i in range(len(results.segments)):
        distance = (results.segments[i].conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
        time = results.segments[i].conditions.frames.inertial.time[:, 0] / Units.min
#        oswald = results.segments[i].conditions.aerodynamics.drag_breakdown.induced.oswald_efficiency_factor[:,0]
#        axes.plot(distance, oswald, 'o-', color='blue', linewidth=1.0, markersize=markersize)
    axes.set_xlabel('Distance (nm)')
    axes.set_ylabel('Oswald')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    fig.suptitle('Throttle History', fontsize=14)
    axes.grid(True)

    #     ------------------------------------------------------------------
    #     Mission Profile
    #     ------------------------------------------------------------------
    #f4 = open(Mission_name + '_profile.dat', 'w')
    #f4.write('Distance [nm], Altitude [ft], Speed [kts], Mach number, Rate of climb [ft/min]\n')
    fig = plt.figure("Mission Profile")
    for segment in results.segments.values():
        distance = (segment.conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        speed = segment.conditions.freestream.velocity[:, 0] / Units['knot']
        mach_number = segment.conditions.freestream.mach_number[:, 0]
        altitude = segment.conditions.freestream.altitude[:, 0] / Units.ft
        velocity = segment.conditions.freestream.velocity[:, 0]
        weight = segment.conditions.weights.total_mass[:, 0]
        drag = -segment.conditions.frames.wind.drag_force_vector[:, 0]
        thrust = segment.conditions.frames.body.thrust_force_vector[:, 0]
        ROC = (velocity * (thrust - drag) / (weight * 9.81)) * 196.8504  # ft/min

        axes = plt.subplot(4, 1, 1)
        axes.plot(distance, altitude, 'o-', color='blue', linewidth=1.0, markersize=markersize)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.set_ylabel('Altitude [ft]')
        axes.grid(True)

        axes = plt.subplot(4, 1, 2, sharex = axes)
        axes.plot(distance, speed, 'o-', color='blue', linewidth=1.0, markersize=markersize)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.set_ylabel('Speed [kts]')
        axes.grid(True)

        axes = plt.subplot(4, 1, 3, sharex = axes)
        axes.plot(distance, mach_number, 'o-', color='blue', linewidth=1.0, markersize=markersize)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.set_ylabel('Mach number')
        axes.grid(True)

        axes = plt.subplot(4, 1, 4, sharex = axes)
        axes.plot(distance, ROC, 'bo-', markersize=markersize)
        axes.set_xlabel('Distance (nm)')
        axes.set_ylabel('Rate of climb [ft/min]')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True, which='both')

        fig.suptitle('Mission Profile', fontsize=14)

        #for i,d in enumerate(distance):
            #f4.write('%f,%f,%f,%f,%f\n' %(d, altitude[i], speed[i], mach_number[i], ROC[i]))

    #f4.close()

    #####################################
    # SFC
    #####################################

    axis_font = {'size': '14'}
    fig = plt.figure("Altitude_SFC_Weight")
    fig.set_size_inches(10, 8)
    for segment in results.segments.values():
        if segment.tag == "reserve_descent_5":
            a = 5
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        mass = segment.conditions.weights.total_mass[:, 0] / Units.lb
        altitude = segment.conditions.freestream.altitude[:, 0] / Units.ft
        mdot = segment.conditions.weights.vehicle_mass_rate[:, 0]
        thrust = segment.conditions.frames.body.thrust_force_vector[:, 0]

        power = segment.conditions.propulsion.power[:, 0]
        power_single_hp = power / Units.hp / 2
        #sfc = (mdot / Units.lb) / (thrust / Units.lbf) * Units.hr
        sfc_p = (mdot / Units.lb) / (power / Units.hp) * Units.hr
        #mdot_kg_hr = mdot * Units.hr

        axes = plt.subplot(3, 1, 1)
        axes.plot(time, altitude, 'bo-')
        axes.set_ylabel('Altitude (ft)', axis_font)
        axes.grid(True)
        #set_axes(axes)

        axes = plt.subplot(3, 1, 3)
        #axes.set_ylim(0,4)
        #axes.plot(time, sfc, 'bo-')
        #axes.set_ylabel('sfc (lb/lbf-hr)', axis_font)

        axes.plot(time, sfc_p, 'bo-')
        #axes.plot(time, mdot_kg_hr, 'bo-')

        axes.set_ylabel('sfc_p (lb/hp-hr)', axis_font)
        axes.set_xlabel('Time (min)', axis_font)
        axes.grid(True)
        #set_axes(axes)

        axes = plt.subplot(3, 1, 2)
        axes.plot(time, mdot, 'ro-')
        axes.set_ylabel('mdot', axis_font)
        axes.grid(True)
        #set_axes(axes)

    #####################################
    # SFC
    #####################################

    axis_font = {'size': '14'}
    fig = plt.figure("Altitude_Power")
    fig.set_size_inches(10, 8)
    for segment in results.segments.values():
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        mass = segment.conditions.weights.total_mass[:, 0] / Units.lb
        altitude = segment.conditions.freestream.altitude[:, 0] / Units.ft
        mdot = segment.conditions.weights.vehicle_mass_rate[:, 0]
        thrust = segment.conditions.frames.body.thrust_force_vector[:, 0]

        power = segment.conditions.propulsion.power
        #powerBLI = segment.conditions.propulsion.power_BLI

        sfc = (mdot / Units.lb) / (thrust / Units.lbf) * Units.hr
        sfc_p = (mdot / Units.lb) / (power / Units.hp) * Units.hr

        rho  = segment.conditions.freestream.density[:,0]

        TC = thrust/vehicle.propulsors.network.number_of_engines / (rho * segment.conditions.freestream.velocity[:, 0]**2 * (vehicle.propulsors.network.propeller.tip_radius*2)**2)

        axes = plt.subplot(3, 1, 1)
        axes.plot(time, altitude, 'bo-')
        axes.set_ylabel('Altitude (ft)', axis_font)
        axes.grid(True)
        # set_axes(axes)

        axes = plt.subplot(3, 1, 2)
        # axes.plot(time, sfc, 'bo-')
        # axes.set_ylabel('sfc (lb/lbf-hr)', axis_font)
        axes.plot(time, power, 'bo-')
        #axes.plot(time, powerBLI, 'ro-')
        axes.set_ylabel('Power', axis_font)
        axes.set_xlabel('Time (min)', axis_font)
        axes.grid(True)
        # set_axes(axes)

        axes = plt.subplot(3, 1, 3)
        axes.plot(time, TC, 'bo-')
        axes.set_ylabel('Thrust Coefficient', axis_font)
        axes.grid(True)
    # set_axes(axes)


    axis_font = {'size':'14'}
    fig = plt.figure("Stability")
    fig.set_size_inches(12, 10)

    for segment in results.segments.values():
        time     = segment.conditions.frames.inertial.time[:,0] / Units.min
        cm       = segment.conditions.stability.static.CM[:,0]
        cm_alpha = segment.conditions.stability.static.Cm_alpha[:,0]
        SM       = segment.conditions.stability.static.static_margin[:,0]*100
        aoa      = segment.conditions.aerodynamics.angle_of_attack[:,0] / Units.deg
        percent_mac = segment.conditions.stability.static.percent_mac[:,0]
        mass        = segment.conditions.weights.total_mass

        axes = plt.subplot(2,2,1)
        axes.plot( time , aoa, 'bo-')
        axes.set_ylabel(r'$AoA$',axis_font)
        axes.grid(True)


        axes = plt.subplot(2,2,2)
        axes.plot( time , cm, 'bo-')
        axes.set_ylabel(r'$C_M$',axis_font)
        axes.grid(True)


        axes = plt.subplot(2,2,3)
        axes.plot( percent_mac ,mass , 'bo-')
        axes.set_xlabel('Percent MAC',axis_font)
        axes.set_ylabel('Mass (kg)',axis_font)
        axes.grid(True)


        axes = plt.subplot(2,2,4)
        axes.plot( time , SM, 'bo-')
        axes.set_xlabel('Time (min)',axis_font)
        axes.set_ylabel('Static Margin (%)',axis_font)
        axes.grid(True)





def plot_mission_single(results):
    markersize = 1.5
    # ------------------------------------------------------------------
    #   Aerodynamics
    # ------------------------------------------------------------------
    fig = plt.figure("Aerodynamic Coefficients_1")
    #for segment in results.segments.climb_9_5:
    segment = results.segments.cruise
    distance = (segment.conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
    time = segment.conditions.frames.inertial.time[:, 0] / Units.min
    CLift = segment.conditions.aerodynamics.lift_coefficient[:, 0]
    CDrag = segment.conditions.aerodynamics.drag_coefficient[:, 0]
    Drag = -segment.conditions.frames.wind.drag_force_vector[:, 0]
    Thrust = segment.conditions.frames.body.thrust_force_vector[:, 0]

    LoverD = CLift / CDrag

    axes = plt.subplot(4, 1, 1)
    axes.plot(distance, CLift, 'bo-', markersize=markersize)
    axes.set_ylabel('CL')
    axes.grid(True, which='both')
    axes.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.2f}"))

    axes = plt.subplot(4, 1, 2, sharex=axes)
    axes.plot(distance, CDrag, 'bo-', markersize=markersize)
    axes.set_ylabel('CD')
    axes.grid(True, which='both')
    axes.yaxis.set_major_formatter(ticker.StrMethodFormatter("{x:.4f}"))

    axes = plt.subplot(4, 1, 3, sharex=axes)
    axes.plot(distance, LoverD, 'bo-', markersize=markersize)
    axes.set_ylabel('L/D')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    axes.grid(True, which='both')

    axes = plt.subplot(4, 1, 4, sharex=axes)
    axes.plot(distance, Drag, 'bo-', markersize=markersize)
    axes.plot(distance, Thrust, 'ro-', markersize=markersize)
    axes.set_xlabel('Distance (nm)')
    axes.set_ylabel('Drag and Thrust (N)')
    axes.get_yaxis().get_major_formatter().set_scientific(False)
    axes.get_yaxis().get_major_formatter().set_useOffset(False)
    axes.grid(True, which='both')

    #     ------------------------------------------------------------------
    #     Mission Profile new
    #     ------------------------------------------------------------------

    fig = plt.figure("Mission Profile 1")
    for segment in results.segments.values():
        distance = (segment.conditions.frames.inertial.position_vector[:, 0]) / Units['nautical_mile']
        time = segment.conditions.frames.inertial.time[:, 0] / Units.min
        speed = segment.conditions.freestream.velocity[:, 0] / Units['knot']
        mach_number = segment.conditions.freestream.mach_number[:, 0]
        altitude = segment.conditions.freestream.altitude[:, 0] / Units.ft
        velocity = segment.conditions.freestream.velocity[:, 0]
        weight = segment.conditions.weights.total_mass[:, 0]
        drag = -segment.conditions.frames.wind.drag_force_vector[:, 0]
        thrust = segment.conditions.frames.body.thrust_force_vector[:, 0]
        ROC = (velocity * (thrust - drag) / (weight * 9.81)) * 196.8504  # ft/min
        eta = segment.conditions.propulsion.throttle[:, 0]

        axes = plt.subplot(3, 1, 1)
        axes.plot(distance, altitude, 'o-', color='blue', linewidth=1.0, markersize=markersize)
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.set_ylabel('Altitude [ft]')
        axes.grid(True)

        axes = plt.subplot(3, 1, 2, sharex=axes)
        axes.plot(distance, eta, 'bo-', markersize=markersize)
        axes.set_xlabel('Distance (nm)')
        axes.set_ylabel('eta')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True, which='both')


        axes = plt.subplot(3, 1, 3, sharex=axes)
        axes.plot(distance, ROC, 'bo-', markersize=markersize)
        axes.set_xlabel('Distance (nm)')
        axes.set_ylabel('Rate of climb [ft/min]')
        axes.get_yaxis().get_major_formatter().set_scientific(False)
        axes.get_yaxis().get_major_formatter().set_useOffset(False)
        axes.grid(True, which='both')

        fig.suptitle('Mission Profile', fontsize=14)

    #plt.show()