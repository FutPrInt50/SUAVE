## @ingroup Methods-Figures_of_Merit
# noise_certification.py
#
# Created:  May 2022, J. Frank


# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import numpy as np
import copy
import matplotlib.pyplot as plt
import SUAVE
from SUAVE.Core import Units
from SUAVE.Core import Data
from SUAVE.Methods.Noise.Fidelity_One.Airframe    import noise_airframe_Fink
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import noise_geometric
from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.decibel_arithmetic import SPL_arithmetic
from SUAVE.Methods.Figures_of_Merit.Supporting_Functions.propeller_noise import propeller_noise
from SUAVE.Methods.Figures_of_Merit.Supporting_Functions.arithmetic_mean import arithmetic_mean
from scipy.interpolate import interp1d

# ----------------------------------------------------------------------
#  Method
# ----------------------------------------------------------------------
## @ingroup Methods-Figures_of_Merit
def noise_certification(results, configs, settings, weighting):
    """ This method estimates the perceived noise level under the in ANNEX16 of ICAO described conditions.
    """
    vehicle = configs.base
    noise = Data()
    noise.sideline = Data()
    noise.flyover = Data()
    noise.approach = Data()

    #SIDELINE NOISE ESTIMATION
    num_mic_sideline = settings.number_of_microfones_sideline
    mic_pos_sideline_min = settings.pos_fist_microfone_sideline
    mic_pos_sideline_max = settings.pos_last_microfone_sideline

    mic_pos_sideline = np.zeros((num_mic_sideline, 3))
    mic_pos_sideline[:,0] = np.linspace(mic_pos_sideline_min, mic_pos_sideline_max, num_mic_sideline) * Units.meter
    mic_pos_sideline[:,1] = 450 * Units.meter
    mic_pos_sideline[:,2] = 0.

    take_off_length = settings.aircraft_take_off_distance
    mic_pos_flyover = [6500. - take_off_length / Units.meter, 0, 0] * Units.meter

    climb_segments = [key for key in results.segments.keys() if
                      (('climb' in key) and ('reserve' not in key) and ('second_leg' not in key))]
    n_climb_segments = len(climb_segments)

    # UNPACK DATA FOR PROPELLER ANALYSIS
    B = vehicle.propulsors.network.propeller.number_of_blades
    D = 2 * vehicle.propulsors.network.propeller.tip_radius / Units.ft
    n_props = vehicle.propulsors.network.number_of_engines

    hybrid = False
    if 'propellerWTP' in vehicle.propulsors.network.keys():
        hybrid = True
        B_WTP = vehicle.propulsors.network.propellerWTP.number_of_blades
        D_WTP = 2 * vehicle.propulsors.network.propellerWTP.tip_radius / Units.ft

    old_prop_noise_sideline = 0.
    old_prop_noise_flyover = 0.
    old_prop_noise_approach = 0.

    # AIRFRAME NOISE
    # INITIALIZE NECESSARY VALUES
    setting = Data()
    setting.center_frequencies = np.array([16, 20, 25, 31.5, 40, 50, 63, 80, 100, 125, 160, 200, 250, 315, 400, \
                                            500, 630, 800, 1000, 1250, 1600, 2000, 2500, 3150,
                                            4000, 5000, 6300, 8000, 10000])

    prop_noise_vec = np.zeros((1,num_mic_sideline,1))
    airframe_noise_vec = np.zeros((1,num_mic_sideline,1))
    gas_turbine_noise_vec = np.zeros((1,num_mic_sideline,1))
    motorWTP_noise_vec = np.zeros((1, num_mic_sideline, 1))
    gear_box_noise_vec = np.zeros((1,num_mic_sideline,1))

    prop_noise_vec_flyover = np.zeros((1, 1, 1))
    airframe_noise_vec_flyover = np.zeros((1, 1, 1))
    gas_turbine_noise_vec_flyover = np.zeros((1, 1, 1))
    motorWTP_noise_vec_flyover = np.zeros((1, 1, 1))
    gear_box_noise_vec_flyover = np.zeros((1, 1, 1))

    prop_noise_vec_approach = np.zeros((1, 1, 1))
    airframe_noise_vec_approach = np.zeros((1, 1, 1))
    gas_turbine_noise_vec_approach = np.zeros((1, 1, 1))
    motorWTP_noise_vec_approach = np.zeros((1, 1, 1))
    gear_box_noise_vec_approach = np.zeros((1, 1, 1))

    # SIDELINE NOISE PREDICTION

    for i in range(n_climb_segments):
        segment = results.segments[climb_segments[i]]
        vehicle = results.segments[climb_segments[i]].analyses.aerodynamics.geometry
        analyses = results.segments[climb_segments[i]].analyses
        M_tip = results.segments[climb_segments[i]].conditions.propulsion.propeller_tip_mach[:, 0]
        if hybrid == True:
            Input_Power = results.segments[climb_segments[i]].conditions.propulsion.power_propeller_turboprop / Units.hp
            Input_Power_WTP = results.segments[climb_segments[i]].conditions.propulsion.power_propeller_WTP / Units.hp
            M_tip_WTP = results.segments[climb_segments[i]].conditions.propulsion.propellerWTP_tip_mach[:, 0]
        else:
            Input_Power = results.segments[climb_segments[i]].conditions.propulsion.power / 2 / Units.hp
        Rotational_Speed = results.segments[climb_segments[i]].conditions.propulsion.propeller_rpm

        if 'angle' in vehicle.wings.main_wing.control_surfaces.flap.keys():
            vehicle.wings.main_wing.control_surfaces.flap.deflection = vehicle.wings.main_wing.control_surfaces.flap.angle
        else:
            vehicle.wings.main_wing.control_surfaces.flap.deflection = 0.

        if 'landing_gear_extracted' in vehicle.landing_gear.keys():
            if vehicle.landing_gear.landing_gear_extracted == True:
                vehicle.landing_gear.gear_condition = 'down'
            else:
                vehicle.landing_gear.gear_condition = 'up'
        # vehicle.landing_gear.gear_condition = 'down'

        vehicle.wings.main_wing.control_surfaces.flap.area = vehicle.wings.main_wing.flap_ratio * vehicle.wings.main_wing.areas.reference

        vehicle.wings.main_wing.control_surfaces.flap.configuration_type = vehicle.wings.main_wing.control_surfaces.flap.type

        vehicle.wings.main_wing.control_surfaces.flap.chord_dimensional = vehicle.wings.main_wing.control_surfaces.flap.chord_fraction * vehicle.wings.main_wing.chords.mean_aerodynamic

        aircraft_pos_airframe_list = results.segments[climb_segments[i]].conditions.frames.inertial.position_vector
        velocity_direction_airframe_list = results.segments[climb_segments[i]].conditions.frames.inertial.velocity_vector

        aircraft_pos_airframe = np.array(aircraft_pos_airframe_list)
        velocity_direction_airframe = np.array(velocity_direction_airframe_list)

        segment.dist = np.zeros(len(aircraft_pos_airframe[:,0]))
        segment.theta = np.zeros(len(aircraft_pos_airframe[:,0]))
        segment.phi = np.zeros(len(aircraft_pos_airframe[:,0]))
        noise_direction_airframe = np.zeros((len(aircraft_pos_airframe[:,0]), 3))

        prop_noise = np.zeros((len(aircraft_pos_airframe[:,0]), num_mic_sideline, 1))
        airframe_noise = np.zeros((len(aircraft_pos_airframe[:,0]), num_mic_sideline, 1))
        gas_turbine_noise = np.zeros((len(aircraft_pos_airframe[:,0]), num_mic_sideline, 1))
        motorWTP_noise = np.zeros((len(aircraft_pos_airframe[:,0]), num_mic_sideline, 1))
        gear_box_noise = np.zeros((len(aircraft_pos_airframe[:,0]), num_mic_sideline, 1))

        for mic in range(num_mic_sideline):
            # BERECHNUNG DISTANCE, THETA, PHI
            for cntrl_point in range(len(aircraft_pos_airframe[:,0])):
                noise_direction_airframe[cntrl_point, :] = mic_pos_sideline[mic, :] - aircraft_pos_airframe[cntrl_point, :]
                distance = np.sqrt(np.vdot(noise_direction_airframe[cntrl_point,:], noise_direction_airframe[cntrl_point,:]))
                theta = np.arccos(np.vdot(noise_direction_airframe[cntrl_point,:], velocity_direction_airframe[cntrl_point,:]) / (distance * np.sqrt(np.vdot(velocity_direction_airframe[cntrl_point,:], velocity_direction_airframe[cntrl_point,:]))))

                gamma = - np.arctan(velocity_direction_airframe[cntrl_point,2] / velocity_direction_airframe[cntrl_point,0])
                b_length = - aircraft_pos_airframe[cntrl_point,2] / np.cos(gamma)
                phi = np.arctan(mic_pos_sideline[mic, 1] / b_length)

                segment.dist[cntrl_point] = distance
                segment.theta[cntrl_point] = theta
                segment.phi[cntrl_point] = phi

                noise_prop = propeller_noise(M_tip[cntrl_point], Input_Power[cntrl_point], distance, theta * 180 / np.pi, B, D, n_props)
                if hybrid == True:
                    noise_prop_WTP = propeller_noise(M_tip_WTP[cntrl_point], Input_Power_WTP[cntrl_point], distance, theta * 180 / np.pi, B_WTP, D_WTP, 2)
                    noise_prop = 10 * np.log10(10 ** (0.1 * noise_prop) + 10 ** (0.1 * noise_prop_WTP))
                prop_noise[cntrl_point, mic] = noise_prop
                # DIESEN ABSCHNITT SPÄTER ENTFERNEN
                if noise_prop > old_prop_noise_sideline:
                    old_prop_noise_sideline = noise_prop
                    max_sideline_M_tip = M_tip[cntrl_point]
                    max_sideline_Input_Power = Input_Power[cntrl_point]
                    max_sideline_distance = distance
                    max_sideline_theta = theta * 180 / np.pi
                    max_sideline_B = B
                    max_sideline_D = D
                    max_sideline_n_props = n_props
                    if hybrid == True:
                        max_sideline_noise_WTP = noise_prop_WTP
                    else:
                        max_sideline_noise_WTP = 0.

                if 'gas_turbine' or 'engine' in vehicle.propulsors.network.keys():
                    if hybrid == True:
                        p_gt = results.segments[climb_segments[i]].conditions.propulsion.power_turboshaft[cntrl_point] / 1000 / Units.kW
                    else:
                        p_gt = Input_Power[cntrl_point] * Units.hp / 1000 / Units.kW
                    L_w_Casing = 118 + 5 * np.log10(p_gt)
                    L_w_Exhaust = 129 + 10 * np.log10(p_gt)
                    L_w_Intake = 127 + 15 * np.log10(p_gt)
                    L_w_gt = 10 * np.log10(10 **(0.1*L_w_Casing) + 10**(0.1*L_w_Exhaust) + 10**(0.1*L_w_Intake))
                    L_p_gt = L_w_gt - abs( 10 * np.log10(1/(4*np.pi*distance**2)))
                    gas_turbine_noise[cntrl_point, mic] = L_p_gt

                if 'motorWTP' in vehicle.propulsors.network.keys():
                    p_em = results.segments[climb_segments[i]].conditions.propulsion.power_WTP[cntrl_point] / Units.kW
                    n_em = Rotational_Speed[cntrl_point] # no translation assumed?
                    S = 1. # Annahme 1 m^2 Fläche
                    if p_em < 40.:
                        L_w_em = 16 + 17 * np.log10(p_em) + 15 * np.log10(n_em) + 10 * np.log10(S)
                    else:
                        L_w_em = 27 + 10 * np.log10(p_em) + 15 * np.log10(n_em) + 10 * np.log10(S)
                    L_p_em = L_w_em - abs(10 * np.log10(1 / (4 * np.pi * distance ** 2)))
                    L_p_em = L_p_em + 3.01 # zwei identisch arbeitende Motoren werden summiert
                    motorWTP_noise[cntrl_point, mic] = L_p_em

                if 'gearbox' in vehicle.propulsors.network.keys():
                    p_sp = Input_Power[cntrl_point] * Units.hp / Units.kW
                    n_p = Rotational_Speed[cntrl_point]
                    S = 1. # Annahme 1 m^2 Fläche
                    L_w_gb = 86 + 4 * np.log10(p_sp) + 3 * np.log10(n_p) + 10 * np.log10(S)
                    L_p_gb = L_w_gb - abs( 10 * np.log10(1/(4*np.pi*distance**2)))
                    gear_box_noise[cntrl_point, mic] = L_p_gb

            noise_airframe = noise_airframe_Fink(segment, analyses, vehicle, setting)
            airframe_noise[:, mic] = noise_airframe.SPL_dBA
            #print('airframe noise %.2f dBA' % noise_airframe.SPL_dBA)

        prop_noise_vec = np.append(prop_noise_vec, prop_noise, axis = 0)
        airframe_noise_vec = np.append(airframe_noise_vec, airframe_noise, axis = 0)
        gas_turbine_noise_vec = np.append(gas_turbine_noise_vec, gas_turbine_noise, axis = 0)
        motorWTP_noise_vec = np.append(motorWTP_noise_vec, motorWTP_noise, axis = 0)
        gear_box_noise_vec = np.append(gear_box_noise_vec, gear_box_noise, axis = 0)


    total_noise_vec = np.append(prop_noise_vec,airframe_noise_vec,axis=2)
    total_noise_vec = np.append(total_noise_vec,gas_turbine_noise_vec,axis=2)
    total_noise_vec = np.append(total_noise_vec, motorWTP_noise_vec, axis=2)
    total_noise_vec = np.append(total_noise_vec,gear_box_noise_vec,axis=2)
    total_noise_vec = SPL_arithmetic(total_noise_vec, sum_axis=2)

    noise.sideline.prop = np.max(prop_noise_vec)
    noise.sideline.airframe = np.max(airframe_noise_vec)
    noise.sideline.gas_turbine = np.max(gas_turbine_noise_vec)
    noise.sideline.motorWTP = np.max(motorWTP_noise_vec)
    noise.sideline.gear_box = np.max(gear_box_noise_vec)
    noise.sideline.total = np.max(total_noise_vec)
    noise.sideline.total += settings.correction_sideline
    noise.sideline.total_sone = 2**((noise.sideline.total - 40)/10)


    #FLYOVER NOISE PREDICTION

    for i in range(n_climb_segments):
        segment = results.segments[climb_segments[i]]
        vehicle = results.segments[climb_segments[i]].analyses.aerodynamics.geometry
        analyses = results.segments[climb_segments[i]].analyses
        M_tip = results.segments[climb_segments[i]].conditions.propulsion.propeller_tip_mach[:, 0]
        if hybrid == True:
            Input_Power = results.segments[climb_segments[i]].conditions.propulsion.power_propeller_turboprop / Units.hp
            Input_Power_WTP = results.segments[climb_segments[i]].conditions.propulsion.power_propeller_WTP / Units.hp
            M_tip_WTP = results.segments[climb_segments[i]].conditions.propulsion.propellerWTP_tip_mach[:, 0]
        else:
            Input_Power = results.segments[climb_segments[i]].conditions.propulsion.power / 2 / Units.hp
        Rotational_Speed = results.segments[climb_segments[i]].conditions.propulsion.propeller_rpm

        if 'angle' in vehicle.wings.main_wing.control_surfaces.flap.keys():
            vehicle.wings.main_wing.control_surfaces.flap.deflection = vehicle.wings.main_wing.control_surfaces.flap.angle
        else:
            vehicle.wings.main_wing.control_surfaces.flap.deflection = 0.

        if 'landing_gear_extracted' in vehicle.landing_gear.keys():
            if vehicle.landing_gear.landing_gear_extracted == True:
                vehicle.landing_gear.gear_condition = 'down'
            else:
                vehicle.landing_gear.gear_condition = 'up'
        # vehicle.landing_gear.gear_condition = 'down'

        vehicle.wings.main_wing.control_surfaces.flap.area = vehicle.wings.main_wing.flap_ratio * vehicle.wings.main_wing.areas.reference

        vehicle.wings.main_wing.control_surfaces.flap.configuration_type = vehicle.wings.main_wing.control_surfaces.flap.type

        vehicle.wings.main_wing.control_surfaces.flap.chord_dimensional = vehicle.wings.main_wing.control_surfaces.flap.chord_fraction * vehicle.wings.main_wing.chords.mean_aerodynamic

        aircraft_pos_airframe_list = results.segments[climb_segments[i]].conditions.frames.inertial.position_vector
        velocity_direction_airframe_list = results.segments[climb_segments[i]].conditions.frames.inertial.velocity_vector

        aircraft_pos_airframe = np.array(aircraft_pos_airframe_list)
        velocity_direction_airframe = np.array(velocity_direction_airframe_list)

        segment.dist = np.zeros(len(aircraft_pos_airframe[:,0]))
        segment.theta = np.zeros(len(aircraft_pos_airframe[:,0]))
        segment.phi = np.zeros(len(aircraft_pos_airframe[:,0]))
        noise_direction_airframe = np.zeros((len(aircraft_pos_airframe[:,0]), 3))

        prop_noise_flyover = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))
        airframe_noise_flyover = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))
        gas_turbine_noise_flyover = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))
        motorWTP_noise_flyover = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))
        gear_box_noise_flyover = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))

        # BERECHNUNG DISTANCE, THETA, PHI
        for cntrl_point in range(len(aircraft_pos_airframe[:,0])):
            noise_direction_airframe[cntrl_point, :] = mic_pos_flyover[:] - aircraft_pos_airframe[cntrl_point, :]
            #distance = np.sqrt(np.vdot(noise_direction_airframe[cntrl_point,:], noise_direction_airframe[cntrl_point,:]))
            #theta = np.arccos(np.vdot(noise_direction_airframe[cntrl_point,:], velocity_direction_airframe[cntrl_point,:]) / (distance * np.sqrt(np.vdot(velocity_direction_airframe[cntrl_point,:], velocity_direction_airframe[cntrl_point,:]))))
            distance = 550.
            theta = 105. / 180 * np.pi


            gamma = - np.arctan(velocity_direction_airframe[cntrl_point,2] / velocity_direction_airframe[cntrl_point,0])
            b_length = - aircraft_pos_airframe[cntrl_point,2] / np.cos(gamma)
            phi = np.arctan(mic_pos_flyover[1] / b_length)

            segment.dist[cntrl_point] = distance
            segment.theta[cntrl_point] = theta
            segment.phi[cntrl_point] = phi

            noise_prop = propeller_noise(M_tip[cntrl_point], Input_Power[cntrl_point], distance, theta * 180 / np.pi, B, D, n_props)

            if hybrid == True:
                noise_prop_WTP = propeller_noise(M_tip_WTP[cntrl_point], Input_Power_WTP[cntrl_point], distance,
                                                 theta * 180 / np.pi, B_WTP, D_WTP, 2)
                noise_prop = 10 * np.log10(10 ** (0.1 * noise_prop) + 10 ** (0.1 * noise_prop_WTP))
            prop_noise_flyover[cntrl_point, 0] = noise_prop
            # DIESEN ABSCHNITT SPÄTER ENTFERNEN
            if noise_prop > old_prop_noise_flyover:
                old_prop_noise_flyover = noise_prop
                max_flyover_M_tip = M_tip[cntrl_point]
                max_flyover_Input_Power = Input_Power[cntrl_point]
                max_flyover_distance = distance
                max_flyover_theta = theta * 180 / np.pi
                max_flyover_B = B
                max_flyover_D = D
                max_flyover_n_props = n_props
                if hybrid == True:
                    max_flyover_noise_WTP = noise_prop_WTP
                else:
                    max_flyover_noise_WTP = 0.

            if 'gas_turbine' or 'engine' in vehicle.propulsors.network.keys():
                if hybrid == True:
                    p_gt = results.segments[climb_segments[i]].conditions.propulsion.power_turboshaft[
                               cntrl_point] / 1000 / Units.kW
                else:
                    p_gt = Input_Power[cntrl_point] * Units.hp / 1000 / Units.kW
                L_w_Casing = 118 + 5 * np.log10(p_gt)
                L_w_Exhaust = 129 + 10 * np.log10(p_gt)
                L_w_Intake = 127 + 15 * np.log10(p_gt)
                L_w_gt = 10 * np.log10(10 **(0.1*L_w_Casing) + 10**(0.1*L_w_Exhaust) + 10**(0.1*L_w_Intake))
                L_p_gt = L_w_gt - abs( 10 * np.log10(1/(4*np.pi*distance**2)))
                gas_turbine_noise_flyover[cntrl_point, 0] = L_p_gt

            if 'motorWTP' in vehicle.propulsors.network.keys():
                p_em = results.segments[climb_segments[i]].conditions.propulsion.power_WTP[cntrl_point] / Units.kW
                n_em = Rotational_Speed[cntrl_point]  # no translation assumed?
                S = 1.  # Annahme 1 m^2 Fläche
                if p_em < 40.:
                    L_w_em = 16 + 17 * np.log10(p_em) + 15 * np.log10(n_em) + 10 * np.log10(S)
                else:
                    L_w_em = 27 + 10 * np.log10(p_em) + 15 * np.log10(n_em) + 10 * np.log10(S)
                L_p_em = L_w_em - abs(10 * np.log10(1 / (4 * np.pi * distance ** 2)))
                L_p_em = L_p_em + 3.01  # zwei identisch arbeitende Motoren werden summiert
                motorWTP_noise_flyover[cntrl_point, 0] = L_p_em

            if 'gearbox' in vehicle.propulsors.network.keys():
                p_sp = Input_Power[cntrl_point] * Units.hp / Units.kW
                n_p = Rotational_Speed[cntrl_point]
                S = 1. # Annahme 1 m^2 Fläche
                L_w_gb = 86 + 4 * np.log10(p_sp) + 3 * np.log10(n_p) + 10 * np.log10(S)
                L_p_gb = L_w_gb - abs( 10 * np.log10(1/(4*np.pi*distance**2)))
                gear_box_noise_flyover[cntrl_point, 0] = L_p_gb

        noise_airframe = noise_airframe_Fink(segment, analyses, vehicle, setting)
        airframe_noise_flyover[:, 0] = noise_airframe.SPL_dBA
        prop_noise_vec_flyover = np.append(prop_noise_vec_flyover, prop_noise_flyover, axis=0)
        airframe_noise_vec_flyover = np.append(airframe_noise_vec_flyover, airframe_noise_flyover, axis=0)
        gas_turbine_noise_vec_flyover = np.append(gas_turbine_noise_vec_flyover, gas_turbine_noise_flyover, axis=0)
        motorWTP_noise_vec_flyover = np.append(motorWTP_noise_vec_flyover, motorWTP_noise_flyover, axis=0)
        gear_box_noise_vec_flyover = np.append(gear_box_noise_vec_flyover, gear_box_noise_flyover, axis=0)

    total_noise_vec_flyover = np.append(prop_noise_vec_flyover, airframe_noise_vec_flyover, axis=2)
    total_noise_vec_flyover = np.append(total_noise_vec_flyover, gas_turbine_noise_vec_flyover, axis=2)
    total_noise_vec_flyover = np.append(total_noise_vec_flyover, motorWTP_noise_vec_flyover, axis=2)
    total_noise_vec_flyover = np.append(total_noise_vec_flyover, gear_box_noise_vec_flyover, axis=2)
    total_noise_vec_flyover = SPL_arithmetic(total_noise_vec_flyover, sum_axis=2)


    noise.flyover.prop = np.max(prop_noise_vec_flyover)
    noise.flyover.airframe = np.max(airframe_noise_vec_flyover)
    noise.flyover.gas_turbine = np.max(gas_turbine_noise_vec_flyover)
    noise.flyover.motorWTP = np.max(motorWTP_noise_flyover)
    noise.flyover.gear_box = np.max(gear_box_noise_vec_flyover)
    noise.flyover.total = np.max(total_noise_vec_flyover)
    noise.flyover.total += settings.correction_flyover
    noise.flyover.total_sone = 2**((noise.flyover.total - 40)/10)


    # APPROACH NOISE PREDICTION

    descent_segments = [key for key in results.segments.keys() if
                      (('descent' in key) and ('reserve' not in key) and ('second_leg' not in key))]
    n_descent_segments = len(descent_segments)

    aircraft_pos_list_descent = []
    M_tip_list_descent = []

    for i in range(n_descent_segments):
        aircraft_pos_list_descent.extend(results.segments[descent_segments[i]].conditions.frames.inertial.position_vector)
        M_tip_list_descent.extend(results.segments[descent_segments[i]].conditions.propulsion.propeller_tip_mach[:,0])

    aircraft_pos_descent = np.array(aircraft_pos_list_descent)
    M_tip_descent = np.array(M_tip_list_descent)

    num_control_points_descent = len(M_tip_list_descent)
    tick = False
    #
    for control_point in range(num_control_points_descent):
        if (tick == False) and abs(aircraft_pos_descent[control_point,2]) <= 120 * Units.meter:
            mic_pos_approach = [aircraft_pos_descent[control_point,0], 0, 0]
            tick = True

    for i in range(n_descent_segments):
        segment = results.segments[descent_segments[i]]
        vehicle = results.segments[descent_segments[i]].analyses.aerodynamics.geometry
        analyses = results.segments[descent_segments[i]].analyses
        M_tip = results.segments[descent_segments[i]].conditions.propulsion.propeller_tip_mach[:, 0]
        if hybrid == True:
            Input_Power = results.segments[descent_segments[i]].conditions.propulsion.power_propeller_turboprop / Units.hp
            Input_Power_WTP = results.segments[descent_segments[i]].conditions.propulsion.power_propeller_WTP / Units.hp
            M_tip_WTP = results.segments[descent_segments[i]].conditions.propulsion.propellerWTP_tip_mach[:, 0]
        else:
            Input_Power = results.segments[descent_segments[i]].conditions.propulsion.power / 2 / Units.hp
        Rotational_Speed = results.segments[descent_segments[i]].conditions.propulsion.propeller_rpm

        if 'angle' in vehicle.wings.main_wing.control_surfaces.flap.keys():
            vehicle.wings.main_wing.control_surfaces.flap.deflection = vehicle.wings.main_wing.control_surfaces.flap.angle
        else:
            vehicle.wings.main_wing.control_surfaces.flap.deflection = 0.

        if 'landing_gear_extracted' in vehicle.landing_gear.keys():
            if vehicle.landing_gear.landing_gear_extracted == True:
                vehicle.landing_gear.gear_condition = 'down'
            else:
                vehicle.landing_gear.gear_condition = 'up'
        # vehicle.landing_gear.gear_condition = 'down'

        vehicle.wings.main_wing.control_surfaces.flap.area = vehicle.wings.main_wing.flap_ratio * vehicle.wings.main_wing.areas.reference

        vehicle.wings.main_wing.control_surfaces.flap.configuration_type = vehicle.wings.main_wing.control_surfaces.flap.type

        vehicle.wings.main_wing.control_surfaces.flap.chord_dimensional = vehicle.wings.main_wing.control_surfaces.flap.chord_fraction * vehicle.wings.main_wing.chords.mean_aerodynamic

        aircraft_pos_airframe_list = results.segments[descent_segments[i]].conditions.frames.inertial.position_vector
        velocity_direction_airframe_list = results.segments[descent_segments[i]].conditions.frames.inertial.velocity_vector

        aircraft_pos_airframe = np.array(aircraft_pos_airframe_list)
        velocity_direction_airframe = np.array(velocity_direction_airframe_list)

        segment.dist = np.zeros(len(aircraft_pos_airframe[:,0]))
        segment.theta = np.zeros(len(aircraft_pos_airframe[:,0]))
        segment.phi = np.zeros(len(aircraft_pos_airframe[:,0]))
        noise_direction_airframe = np.zeros((len(aircraft_pos_airframe[:,0]), 3))

        prop_noise_approach = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))
        airframe_noise_approach = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))
        gas_turbine_noise_approach = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))
        motorWTP_noise_approach = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))
        gear_box_noise_approach = np.zeros((len(aircraft_pos_airframe[:,0]), 1, 1))


        # BERECHNUNG DISTANCE, THETA, PHI
        for cntrl_point in range(len(aircraft_pos_airframe[:,0])):
            noise_direction_airframe[cntrl_point, :] = mic_pos_approach[:] - aircraft_pos_airframe[cntrl_point, :]
            distance = np.sqrt(np.vdot(noise_direction_airframe[cntrl_point,:], noise_direction_airframe[cntrl_point,:]))
            theta = np.arccos(np.vdot(noise_direction_airframe[cntrl_point,:], velocity_direction_airframe[cntrl_point,:]) / (distance * np.sqrt(np.vdot(velocity_direction_airframe[cntrl_point,:], velocity_direction_airframe[cntrl_point,:]))))

            gamma = - np.arctan(velocity_direction_airframe[cntrl_point,2] / velocity_direction_airframe[cntrl_point,0])
            b_length = - aircraft_pos_airframe[cntrl_point,2] / np.cos(gamma)
            phi = np.arctan(mic_pos_flyover[1] / b_length)

            segment.dist[cntrl_point] = distance
            segment.theta[cntrl_point] = theta
            segment.phi[cntrl_point] = phi

            noise_prop = propeller_noise(M_tip[cntrl_point], Input_Power[cntrl_point], distance, theta * 180 / np.pi, B, D, n_props)
            if hybrid == True:
                noise_prop_WTP = propeller_noise(M_tip_WTP[cntrl_point], Input_Power_WTP[cntrl_point], distance, theta * 180 / np.pi, B_WTP, D_WTP, 2)
                if noise_prop_WTP < 0:
                    noise_prop_WTP = 0.
                noise_prop = 10 * np.log10(10 ** (0.1 * noise_prop) + 10 ** (0.1 * noise_prop_WTP))
            prop_noise_approach[cntrl_point, 0] = noise_prop
            # DIESEN ABSCHNITT SPÄTER ENTFERNEN
            if noise_prop > old_prop_noise_approach:
                old_prop_noise_approach = noise_prop
                max_approach_M_tip = M_tip[cntrl_point]
                max_approach_Input_Power = Input_Power[cntrl_point]
                max_approach_distance = distance
                max_approach_theta = theta * 180 / np.pi
                max_approach_B = B
                max_approach_D = D
                max_approach_n_props = n_props
                if hybrid == True:
                    max_approach_noise_WTP = noise_prop_WTP
                else:
                    max_approach_noise_WTP = 0.

            if 'gas_turbine' or 'engine' in vehicle.propulsors.network.keys():
                if hybrid == True:
                    p_gt = results.segments[descent_segments[i]].conditions.propulsion.power_turboshaft[
                               cntrl_point] / 1000 / Units.kW
                else:
                    p_gt = Input_Power[cntrl_point] * Units.hp / 1000 / Units.kW
                L_w_Casing = 118 + 5 * np.log10(p_gt)
                L_w_Exhaust = 129 + 10 * np.log10(p_gt)
                L_w_Intake = 127 + 15 * np.log10(p_gt)
                L_w_gt = 10 * np.log10(10 **(0.1*L_w_Casing) + 10**(0.1*L_w_Exhaust) + 10**(0.1*L_w_Intake))
                L_p_gt = L_w_gt - abs( 10 * np.log10(1/(4*np.pi*distance**2)))
                gas_turbine_noise_approach[cntrl_point, 0] = L_p_gt

            if 'motorWTP' in vehicle.propulsors.network.keys():
                p_em = results.segments[descent_segments[i]].conditions.propulsion.power_WTP[cntrl_point] / Units.kW
                n_em = Rotational_Speed[cntrl_point]  # no translation assumed?
                S = 1.  # Annahme 1 m^2 Fläche
                if p_em < 40.:
                    L_w_em = 16 + 17 * np.log10(p_em) + 15 * np.log10(n_em) + 10 * np.log10(S)
                else:
                    L_w_em = 27 + 10 * np.log10(p_em) + 15 * np.log10(n_em) + 10 * np.log10(S)
                L_p_em = L_w_em - abs(10 * np.log10(1 / (4 * np.pi * distance ** 2)))
                L_p_em = L_p_em + 3.01  # zwei identisch arbeitende Motoren werden summiert
                motorWTP_noise_approach[cntrl_point, 0] = L_p_em

            if 'gearbox' in vehicle.propulsors.network.keys():
                p_sp = Input_Power[cntrl_point] * Units.hp / Units.kW
                n_p = Rotational_Speed[cntrl_point]
                S = 1. # Annahme 1 m^2 Fläche
                L_w_gb = 86 + 4 * np.log10(p_sp) + 3 * np.log10(n_p) + 10 * np.log10(S)
                L_p_gb = L_w_gb - abs( 10 * np.log10(1/(4*np.pi*distance**2)))
                gear_box_noise_approach[cntrl_point, 0] = L_p_gb

        noise_airframe = noise_airframe_Fink(segment, analyses, vehicle, setting)
        airframe_noise_approach[:, 0] = noise_airframe.SPL_dBA
        #print('airframe noise %.2f dBA' % noise_airframe.SPL_dBA)

        prop_noise_vec_approach = np.append(prop_noise_vec_approach, prop_noise_approach, axis=0)
        airframe_noise_vec_approach = np.append(airframe_noise_vec_approach, airframe_noise_approach, axis=0)
        gas_turbine_noise_vec_approach = np.append(gas_turbine_noise_vec_approach, gas_turbine_noise_approach, axis=0)
        motorWTP_noise_vec_approach = np.append(motorWTP_noise_vec_approach, motorWTP_noise_approach, axis=0)
        gear_box_noise_vec_approach = np.append(gear_box_noise_vec_approach, gear_box_noise_approach, axis=0)

    total_noise_vec_approach = np.append(prop_noise_vec_approach, airframe_noise_vec_approach, axis=2)
    total_noise_vec_approach = np.append(total_noise_vec_approach, gas_turbine_noise_vec_approach, axis=2)
    total_noise_vec_approach = np.append(total_noise_vec_approach, motorWTP_noise_vec_approach, axis=2)
    total_noise_vec_approach = np.append(total_noise_vec_approach, gear_box_noise_vec_approach, axis=2)
    total_noise_vec_approach = SPL_arithmetic(total_noise_vec_approach, sum_axis=2)


    noise.approach.prop = np.max(prop_noise_vec_approach)
    noise.approach.airframe = np.max(airframe_noise_vec_approach)
    noise.approach.gas_turbine = np.max(gas_turbine_noise_vec_approach)
    noise.approach.motorWTP = np.max(motorWTP_noise_vec_approach)
    noise.approach.gear_box = np.max(gear_box_noise_vec_approach)
    noise.approach.total = np.max(total_noise_vec_approach)
    noise.approach.total += settings.correction_approach
    #noise.approach.total_sone = phon_to_sone(noise.approach.total)
    noise.approach.total_sone = 2 ** ((noise.approach.total - 40) / 10)

    noise.total_sone = arithmetic_mean([noise.sideline.total_sone, noise.flyover.total_sone, noise.approach.total_sone], weighting)

    if settings.print_flag == True:
    #     print('\n')
    #     print('\n')
    #     print('total noise sone: %.9f sone' % noise.total_sone)
    #     print('\n')
    #     print('the maximum noise levels were estimated based on the following inputs')
    #     print('SIDELINE')
    #     print('max noise level: %.2f dB' % noise.sideline.prop)
    #     print('max noise level WTP: %.2f dB' % max_sideline_noise_WTP)
    #     print('M_tip: %.2f' % max_sideline_M_tip)
    #     print('Input Power: %.2f hp' % max_sideline_Input_Power)
    #     print('Distance: %.2f m' % max_sideline_distance)
    #     print('Theta: %.2f deg' % max_sideline_theta)
    #     print('B: %1.f' % max_sideline_B)
    #     print('D: %.1f' % max_sideline_D)
    #     print('n_props: %.1f' % max_sideline_n_props)
         print('SIDELINE NOISE')
         print('maximum prop noise: %.9f dB' % noise.sideline.prop)
         print('maximum airframe noise: %.9f dB' % noise.sideline.airframe)
    #     print('maximum gas turbine noise: %.9f dB' % noise.sideline.gas_turbine)
    #     print('maximum electric motor noise: %.9f dB' % noise.sideline.motorWTP)
    #     print('maximum gear box noise: %.9f dB' % noise.sideline.gear_box)
    #     print('maximum total noise: %.9f dB' % noise.sideline.total)
    #     print('maximum total noise: %.9f sone' % noise.sideline.total_sone)
    #     print('\n')
    #     print('FLYOVER')
    #     print('flyover mic pos: %.1f m' % mic_pos_flyover[0])
    #     print('max noise level: %.2f dB' % noise.flyover.prop)
    #     print('max noise level WTP: %.2f dB' % max_flyover_noise_WTP)
    #     print('M_tip: %.2f' % max_flyover_M_tip)
    #     print('Input Power: %.2f hp' % max_flyover_Input_Power)
    #     print('Distance: %.2f m' % max_flyover_distance)
    #     print('Theta: %.2f deg' % max_flyover_theta)
    #     print('B: %1.f' % max_flyover_B)
    #     print('D: %.1f' % max_flyover_D)
    #     print('n_props: %.1f' % max_flyover_n_props)
    #     print('\n')
         print('FLYOVER NOISE')
         print('maximum prop noise: %.9f dB' % noise.flyover.prop)
         print('maximum airframe noise: %.9f dB' % noise.flyover.airframe)
    #     print('maximum gas turbine noise: %.9f dB' % noise.flyover.gas_turbine)
    #     print('maximum electric motor noise: %.9f dB' % noise.flyover.motorWTP)
    #     print('maximum gear box noise: %.9f dB' % noise.flyover.gear_box)
    #     print('maximum total noise: %.9f dB' % noise.flyover.total)
    #     print('maximum total noise: %.9f sone' % noise.flyover.total_sone)
    #     print('APPROACH')
    #     print('max noise level: %.2f dB' % noise.approach.prop)
    #     print('max noise level WTP: %.2f dB' % max_approach_noise_WTP)
    #     print('M_tip: %.2f' % max_approach_M_tip)
    #     print('Input Power: %.2f hp' % max_approach_Input_Power)
    #     print('Distance: %.2f m' % max_approach_distance)
    #     print('Theta: %.2f deg' % max_approach_theta)
    #     print('B: %1.f' % max_approach_B)
    #     print('D: %.1f' % max_approach_D)
    #     print('n_props: %.1f' % max_approach_n_props)
         print('APPROACH NOISE')
         print('maximum prop noise: %.9f dB' % noise.approach.prop)
         print('maximum airframe noise: %.9f dB' % noise.approach.airframe)
    #     print('maximum gas turbine noise: %.9f dB' % noise.approach.gas_turbine)
    #     print('maximum electric motor noise: %.9f dB' % noise.approach.motorWTP)
    #     print('maximum gear box noise: %.9f dB' % noise.approach.gear_box)
    #     print('maximum total noise: %.9f dB' % noise.approach.total)
    #     print('maximum total noise: %.9f sone' % noise.approach.total_sone)

    return noise