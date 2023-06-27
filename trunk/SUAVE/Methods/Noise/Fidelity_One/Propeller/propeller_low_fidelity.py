## @ingroupMethods-Noise-Fidelity_One-Propeller
# noise_propeller_low_fidelty.py
#
# Created:  May 2022, J Frank

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------
import SUAVE
from SUAVE.Core import Data, Units
import numpy as np
from scipy.interpolate import interp1d

# from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools.decibel_arithmetic import pressure_ratio_to_SPL_arithmetic
# from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import SPL_arithmetic
# from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import SPL_spectra_arithmetic
# from SUAVE.Methods.Noise.Fidelity_One.Noise_Tools import compute_point_source_coordinates
#
# from SUAVE.Methods.Noise.Fidelity_One.Propeller.compute_broadband_noise import compute_broadband_noise
# from SUAVE.Methods.Noise.Fidelity_One.Propeller.compute_harmonic_noise import compute_harmonic_noise


## @ingroupMethods-Noise-Fidelity_One-Propeller
def propeller_low_fidelity(net,propeller,segment,settings,gm_phi,gm_theta,alt):
    ''' This computes the acoustic signature (sound pressure level, weighted sound pressure levels,
    and frequency spectrums of a system of rotating blades (i.e. propellers and rotors)

    Assumptions:
    None

    Source:
    Roskam Airplane Aerodynamics and Performance, Chapter 7 "Propeller Noise"

    Inputs:
        network                 - vehicle energy network data structure               [None]
        segment                 - flight segment data structure                       [None]
        mic_loc                 - microhone location                                  [m]
        propeller               - propeller class data structure                      [None]
        auc_opts                - data structure of acoustic data                     [None]
        settings                - accoustic settings                                  [None]

    Outputs:
        Results.
            SPL                 - SPL                                                 [dB]
            SPL_dBA             - dbA-Weighted SPL                                    [dBA]
            SPL_bb_spectrum     - broadband contribution to total SPL                 [dB]
            SPL_spectrum        - 1/3 octave band SPL                                 [dB]
            SPL_tonal_spectrum  - harmonic contribution to total SPL                  [dB]
            SPL_bpfs_spectrum   - 1/3 octave band harmonic contribution to total SPL  [dB]

    Properties Used:
        N/A
    '''

    propeller_noise = Data()
    # unpack
    M_tip = segment.conditions.propulsion.propeller_tip_mach
    Input_Power = segment.conditions.propulsion.propeller_power / Units.hp
    B = propeller.number_of_blades
    D = 2 * propeller.tip_radius / Units.ft
    # theta = gm_theta
    # phi = gm_phi
    # alt = segment.conditions.
    # distance =  / Units.ft  is this correct ?
    n_props = net.number_of_engines
    if 'motorWTP' in net.keys():
        n_props +=2
    dim_alt = len(alt)
    dim_phi = len(gm_phi)
    dim_theta = len(gm_theta)
    num_mic = dim_phi * dim_theta

    # dimension:[control point, theta, phi]
    theta = np.repeat(np.repeat(np.atleast_2d(gm_theta).T, dim_phi, axis=1)[np.newaxis, :, :], dim_alt, axis=0) / Units.deg
    phi = np.repeat(np.repeat(np.atleast_2d(gm_phi), dim_theta, axis=0)[np.newaxis, :, :], dim_alt, axis=0) / Units.deg
    altitude = np.repeat(np.repeat(np.atleast_2d(alt).T, dim_theta, axis=1)[:, :, np.newaxis], dim_phi, axis=2)
    x_vals = altitude / np.tan(theta)
    y_vals = altitude / np.tan(phi)
    z_vals = altitude
    distance = np.sqrt(x_vals**2 + y_vals**2 + z_vals**2) / Units.ft

    for i in range(dim_alt):
        if M_tip[i] <= 0.4:
            M_tip[i] = 0.4
        elif M_tip[i] >= 0.9:
            M_tip[i] = 0.9

        if Input_Power[i] <= 30:
            Input_Power[i] = 30
        elif Input_Power[i] >= 10000:
            Input_Power[i] = 10000

        for j in range(dim_theta):
            for k in range(dim_phi):
                if distance[i,j,k] < 40:
                    distance[i,j,k] = 40
                elif distance[i,j,k] >= 9830:  # TODO: possibly better to extrapolate to get lower noise pressure levels for a very high distance
                    distance[i,j,k] = 9830

    if B <= 2:
        B = 2
    elif B >= 8:
        B = 8

    if D <= 2:
        D = 2
    elif D >= 50:
        D = 50

    FL1 = 6.796 * np.log(Input_Power) + 36.943 * M_tip + 17.67

    B_data = [2, 3, 4, 6, 8]
    FL2_const_data = [27.105, 24.315, 21.187, 18.127, 15.767]
    FL2_const_interpol = interp1d(B_data, FL2_const_data)

    FL2 = -8.887 * np.log(D) + FL2_const_interpol(B)

    distance_data = [40, 135, 190, 361, 712, 910, 1756, 1913, 4092, 4695, 9830]
    FL3_data = [22, 11.5, 8.4, 2.8, -3.6, -6.2, -13.8, -14.8, -24.4, -26.3, -37.5]
    FL3_interpol = interp1d(distance_data,FL3_data)

    FL3 = FL3_interpol(distance)

    theta_data = [20, 25.5, 32.1, 37.1, 42.7, 47.7, 54.3, 60.8, 67.1, 72.2, 77.2, 83.1, 87.3, 93.6, 99.6, 105.3, 110, 115.4, 121.9, 127.7, 132, 135.6, 139.8, 143.9, 148.6, 153.8, 156.7, 160]
    DI_data = [-4.14, -3.32, -2.43, -1.89, -1.42, -1.14, -1.03, -0.97, -0.61, -0.47, -0.44, -0.38, -0.14, 0.21, 0.42, 0.52, 0.47, 0.27, -0.26, -1.18, -2.01, -2.88, -3.99, -5.22, -6.78, -8.64, -9.81, -11.01]
    DI_interpol = interp1d(theta_data, DI_data)

    DI = DI_interpol(theta)

    n_props_data = [1, 2, 4, 8, 16]
    NC_data = [0, 3, 6, 9, 12]
    NC_interpol = interp1d(n_props_data,NC_data)

    NC = NC_interpol(n_props)

    FL1_vec = np.zeros((dim_alt,dim_phi,dim_theta))
    for i in range(dim_alt):
        FL1_vec[i,:,:] = FL1[i]
    FL2_vec = np.ones((dim_alt,dim_phi,dim_theta)) * FL2
    FL3_vec = FL3
    DI_vec = DI
    NC_vec = np.ones((dim_alt,dim_phi,dim_theta)) * NC

    OSPL_vec = FL1_vec + FL2_vec + FL3_vec + DI_vec + NC_vec

    OSPL = OSPL_vec.reshape(dim_alt,num_mic)
    propeller_noise.SPL_dBA = OSPL

    return propeller_noise