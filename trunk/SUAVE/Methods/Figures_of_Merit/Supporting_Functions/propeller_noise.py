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
from scipy.interpolate import interp1d, interp2d

## @ingroupMethods-Figures_of_Merit-Supporting_Functions
def propeller_noise(M_tip, Input_Power, distance, theta, B, D, n_props):
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

    FL1 = 6.796 * np.log(Input_Power) + 36.943 * M_tip + 17.67

    B_data = [2, 3, 4, 6, 8]
    FL2_const_data = [27.105, 24.315, 21.187, 18.127, 15.767]
    FL2_const_interpol = interp1d(B_data, FL2_const_data, kind = 'cubic', fill_value = 'extrapolate')

    FL2 = -8.887 * np.log(D) + FL2_const_interpol(B)

    distance_data = [40, 135, 190, 361, 712, 910, 1756, 1913, 4092, 4695, 9830]
    FL3_data = [22, 11.5, 8.4, 2.8, -3.6, -6.2, -13.8, -14.8, -24.4, -26.3, -37.5]
    FL3_interpol = interp1d(distance_data,FL3_data, kind = 'cubic', bounds_error=False, fill_value = -37.5)

    FL3 = FL3_interpol(distance)

    theta_data = [20, 25.5, 32.1, 37.1, 42.7, 47.7, 54.3, 60.8, 67.1, 72.2, 77.2, 83.1, 87.3, 93.6, 99.6, 105.3, 110, 115.4, 121.9, 127.7, 132, 135.6, 139.8, 143.9, 148.6, 153.8, 156.7, 160]
    DI_data = [-4.14, -3.32, -2.43, -1.89, -1.42, -1.14, -1.03, -0.97, -0.61, -0.47, -0.44, -0.38, -0.14, 0.21, 0.42, 0.52, 0.47, 0.27, -0.26, -1.18, -2.01, -2.88, -3.99, -5.22, -6.78, -8.64, -9.81, -11.01]
    DI_interpol = interp1d(theta_data, DI_data, kind = 'cubic', fill_value = 'extrapolate')

    DI = DI_interpol(theta)

    n_props_data = [1, 2, 4, 8, 16]
    NC_data = [0, 3, 6, 9, 12]
    NC_interpol = interp1d(n_props_data,NC_data, kind = 'cubic', fill_value = 'extrapolate')

    NC = NC_interpol(n_props)

    OSPL = FL1 + FL2 + FL3 + DI + NC

    M_tip_data = [0.5, 0.6, 0.7, 0.8, 0.85, 0.9]

    if B == 2:
        D_data = [5., 7.5, 10., 13.2, 15.3, 17.5, 20.8, 23.7, 25.]
        delta_PNL_data = [[0.8, -2.3, -5.8, -8.8, -10.1, -10.8, -11.2, -11.4, -11.5],
                          [1.7, -1.1, -3.7, -7.2, -8.7, -9.8, -10.4, -10.7, -10.7],
                          [2.3, 0., -1.8, -4.6, -6.3, -7.6, -8.4, -8.7, -8.8],
                          [3.6, 1.3, -0.4, -2.5, -3.9, -4.9, -5.6, -5.8, -5.8],
                          [4.3, 2.4, 0.9, -0.6, -1.9, -3., -3.5, -3.9, -4.1],
                          [5.5, 3.4, 2.4, 1.4, 0.4, -0.7, -1.7, -2.1, -2.3]]
    elif B == 3:
        D_data = [5., 7.8, 9.9, 12.2, 14.4, 16.7, 19.4, 21.5, 25]
        delta_PNL_data = [[3.8, 1., -0.3, -1.4, -3.2, -5.2, -6.7, -7.5, -8.7],
                          [4.4, 2.2, 1., -0.3, -2., -4.2, -5.9, -6.7, -7.8],
                          [4.8, 2.9, 1.6, 0.6, -0.8, -2.8, -4.4, -5.2, -6.2],
                          [5.4, 3.8, 2.5, 1.4, 0.6, -0.6, -1.9, -2.7, -4.1],
                          [6., 4.4, 3.2, 2.3, 1.9, 1.1, 0.1, -0.7, -2.3],
                          [7.2, 5.9, 4.8, 3.8, 3.3, 2.6, 1.7, 1., -0.2]]
    elif B == 4:
        D_data = [5., 7.5, 10.1, 12.5, 15.1, 17.7, 19.9, 22.2, 25.]
        delta_PNL_data = [[5.1, 3., 1., -0.1, -1.5, -3.3, -4.4, -5., -5.6],
                          [5.5, 3.5, 1.9, 0.8, -0.6, -2.3, -3.3, -3.8, -4.4],
                          [5.9, 4.1, 2.6, 1.5, 0.3, -1., -1.8, -2.4, -3.],
                          [6.7, 4.8, 3.4, 2.6, 1.6, 0.5, -0.1, -0.6, -1.2],
                          [7.3, 5.8, 4.5, 3.5, 2.4, 1.6, 1., 0.5, 0.],
                          [8.1, 7.3, 6.2, 5., 4.2, 3.2, 2.7, 2.3, 2.]]
    else:
        D_data = [5., 7.4, 10., 12.5, 15., 17.4, 19.9, 22.7, 25.]
        delta_PNL_data = [[5.8, 4., 2.7, 2.1, 1.1, -0.2, -1., -1.7, -2.1],
                          [6.3, 4.6, 3.1, 2.7, 1.7, 0.4, -0.4, -1., -1.3],
                          [6.8, 5.2, 3.7, 3.3, 2.3, 1.2, 0.5, -0.1, -0.5],
                          [7.3, 5.6, 4.3, 4., 3.2, 2.1, 1.4, 0.9, 0.5],
                          [8.0, 6.2, 5.0, 4.6, 3.9, 2.9, 2.1, 1.7, 1.4],
                          [9.1, 7.0, 5.8, 5.6, 4.9, 4.0, 3.3, 2.8, 2.6]]

    PNL_interpol = interp2d(D_data, M_tip_data, delta_PNL_data, kind = 'cubic', fill_value=0.)
    delta_PNL = PNL_interpol(D, M_tip)

    PNL = OSPL + delta_PNL
    dB_A = PNL - 14.

    return dB_A