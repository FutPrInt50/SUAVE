## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Helper_Functions
# propeller_wing_interaction_delta_lift_drag.py
# 
# Created:  Aug 2022, J. Mangold
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUAVE Imports
import SUAVE
import numpy as np
from SUAVE.Core import Units, Data
import scipy.io as sio
import os
import time

# ----------------------------------------------------------------------
#  Compute delta lift and drag of propeller wing interaction
# ----------------------------------------------------------------------

## @ingroup Methods-Aerodynamics-Common-Fidelity_Zero-Helper_Functions
def propeller_wing_interaction_delta_lift_drag(state,settings,geometry):
    """Compute delta lift and drag of propeller wing interaction

    Definition:
    This function evaluates a polynomial fit of order 5 over 6 variables,
    namely the CL-isolated, CD-isolated, CL-installed, CD-isntalled,
    delta CL and delta CD using 8 input variables.

    Only the delta terms are being given as output.

    Assumptions:
    -

    Source:
    TU Delft tbd

    Inputs:
    Input:
    AoA: 1D array of length N containing the angle of attack values
    AR: 1D array of length N containing the aspect ratio values
    taper: 1D array of length N containing the taper values
    RoverB_mainProp: 1D array of length N containing the main propellers
    radii values (as fraction of the span).
        RoverB_tipProp: 1D array of length N containing the tip propellers
    radii values (as fraction of the span).
        YoverB_mainprop: 1D array of length N containing the position of the
    main propellers (as fraction of the span). Note that the tip propellers are always at Y/B=±0.5.
        TC_mainProp: 1D array of length N containing the TC values of the
    main props
        TC_tipProp: 1D array of length N containing the TC values of the tip
    props


    note:   TC = T/(rho * Vinf^2 * Dprop^2)
    Advance ratio (J) is automatically determined by emprical relation
    J = 0.308*TC^-0.73      based on XPROP experiments

    Outputs:
    deltaCL: Difference in CL between installed and uninstalled case
    deltaCD: Difference in CD between installed and uninstalled case

    Note that response surface model is generated using certain bounds (see below). The model may not be accurate
    when inputs are given outside ofthese bounds. Furthermore, all inputs must be normalized for their repsective range,
    determined by the bounds.

    Properties Used:
    N/A
    """ 
    # ==============================================
	# Unpack
    # ==============================================
    vehicle    = geometry
    conditions = state.conditions

    if vehicle.propulsors.network.propeller_wing_interaction == False:

        deltaCL = np.zeros_like(conditions.freestream.density)
        deltaCD = np.zeros_like(conditions.freestream.density)
        
    elif vehicle.propulsors.network.propeller_wing_interaction == True:

        rho = conditions.freestream.density
        v_inf = conditions.freestream.velocity
        thrust_mainProp = conditions.propulsion.thrust_turboprop[:,[0]]
        thrust_tipProp = conditions.propulsion.thrust_WTP[:,[0]]

        AoA = conditions.aerodynamics.angle_of_attack / Units.deg
        AR = vehicle.wings.main_wing.aspect_ratio * np.ones_like(AoA)
        taper = vehicle.wings.main_wing.taper * np.ones_like(AoA)
        RoverB_mainProp = vehicle.propulsors.network.propeller.tip_radius / vehicle.wings.main_wing.spans.projected * np.ones_like(AoA)
        RoverB_tipProp = vehicle.propulsors.network.propellerWTP.tip_radius / vehicle.wings.main_wing.spans.projected * np.ones_like(AoA)# add wing tip propeller
        YoverB_mainProp = vehicle.propulsors.network.propeller.origin[0][1]   / vehicle.wings.main_wing.spans.projected * np.ones_like(AoA)

        TC_mainProp = thrust_mainProp / (rho * v_inf**2 * (2 * vehicle.propulsors.network.propeller.tip_radius)**2)
        TC_tipProp = thrust_tipProp / (rho * v_inf**2 * (2 * vehicle.propulsors.network.propellerWTP.tip_radius)**2)

        if np.any(TC_tipProp < 0.01) or np.any(TC_mainProp < 0.01):
            deltaCL = np.zeros_like(conditions.freestream.density)
            deltaCD = np.zeros_like(conditions.freestream.density)

        else:



            # inputs = [AoA, AR, taper, RoverB_mainProp, RoverB_tipProp, YoverB_mainProp, TC_mainProp, TC_tipProp]
            # # Check inputs
            # for i in inputs:
            #     clm(i) = iscolumn(eval(inputs{i}))
            #     lng(i) = length(eval(inputs{i}))
            # end
            #
            # if ~all(clm)
            #     error('Please give inputs as column vectors')
            # end
            #
            # if length(unique(lng))~=1
            #     error('Inputs all must have the same length')
            # end


            # for faster run time: now included at vehile_setup
            # file_name = 'coeff5_full.mat'
            # input_file = os.path.join(SUAVE.__path__[0], 'Data_Files\\Aero\\', file_name)
            # coeff5_full = sio.loadmat(input_file)
            # coeff = coeff5_full['coeff']
            #
            # file_name = 'E5_full.mat'
            # input_file = os.path.join(SUAVE.__path__[0], 'Data_Files\\Aero\\', file_name)
            # E5_full = sio.loadmat(input_file)
            # E = E5_full['E']

            #coeff = vehicle.propulsors.network.propeller.coeff
            coeff_new = vehicle.propulsors.network.propeller.coeff_new
            #E = vehicle.propulsors.network.propeller.E


            # coeff: 1D array of length M containing the monomial coefficients,
            # where M equals the number of monomials. For the 5th-order
            # polynomial used and the 8 input variables, M = 1287
            # E: 2D array of size [1287 x 8] containing the exponents of the variables
            # in each monomial.

            # Number of input variables
            K = 8

            # Number of monomials
            M = 1287

            #Bounds used on input variables
            bounds = Data()
            bounds.AoA             = np.array([-5,15])      # geometric angle of attack [degrees]
            bounds.AR              = np.array([8,16])        # wing aspect ratio
            bounds.taper           = np.array([0.4,1])       # wing taper ratio
            bounds.RoverB_mainProp = np.array([0.025,0.1])   # main prop radius relative to full wing span
            bounds.RoverB_tipProp  = np.array([0.025,0.1])   # tip prop radius relative to full wing span
            bounds.YoverB_mainProp = np.array([0.15,0.25])   # main prop lateral position w.r.t. full span
            bounds.TC_mainProp     = np.array([0.01,1.0])    # main prop thrust coefficient relative to rho*V^2*D^2
            bounds.TC_tipProp      = np.array([0.01,1.0])    # distributed prop thrust coefficient relative to rho*V^2*D^2

            inputs = np.array(['AoA','AR','taper','RoverB_mainProp','RoverB_tipProp','YoverB_mainProp','TC_mainProp','TC_tipProp'])

            # input vector
            x = np.c_[AoA,AR,taper,RoverB_mainProp,RoverB_tipProp,YoverB_mainProp,TC_mainProp,TC_tipProp]



            # Scale input variables

            xRange = np.zeros((8))
            xMean = np.zeros((8))

            #X = np.zeros((1,8))
            #X = np.zeros((8, 4))

            for i in np.arange(0,8):
                xRange[i] = getattr(bounds,(inputs[i]))[1] - getattr(bounds,(inputs[i]))[0]
                xMean[i] = np.mean(getattr(bounds,(inputs[i])))
                #X[i] = (x[i] - xMean[i])/xRange[i]

            X = (x - xMean) / xRange


            # #X = (x - xMean) / xRange
            # # Number of data points
            # n = X.shape[0]
            # # Loop over ouput variables
            # Y = np.zeros((n,6))
            #
            # #for i in np.arange(4,6):
            # for i in np.arange(4,6): # for faster run time
            #     # Loop over monomials
            #     mon = np.ones((n,M))
            #     for j in np.arange(0,M):
            #         # Multiply 1 by monomial coefficient
            #         mon[:,j] = mon[:,j]  * coeff[0][i][j][0]
            #         # Multiply monomial by all variables and their corresponding exponent
            #         for k in np.arange(0,K):
            #             #mon[:,j] = np.multiply(mon[:,j],X[:,k] ** E[i][j][k])
            #             mon[:,j] = np.multiply(mon[:,j],X[:,k] ** E[0][i][j][k])
            #         # Add monomials to get function value
            #         Y[:,i] = np.sum(mon, 1)


            from sklearn.preprocessing import PolynomialFeatures
            poly = PolynomialFeatures(degree = 5)

            if (np.isfinite(X).all()) != True:
                #print('Info: Inputs for Propeller Wing Interaction NAN')
                #X = np.ones_like(X)
                X = (np.zeros_like(x) - xMean) / xRange

            aaaa = poly.fit_transform(X)

            # Number of data points
            n = X.shape[0]
            # # Loop over ouput variables
            Y = np.zeros((n,4))

            Y4 = np.matmul(aaaa,coeff_new[:,4])
            Y5 = np.matmul(aaaa, coeff_new[:,5])

            Y = np.c_[Y,Y4,Y5]

            # Also outputs are scaled, so rescale to get real values
            yRange = np.array([1.86712167618919,0.0710608194556417,2.83575841489857,0.100868095878334,1.06673845433005,0.0338795784377658])
            yMean = np.array([0.433016424311606,0.0217860052482334,0.570234330880355,0.0168931963627903,0.137217906568754,-0.00489280888544329])

            y = np.multiply(Y,yRange) + yMean

            # CL_iso = y[:,[0]] # for faster run time
            # CD_iso = y[:,[1]] # for faster run time
            # CL_ins = y[:,[2]] # for faster run time
            # CD_ins = y[:,[3]] # for faster run time
            deltaCL = y[:,[4]]
            deltaCD = y[:,[5]]

            # # The deltas can also be defined by: installed - iso
            # deltaCL = CL_ins - CL_iso
            # deltaCD = CD_ins - CD_iso

    propeller_wing_interaction_delta_lift_drag.deltaCL = deltaCL
    propeller_wing_interaction_delta_lift_drag.deltaCD = deltaCD

    # dump data to state
    state.conditions.aerodynamics.propeller_wing_interaction_delta_lift = deltaCL
    state.conditions.aerodynamics.drag_breakdown.propeller_wing_interaction_delta_drag = deltaCD

    return propeller_wing_interaction_delta_lift_drag