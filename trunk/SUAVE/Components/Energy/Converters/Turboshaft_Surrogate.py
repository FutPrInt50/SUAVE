## @ingroup Components-Energy-Networks
# Turboshaft_Surrogate.py
#
# Created:  Mar 2017, E. Botero
# Modified: Jan 2020, T. MacDonald
#           May 2021, E. Botero
#           Sep 2021, J. Mangold
#           Jul 2022, D. Eisenhut

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
from copy import deepcopy
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from SUAVE.Methods.Utilities.Cubic_Spline_Blender import Cubic_Spline_Blender

from SUAVE.Core import Data, Units
import sklearn
from sklearn import gaussian_process
from sklearn.gaussian_process.kernels import RationalQuadratic, ConstantKernel, RBF, Matern
from sklearn import neighbors
from sklearn import svm, linear_model
from scipy.interpolate import LinearNDInterpolator

import pickle

# ----------------------------------------------------------------------
#  Network
# ----------------------------------------------------------------------

## @ingroup Components-Energy-Networks
class Turboshaft_Surrogate(Energy_Component):
    """ This is a way for you to load engine data from a source.
        A .csv file is read in, a surrogate made, that surrogate is used during the mission analysis.
        
        You need to use build surrogate first when setting up the vehicle to make this work.
        
        Assumptions:
        The input format for this should be Altitude, Mach, Throttle, Power, SFC(power-related)
        
        Source:
        None
    """        
    def __defaults__(self):
        """ This sets the default values for the network to function
        
            Assumptions:
            None
            
            Source:
            N/A
            
            Inputs:
            None
            
            Outputs:
            None
            
            Properties Used:
            N/A
        """          
        self.nacelle_diameter         = None
        self.engine_length            = None
        self.number_of_engines        = None
        self.tag                      = 'Engine_Deck_Surrogate'
        self.input_file               = None
        self.sfc_surrogate            = None
        self.thrust_surrogate         = None
        self.thrust_angle             = 0.0
        self.areas                    = Data()
        self.surrogate_type           = 'gaussian'
        self.altitude_input_scale     = 1.
        self.power_input_scale        = 1.
        self.power_anchor_scale        = 1.
        self.sfc_anchor               = None
        self.sfc_anchor_scale         = 1.
        self.sfc_anchor_conditions    = np.array([[1.,1.,1.]])
        self.thrust_anchor            = None
        self.thrust_anchor_scale      = 1.
        self.thrust_anchor_conditions = np.array([[1.,1.,1.]])
        self.sfc_rubber_scale         = 1.
        self.use_extended_surrogate   = False
        self.sealevel_static_thrust   = 0.0
        self.negative_throttle_values = False

        # new ones
        self.off_takes                = 0.0
        self.residual_thrust_surrogate  = None
        self.residual_thrust_input_scale = 1.
        self.sfc_scaling_factor          = 1.
        self.power_scaling_factor        = 1.
        self.power_sea_level_static      = 0.0
        self.sea_level_power             = 0
        self.p3t3_method                 = False
        self.deltaisa_method             = False
        self.interpolate_nominal_power   = False

   
    # manage process with a driver function
    def power(self,conditions):
        """ Calculate thrust given the current state of the vehicle
        
            Assumptions:
            None
            
            Source:
            N/A
            
            Inputs:
            state [state()]
            
            Outputs:
            results.thrust_force_vector [newtons]
            results.vehicle_mass_rate   [kg/s]
            
            Properties Used:
            Defaulted values
        """
        
        # Unpack the surrogate
        sfc_surrogate = self.sfc_surrogate
        power_surrogate = self.power_surrogate
        residual_thrust_surrogate = self.residual_thrust_surrogate

        if self.p3t3_method == True:
            p3_surrogate = self.p3_surrogate
            t3_surrogate = self.t3_surrogate
            far_surrogate = self.far_surrogate
        
        # Unpack the conditions
        #conditions = state.conditions
        # rescale altitude for proper surrogate performance
        altitude   = conditions.freestream.altitude/self.altitude_input_scale
        mach       = conditions.freestream.mach_number
        throttle   = conditions.propulsion.throttle
        if self.deltaisa_method:
            deltaisa   = conditions.freestream.delta_ISA/self.deltaisa_input_scale

        if self.interpolate_nominal_power:
            nominal_power = np.ones((len(altitude), 1)) * self.sea_level_power / self.nominal_power_input_scale

        power_factor = 1

        if self.surrogate_type == 'LinearNDI':
            if any(altitude < self.altitude_min_max[0]) or any(altitude > self.altitude_min_max[1]):
                #print(f'replacing altitude out of bounds\n{altitude}')
                altitude[altitude < self.altitude_min_max[0]] = self.altitude_min_max[0]
                altitude[altitude > self.altitude_min_max[1]] = self.altitude_min_max[1]

            if any(mach < self.mach_min_max[0]) or any(mach > self.mach_min_max[1]):
                #print(f'replacing mach out of bounds\n{mach}')
                mach[mach < self.mach_min_max[0]] = self.mach_min_max[0]
                mach[mach > self.mach_min_max[1]] = self.mach_min_max[1]

            if any(throttle < self.throttle_min_max[0]) or any(throttle > self.throttle_min_max[1]):
                #print(f'replacing throttle out of bounds\n{throttle}')
                throttle[throttle < self.throttle_min_max[0]] = self.throttle_min_max[0]
                throttle[throttle > self.throttle_min_max[1]] = self.throttle_min_max[1]

            if self.deltaisa_method:
                if any(deltaisa < self.deltaisa_min_max[0]) or any(deltaisa > self.deltaisa_min_max[1]):
                    #print(f'replacing deltaisa out of bounds\n{deltaisa}')
                    deltaisa[deltaisa < self.deltaisa_min_max[0]] = self.deltaisa_min_max[0]
                    deltaisa[deltaisa > self.deltaisa_min_max[1]] = self.deltaisa_min_max[1]


            if self.interpolate_nominal_power:
                if any(nominal_power < self.nominal_power_min_max[0]) or any(nominal_power > self.nominal_power_min_max[1]):
                    nominal_power[nominal_power < self.nominal_power_min_max[0]] = self.nominal_power_min_max[0]
                    nominal_power[nominal_power > self.nominal_power_min_max[1]] = self.nominal_power_min_max[1]

                    power_factor = self.sea_level_power / self.nominal_power_input_scale


        if self.interpolate_nominal_power:
            cond = np.hstack([altitude, mach, throttle, deltaisa, nominal_power])
        else:
            if self.deltaisa_method:
                cond = np.hstack([altitude, mach, throttle, deltaisa])
            else:
                cond = np.hstack([altitude, mach, throttle])



        if self.surrogate_type == 'LinearNDI':
            sfc = sfc_surrogate(cond)

            # if np.max(conditions.propulsion.throttle) > 1.0:
            power = power_surrogate(cond) * power_factor  #* conditions.propulsion.throttle
            power[conditions.propulsion.throttle > self.throttle_min_max[1]] = (power * conditions.propulsion.throttle)[0]
            # else:
            #     power = power_surrogate(cond)

            residual_thrust = residual_thrust_surrogate(cond)

            if self.p3t3_method == True:
                p3 = p3_surrogate(cond)
                t3 = t3_surrogate(cond)
                far = far_surrogate(cond)


        elif self.use_extended_surrogate == True:
            lo_blender = Cubic_Spline_Blender(0, .01)
            hi_blender = Cubic_Spline_Blender(0.99, 1)            
            sfc = self.extended_sfc_surrogate(sfc_surrogate, cond, lo_blender, hi_blender)
            power = self.extended_thrust_surrogate(power_surrogate, cond, lo_blender, hi_blender)
            residual_thrust = self.extended_sfc_surrogate(residual_thrust_surrogate, cond, lo_blender, hi_blender)
            if self.p3t3_method == True:
                p3 = self.extended_sfc_surrogate(p3_surrogate, cond, lo_blender, hi_blender)
                t3 = self.extended_sfc_surrogate(t3_surrogate, cond, lo_blender, hi_blender)
                far = self.extended_sfc_surrogate(far_surrogate, cond, lo_blender, hi_blender)

        else:
            sfc = sfc_surrogate.predict(cond)
            power = power_surrogate.predict(cond)
            residual_thrust = residual_thrust_surrogate.predict(cond)
            if self.p3t3_method == True:
                p3 = p3_surrogate.predict(cond)
                t3 = t3_surrogate.predict(cond)
                far = far_surrogate.predict(cond)

        sfc = sfc*self.sfc_input_scale*self.sfc_anchor_scale  * 1e-9
        power = power * self.power_input_scale * self.power_anchor_scale
        mdot_one_engine = power * sfc

        power_off_takes = self.sea_level_power * self.off_takes
        power -= power_off_takes

        residual_thrust_one_engine = residual_thrust*self.residual_thrust_input_scale
        if self.p3t3_method == True:
            p3 = p3 * self.p3_input_scale
            t3 = t3 * self.t3_input_scale
            far = far * self.far_input_scale

        ################################################################################################################
        # store to outputs

        self.outputs.power = power
        self.outputs.power_specific_fuel_consumption = sfc
        self.outputs.mdot_one_engine = mdot_one_engine
        self.outputs.residual_thrust_one_engine = residual_thrust_one_engine
        if self.p3t3_method == True:
            self.outputs.p3 = p3
            self.outputs.t3 = t3
            self.outputs.far = far

        return self.outputs
    
    def build_surrogate(self):
        """ Build a surrogate. Multiple options for models are available including:
            -Gaussian Processes
            -KNN
            -SVR
            
            Assumptions:
            None
            
            Source:
            N/A
            
            Inputs:
            state [state()]
            
            Outputs:
            self.sfc_surrogate    [fun()]
            self.thrust_surrogate [fun()] #old
            self.power_surrogate [fun()] #new

            
            Properties Used:
            Defaulted values
        """          
       
        # file name to look for

        if self.gas_turbine_use_pickle:
            if self.interpolate_nominal_power:
                loaded_file = pickle.load(open(self.input_file, 'rb'))

                self.altitude_input_scale = loaded_file.altitude_input_scale
                self.nominal_power_input_scale = loaded_file.nominal_power_input_scale
                self.power_input_scale = loaded_file.power_input_scale
                self.sfc_input_scale = loaded_file.sfc_input_scale
                self.residual_thrust_input_scale = loaded_file.residual_thrust_input_scale
                self.deltaisa_input_scale = loaded_file.deltaisa_input_scale
                self.p3_input_scale = loaded_file.p3_input_scale
                self.t3_input_scale = loaded_file.t3_input_scale
                self.far_input_scale = loaded_file.far_input_scale
                self.altitude_min_max = loaded_file.altitude_min_max
                self.mach_min_max = loaded_file.mach_min_max
                self.throttle_min_max = loaded_file.throttle_min_max
                self.deltaisa_min_max = loaded_file.deltaisa_min_max
                self.nominal_power_min_max = loaded_file.nominal_power_min_max
                self.sfc_surrogate = loaded_file.sfc_surrogate
                self.power_surrogate = loaded_file.power_surrogate
                self.residual_thrust_surrogate = loaded_file.residual_thrust_surrogate
                self.p3_surrogate = loaded_file.p3_surrogate
                self.t3_surrogate = loaded_file.t3_surrogate
                self.far_surrogate = loaded_file.far_surrogate
            else:
                raise NotImplementedError('Using pickle for turboshaft surrogate currently only supported for interpolate nominal power!')

        else:
            file_name = self.input_file

            # Load the CSV file
            #my_data = np.genfromtxt(file_name, delimiter=',')
            my_data = np.genfromtxt(file_name, delimiter=';')

            # Remove the header line
            my_data = np.delete(my_data,np.s_[0],axis=0)

            # Clean up to remove redundant lines
            b = np.ascontiguousarray(my_data).view(np.dtype((np.void, my_data.dtype.itemsize * my_data.shape[1])))
            _, idx = np.unique(b, return_index=True)

            my_data = my_data[idx]

            xy  = my_data[:,:3] # Altitude, Mach, Throttle

            power = np.transpose(np.atleast_2d(my_data[:, 3]))  #Power
            sfc = np.transpose(np.atleast_2d(my_data[:, 4])) # SFC
            residual_thrust = np.transpose(np.atleast_2d(my_data[:, 15]))#/2.39  # Residual Thrust

            if self.p3t3_method == True:
                p3 = np.transpose(np.atleast_2d(my_data[:, 12]))   # P3
                t3 = np.transpose(np.atleast_2d(my_data[:, 13]))   # T3
                far = np.transpose(np.atleast_2d(my_data[:, 14]))  # fuel to air ratio

            if self.interpolate_nominal_power:
                delta_isa = np.transpose(np.atleast_2d(my_data[:, 11]))
                xy = np.c_[xy, delta_isa]
                nominal_power = np.transpose(np.atleast_2d(my_data[:, 16]))
                xy = np.c_[xy, nominal_power]

            else:

                if self.deltaisa_method:
                    delta_isa = np.transpose(np.atleast_2d(my_data[:, 11]))
                    xy = np.c_[xy, delta_isa]
                    sea_level_static_row_index = np.where(np.all(xy == [0, 0, 1, 0], axis=1))
                    self.power_sea_level_static = power[sea_level_static_row_index]
                    self.power_loading_scaling = self.sea_level_power / self.power_sea_level_static[0][0]

                else:
                    sea_level_static_row_index = np.where(np.all(xy == [0, 0, 1], axis=1))
                    self.power_sea_level_static = power[sea_level_static_row_index]
                    self.power_loading_scaling = self.sea_level_power / self.power_sea_level_static[0][0]


            self.altitude_input_scale = np.max(xy[:,0])

            if self.interpolate_nominal_power:
                self.nominal_power_input_scale = np.max(xy[:,4])
                self.deltaisa_input_scale = np.max(np.array([np.max(xy[:, 3]), np.abs(np.min(xy[:, 3]))]))
            else:
                power = power * self.power_loading_scaling
                if self.deltaisa_method:
                    self.deltaisa_input_scale = np.max(np.array([np.max(xy[:, 3]), np.abs(np.min(xy[:, 3]))]))


            sfc = sfc * self.sfc_scaling_factor

            self.power_input_scale = np.max(power)
            self.sfc_input_scale      = np.max(sfc)
            self.residual_thrust_input_scale = np.max(residual_thrust)

            if self.p3t3_method == True:
                self.p3_input_scale = np.max(p3)
                self.t3_input_scale = np.max(t3)
                self.far_input_scale = np.max(far)

            # normalize for better surrogate performance
            xy[:,0] /= self.altitude_input_scale
            #thr     /= self.thrust_input_scale
            power /= self.power_input_scale
            sfc     /= self.sfc_input_scale
            residual_thrust /= self.residual_thrust_input_scale

            if self.interpolate_nominal_power:
                xy[:,4] /= self.nominal_power_input_scale
                xy[:,3] /= self.deltaisa_input_scale
            else:
                if self.deltaisa_method:
                    xy[:,3] /= self.deltaisa_input_scale

            if self.p3t3_method == True:
                p3 /= self.p3_input_scale
                t3 /= self.t3_input_scale
                far /= self.far_input_scale


            # Pick the type of process
            if self.surrogate_type  == 'gaussian':
                gp_kernel = Matern()
                regr_sfc = gaussian_process.GaussianProcessRegressor(kernel=gp_kernel)
                regr_power = gaussian_process.GaussianProcessRegressor(kernel=gp_kernel)
                regr_residual_thrust = gaussian_process.GaussianProcessRegressor(kernel=gp_kernel)
                power_surrogate = regr_power.fit(xy, power)
                sfc_surrogate = regr_sfc.fit(xy, sfc)
                residual_thrust_surrogate = regr_residual_thrust.fit(xy, residual_thrust)

                if self.p3t3_method == True:
                    regr_p3 = gaussian_process.GaussianProcessRegressor(kernel=gp_kernel)
                    regr_t3 = gaussian_process.GaussianProcessRegressor(kernel=gp_kernel)
                    regr_far = gaussian_process.GaussianProcessRegressor(kernel=gp_kernel)
                    p3_surrogate = regr_p3.fit(xy, p3)
                    t3_surrogate = regr_t3.fit(xy, t3)
                    far_surrogate = regr_far.fit(xy, far)

            elif self.surrogate_type  == 'knn':
                regr_sfc = neighbors.KNeighborsRegressor(n_neighbors=1,weights='distance')
                #regr_thr = neighbors.KNeighborsRegressor(n_neighbors=1,weights='distance')
                regr_power = neighbors.KNeighborsRegressor(n_neighbors=1, weights='distance')
                sfc_surrogate = regr_sfc.fit(xy, sfc)
                #thr_surrogate = regr_thr.fit(xy, thr)
                power_surrogate = regr_power.fit(xy, power)

            elif self.surrogate_type  == 'svr':
                regr_power = svm.SVR(C=500.)
                regr_sfc = svm.SVR(C=500.)
                sfc_surrogate  = regr_sfc.fit(xy, sfc)
                power_surrogate = regr_power.fit(xy, power)

            elif self.surrogate_type == 'linear':
                regr_power = linear_model.LinearRegression()
                regr_sfc = linear_model.LinearRegression()
                regr_residual_thrust = linear_model.LinearRegression()
                sfc_surrogate  = regr_sfc.fit(xy, sfc)
                power_surrogate = regr_power.fit(xy, power)
                residual_thrust_surrogate = regr_residual_thrust.fit(xy, residual_thrust)

                if self.p3t3_method == True:
                    regr_p3 = linear_model.LinearRegression()
                    regr_t3 = linear_model.LinearRegression()
                    regr_far = linear_model.LinearRegression()
                    p3_surrogate = regr_p3.fit(xy, p3)
                    t3_surrogate = regr_t3.fit(xy, t3)
                    far_surrogate = regr_far.fit(xy, far)

            elif self.surrogate_type == 'LinearNDI':
                power_surrogate = LinearNDInterpolator(xy, power)
                sfc_surrogate = LinearNDInterpolator(xy, sfc)
                residual_thrust_surrogate = LinearNDInterpolator(xy, residual_thrust)
                if self.p3t3_method == True:
                    p3_surrogate = LinearNDInterpolator(xy, p3)
                    t3_surrogate = LinearNDInterpolator(xy, t3)
                    far_surrogate = LinearNDInterpolator(xy, far)

                self.altitude_min_max = np.array([np.amin(xy[:, 0]), np.amax(xy[:, 0])])
                self.mach_min_max     = np.array([np.amin(xy[:, 1]), np.amax(xy[:, 1])])
                self.throttle_min_max = np.array([np.amin(xy[:, 2]), np.amax(xy[:, 2])])

                if self.interpolate_nominal_power:
                    self.nominal_power_min_max = np.array([np.amin(xy[:, 4]), np.amax(xy[:, 4])])
                    self.deltaisa_min_max = np.array([np.amin(xy[:, 3]), np.amax(xy[:, 3])])
                else:
                    if self.deltaisa_method:
                        self.deltaisa_min_max = np.array([np.amin(xy[:, 3]), np.amax(xy[:, 3])])

            else:
                raise NotImplementedError('Selected surrogate method has not been implemented')


            if self.thrust_anchor is not None:
                cons = deepcopy(self.thrust_anchor_conditions)
                cons[0,0] /= self.altitude_input_scale
                base_thrust_at_anchor = thr_surrogate.predict(cons)
                self.thrust_anchor_scale = self.thrust_anchor/(base_thrust_at_anchor*self.power_input_scale)

            if self.sfc_anchor is not None:
                cons = deepcopy(self.sfc_anchor_conditions)
                cons[0,0] /= self.altitude_input_scale
                base_sfc_at_anchor = sfc_surrogate.predict(cons)
                self.sfc_anchor_scale = self.sfc_anchor/(base_sfc_at_anchor*self.sfc_input_scale)

            # Save the output
            self.sfc_surrogate    = sfc_surrogate
            self.power_surrogate = power_surrogate
            self.residual_thrust_surrogate = residual_thrust_surrogate
            if self.p3t3_method == True:
                self.p3_surrogate = p3_surrogate
                self.t3_surrogate = t3_surrogate
                self.far_surrogate = far_surrogate

    def extended_thrust_surrogate(self, thr_surrogate, cond, lo_blender, hi_blender):
        """ Fixes thrust values outside of the standard throttle range in order to provide
            reasonable values outside of the typical surrogate coverage area. 
            
            Assumptions:
            None
            
            Source:
            N/A
            
            Inputs:
            thr_surrogate     - Trained sklearn surrogate that outputs a scaled thrust value
            cond              - nx3 numpy array with input conditions for the surrogate
            lo_blender        - Cubic spline blending class that is used at the low throttle cutoff
            hi_blender        - Cubic spline blending class that is used at the high throttle cutoff
            
            Outputs:
            T                 [nondim]
            
            Properties Used:
            None
        """            
        # initialize
        cond_zero_eta      = deepcopy(cond)
        cond_one_eta       = deepcopy(cond)
        cond_zero_eta[:,2] = 0
        cond_one_eta[:,2]  = 1
        
        min_thrs = thr_surrogate.predict(cond_zero_eta)
        max_thrs = thr_surrogate.predict(cond_one_eta)
        dTdetas  = max_thrs - min_thrs
        
        etas          = cond[:,2]
        mask_low      = etas < 0
        mask_lo_blend = np.logical_and(etas >= 0, etas < 0.01)
        mask_mid      = np.logical_and(etas >= 0.01, etas < 0.99)
        mask_hi_blend = np.logical_and(etas >= 0.99, etas < 1)
        mask_high     = etas >= 1
        
        etas = np.atleast_2d(etas).T
        T = np.zeros_like(etas)
        
        # compute thrust
        T[mask_low] = min_thrs[mask_low] + etas[mask_low]*dTdetas[mask_low]
        
        if np.sum(mask_lo_blend) > 0:
            lo_weight = lo_blender.compute(etas[mask_lo_blend])
            T[mask_lo_blend] = (min_thrs[mask_lo_blend] + etas[mask_lo_blend]*dTdetas[mask_lo_blend])*lo_weight + \
                               thr_surrogate.predict(cond[mask_lo_blend])*(1-lo_weight)
        
        if np.sum(mask_mid) > 0:
            T[mask_mid] = thr_surrogate.predict(cond[mask_mid])
        
        if np.sum(mask_hi_blend) > 0:
            hi_weight = hi_blender.compute(etas[mask_hi_blend])
            T[mask_hi_blend] = thr_surrogate.predict(cond[mask_hi_blend])*hi_weight + \
                               (max_thrs[mask_hi_blend] + (etas[mask_hi_blend]-1)*dTdetas[mask_hi_blend])*(1-hi_weight)
        
        T[mask_high] = max_thrs[mask_high] + (etas[mask_high]-1)*dTdetas[mask_high]
        
        return T
    
    def extended_sfc_surrogate(self, sfc_surrogate, cond, lo_blender, hi_blender):
        """ Fixes sfc values outside of the standard throttle range in order to provide
            reasonable values outside of the typical surrogate coverage area. 
            
            Assumptions:
            None
            
            Source:
            N/A
            
            Inputs:
            sfc_surrogate     - Trained sklearn surrogate that outputs a scaled sfc value
            cond              - nx3 numpy array with input conditions for the surrogate
            lo_blender        - Cubic spline blending class that is used at the low throttle cutoff
            hi_blender        - Cubic spline blending class that is used at the high throttle cutoff
            
            Outputs:
            sfcs              [nondim]
            
            Properties Used:
            None
        """           
        # initialize
        cond_zero_eta      = deepcopy(cond)
        cond_one_eta       = deepcopy(cond)
        cond_zero_eta[:,2] = 0
        cond_one_eta[:,2]  = 1  
        
        etas          = cond[:,2]
        mask_low      = etas < 0
        mask_lo_blend = np.logical_and(etas >= 0, etas < 0.01)
        mask_mid      = np.logical_and(etas >= 0.01, etas < 0.99)
        mask_hi_blend = np.logical_and(etas >= 0.99, etas < 1)
        mask_high     = etas >= 1 
        
        etas = np.atleast_2d(etas).T
        sfcs = np.zeros_like(etas)
        
        # compute sfc
        if np.sum(mask_low) > 0:
            sfcs[mask_low] = sfc_surrogate.predict(cond_zero_eta[mask_low])
        
        if np.sum(mask_lo_blend) > 0:
            lo_weight = lo_blender.compute(etas[mask_lo_blend])
            sfcs[mask_lo_blend] = sfc_surrogate.predict(cond_zero_eta[mask_lo_blend])*lo_weight + \
                               sfc_surrogate.predict(cond[mask_lo_blend])*(1-lo_weight)
        
        if np.sum(mask_mid) > 0:
            sfcs[mask_mid] = sfc_surrogate.predict(cond[mask_mid])
        
        if np.sum(mask_hi_blend) > 0:
            hi_weight = hi_blender.compute(etas[mask_hi_blend])
            sfcs[mask_hi_blend] = sfc_surrogate.predict(cond[mask_hi_blend])*hi_weight + \
                               sfc_surrogate.predict(cond_one_eta[mask_hi_blend])*(1-hi_weight)
        
        if np.sum(mask_high) > 0:
            sfcs[mask_high] = sfc_surrogate.predict(cond_one_eta[mask_high])
            
        return sfcs