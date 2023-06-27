## @ingroup Components-Energy-Converters
# Propeller_Lo_Fid.py
#
# Created:  Jun 2014, E. Botero
# Modified: Jan 2016, T. MacDonald

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# suave imports
import SUAVE

# package imports
import numpy as np
import scipy.optimize
from SUAVE.Components.Energy.Energy_Component import Energy_Component
from warnings import warn

# ----------------------------------------------------------------------
#  Propeller Class
# ----------------------------------------------------------------------    
## @ingroup Components-Energy-Converters
class Delft_Propeller_Lo_Fid(Energy_Component):
    """This is a low-fidelity propeller component.
    
    Assumptions:
    None

    Source:
    None
    """    
    def __defaults__(self):
        """This sets the default values for the component to function.

        Assumptions:
        None

        Source:
        N/A

        Inputs:
        None

        Outputs:
        None

        Properties Used:
        None
        """
        self.tag                   = 'propeller'
        self.tip_radius            = 0.0
        self.propulsive_efficiency = 0.0
        self.thrust_calibration    = 1.

        
    def spin(self,conditions):
        """Analyzes a propeller given geometry and operating conditions.

        Assumptions:
        Six Blade Propeller

        Source:
        Delft Propeller Performance Maps 2022-2-28

        Inputs:
        self.inputs.omega            [radian/s]
        self.inputs.torque           [Nm]
        conditions.freestream.
          density                    [kg/m^3]
          velocity                   [m/s]
          mach_number                [-]

        Outputs:
        conditions.propulsion.etap   [-]  (propulsive efficiency)
        thrust                       [N]
        Qm                           [Nm] (torque)
        power                        [W]
        Cp                           [-]  (coefficient of power)

        Properties Used:
        self.tip_radius              [m]
        """
           
        # Unpack    
        R     = self.tip_radius
        omega = self.inputs.omega
        Qm    = self.inputs.torque
        rho   = conditions.freestream.density[:,0,None]
        # mu    = conditions.freestream.dynamic_viscosity[:,0,None]
        V     = conditions.freestream.velocity[:,0,None]
        # a     = conditions.freestream.speed_of_sound[:,0,None]
        M     = conditions.freestream.mach_number[:,0,None]
        
        # Get Basic Inputs
        power  = Qm*omega
        n      = omega/(2.*np.pi) 
        D      = 2*R
        Cp     = power / (rho * n ** 3 * D ** 5)
        J      = V / (n * D)

        # CP FOR POSITIVE THRUST REGIME
        def fun1(beta, J, M, Cp):
            res = -0.0010837 * J ** 4 - 0.009782 * J ** 3 * M + 0.10723 * J ** 3 * (beta / 30) \
                - 0.20454 * J ** 3 - 1.2645 * J ** 2 * M ** 2 + 0.23304 * J ** 2 * M * (beta / 30) \
                + 0.52609 * J ** 2 * M - 0.4067 * J ** 2 * (beta / 30) ** 2 + 0.86141 * J ** 2 * (beta / 30) \
                - 0.49031 * J ** 2 + 5.6305 * J * M ** 3 + 3.8971 * J * M ** 2 * (beta / 30) - 5.775 * J * M ** 2 \
                - 0.42956 * J * M * (beta / 30) ** 2 - 2.1867 * J * M * (beta / 30) + 1.6682 * J * M \
                + 0.65723 * J * (beta / 30) ** 3 - 1.0277 * J * (beta / 30) ** 2 + 0.29945 * J * (beta / 30) \
                + 0.041967 * J - 9.0409 * M ** 4 - 11.2652 * M ** 3 * (beta / 30) + 14.7799 * M ** 3 \
                - 0.16207 * M ** 2 * (beta / 30) ** 2 + 4.9335 * M ** 2 * (beta / 30) - 4.8181 * M ** 2 \
                + 0.20232 * M * (beta / 30) ** 3 + 0.64152 * M * (beta / 30) ** 2 + 0.023787 * M * (beta / 30) \
                + 0.063763 * M - 0.39554 * (beta / 30) ** 4 + 0.80622 * (beta / 30) ** 3 \
                - 0.28484 * (beta / 30) ** 2 + 0.32376 * (beta / 30) - 0.064419 - Cp
            return res


        # CP FOR NEGATIVE THRUST REGIME
        def fun2(beta, J, M, Cp):
            res = -0.033274 * J ** 4 + 0.19484 * J ** 3 * M + 0.097143 * J ** 3 * (beta / 30) \
                + 0.32058 * J ** 3 - 1.1806 * J ** 2 * M ** 2 - 1.1119 * J ** 2 * M * (beta / 30) \
                - 0.10236 * J ** 2 * M + 0.0055636 * J ** 2 * (beta / 30) ** 2 - 1.426 * J ** 2 * (beta / 30) \
                - 0.49362 * J ** 2 + 9.1402 * J * M ** 3 + 0.070859 * J * M ** 2 * (beta / 30) - 4.0603 * J * M ** 2 \
                + 3.8901 * J * M * (beta / 30) ** 2 - 0.07318 * J * M * (beta / 30) + 0.676 * J * M \
                + 0.8671 * J * (beta / 30) ** 3 - 1.1438 * J * (beta / 30) ** 2 + 3.4985 * J * (beta / 30) \
                - 0.68421 * J + 102.4464 * M ** 4 - 38.2356 * M ** 3 * (beta / 30) - 118.4115 * M ** 3 \
                + 8.596 * M ** 2 * (beta / 30) ** 2 + 19.2147 * M ** 2 * (beta / 30) + 51.7506 * M ** 2 \
                - 5.6479 * M * (beta / 30) ** 3 + 0.96003 * M * (beta / 30) ** 2 - 5.7829 * M * (beta / 30) \
                - 9.0021 * M - 1.6227 * (beta / 30) ** 4 + 5.1332 * (beta / 30) ** 3 \
                - 5.534 * (beta / 30) ** 2 + 1.7049 * (beta / 30) + 0.67199 - Cp
            return res

        beta = np.zeros_like(omega)
        for x in range(len(beta[:, 0])):
            if power[x] >= 0:
                beta[x, 0] = np.array(scipy.optimize.fsolve(lambda beta: fun1(beta, J[x, 0], M[x, 0], Cp[x, 0]), 20))
                # C_T FOR POSITIVE THRUST REGIME
                Ct = -0.0015551 * J ** 4 - 0.010345 * J ** 3 * M + 0.045256 * J ** 3 * (beta / 30) \
                     - 0.067856 * J ** 3 - 0.38564 * J ** 2 * M ** 2 + 0.080407 * J ** 2 * M * (beta / 30) \
                     + 0.22598 * J ** 2 * M - 0.19903 * J ** 2 * (beta / 30) ** 2 + 0.39981 * J ** 2 * (beta / 30) \
                     - 0.1853 * J ** 2 + 1.4779 * J * M ** 3 + 1.1378 * J * M ** 2 * (beta / 30) - 1.3938 * J * M ** 2 \
                     - 0.23198 * J * M * (beta / 30) ** 2 - 0.2635 * J * M * (beta / 30) - 0.27721 * J * M \
                     + 0.20485 * J * (beta / 30) ** 3 - 0.17388 * J * (beta / 30) ** 2 - 0.14799 * J * (beta / 30) \
                     - 0.21604 * J - 2.3756 * M ** 4 - 2.4553 * M ** 3 * (beta / 30) + 2.8615 * M ** 3 \
                     - 0.42279 * M ** 2 * (beta / 30) ** 2 + 1.6533 * M ** 2 * (beta / 30) - 1.0408 * M ** 2 \
                     + 0.12198 * M * (beta / 30) ** 3 - 0.013931 * M * (beta / 30) ** 2 + 0.39199 * M * (beta / 30) \
                     + 0.20179 * M - 0.091186 * (beta / 30) ** 4 + 0.048423 * (beta / 30) ** 3 \
                     + 0.094583 * (beta / 30) ** 2 + 0.49321 * (beta / 30) - 0.0062199
                Ct = self.thrust_calibration * Ct
                etap = J * Ct / Cp
            else:
                beta[x, 0] = np.array(scipy.optimize.fsolve(lambda beta: fun2(beta, J[x, 0], M[x, 0], Cp[x, 0]), 40))
                # C_T FOR NEGATIVE THRUST REGIME
                Ct = -0.031755 * J ** 4 + 0.050567 * J ** 3 * M + 0.29731 * J ** 3 * (beta / 30) \
                     + 0.023092 * J ** 3 + 0.029649 * J ** 2 * M ** 2 - 0.38605 * J ** 2 * M * (beta / 30) \
                     - 0.040638 * J ** 2 * M - 1.1387 * J ** 2 * (beta / 30) ** 2 + 0.031057 * J ** 2 * (beta / 30) \
                     - 0.059655 * J ** 2 + 1.3302 * J * M ** 3 - 1.0239 * J * M ** 2 * (beta / 30) - 0.65024 * J * M ** 2 \
                     + 1.1757 * J * M * (beta / 30) ** 2 + 0.61876 * J * M * (beta / 30) - 0.49132 * J * M \
                     + 2.229 * J * (beta / 30) ** 3 - 0.90987 * J * (beta / 30) ** 2 + 0.48498 * J * (beta / 30) \
                     - 0.54501 * J + 9.7644 * M ** 4 - 6.3248 * M ** 3 * (beta / 30) - 8.0181 * M ** 3 \
                     + 3.5318 * M ** 2 * (beta / 30) ** 2 + 0.91674 * M ** 2 * (beta / 30) + 3.5247 * M ** 2 \
                     - 1.6305 * M * (beta / 30) ** 3 - 0.35254 * M * (beta / 30) ** 2 + 0.18966 * M * (beta / 30) \
                     - 0.38696 * M - 1.9205 * (beta / 30) ** 4 + 2.3387 * (beta / 30) ** 3 \
                     - 1.7351 * (beta / 30) ** 2 + 1.1539 * (beta / 30) + 0.048624
                etap = J * Cp / Ct

        thrust = Ct * rho * n**2 * D**4

        conditions.propulsion['beta_' + self.tag] = beta
        conditions.propulsion['eta_' + self.tag] = etap * np.ones_like(omega)
        conditions.propulsion['cp_' + self.tag] = Cp * np.ones_like(omega)
        conditions.propulsion['ct_' + self.tag] = Ct * np.ones_like(omega)
        conditions.propulsion['j_' + self.tag] = J * np.ones_like(omega)
        conditions.propulsion['rpm_' + self.tag] = n * 60 * np.ones_like(omega)
        conditions.propulsion['thrust_' + self.tag] = thrust * np.ones_like(omega)

        return thrust, Qm, power, Cp
