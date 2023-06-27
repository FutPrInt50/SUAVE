## @ingroup Analyses-Mission-Segments-Taxi
# Taxi_Constant_Speed_Set_Time.py
#
# Created:  Aug 2022, D. Eisenhut
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

# SUAVE imports
from SUAVE.Analyses.Mission.Segments import Aerodynamic
from SUAVE.Analyses.Mission.Segments import Conditions

from SUAVE.Methods.Missions import Segments as Methods
from SUAVE.Methods.skip import skip

from SUAVE.Analyses import Process

# Units
from SUAVE.Core import Units, Data


# ----------------------------------------------------------------------
#  Segment
# ----------------------------------------------------------------------

## @ingroup Analyses-Mission-Segments-Taxi
class Taxi_Constant_Speed_Set_Time(Aerodynamic):
    """ Taxi at constant velocity for a given time.

        Assumptions:


        Source:
        None
    """

    def __defaults__(self):
        """ This sets the default solver flow. Anything in here can be modified after initializing a segment.

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

        # --------------------------------------------------------------
        #   User inputs
        # --------------------------------------------------------------

        self.altitude_start = None  # Optional
        self.air_speed = 10 * Units.m / Units.s
        self.ground_incline = 0 * Units.degree          # TODO currently not used
        self.friction_coefficient = 0.04
        self.time = 100 * Units.seconds

        self.true_course = 0.0 * Units.degrees

        # --------------------------------------------------------------
        #   State
        # --------------------------------------------------------------

        # conditions
        self.state.conditions.update(Conditions.Aerodynamics())

        # initials and unknowns
        ones_row = self.state.ones_row

        self.state.unknowns.throttle     = ones_row(1) * 0.5
        self.state.residuals.total_force = ones_row(1) * 0.0

        # Specific ground things
        self.state.conditions.ground = Data()
        self.state.conditions.ground.incline = ones_row(1) * 0.0
        self.state.conditions.ground.friction_coefficient = ones_row(1) * 0.0
        self.state.conditions.frames.inertial.ground_force_vector = ones_row(3) * 0.0

        # --------------------------------------------------------------
        #   The Solving Process
        # --------------------------------------------------------------

        # --------------------------------------------------------------
        #   Initialize - before iteration
        # --------------------------------------------------------------
        initialize = self.process.initialize

        initialize.expand_state  = Methods.expand_state
        initialize.differentials = Methods.Common.Numerics.initialize_differentials_dimensionless
        initialize.conditions    = Methods.Ground.Taxi_Constant_Speed_Set_Time.initialize_conditions


        # --------------------------------------------------------------
        #   Converge - starts iteration
        # --------------------------------------------------------------
        converge = self.process.converge

        converge.converge_root = Methods.converge_root

        # --------------------------------------------------------------
        #   Iterate - this is iterated
        # --------------------------------------------------------------
        iterate = self.process.iterate

        # Update Initials
        iterate.initials = Process()
        iterate.initials.time = Methods.Common.Frames.initialize_time
        iterate.initials.weights = Methods.Common.Weights.initialize_weights
        iterate.initials.inertial_position = Methods.Common.Frames.initialize_inertial_position
        iterate.initials.planet_position = Methods.Common.Frames.initialize_planet_position

        # Unpack Unknowns
        iterate.unknowns = Process()
        iterate.unknowns.mission = Methods.Ground.Taxi_Constant_Speed_Set_Time.unpack_throttle

        # Update Conditions
        iterate.conditions = Process()
        iterate.conditions.differentials = Methods.Common.Numerics.update_differentials_time
        iterate.conditions.altitude = Methods.Common.Aerodynamics.update_altitude
        iterate.conditions.atmosphere = Methods.Common.Aerodynamics.update_atmosphere
        iterate.conditions.gravity = Methods.Common.Weights.update_gravity
        iterate.conditions.freestream = Methods.Common.Aerodynamics.update_freestream
        iterate.conditions.orientations = Methods.Common.Frames.update_orientations
        iterate.conditions.propulsion = Methods.Common.Energy.update_thrust
        iterate.conditions.aerodynamics = Methods.Common.Aerodynamics.update_aerodynamics
        iterate.conditions.stability = Methods.Common.Aerodynamics.update_stability
        iterate.conditions.weights = Methods.Common.Weights.update_weights
        iterate.conditions.forces_ground = Methods.Ground.Common.compute_ground_forces
        iterate.conditions.forces = Methods.Ground.Common.compute_forces
        iterate.conditions.planet_position = Methods.Common.Frames.update_planet_position

        # Solve Residuals
        iterate.residuals = Process()
        iterate.residuals.total_forces = Methods.Ground.Taxi_Constant_Speed_Set_Time.solve_residuals

        # --------------------------------------------------------------
        #   Finalize - after iteration
        # --------------------------------------------------------------
        finalize = self.process.finalize

        # Post Processing
        finalize.post_process = Process()
        finalize.post_process.inertial_position = Methods.Common.Frames.integrate_inertial_horizontal_position
        finalize.post_process.stability = Methods.Common.Aerodynamics.update_stability

        return
