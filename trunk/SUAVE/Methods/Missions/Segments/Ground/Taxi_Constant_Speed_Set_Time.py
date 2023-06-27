## @ingroup Methods-Missions-Segments-Ground
# Taxi_Constant_Speed_Set_Time.py
#
# Created:  Aug 2022, D. Eisenhut
# Modified:

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
import SUAVE
from SUAVE.Core import Units


# ----------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------

## @ingroup Methods-Missions-Segments-Taxi
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Constant taxi speed for a given time

    Source:
    N/A

    Inputs:
    segment.altitude_start                              [meters]
    segment.state.numerics.dimensionless.control_points [Unitless]
    segment.air_speed                                   [meters/second]
    segment.ground_incline                              [radians]
    segment.friction_coefficient                        [Unitless]

    Outputs:
    conditions.frames.inertial.velocity_vector  [meters/second]
    conditions.frames.inertial.position_vector  [meters]
    conditions.ground.incline                   [radians]
    conditions.ground.friction_coefficient      [dimensionless]

    Properties Used:
    N/A
    """

    # unpack
    air_speed = segment.air_speed
    time = segment.time
    alt0 = segment.altitude_start
    ground_incline = segment.ground_incline
    friction_coefficient = segment.friction_coefficient
    t_nondim = segment.state.numerics.dimensionless.control_points

    conditions = segment.state.conditions

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1, 2]

    # pack conditions
    conditions.frames.inertial.time = t_nondim * time
    conditions.frames.inertial.velocity_vector[:, 0] = air_speed
    conditions.ground.incline[:, 0] = ground_incline        # TODO ground incline currently not implemented
    conditions.ground.friction_coefficient[:, 0] = friction_coefficient
    conditions.propulsion.throttle[:, 0] = 0.2      # initial value # TODO check if necessary!

    conditions.frames.inertial.position_vector[:, 2] = -alt0  # z points down


def unpack_throttle(segment):
    """Unpacks and sets the proper value for throttle

        Assumptions:
        N/A

        Source:
        N/A

        Inputs:
        segment.state.unknowns.throttle                [dimensionless]

        Outputs:
        segment.state.conditions.propulsion.throttle   [dimensionless]

        Properties Used:
        N/A
        """

    # unpack unknowns
    throttle = segment.state.unknowns.throttle

    # apply unknowns
    segment.state.conditions.propulsion.throttle[:, 0] = throttle[:, 0]


def solve_residuals(segment):
    total_forces = segment.state.conditions.frames.inertial.total_force_vector

    segment.state.residuals.total_force = np.linalg.norm(total_forces, axis=1)

