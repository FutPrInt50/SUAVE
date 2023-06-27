## @ingroup Methods-Missions-Segments-Climb
# Constant_Throttle_Constant_CAS.py
#
# Created:  Jan 2022, D. Eisenhut
# Modified: Jul 2022, D. Eisenhut

# ----------------------------------------------------------------------
#  Imports
# ----------------------------------------------------------------------

import numpy as np
import SUAVE
from SUAVE.Core import Units


# ----------------------------------------------------------------------
#  Unpack Unknowns
# ----------------------------------------------------------------------

## @ingroup Methods-Missions-Segments-Climb
def unpack_body_angle(segment):
    """Unpacks and sets the proper value for body angle

    Assumptions:
    N/A

    Source:
    N/A

    Inputs:
    state.unknowns.body_angle                      [Radians]

    Outputs:
    state.conditions.frames.body.inertial_rotation [Radians]

    Properties Used:
    N/A
    """

    # unpack unknowns
    theta = segment.state.unknowns.body_angle

    # apply unknowns
    segment.state.conditions.frames.body.inertial_rotations[:, 1] = theta[:, 0]


# ----------------------------------------------------------------------
#  Initialize Conditions
# ----------------------------------------------------------------------

## @ingroup Methods-Missions-Segments-Climb
def initialize_conditions(segment):
    """Sets the specified conditions which are given for the segment type.

    Assumptions:
    Constant throttle estting, with a constant calibrated airspeed

    Source:
    N/A

    Inputs:
    segment.calibrated_air_speed                        [meters/second]
    segment.throttle                                    [Unitless]
    segment.altitude_start                              [meters]
    segment.altitude_end                                [meters]
    segment.state.numerics.dimensionless.control_points [Unitless]
    conditions.freestream.density                       [kilograms/meter^3]

    Outputs:
    conditions.frames.inertial.velocity_vector  [meters/second]
    conditions.propulsion.throttle              [Unitless]

    Properties Used:
    N/A
    """

    # unpack
    throttle = segment.throttle
    cas = segment.calibrated_air_speed
    alt0 = segment.altitude_start
    conditions = segment.state.conditions

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1, 2]

    # pack conditions
    conditions.propulsion.throttle[:, 0] = throttle
    conditions.frames.inertial.velocity_vector[:, 0] = cas  # start up value


## @ingroup Methods-Missions-Segments-Climb
def update_differentials_altitude(segment):
    """On each iteration creates the differentials and integration funcitons from knowns about the problem. Sets the time at each point. Must return in dimensional time, with t[0] = 0

    Assumptions:
    Constant throttle setting, with a constant calibrated airspeed.

    Source:

   N/A

    Inputs:
    segment.climb_angle                         [radians]
    state.conditions.frames.inertial.velocity_vector [meter/second]
    segment.altitude_start                      [meters]
    segment.altitude_end                        [meters]

    Outputs:
    state.conditions.frames.inertial.time       [seconds]
    conditions.frames.inertial.position_vector  [meters]
    conditions.freestream.altitude              [meters]

    Properties Used:
    N/A
    """

    # unpack
    t = segment.state.numerics.dimensionless.control_points
    D = segment.state.numerics.dimensionless.differentiate
    I = segment.state.numerics.dimensionless.integrate

    # Unpack segment initials
    alt0 = segment.altitude_start
    altf = segment.altitude_end
    conditions = segment.state.conditions
    v = segment.state.conditions.frames.inertial.velocity_vector

    # check for initial altitude
    if alt0 is None:
        if not segment.state.initials: raise AttributeError('initial altitude not set')
        alt0 = -1.0 * segment.state.initials.conditions.frames.inertial.position_vector[-1, 2]

        # get overall time step
    vz = -v[:, 2, None]  # Inertial velocity is z down
    dz = altf - alt0
    dt = dz / np.dot(I[-1, :], vz)[-1]  # maintain column array

    # Integrate vz to get altitudes
    alt = alt0 + np.dot(I * dt, vz)

    # rescale operators
    t = t * dt

    # pack
    t_initial = segment.state.conditions.frames.inertial.time[0, 0]
    segment.state.conditions.frames.inertial.time[:, 0] = t_initial + t[:, 0]
    conditions.frames.inertial.position_vector[:, 2] = -alt[:, 0]  # z points down
    conditions.freestream.altitude[:, 0] = alt[:, 0]  # positive altitude in this context

    return


# ----------------------------------------------------------------------
#  Update Velocity Vector from Wind Angle
# ----------------------------------------------------------------------

## @ingroup Methods-Missions-Segments-Climb
def update_velocity_vector_from_wind_angle(segment):
    # unpack
    conditions = segment.state.conditions
    # cas = segment.calibrated_air_speed
    v_mag = segment.state.unknowns.tas[:, 0][:, None]
    alpha = segment.state.unknowns.wind_angle[:, 0][:, None]
    theta = segment.state.unknowns.body_angle[:, 0][:, None]

    # # determine airspeed from calibrated airspeed
    # SUAVE.Methods.Missions.Segments.Common.Aerodynamics.update_atmosphere(segment)  # get density for airspeed
    # density = conditions.freestream.density[:, 0]
    # pressure = conditions.freestream.pressure[:, 0]
    #
    # MSL_data = segment.analyses.atmosphere.compute_values(0.0, 0.0)
    # pressure0 = MSL_data.pressure[0]
    #
    # a0 = 340.29 * Units["m/s"]
    #
    # qc = pressure0 * (((cas/a0)**2 / 5 + 1)**(7/2) - 1)
    # mach = np.sqrt(5 * ((qc / pressure + 1)**(2/7) - 1))
    #
    # eas = a0 * mach * np.sqrt(pressure / pressure0)
    #
    # # air_speed = eas * np.sqrt(MSL_data.density[0] / density)
    #
    # v_mag = air_speed

    # Flight path angle
    gamma = np.transpose(theta - alpha)

    # process
    v_x = np.multiply(v_mag, np.cos(gamma).T)
    v_z = np.multiply(-v_mag, np.sin(gamma).T) # z points down)

    # pack
    conditions.frames.inertial.velocity_vector[:, 0] = v_x.flatten()
    conditions.frames.inertial.velocity_vector[:, 2] = v_z.flatten()

    return conditions


## @ingroup Methods-Missions-Segments-Climb
def residual_cas(segment):
    tas = segment.state.conditions.freestream.velocity
    conditions = segment.state.conditions

    SUAVE.Methods.Missions.Segments.Common.Aerodynamics.update_atmosphere(segment)  # get density for airspeed
    density = conditions.freestream.density[:, 0]
    pressure = conditions.freestream.pressure[:, 0]

    MSL_data = segment.analyses.atmosphere.compute_values(0.0, 0.0)
    pressure0 = MSL_data.pressure[0]

    a0 = 340.29 * Units["m/s"]

    eas = tas.T * np.sqrt(density/MSL_data.density[0])

    M = eas/a0 * np.sqrt(pressure0 / pressure)

    qc = pressure * ((1 + 0.2 * M**2)**(7/2) - 1)

    cas = a0 * np.sqrt(5 * ((qc / pressure0 + 1)**(2/7)-1))

    segment.state.residuals.cas[:, 0] = (cas - segment.calibrated_air_speed) / segment.calibrated_air_speed

    return
