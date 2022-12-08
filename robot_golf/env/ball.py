import numpy as np
import casadi
import pybullet as p

from .prefix import GRAVITY, AIR_DAMPING


def ball_flying_dyn(x):
    """
    Flying dynamics of the ball.

    Parameters
    ----------
    x: [x, y, z, vx, vy, vz]

    Returns
    -------
    xdot: [vx, vy, vz, ax, ay, az]
    """
    if isinstance(x, np.ndarray):
        xdot = np.array([x[3], x[4], x[5], -AIR_DAMPING * x[3], -AIR_DAMPING * x[4], GRAVITY - AIR_DAMPING * x[5]])
    else:
        xdot = casadi.MX.zeros(6)
        xdot[:3] = x[3:]
        xdot[3] = -AIR_DAMPING * x[3]
        xdot[4] = -AIR_DAMPING * x[4]
        xdot[5] = GRAVITY - AIR_DAMPING * x[5]
    return xdot


def ball_aerodynamics(ball_id):
    v = p.getBaseVelocity(ball_id)[0]
    pos = p.getBasePositionAndOrientation(ball_id)[0]
    force = -AIR_DAMPING * np.array(v) * p.getDynamicsInfo(ball_id, -1)[0]
    p.applyExternalForce(ball_id, -1, force, pos, p.WORLD_FRAME)
