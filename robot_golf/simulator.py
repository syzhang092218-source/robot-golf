import pybullet as p
import time
import numpy as np

from .env import create_env
from .env.prefix import GRAVITY, BALL_COLLISION_RADIUS, TIMESTEP
from .env.ball import ball_aerodynamics
from .planner.robot import get_club_init_transform
from .planner.ball import solve_init_velocity


def set_simulator(ball_pos):
    """SET UP SIMULATOR & CALCULATE TARGET INIT BALL VELOCITY"""
    robot_id, ball_id, terrainId, hole_pos = create_env(ball_pos=ball_pos)

    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1
    p.setGravity(0, 0, GRAVITY)
    p.setTimeStep(TIMESTEP)
    p.changeDynamics(ball_id, linkIndex=-1, linearDamping=0, contactDamping=1, contactStiffness=100, restitution=0.98,
                     lateralFriction=0., spinningFriction=0., rollingFriction=0.)
    p.changeDynamics(terrainId, linkIndex=-1, contactDamping=1, contactStiffness=100, restitution=0.02,
                     lateralFriction=1, spinningFriction=1, rollingFriction=1)

    # wait for the ball to land on the tee holder
    for _ in range(int(1/TIMESTEP)):
        p.stepSimulation()

    v_ball_init, dt = solve_init_velocity(
        hole_pos=hole_pos - np.array(p.getBasePositionAndOrientation(ball_id)[0]),
        hole_radius=0.1,
        ball_radius=0.08,
        max_land_angle=np.pi / 2 * 0.15,
        v_constraint=[np.array([-100000, -100000, 0]), np.array([100000, 100000, 100000])]
    )

    # v_ball_init = np.array([1, 0, 0])

    ball_center = np.array(p.getBasePositionAndOrientation(ball_id)[0])

    # robot initial position
    p.resetJointState(robot_id, 0, -2, 0)
    p.stepSimulation()

    return robot_id, ball_id, ball_center, v_ball_init
