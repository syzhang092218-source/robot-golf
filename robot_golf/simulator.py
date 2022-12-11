import pybullet as p
import time
import numpy as np

from .env import create_env
from .env.prefix import GRAVITY, BALL_COLLISION_RADIUS
from .env.ball import ball_aerodynamics
from .planner.robot import get_club_init_transform
from .planner.ball import solve_init_velocity


def set_simulator():
    ball_pos = np.array([0.0, -0.5, 0.35])
    robot_id, ball_id, terrainId, hole_pos = create_env(ball_pos=ball_pos, GUI=True)

    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1
    p.setGravity(0, 0, GRAVITY)
    p.changeDynamics(ball_id, linkIndex=-1, linearDamping=0, contactDamping=1, contactStiffness=100, restitution=0.98,
                     lateralFriction=0., spinningFriction=0., rollingFriction=0.)
    p.setTimeStep(1 / 100)
    p.changeDynamics(terrainId, linkIndex=-1, contactDamping=1, contactStiffness=100, restitution=0.1)

    # wait for the ball to land on the tee holder
    for _ in range(100):
        p.stepSimulation()

    v_ball_init, dt = solve_init_velocity(
        hole_pos=hole_pos - np.array(p.getBasePositionAndOrientation(ball_id)[0]),
        hole_radius=0.1,
        ball_radius=0.08,
        max_land_angle=np.pi / 2 * 0.15,
        v_constraint=[np.array([-100000, -100000, 0]), np.array([100000, 100000, 100000])]
    )
    p.setTimeStep(1/200)



    # v_ball_init = np.array([1, 0, 0])

    ball_center = np.array(p.getBasePositionAndOrientation(ball_id)[0])
    contact_point = ball_center - (BALL_COLLISION_RADIUS + 0.005) * np.array([np.linalg.norm(v_ball_init[:2]), 0, v_ball_init[2]]) / np.linalg.norm(v_ball_init)
    X_WC = get_club_init_transform(contact_point, v_ball_init, robot_id)

    # ball_bounding_box = np.array(p.getAABB(ball_id))
    # ball_radius = (ball_bounding_box[1, :] - ball_bounding_box[0, :]).mean() / 2
    # contact_point = ball_center - (BALL_COLLISION_RADIUS + 0.005) * v_ball_init / np.linalg.norm(v_ball_init)
    # X_WC = get_club_init_transform(contact_point, v_ball_init, robot_id)
    # p.addUserDebugPoints([X_WC[0]], [[1, 0, 0]], pointSize=5)

    joint_pos = p.calculateInverseKinematics(robot_id, club_link_id, X_WC[0], X_WC[1])

    # robot initial position
    p.resetJointState(robot_id, 0, -2, 0)
    p.stepSimulation()

    return robot_id, ball_id, joint_pos, v_ball_init
