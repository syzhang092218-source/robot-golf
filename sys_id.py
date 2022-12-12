import pybullet as p
import time
import numpy as np

from robot_golf.env import create_env
from robot_golf.env.prefix import GRAVITY, BALL_COLLISION_RADIUS, TIMESTEP
from robot_golf.env.ball import ball_aerodynamics
from robot_golf.planner.robot import get_club_init_transform, solve_v_align_ik
from robot_golf.planner.ball import solve_init_velocity
from robot_golf.simulator import set_simulator


def prepare(ball_pos):
    robot_id, ball_id, ball_center, v_ball_init = set_simulator(ball_pos)
    print(f'v_ball_init: {v_ball_init}')
    contact_point_pre = ball_center - (BALL_COLLISION_RADIUS + 0.005) * np.array(
        v_ball_init) / np.linalg.norm(v_ball_init)
    X_WL_pre = get_club_init_transform(contact_point_pre, v_ball_init, robot_id)

    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1

    joint_pos_pre = solve_v_align_ik(robot_id, club_link_id, X_WL_pre, v_ball_init, ball_center)

    # pre-hitting motion, position-controlled
    for _ in range(5000):
        p.stepSimulation()
        for i in range(1, n_joints - 2):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos_pre[i], 0, maxVelocity=0.1)
    for _ in range(5000):
        p.stepSimulation()
        p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, joint_pos_pre[0], 0, maxVelocity=0.05)

    ball_center = np.array(p.getBasePositionAndOrientation(ball_id)[0])
    contact_point = ball_center - BALL_COLLISION_RADIUS * np.array(v_ball_init) / np.linalg.norm(v_ball_init)
    X_WL = get_club_init_transform(contact_point, v_ball_init, robot_id)
    joint_pos = solve_v_align_ik(robot_id, club_link_id, X_WL, v_ball_init, ball_center)

    for _ in range(int(0.2 / TIMESTEP)):
        for i in range(n_joints - 3):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
        p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], 0, maxVelocity=0.1)
        p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], 0, maxVelocity=0.1)
        ball_aerodynamics(ball_id)
        p.stepSimulation()
    return robot_id, joint_pos, ball_id, v_ball_init

def generate_data(robot_id, joint_pos, ball_id, qdot_1, qdot_2):
    # todo: collect data, learn (vx, 0, vz) -> (q_-1 dot, q_-2 dot)
    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1

    pre_hit_flag = True
    continuous_seperation = 0
    distance_list = []
    velocity = []

    for _ in range(5000):
        for i in range(n_joints - 3):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
        p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], qdot_1, maxVelocity=10000)
        p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], qdot_2, maxVelocity=10000)

        ball_aerodynamics(ball_id)
        p.stepSimulation()
        p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=5)
        p.addUserDebugPoints([p.getLinkState(robot_id, club_link_id)[0]], [[0, 0, 1]], pointSize=5)

        # contact detection
        if pre_hit_flag and len(p.getContactPoints(robot_id, ball_id, club_link_id, -1)) > 0:
            pre_hit_flag = False

        if not pre_hit_flag:
            distance_list.append(p.getClosestPoints(robot_id, ball_id, 1, club_link_id, -1)[0][8])
            a = p.getBaseVelocity(ball_id)[0]
            velocity.append(p.getBaseVelocity(ball_id)[0])
            if len(p.getContactPoints(robot_id, ball_id, club_link_id, -1)) > 0:
                continuous_seperation = 0
            else:
                continuous_seperation += 1
                if continuous_seperation > 5:
                    return velocity[-5]

    return [None, None, None]

