import pybullet as p
import time
import numpy as np

from robot_golf.env.prefix import GRAVITY, BALL_COLLISION_RADIUS
from robot_golf.env.ball import ball_aerodynamics
from robot_golf.planner.robot import get_club_init_transform
from robot_golf.simulator import set_simulator

from sys_id import generate_data


if __name__ == "__main__":
    generate_data()
    # ball_pos = np.array([0.0, -0.5, 0.35])
    # robot_id, ball_id, X_WL, v_ball_init = set_simulator(ball_pos)
    #
    # n_joints = p.getNumJoints(robot_id)
    # base_x_joint = 0
    # base_y_joint = 1
    # base_theta_joint = 2
    # club_link_id = n_joints - 1
    #
    # joint_pos = p.calculateInverseKinematics(robot_id, club_link_id, X_WL[0], X_WL[1])
    #
    # # pre-hitting motion, position-controlled
    # for _ in range(5000):
    #     p.stepSimulation()
    #     for i in range(1, n_joints - 2):
    #         p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0, maxVelocity=0.1)
    # for _ in range(5000):
    #     p.stepSimulation()
    #     p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, joint_pos[0], 0, maxVelocity=0.05)
    #
    # # joint_pos1 = joint_pos
    # #
    # # ball_center = np.array(p.getBasePositionAndOrientation(ball_id)[0])
    # # contact_point = ball_center - BALL_COLLISION_RADIUS * np.array([np.linalg.norm(v_ball_init[:2]), 0, v_ball_init[2]]) / np.linalg.norm(v_ball_init)
    # # X_WC = get_club_init_transform(contact_point, v_ball_init, robot_id)
    # # joint_pos = p.calculateInverseKinematics(robot_id, club_link_id, X_WC[0], X_WC[1])
    #
    # # todo: calculate new robot position and rotation angle, update joint_pos[:3] (x, y, theta), set joint state
    # # todo: collect data, learn (vx, 0, vz) -> (q_-1 dot, q_-2 dot)
    #
    # for i in range(n_joints - 2):
    #     p.resetJointState(robot_id, i, joint_pos[i])
    #
    # # q_dots = v_to_qdot(v_ball_init)  # mean(-100, 20)
    #
    # # p.resetBaseVelocity(ball_id, v_ball_init)
    #
    # while True:
    #     p.stepSimulation()
    #     for i in range(n_joints - 3):
    #         p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
    #     p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], -100, maxVelocity=10000)
    #     p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], 100, maxVelocity=10000)
    #     ball_aerodynamics(ball_id)
    #     p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=5)
    #     p.addUserDebugPoints([p.getLinkState(robot_id, club_link_id)[0]], [[0, 0, 1]], pointSize=5)
