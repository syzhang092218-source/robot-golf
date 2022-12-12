import pybullet as p
import pybullet_data
import time
import numpy as np
import scipy

from sklearn.preprocessing import PolynomialFeatures
from sklearn import linear_model

from robot_golf.env.prefix import GRAVITY, BALL_COLLISION_RADIUS
from robot_golf.env.ball import ball_aerodynamics
from robot_golf.planner.robot import get_club_init_transform
from robot_golf.simulator import set_simulator

from sys_id import prepare, generate_data


if __name__ == "__main__":
    GUI = True
    # init
    p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
    # p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    ball_pos = np.array([0.0, -0.5, 0.35])

    # v_balls = []
    # for i in range(100):
    #     robot_id, joint_pos, ball_id, v_ball_init = prepare(ball_pos)
    #     v_balls.append(v_ball_init)
    #     p.resetSimulation()
    # print(f'v_mean: {np.mean(v_balls)}')

    robot_id, joint_pos, ball_id, v_ball_init = prepare(ball_pos)

    # fitting
    valid_qdots = np.load('qdots.npy', allow_pickle=True)
    valid_qdots = valid_qdots.astype(np.float64)
    valid_vs = np.load('vs.npy', allow_pickle=True)
    valid_vs = valid_vs.astype(np.float64)

    input_all = valid_vs.copy()
    output_x = valid_qdots[:, 0]
    output_y = valid_qdots[:, 1]

    poly_second_order = PolynomialFeatures(degree=2)
    input_second_order = poly_second_order.fit_transform(input_all)

    clf_second_order_x = linear_model.LinearRegression()
    clf_second_order_x.fit(input_second_order, output_x)
    clf_second_order_y = linear_model.LinearRegression()
    clf_second_order_y.fit(input_second_order, output_y)

    # test
    test_input_all = np.array(v_ball_init).reshape(1, -1)
    test_input_second_order = poly_second_order.fit_transform(test_input_all)
    # qdot_1_coef = [0., 1.09470062, 5.92428558, 3.11208748, -0.34473883, -0.41970851, 0.02019625, 0.37260651, -0.12307135, -0.23749958]
    # qdot_2_coef = [0., 4.10949569, -7.64972368, 3.16672815, -0.35112686, 0.82761246, -0.32129703, -1.81880136, 0.62503639, -0.1596113]
    # clf_second_order_x.set_params(qdot_1_coef)
    # clf_second_order_y.set_params(qdot_2_coef)
    qdot_1 = clf_second_order_x.predict(test_input_second_order)
    qdot_2 = clf_second_order_y.predict(test_input_second_order)

    # v_id = np.argpartition(np.linalg.norm(valid_vs - v_ball_init, axis=1), 5)[:5]
    # q_dot_1_interp = scipy.interpolate.LinearNDInterpolator(valid_vs[v_id], valid_qdots[v_id, 0])
    # q_dot_2_interp = scipy.interpolate.LinearNDInterpolator(valid_vs[v_id], valid_qdots[v_id, 1])
    # q_dots = [q_dot_1_interp(v_ball_init),
    #           q_dot_2_interp(v_ball_init)]



    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1
    pre_hit_flag = True

    # pre_hit_flag = True
    continuous_seperation = 0
    distance_list = []
    velocity = []
    while True:
        for i in range(n_joints - 3):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
        p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], qdot_1, maxVelocity=10000)
        p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], qdot_2, maxVelocity=10000)

        ball_aerodynamics(ball_id)
        p.stepSimulation()



        p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=5)
        p.addUserDebugPoints([p.getLinkState(robot_id, club_link_id)[0]], [[0, 0, 1]], pointSize=5)

        if pre_hit_flag and len(p.getContactPoints(robot_id, ball_id, club_link_id, -1)) > 0 and p.getContactPoints(robot_id, ball_id, club_link_id, -1)[0][9] >= 0:
            pre_hit_flag = False

        if not pre_hit_flag:
            distance_list.append(p.getClosestPoints(robot_id, ball_id, 50, club_link_id, -1)[0][8])
            a = p.getBaseVelocity(ball_id)[0]
            velocity.append(p.getBaseVelocity(ball_id)[0])
            if len(p.getContactPoints(robot_id, ball_id, club_link_id, -1)) > 0 and p.getContactPoints(robot_id, ball_id, club_link_id, -1)[0][9] >= 0:
                continuous_seperation = 0
            else:
                continuous_seperation += 1
                if continuous_seperation > 5:
                    print('--------')
                    print(v_ball_init)
                    print(velocity[-5])
                    print('---------')

    # todo: robot final pos, camera


    # fitting


    # test


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
