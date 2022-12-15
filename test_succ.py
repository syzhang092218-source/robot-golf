import pybullet as p
import pybullet_data
import time
import numpy as np
import scipy
import torch
import matplotlib.pyplot as plt
import cv2
import os

from PIL import Image

from sklearn.preprocessing import PolynomialFeatures
from sklearn import linear_model

from robot_golf.env.prefix import GRAVITY, BALL_COLLISION_RADIUS
from robot_golf.env.ball import ball_aerodynamics
from robot_golf.planner.robot import get_club_init_transform
from robot_golf.simulator import set_simulator

from sys_id import prepare, generate_data
from robot_golf.sys_id.predictor import QDotPredictor
from robot_golf.sys_id.dyn_trainer import DynTrainer
from robot_golf.video_maker import make_video_linear


if __name__ == "__main__":
    GUI = False
    # init
    # p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
    p.connect(p.DIRECT)

    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    ball_pos = np.array([0.0, -0.5, 0.35])

    succ = 0
    n_iter = 100
    for j in range(n_iter):

        robot_id, joint_pos, ball_id, v_ball_init, hole_pos = prepare(ball_pos)

        # learn dynamics
        device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')

        # qdot_predictor = torch.load('./sys_id_model/40960.pkl', map_location=device)
        # qdots = qdot_predictor(torch.tensor(v_ball_init, device=device).unsqueeze(0).type(torch.float))
        # qdot_1 = qdots[0, 0]
        # qdot_2 = qdots[0, 1]

        valid_qdots = np.load('robot_golf/sys_id/data/qdots_6300.npy', allow_pickle=True)
        valid_qdots = valid_qdots.astype(np.float64)
        valid_vs = np.load('robot_golf/sys_id/data/vs_6300.npy', allow_pickle=True)
        valid_vs = valid_vs.astype(np.float64)
        input_all = valid_vs.copy()
        ransac_qdot1 = linear_model.RANSACRegressor(max_trials=1000000, residual_threshold=2)
        ransac_qdot1.fit(input_all, valid_qdots[:, 0])
        ransac_qdot2 = linear_model.RANSACRegressor()
        ransac_qdot2.fit(input_all, valid_qdots[:, 1])
        qdot_1 = ransac_qdot1.predict(v_ball_init.reshape(1, -1))[0]
        qdot_2 = ransac_qdot2.predict(v_ball_init.reshape(1, -1))[0]


        n_joints = p.getNumJoints(robot_id)
        club_link_id = n_joints - 1

        i_step = 0
        rest_in_hole = False
        while True:
            i_step += 1

            for i in range(n_joints - 3):
                p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
            p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], qdot_1, maxVelocity=10000)
            p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], qdot_2, maxVelocity=10000)

            ball_aerodynamics(ball_id)
            p.stepSimulation()
            # p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=3)

            # if np.linalg.norm(np.array(p.getBasePositionAndOrientation(ball_id)[0])[:2] - np.array(
            #         [hole_pos[0], hole_pos[1]])) < 0.1 and np.linalg.norm(np.array(p.getBaseVelocity(ball_id)[0])[2]) < 0.1:
            #     rest_in_hole = True
            #     succ += 1
            #     break

            if np.linalg.norm(np.array(p.getBaseVelocity(ball_id)[0])[2]) < 0.1 and \
                np.array(p.getBasePositionAndOrientation(ball_id)[0])[2] < -0.3:
                rest_in_hole = True
                succ += 1
                break

            if i_step > 700:
                rest_in_hole = False
                break

        p.resetSimulation()
        print(f'iter: {j}, succ rate: {succ / (j + 1)}')

    print(f'succ rate: {succ / n_iter}')
