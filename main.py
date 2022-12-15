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
    GUI = True
    # init
    p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
    p.setAdditionalSearchPath(pybullet_data.getDataPath())
    ball_pos = np.array([0.0, -0.5, 0.35])
    robot_id, joint_pos, ball_id, v_ball_init, hole_pos = prepare(ball_pos)

    # fitting
    valid_qdots = np.load('robot_golf/sys_id/data/qdots_6300.npy', allow_pickle=True)
    valid_qdots = valid_qdots.astype(np.float64)
    valid_vs = np.load('robot_golf/sys_id/data/vs_6300.npy', allow_pickle=True)
    valid_vs = valid_vs.astype(np.float64)

    input_all = valid_vs.copy()
    output_x = valid_qdots[:, 0]
    output_y = valid_qdots[:, 1]

    # ransac
    ransac_qdot1 = linear_model.RANSACRegressor(max_trials=1000000, residual_threshold=2)
    ransac_qdot1.fit(input_all, valid_qdots[:, 0])
    ransac_qdot2 = linear_model.RANSACRegressor()
    ransac_qdot2.fit(input_all, valid_qdots[:, 1])
    qdot_1 = ransac_qdot1.predict(v_ball_init.reshape(1, -1))[0]
    qdot_2 = ransac_qdot2.predict(v_ball_init.reshape(1, -1))[0]

    inlier_mask = ransac_qdot1.inlier_mask_
    outlier_mask = np.logical_not(inlier_mask)

    # plt.rcParams['text.usetex'] = True
    parameters = {
        # 'font.family': 'Times New Roman',
        'axes.labelsize': 18,
        'axes.titlesize': 18,
        'xtick.labelsize': 18,
        'ytick.labelsize': 18,
        'legend.fontsize': 18,
        'legend.loc': 'upper right'
    }
    plt.rcParams.update(parameters)
    fig = plt.figure(figsize=(8, 6), dpi=50)
    ax = plt.axes(projection='3d')
    # ax.scatter(valid_qdots[outlier_mask, 0], valid_qdots[outlier_mask, 1], valid_vs[outlier_mask, 2], alpha=0.3)
    # ax.scatter(valid_qdots[inlier_mask, 0], valid_qdots[inlier_mask, 1], valid_vs[inlier_mask, 2], c='red', alpha=0.3)
    ax.scatter(valid_qdots[:, 0], valid_qdots[:, 1], valid_vs[:, 0], c='red', alpha=0.3)
    # ax.set_xlabel(r'$\dot{q}_1$')
    # ax.set_ylabel(r'$\dot{q}_2$')
    # ax.set_zlabel(r'$v_x$')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    ax.view_init(elev=0, azim=-70)
    fig.canvas.draw()
    fig.tight_layout()
    plt.savefig('vx.pdf')
    plt.show()

    fig = plt.figure(figsize=(8, 6), dpi=50)
    ax = plt.axes(projection='3d')
    ax.scatter(valid_qdots[:, 0], valid_qdots[:, 1], valid_vs[:, 1], c='red', alpha=0.3)
    # ax.set_xlabel(r'$\dot{q}_1$')
    # ax.set_ylabel(r'$\dot{q}_2$')
    # ax.set_zlabel(r'$v_y$')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    ax.view_init(elev=0, azim=-20)
    fig.canvas.draw()
    fig.tight_layout()
    plt.savefig('vy.pdf')
    plt.show()

    fig = plt.figure(figsize=(8, 6), dpi=50)
    ax = plt.axes(projection='3d')
    ax.scatter(valid_qdots[:, 0], valid_qdots[:, 1], valid_vs[:, 2], c='red', alpha=0.3)
    # ax.set_xlabel(r'$\dot{q}_1$')
    # ax.set_ylabel(r'$\dot{q}_2$')
    # ax.set_zlabel(r'$v_z$')
    # ax.set_xticklabels([])
    # ax.set_yticklabels([])
    # ax.set_zticklabels([])
    ax.view_init(elev=0, azim=-20)
    fig.canvas.draw()
    fig.tight_layout()
    plt.savefig('vz.pdf')
    plt.show()

    # fig = plt.figure(figsize=(8, 6), dpi=50)
    # ax = plt.axes(projection='3d')
    # ax.scatter(valid_qdots[inlier_mask, 0], valid_qdots[inlier_mask, 1], valid_vs[inlier_mask, 0], c='blue', alpha=0.3)
    # # ax.set_xlabel(r'$\dot{q}_1$')
    # # ax.set_ylabel(r'$\dot{q}_2$')
    # # ax.set_zlabel(r'$v_x$')
    # # ax.set_xticklabels([])
    # # ax.set_yticklabels([])
    # # ax.set_zticklabels([])
    # ax.view_init(elev=0, azim=-70)
    # fig.canvas.draw()
    # fig.tight_layout()
    # plt.savefig('vx_in.pdf')
    # plt.show()
    #
    # fig = plt.figure(figsize=(8, 6), dpi=50)
    # ax = plt.axes(projection='3d')
    # ax.scatter(valid_qdots[inlier_mask, 0], valid_qdots[inlier_mask, 1], valid_vs[inlier_mask, 1], c='blue', alpha=0.3)
    # # ax.set_xlabel(r'$\dot{q}_1$')
    # # ax.set_ylabel(r'$\dot{q}_2$')
    # # ax.set_zlabel(r'$v_y$')
    # # ax.set_xticklabels([])
    # # ax.set_yticklabels([])
    # # ax.set_zticklabels([])
    # ax.view_init(elev=0, azim=-20)
    # fig.canvas.draw()
    # fig.tight_layout()
    # plt.savefig('vy_in.pdf')
    # plt.show()
    #
    # fig = plt.figure(figsize=(8, 6), dpi=50)
    # ax = plt.axes(projection='3d')
    # ax.scatter(valid_qdots[inlier_mask, 0], valid_qdots[inlier_mask, 1], valid_vs[inlier_mask, 2], c='blue', alpha=0.3)
    # # ax.set_xlabel(r'$\dot{q}_1$')
    # # ax.set_ylabel(r'$\dot{q}_2$')
    # # ax.set_zlabel(r'$v_z$')
    # # ax.set_xticklabels([])
    # # ax.set_yticklabels([])
    # # ax.set_zticklabels([])
    # ax.view_init(elev=0, azim=-20)
    # fig.canvas.draw()
    # fig.tight_layout()
    # plt.savefig('vz_in.pdf')
    # plt.show()

    v_id = np.argpartition(np.linalg.norm(valid_vs - v_ball_init, axis=1), 5)[:5]
    # q_dot_1_interp = scipy.interpolate.LinearNDInterpolator(valid_vs[v_id], valid_qdots[v_id, 0])
    # q_dot_2_interp = scipy.interpolate.LinearNDInterpolator(valid_vs[v_id], valid_qdots[v_id, 1])
    # q_dots = [q_dot_1_interp(v_ball_init),
    #           q_dot_2_interp(v_ball_init)]
    q_dot_1 = valid_qdots[v_id, 0]
    q_dot_2 = valid_qdots[v_id, 1]
    print('xxxxxxxxxx')
    print(v_ball_init)
    print(valid_vs[v_id])

    # learn dynamics
    device = torch.device('cuda' if torch.cuda.is_available() else 'cpu')
    qdot_predictor = QDotPredictor(device=device).to(device)
    dyn_trainer = DynTrainer(qdot_predictor, device, qdots=valid_qdots[inlier_mask,:], vs=valid_vs[inlier_mask, :])
    # dyn_trainer.train(n_iter=40960)

    # qdot_predictor.load_state_dict(torch.load('./sys_id_model/40960.pkl', map_location=device))
    qdot_predictor = torch.load('./sys_id_model/40960.pkl', map_location=device)
    qdots = qdot_predictor(torch.tensor(v_ball_init, device=device).unsqueeze(0).type(torch.float))
    qdot_1 = qdots[0, 0]
    qdot_2 = qdots[0, 1]

    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1
    pre_hit_flag = True

    # pre_hit_flag = True
    continuous_seperation = 0
    distance_list = []
    velocity = []

    prev_ball_h = p.getBasePositionAndOrientation(ball_id)[0][2]

    peak_step = 211
    video = []
    step_per_frame = 2
    n_frame = peak_step // step_per_frame

    # # params for ball go up
    # camera_init_pos = np.array([-3, -2.5, 1.2])
    # camera_final_pos = np.array([3, -9, 1.2])
    # target_init_pos = ball_pos
    # target_final_pos = np.array([3, 1., 4.])
    # camera_step = (camera_final_pos - camera_init_pos) / n_frame
    # target_step = (target_final_pos - target_init_pos) / n_frame
    # fov_init = 40.
    # fov_final = 60.
    # fov_step = (fov_final - fov_init) / n_frame
    # width = 1280
    # height = 720
    # i_frame = 0
    # for i_step in range(peak_step):
    #     ball_prev_pos = p.getBasePositionAndOrientation(ball_id)[0]
    #     for i in range(n_joints - 3):
    #         p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
    #     p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], qdot_1,
    #                             maxVelocity=10000)
    #     p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], qdot_2,
    #                             maxVelocity=10000)
    #
    #     ball_aerodynamics(ball_id)
    #     p.stepSimulation()
    #
    #     debug_visid = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1])
    #     # debug_colid = p.createCollisionShape(p.GEOM_SPHERE, radius=0.00000)
    #     p.createMultiBody(0, -1, debug_visid, ball_prev_pos)
    #
    #     if i_step % step_per_frame == 0:
    #         projectionMatrix = p.computeProjectionMatrixFOV(
    #             fov=fov_init + i_frame * fov_step,
    #             aspect=width / height,
    #             nearVal=0.1,
    #             farVal=50
    #         )
    #         viewMatrix = p.computeViewMatrix(
    #             cameraEyePosition=list(i_frame * camera_step + camera_init_pos),
    #             cameraTargetPosition=list(i_frame * target_step + target_init_pos),
    #             cameraUpVector=[0, 0, 1]
    #         )
    #         width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    #             width=width,
    #             height=height,
    #             viewMatrix=viewMatrix,
    #             projectionMatrix=projectionMatrix,
    #             renderer=p.ER_BULLET_HARDWARE_OPENGL
    #         )
    #         i_frame += 1
    #         video.append(rgbImg)
    #         print(i_step)
    #         print(i_frame)
    #
    #
    # # debug_visid = p.createVisualShape(p.GEOM_SPHERE, radius=0.2, rgbaColor=[1, 0, 0, 1])
    # # debug_colid = p.createCollisionShape(p.GEOM_SPHERE, radius=0.2)
    # # p.createMultiBody(0, debug_colid, debug_visid, [3, -6, 1.2])
    #
    # # ball go down
    # down_step = 400
    # camera_init_pos = camera_final_pos
    # camera_final_pos = np.array([hole_pos[0] - 2, hole_pos[1] - 2, 1])
    # target_init_pos = target_final_pos
    # target_final_pos = hole_pos
    # fov_init = fov_final
    # fov_final = 40.
    # n_frame = down_step // step_per_frame
    # camera_step = (camera_final_pos - camera_init_pos) / n_frame
    # target_step = (target_final_pos - target_init_pos) / n_frame
    # fov_step = (fov_final - fov_init) / n_frame
    # i_frame = 0
    # for i_step in range(down_step):
    #     if i_step == 50:
    #         a = 0
    #     ball_prev_pos = p.getBasePositionAndOrientation(ball_id)[0]
    #     for i in range(n_joints - 3):
    #         p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
    #     p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], qdot_1,
    #                             maxVelocity=10000)
    #     p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], qdot_2,
    #                             maxVelocity=10000)
    #
    #     ball_aerodynamics(ball_id)
    #     p.stepSimulation()
    #
    #     debug_visid = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1])
    #     # debug_colid = p.createCollisionShape(p.GEOM_SPHERE, radius=0.00000)
    #     p.createMultiBody(0, -1, debug_visid, ball_prev_pos)
    #
    #     if i_step < down_step and i_step % step_per_frame == 0:
    #         projectionMatrix = p.computeProjectionMatrixFOV(
    #             fov=fov_init + i_frame * fov_step,
    #             aspect=width / height,
    #             nearVal=0.1,
    #             farVal=50
    #         )
    #         viewMatrix = p.computeViewMatrix(
    #             cameraEyePosition=list(i_frame * camera_step + camera_init_pos),
    #             cameraTargetPosition=list(i_frame * target_step + target_init_pos),
    #             cameraUpVector=[0, 0, 1]
    #         )
    #         width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    #             width=width,
    #             height=height,
    #             viewMatrix=viewMatrix,
    #             projectionMatrix=projectionMatrix,
    #             renderer=p.ER_BULLET_HARDWARE_OPENGL
    #         )
    #         i_frame += 1
    #         video.append(rgbImg)
    #
    # # post strike
    # post_step = 200
    # camera_init_pos = camera_final_pos
    # camera_final_pos = np.array([hole_pos[0] + 5, hole_pos[1] - 5, 1])
    # target_init_pos = target_final_pos
    # target_final_pos = np.array([3, 1., 3.5])
    # fov_init = fov_final
    # fov_final = 60.
    # n_frame = post_step // step_per_frame
    # camera_step = (camera_final_pos - camera_init_pos) / n_frame
    # target_step = (target_final_pos - target_init_pos) / n_frame
    # fov_step = (fov_final - fov_init) / n_frame
    # i_frame = 0
    # for i_step in range(post_step):
    #     if i_step == 50:
    #         a = 0
    #     ball_prev_pos = p.getBasePositionAndOrientation(ball_id)[0]
    #     for i in range(n_joints - 3):
    #         p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
    #     p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], qdot_1,
    #                             maxVelocity=10000)
    #     p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], qdot_2,
    #                             maxVelocity=10000)
    #
    #     ball_aerodynamics(ball_id)
    #     p.stepSimulation()
    #
    #     debug_visid = p.createVisualShape(p.GEOM_SPHERE, radius=0.02, rgbaColor=[1, 0, 0, 1])
    #     # debug_colid = p.createCollisionShape(p.GEOM_SPHERE, radius=0.00000)
    #     p.createMultiBody(0, -1, debug_visid, ball_prev_pos)
    #
    #     if i_step % step_per_frame == 0:
    #         projectionMatrix = p.computeProjectionMatrixFOV(
    #             fov=fov_init + i_frame * fov_step,
    #             aspect=width / height,
    #             nearVal=0.1,
    #             farVal=50
    #         )
    #         viewMatrix = p.computeViewMatrix(
    #             cameraEyePosition=list(i_frame * camera_step + camera_init_pos),
    #             cameraTargetPosition=list(i_frame * target_step + target_init_pos),
    #             cameraUpVector=[0, 0, 1]
    #         )
    #         width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    #             width=width,
    #             height=height,
    #             viewMatrix=viewMatrix,
    #             projectionMatrix=projectionMatrix,
    #             renderer=p.ER_BULLET_HARDWARE_OPENGL
    #         )
    #         i_frame += 1
    #         video.append(rgbImg)
    #
    # name = 'strike'
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # frame_rate = 24
    # out = cv2.VideoWriter(f'{name}.mp4', fourcc, frame_rate, (width, height))
    # if not os.path.exists(f'video/{name}'):
    #     os.mkdir(f'video/{name}')
    # for i_img, img in enumerate(video):
    #     img_pil = Image.fromarray(img, 'RGBA')
    #     img_rgb = img_pil.convert('RGB')
    #     img_rgb.save(f'video/{name}/{i_img}.png')
    #     img_new = cv2.imread(f'video/{name}/{i_img}.png')
    #     out.write(img_new)
    # out.release()

    i_step = 0
    while True:


        i_step += 1

        for i in range(n_joints - 3):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
        p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], qdot_1, maxVelocity=10000)
        p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], qdot_2, maxVelocity=10000)

        ball_aerodynamics(ball_id)
        p.stepSimulation()
        p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=3)

        if np.linalg.norm(np.array(p.getBasePositionAndOrientation(ball_id)[0]) - np.array(
                [hole_pos[0], hole_pos[1], -.626])) < 0.05 and np.linalg.norm(np.array(p.getBaseVelocity(ball_id)[0])) < 0.3:
            rest_in_hole = True

        # ball_h = p.getBasePositionAndOrientation(ball_id)[0][2]
        # if ball_h < prev_ball_h:
        #     a = 0
        # prev_ball_h = ball_h



        # p.addUserDebugPoints([p.getLinkState(robot_id, club_link_id)[0]], [[0, 0, 1]], pointSize=5)

        # if pre_hit_flag and len(p.getContactPoints(robot_id, ball_id, club_link_id, -1)) > 0 and p.getContactPoints(robot_id, ball_id, club_link_id, -1)[0][9] >= 0:
        #     pre_hit_flag = False
        #
        # if not pre_hit_flag:
        #     distance_list.append(p.getClosestPoints(robot_id, ball_id, 50, club_link_id, -1)[0][8])
        #     a = p.getBaseVelocity(ball_id)[0]
        #     velocity.append(p.getBaseVelocity(ball_id)[0])
        #     if len(p.getContactPoints(robot_id, ball_id, club_link_id, -1)) > 0 and p.getContactPoints(robot_id, ball_id, club_link_id, -1)[0][9] >= 0:
        #         continuous_seperation = 0
        #     else:
        #         continuous_seperation += 1
        #         if continuous_seperation > 5:
        #             # pass
        #             print('--------')
        #             print(v_ball_init)
        #             print(velocity[-5])
        #             print('---------')

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
