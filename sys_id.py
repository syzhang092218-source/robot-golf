import pybullet as p
import time
import numpy as np
import cv2
import imageio
import matplotlib.pyplot as plt

from PIL import Image

from robot_golf.env import create_env
from robot_golf.env.prefix import GRAVITY, BALL_COLLISION_RADIUS, TIMESTEP
from robot_golf.env.ball import ball_aerodynamics
from robot_golf.planner.robot import get_club_init_transform, solve_v_align_ik
from robot_golf.planner.ball import solve_init_velocity
from robot_golf.simulator import set_simulator
from robot_golf.video_maker import make_video_linear


def prepare(ball_pos):
    robot_id, ball_id, ball_center, v_ball_init, hole_pos = set_simulator(ball_pos)
    print(f'v_ball_init: {v_ball_init}')
    contact_point_pre = ball_center - (BALL_COLLISION_RADIUS + 0.005) * np.array(
        v_ball_init) / np.linalg.norm(v_ball_init)
    X_WL_pre = get_club_init_transform(contact_point_pre, v_ball_init, robot_id)

    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1

    joint_pos_pre = solve_v_align_ik(robot_id, club_link_id, X_WL_pre, v_ball_init, ball_center)

    # video to show the platform
    # fourcc = cv2.VideoWriter_fourcc(*'mp4v')
    # video_platform = []
    # projectionMatrix = p.computeProjectionMatrixFOV(
    #     fov=45.0,
    #     aspect=16/9,
    #     nearVal=0.1,
    #     farVal=50)
    # camera_r = 10
    # camera_theta0 = -np.pi / 4 * 3
    # n_img = 240
    # for i_img in range(n_img):
    #     camera_theta = camera_theta0 + np.pi * 2 / n_img * i_img
    #     camera_z_theta = np.pi * 2 / n_img * i_img
    #     viewMatrix = p.computeViewMatrix(
    #         cameraEyePosition=[camera_r * np.cos(camera_theta), camera_r * np.sin(camera_theta), 3 - 1 * np.sin(camera_z_theta)],
    #         cameraTargetPosition=[0, 0, 0],
    #         cameraUpVector=[0, 0, 1])
    #     width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    #         width=1280,
    #         height=720,
    #         viewMatrix=viewMatrix,
    #         projectionMatrix=projectionMatrix,
    #         # shadow=True,
    #         # lightDirection=[-1, 1, 1],
    #         # lightDistance=1,
    #         # lightColor=[1, 1, 1],
    #         # lightDiffuseCoeff=1,
    #         # lightAmbientCoeff=0.5,
    #         renderer=p.ER_BULLET_HARDWARE_OPENGL
    #     )
    #     video_platform.append(rgbImg)
    #
    # out = cv2.VideoWriter('platform.mp4', fourcc, 24.0, (1280, 720))
    # for i_img, img in enumerate(video_platform):
    #     img_pil = Image.fromarray(img, 'RGBA')
    #     img_rgb = img_pil.convert('RGB')
    #     img_rgb.save(f'video/platform/{i_img}.png')
    #     img_new = cv2.imread(f'video/platform/{i_img}.png')
    #     out.write(img_new)
    # out.release()

    # video: focus on the ball
    # make_video_linear(
    #     name='focus_ball',
    #     duration=3,
    #     camera_init_pos=np.array([camera_r * np.cos(camera_theta0), camera_r * np.sin(camera_theta0), 3]),
    #     camera_final_pos=np.array([camera_r * np.cos(camera_theta0), camera_r * np.sin(camera_theta0), 3]) * 0.3,
    #     target_init_pos=np.array([0, 0, 0]),
    #     target_final_pos=np.array([0, -0.5, 0.35]),
    #     fov_init=45.,
    #     fov_final=20.
    # )
    # projectionMatrix = p.computeProjectionMatrixFOV(
    #     fov=20.0,
    #     aspect=16 / 9,
    #     nearVal=0.1,
    #     farVal=50)
    # n_img = 72
    # camera_init_pos = np.array([camera_r * np.cos(camera_theta0), camera_r * np.sin(camera_theta0), 3])
    # camera_target_pos = camera_init_pos * 0.3
    # camera_pos_step = (camera_target_pos - camera_init_pos) / n_img
    # fov_init = 45.
    # fov_target = 20.
    # fov_step = (fov_target - fov_init) / n_img
    # video_focus_ball = []
    # for i_img in range(n_img):
    #     camera_pos = camera_init_pos + camera_pos_step * i_img
    #     camera_obs_pos = i_img / n_img * np.array([0, -0.5, 0.35])
    #     viewMatrix = p.computeViewMatrix(
    #         cameraEyePosition=list(camera_pos),
    #         cameraTargetPosition=list(camera_obs_pos),
    #         cameraUpVector=[0, 0, 1]
    #     )
    #     width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    #         width=1280,
    #         height=720,
    #         viewMatrix=viewMatrix,
    #         projectionMatrix=projectionMatrix,
    #         renderer=p.ER_BULLET_HARDWARE_OPENGL
    #     )
    #     video_focus_ball.append(rgbImg)
    #
    # out = cv2.VideoWriter('focus_ball.mp4', fourcc, 24.0, (1280, 720))
    # for i_img, img in enumerate(video_focus_ball):
    #     img_pil = Image.fromarray(img, 'RGBA')
    #     img_rgb = img_pil.convert('RGB')
    #     img_rgb.save(f'video/focus_ball/{i_img}.png')
    #     img_new = cv2.imread(f'video/focus_ball/{i_img}.png')
    #     out.write(img_new)
    # out.release()

    # video: focus on the robot
    # video_focus_robot = []
    # n_img = 72
    # camera_init_pos = np.array([camera_r * np.cos(camera_theta0), camera_r * np.sin(camera_theta0), 3]) * 0.3
    # camera_target_pos = np.array([2, -2, 1])
    # camera_pos_step = (camera_target_pos - camera_init_pos) / n_img
    # camera_obs_init_pos = np.array([0, -0.5, 0.35])
    # camera_obs_target_pos = np.array([-1, -0.5, 0.4])
    # camera_obs_step = (camera_obs_target_pos - camera_obs_init_pos) / n_img
    # fov_init = 20.
    # fov_target = 50.
    # fov_step = (fov_target - fov_init) / n_img
    # for i_img in range(n_img):
    #     projectionMatrix = p.computeProjectionMatrixFOV(
    #         fov=fov_init + i_img * fov_step,
    #         aspect=16 / 9,
    #         nearVal=0.1,
    #         farVal=50)
    #     camera_obs_pos = i_img * camera_obs_step + camera_obs_init_pos
    #     camera_pos = i_img * camera_pos_step + camera_init_pos
    #     viewMatrix = p.computeViewMatrix(
    #         cameraEyePosition=list(camera_pos),
    #         cameraTargetPosition=list(camera_obs_pos),
    #         cameraUpVector=[0, 0, 1]
    #     )
    #     width, height, rgbImg, depthImg, segImg = p.getCameraImage(
    #         width=1280,
    #         height=720,
    #         viewMatrix=viewMatrix,
    #         projectionMatrix=projectionMatrix,
    #         renderer=p.ER_BULLET_HARDWARE_OPENGL
    #     )
    #     video_focus_robot.append(rgbImg)
    # out = cv2.VideoWriter('focus_robot.mp4', fourcc, 24.0, (1280, 720))
    # for i_img, img in enumerate(video_focus_robot):
    #     img_pil = Image.fromarray(img, 'RGBA')
    #     img_rgb = img_pil.convert('RGB')
    #     img_rgb.save(f'video/focus_robot/{i_img}.png')
    #     img_new = cv2.imread(f'video/focus_robot/{i_img}.png')
    #     out.write(img_new)
    # out.release()

    # pre-hitting motion, position-controlled
    # for _ in range(5000):
    def move():
        p.stepSimulation()
        for i in range(1, n_joints - 2):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos_pre[i], 0, maxVelocity=0.1)

    make_video_linear(
        name='pre_hit1',
        duration=3,
        camera_init_pos=np.array([2, -2, 1]),
        camera_final_pos=np.array([1.7, -2.5, 1.2]),
        target_init_pos=np.array([-1, -0.5, 0.4]),
        target_final_pos=np.array([-1, -0.5, 0.4]),
        fov_init=50.,
        fov_final=50.,
        movement=move,
        simulation_step=5000,
        release=False
    )

    def move():
        p.stepSimulation()
        p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, joint_pos_pre[0], 0, maxVelocity=0.05)

    make_video_linear(
        name='pre_hit2',
        duration=3,
        camera_init_pos=np.array([1.7, -2.5, 1.2]),
        camera_final_pos=np.array([2.7, -2.5, 1.2]),
        target_init_pos=np.array([-1, -0.5, 0.4]),
        target_final_pos=np.array([0, -0.5, 0.4]),
        fov_init=50.,
        fov_final=50.,
        movement=move,
        simulation_step=5000,
        release=False
    )

    # for _ in range(5000):
    #     p.stepSimulation()
    #     p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, joint_pos_pre[0], 0, maxVelocity=0.05)

    ball_center = np.array(p.getBasePositionAndOrientation(ball_id)[0])
    contact_point = ball_center - BALL_COLLISION_RADIUS * np.array(v_ball_init) / np.linalg.norm(v_ball_init)
    X_WL = get_club_init_transform(contact_point, v_ball_init, robot_id)
    joint_pos = solve_v_align_ik(robot_id, club_link_id, X_WL, v_ball_init, ball_center)

    make_video_linear(
        name='pre_hit3',
        duration=3,
        camera_init_pos=np.array([2.7, -2.5, 1.2]),
        camera_final_pos=np.array([-3, -2.5, 1.2]),
        target_init_pos=np.array([0, -0.5, 0.4]),
        target_final_pos=ball_center,
        fov_init=50.,
        fov_final=40.,
        movement=None,
        simulation_step=None,
        release=False
    )

    # def set_precise_init_configuration():
    #     for i in range(n_joints - 3):
    #         p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
    #     p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], 0, maxVelocity=0.1)
    #     p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], 0, maxVelocity=0.1)
    #     ball_aerodynamics(ball_id)
    #     p.stepSimulation()
    #
    # make_video_linear(
    #     name='precise_pre_hit',
    #     duration=1,
    #     camera_init_pos=np.array([-3, -2.5, 1.2]),
    #     camera_final_pos=np.array([-3, -2.5, 1.2]),
    #     target_init_pos=ball_center,
    #     target_final_pos=ball_center,
    #     fov_init=40.,
    #     fov_final=40.,
    #     movement=set_precise_init_configuration(),
    #     simulation_step=int(0.2 / TIMESTEP),
    #     release=False
    # )

    for _ in range(int(0.2 / TIMESTEP)):
        for i in range(n_joints - 3):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
        p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], 0, maxVelocity=0.1)
        p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], 0, maxVelocity=0.1)
        ball_aerodynamics(ball_id)
        p.stepSimulation()
    return robot_id, joint_pos, ball_id, v_ball_init, hole_pos

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

