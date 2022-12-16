import pybullet as p
import numpy as np

from robot_golf.env.prefix import BALL_COLLISION_RADIUS, TIMESTEP
from robot_golf.env.ball import ball_aerodynamics
from robot_golf.planner.robot import get_club_init_transform, solve_v_align_ik
from robot_golf.simulator import set_simulator
from robot_golf.video_maker import make_video_linear


def prepare(ball_pos):
    # set up simulator
    robot_id, ball_id, ball_center, v_ball_init, hole_pos = set_simulator(ball_pos)
    print(f'v_ball_init: {v_ball_init}')

    # calculate pre-hitting position for the club
    contact_point_pre = ball_center - (BALL_COLLISION_RADIUS + 0.005) * np.array(v_ball_init) \
                        / np.linalg.norm(v_ball_init)
    X_WL_pre = get_club_init_transform(contact_point_pre, v_ball_init, robot_id)

    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1

    # solve pre-hitting joint positions
    joint_pos_pre = solve_v_align_ik(robot_id, club_link_id, X_WL_pre, v_ball_init, ball_center)

    # move the joints to hitting position
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

    # move the base to hitting position
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

    # reach precise pre-hitting position
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

    for _ in range(int(0.2 / TIMESTEP)):
        for i in range(n_joints - 3):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
        p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], 0, maxVelocity=0.1)
        p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], 0, maxVelocity=0.1)
        ball_aerodynamics(ball_id)
        p.stepSimulation()
    return robot_id, joint_pos, ball_id, v_ball_init, hole_pos


def generate_data(robot_id, joint_pos, ball_id, qdot_1, qdot_2):
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
