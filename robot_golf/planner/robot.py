import trimesh
import os
import pybullet as p
import numpy as np

from scipy.spatial.transform import Rotation

from ..env.club import get_hit_normal_horizon


def get_club_init_transform(contact_point, v_ball_init, robot_id):
    # link frame in club frame
    P_LC = [0, 0.02, 0.15]
    R_LC = p.getQuaternionFromEuler([0, 3.5, 1.57])
    X_LC = [P_LC, R_LC]
    X_CL = p.invertTransform(X_LC[0], X_LC[1])

    # hitting frame in club frame
    hit_point, hit_normal, hit_horizon = get_hit_normal_horizon()
    hit_normal = hit_normal
    R_CH = Rotation.from_matrix([np.cross(hit_horizon, hit_normal), hit_horizon, hit_normal]).as_quat()
    R_CH = p.invertTransform([0, 0, 0], R_CH)[1]
    X_CH = [hit_point, R_CH]
    X_HC = p.invertTransform(X_CH[0], X_CH[1])

    # desired hitting frame in world frame
    v_dir = v_ball_init / np.linalg.norm(v_ball_init)
    y_dir = np.array([0, -1, 0])
    R_WH_d = Rotation.from_matrix([np.cross(y_dir, v_dir), y_dir, v_dir]).as_quat()
    R_WH_d = p.invertTransform([0, 0, 0], R_WH_d)[1]
    X_WH_d = [contact_point, R_WH_d]

    # get initial orientation of the club in world frame
    X_WC = p.multiplyTransforms(X_WH_d[0], X_WH_d[1], X_HC[0], X_HC[1])
    X_WL = p.multiplyTransforms(X_WC[0], X_WC[1], X_CL[0], X_CL[1])

    # # debug hitting frame
    # X_LH = p.multiplyTransforms(X_LC[0], X_LC[1], X_CH[0], X_CH[1])
    # p_LH = np.array(X_LH[0])
    # for axis in [[1, 0, 0], [0, 1, 0], [0, 0, 1]]:
    #     # y to be parallel with y-axis
    #     # z for contact normal
    #     direction = p.multiplyTransforms([0, 0, 0], X_LH[1], axis, [0, 0, 0, 1])
    #     p.addUserDebugLine(p_LH, p_LH + np.array(direction[0]), lineColorRGB=axis, lineWidth=0.2,
    #                        parentObjectUniqueId=robot_id, parentLinkIndex=9)

    return X_WL


def solve_v_align_ik(robot_id, club_link_id, X_WL, v_ball_init, ball_center):
    """
    base_x_joint = 0
    base_y_joint = 1
    base_theta_joint = 2
    """
    # to v-align frame
    R_WV = p.getQuaternionFromEuler([0, 0, np.arctan(v_ball_init[1]/v_ball_init[0])])
    X_WV = [[ball_center[0], ball_center[1], 0], R_WV]
    X_VW = p.invertTransform(X_WV[0], X_WV[1])
    # v_V = p.multiplyTransforms([0, 0, 0], X_VW[1], v_ball_init, [0, 0, 0, 1])

    # init value set to v-align frame
    x_W, y_W, theta_W = [p.getJointState(robot_id, i)[0] for i in range(3)]
    joints_V = p.multiplyTransforms(X_VW[0], X_VW[1], [x_W, y_W, 0], p.getQuaternionFromEuler([0, 0, theta_W]))
    p.resetJointState(robot_id, 0, joints_V[0][0])
    p.resetJointState(robot_id, 1, joints_V[0][1])
    # p.resetJointState(robot_id, 2, p.getEulerFromQuaternion(joints_V[1])[2])

    # IK
    X_VL = p.multiplyTransforms(X_VW[0], X_VW[1], X_WL[0], X_WL[1])
    sol_V = p.calculateInverseKinematics(robot_id, club_link_id, X_VL[0], X_VL[1], residualThreshold=0.000001)

    # set back to world frame
    p.resetJointState(robot_id, 0, x_W)
    p.resetJointState(robot_id, 1, y_W)
    # p.resetJointState(robot_id, 2, theta_W)

    # back to world frame
    sol_W = p.multiplyTransforms(X_WV[0], X_WV[1], [sol_V[0], sol_V[1], 0], p.getQuaternionFromEuler([0, 0, sol_V[2]]))
    x, y = sol_W[0][:2]
    euler_angle = p.getEulerFromQuaternion(sol_W[1])
    return [x, y, euler_angle[2]] + list(sol_V[3:])

