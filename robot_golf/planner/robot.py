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

    return X_WL
