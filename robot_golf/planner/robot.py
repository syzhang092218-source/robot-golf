import trimesh
import os
import pybullet as p
import numpy as np

from scipy.spatial.transform import Rotation

from ..env.club import get_hit_normal_horizon


def get_club_init_transform(ball_pos, ball_init_v):
    # hitting frame in club frame
    hit_point, hit_normal, hit_horizon = get_hit_normal_horizon()
    R_CH = Rotation.from_matrix([np.cross(hit_horizon, hit_normal), hit_horizon, hit_normal]).as_quat()
    X_CH = [hit_point, R_CH]
    X_HC = p.invertTransform(X_CH[0], X_CH[1])

    # desired hitting frame in world frame
    v_dir = ball_init_v / np.linalg.norm(ball_init_v)
    y_dir = np.array([0, -1, 0])
    R_WH_d = Rotation.from_matrix([np.cross(y_dir, v_dir), y_dir, v_dir]).as_quat()
    X_WH_d = [ball_pos, R_WH_d]

    # get initial orientation of the club in world frame
    X_WC = p.multiplyTransforms(X_WH_d[0], X_WH_d[1], X_HC[0], X_HC[1])

    return X_WC
