import trimesh
import os
import numpy as np
import copy

from .prefix import CLUB_FILE, HIT_FACE_ID, HIT_POINT_1, HIT_POINT_2, HIT_POINT_3, CLUB_SCALE


def get_hit_normal_horizon():
    cur_path = os.path.dirname(__file__)
    club_mesh = trimesh.load(os.path.join(cur_path, CLUB_FILE))
    point1 = club_mesh.vertices[HIT_POINT_1]
    point3 = club_mesh.vertices[HIT_POINT_3]
    point2 = club_mesh.vertices[HIT_POINT_2]
    horizon = point3 - point1
    vec23 = point3 - point2
    normal = np.cross(vec23, horizon)
    normal /= np.linalg.norm(normal)
    horizon /= np.linalg.norm(horizon)
    return point2 * CLUB_SCALE, normal, horizon
