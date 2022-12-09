import trimesh
import os
import numpy as np

from .prefix import CLUB_FILE, HIT_FACE_ID, HIT_POINT_1, HIT_POINT_2, CLUB_SCALE


def get_hit_normal_horizon():
    cur_path = os.path.dirname(__file__)

    club_mesh = trimesh.load(os.path.join(cur_path, CLUB_FILE))
    normal = club_mesh.face_normals[HIT_FACE_ID]
    normal /= np.linalg.norm(normal)
    point1 = club_mesh.vertices[HIT_POINT_1]
    point2 = club_mesh.vertices[HIT_POINT_2]
    horizon = point2 - point1
    horizon /= np.linalg.norm(horizon)
    return point2 * CLUB_SCALE, normal, horizon
