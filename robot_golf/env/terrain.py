import os
import pybullet as p
import numpy as np
import trimesh
import trimesh.creation
import trimesh.boolean


TERRAIN_FILE = 'data/terrain/terrain.obj'
TERRAIN_WITH_HOLE_FILE = 'data/terrain/terrain_with_hole.obj'


def make_terrain(ball_radius: float):
    cur_path = os.path.dirname(__file__)

    # add random hole in the terrain
    terrain_scale = 0.3
    terrain_mesh = trimesh.load(os.path.join(cur_path, TERRAIN_FILE))
    hole_mesh = trimesh.creation.capsule(radius=ball_radius / terrain_scale, height=4)
    hole_mesh.visual.face_colors = [0.5, 0.5, 0.5, 0.5]
    hole_mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0]))
    hole_pos = np.random.rand(2) * np.array([20, 50]) + np.array([70, 10])
    hole_mesh.apply_translation(np.concatenate([hole_pos, np.array([5])]))
    terrain_with_hole_mesh = trimesh.boolean.difference([terrain_mesh, hole_mesh])
    terrain_with_hole_mesh.export(os.path.join(cur_path, TERRAIN_WITH_HOLE_FILE))

    # create terrain
    terrainColId = p.createCollisionShape(
        p.GEOM_MESH,
        fileName=os.path.join(cur_path, TERRAIN_WITH_HOLE_FILE),
        meshScale=[terrain_scale, terrain_scale, terrain_scale],
        flags=p.GEOM_FORCE_CONCAVE_TRIMESH
    )
    terrainVisID = p.createVisualShape(
        p.GEOM_MESH,
        fileName=os.path.join(cur_path, TERRAIN_WITH_HOLE_FILE),
        meshScale=[terrain_scale, terrain_scale, terrain_scale],
        rgbaColor=[54 / 256, 196 / 256, 79 / 256, 0.55]
    )
    terrainId = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=terrainColId,
                                  baseVisualShapeIndex=terrainVisID,
                                  basePosition=[-10, -10, -.8],
                                  baseOrientation=[0, 0, 0, 1])

    return terrainId, hole_pos
