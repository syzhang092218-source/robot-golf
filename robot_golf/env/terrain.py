import os
import pybullet as p
import numpy as np
import trimesh
import trimesh.creation
import trimesh.boolean

from .prefix import TERRAIN_FILE, TERRAIN_WITH_HOLE_FILE


def make_terrain(hole_radius: float):
    cur_path = os.path.dirname(__file__)

    # add random hole in the terrain
    terrain_scale = 0.3
    terrain_mesh = trimesh.load(os.path.join(cur_path, TERRAIN_FILE))

    hole_pos_constraint = [np.array([50, 30, 0.001]), np.array([60, 40, 10])]
    found = False
    vertex_id = 0
    while not found:
        vertex_id = np.random.randint(terrain_mesh.vertices.shape[0])
        if (hole_pos_constraint[0] <= terrain_mesh.vertices[vertex_id]).all() and \
                (terrain_mesh.vertices[vertex_id] <= hole_pos_constraint[1]).all():
            found = True
    hole_pos = terrain_mesh.vertices[vertex_id]
    # testing point
    # hole_pos = terrain_mesh.vertices[44826]

    hole_mesh = trimesh.creation.capsule(radius=hole_radius / terrain_scale, height=4.2)
    cone_mesh = trimesh.creation.cone(radius=hole_radius*5 / terrain_scale, height=2)
    cone_mesh.apply_translation(np.array([0, 0, 2]))
    hole_mesh = trimesh.boolean.union([hole_mesh, cone_mesh])
    hole_mesh.visual.face_colors = [0.5, 0.5, 0.5, 0.5]
    hole_mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0]))
    hole_mesh.apply_translation(np.concatenate([hole_pos[:2], np.array([3.1 + hole_pos[2]])]))
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
    terrain_base_pos = np.array([-10, -10, -.8])
    terrainId = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=terrainColId,
                                  baseVisualShapeIndex=terrainVisID,
                                  basePosition=list(terrain_base_pos),
                                  baseOrientation=[0, 0, 0, 1])

    return terrainId, hole_pos * terrain_scale + terrain_base_pos
