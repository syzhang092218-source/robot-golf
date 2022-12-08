import pybullet as p
import numpy as np
import pybullet_data
import trimesh
import trimesh.creation
import trimesh.boolean

IIWA_FILE = 'robot/kuka_iiwa/model_0.urdf'
BALL_FILE = 'robot/golf_ball/golf_ball.obj'
TERRAIN_FILE = 'robot/terrain/terrain.obj'
TERRAIN_WITH_HOLE_FILE = 'robot/terrain/terrain_with_hole.obj'


def make_terrain(ball_radius: float):
    terrain_scale = 0.3
    terrain_mesh = trimesh.load(TERRAIN_FILE)
    hole_mesh = trimesh.creation.capsule(radius=ball_radius / terrain_scale, height=4)
    hole_mesh.visual.face_colors = [0.5, 0.5, 0.5, 0.5]
    hole_mesh.apply_transform(trimesh.transformations.rotation_matrix(np.pi, [0, 1, 0]))

    # hole_pos = np.random.rand(2) * np.array([20, 50]) + np.array([70, 10])
    hole_pos = np.array([30, 30])
    hole_mesh.apply_translation(np.concatenate([hole_pos, np.array([5])]))
    terrain_with_hole_mesh = trimesh.boolean.difference([terrain_mesh, hole_mesh])
    terrain_with_hole_mesh.export(TERRAIN_WITH_HOLE_FILE)

    terrainColId = p.createCollisionShape(p.GEOM_MESH, fileName=TERRAIN_WITH_HOLE_FILE, meshScale=[terrain_scale, terrain_scale, terrain_scale], flags=p.GEOM_FORCE_CONCAVE_TRIMESH)
    terrainVisID = p.createVisualShape(p.GEOM_MESH, fileName=TERRAIN_WITH_HOLE_FILE, meshScale=[terrain_scale, terrain_scale, terrain_scale], rgbaColor=[54/256, 196/256, 79/256, 0.55])
    terrainId = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=terrainColId,
                                  baseVisualShapeIndex=terrainVisID,
                                  basePosition=[-10, -10, -.8],
                                  baseOrientation=[0, 0, 0, 1])

    return terrainId, hole_pos


def create_env(ball_pos, GUI=False):
    # init
    if GUI:
        p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
    else:
        p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # terrain
    terrainId, hole_pos = make_terrain(ball_radius=0.1)

    # tee holder
    teeColId = p.createCollisionShape(p.GEOM_CYLINDER, height=0.2, radius=0.03)
    teeVisID = p.createVisualShape(p.GEOM_CYLINDER, length=0.2, radius=0.03)
    teeId = p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=teeColId,
                              baseVisualShapeIndex=teeVisID,
                              basePosition=[ball_pos[0], ball_pos[1], ball_pos[2] - 0.23],
                              baseOrientation=[0, 0, 0, 1])

    # ball
    ballColId = p.createCollisionShape(p.GEOM_MESH, fileName=BALL_FILE, meshScale=[0.03, 0.03, 0.03])
    ballVisID = p.createVisualShape(p.GEOM_MESH, fileName=BALL_FILE, meshScale=[0.03, 0.03, 0.03])
    ballId = p.createMultiBody(baseMass=0.02,
                               baseCollisionShapeIndex=ballColId,
                               baseVisualShapeIndex=ballVisID,
                               basePosition=ball_pos,
                               baseOrientation=[0, 0, 0, 1])

    # arm with club
    iiwaId = p.loadURDF(IIWA_FILE, [0, 0, 0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION)

    # collision filter
    p.setCollisionFilterPair(teeId, terrainId, -1, -1, 0)
    for link_index in range(-1, 2):
        p.setCollisionFilterPair(iiwaId, terrainId, link_index, -1, 0)
    return iiwaId, ballId