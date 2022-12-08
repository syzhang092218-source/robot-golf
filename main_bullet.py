import numpy as np
import math
import pybullet as p
import pybullet_data
import trimesh
import trimesh.creation
import trimesh.boolean

from time import sleep

IIWA_FILE = 'robot/kuka_iiwa/model_0.urdf'
BALL_FILE = 'robot/golf_ball/golf_ball.obj'
TERRAIN_FILE = 'robot/terrain/terrain.obj'
TERRAIN_WITH_HOLE_FILE = 'robot/terrain/terrain_with_hole.obj'


def make_terrain(ball_radius: float):
    terrain_mesh = trimesh.load(TERRAIN_FILE)
    hole_mesh = trimesh.creation.capsule(radius=ball_radius, height=4)
    hole_mesh.visual.face_colors = [0.5, 0.5, 0.5, 0.5]
    hole_mesh.apply_transform(trimesh.transformations.rotation_matrix(math.pi, [0, 1, 0]))

    hole_pos = np.random.rand(2) * np.array([20, 50]) + np.array([70, 10])
    hole_mesh.apply_translation(np.concatenate([hole_pos, np.array([5])]))
    terrain_with_hole_mesh = trimesh.boolean.difference([terrain_mesh, hole_mesh])
    terrain_with_hole_mesh.export(TERRAIN_WITH_HOLE_FILE)

    terrainColId = p.createCollisionShape(p.GEOM_MESH, fileName=TERRAIN_WITH_HOLE_FILE, meshScale=[0.3, 0.3, 0.3])
    terrainVisID = p.createVisualShape(p.GEOM_MESH, fileName=TERRAIN_WITH_HOLE_FILE, meshScale=[0.3, 0.3, 0.3])
    terrainId = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=terrainColId,
                                  baseVisualShapeIndex=terrainVisID,
                                  basePosition=[0, 0, -0.5],
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
    terrainId, hole_pos = make_terrain(ball_radius=1)  # todo: change ball radius
    # terrainColId = p.createCollisionShape(p.GEOM_MESH, fileName=TERRAIN_FILE, meshScale=[0.3, 0.3, 0.3])
    # terrainVisID = p.createVisualShape(p.GEOM_MESH, fileName=TERRAIN_FILE, meshScale=[0.3, 0.3, 0.3])
    # terrainId = p.createMultiBody(baseMass=0,
    #                               baseCollisionShapeIndex=terrainColId,
    #                               baseVisualShapeIndex=terrainVisID,
    #                               basePosition=[0, 0, -0.5],
    #                               baseOrientation=[0, 0, 0, 1])

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
    p.setCollisionFilterPair(iiwaId, terrainId, -1, -1, 0)
    return iiwaId, ballId, hole_pos


if __name__ == "__main__":
    robot_id, ball_id, hole_pos = create_env(ball_pos=[0.3, 0.3, 0.3], GUI=True)
    p.setGravity(0, 0, -9.8)
    p.resetBaseVelocity(ball_id, [5, 0, 0])
    while True:
        p.stepSimulation()
        sleep(0.01)
