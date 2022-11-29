import numpy as np
import pybullet as p
import pybullet_data

IIWA_FILE = 'robot/kuka_iiwa/model_0.urdf'
BALL_FILE = 'robot/golf_ball/golf_ball.obj'
TERRAIN_FILE = 'robot/terrain/terrain.obj'


def create_env(ball_pos, GUI=False):
    # init
    if GUI:
        p.connect(p.GUI, options='--background_color_red=1.0 --background_color_green=1.0 --background_color_blue=1.0')
    else:
        p.connect(p.DIRECT)
    p.setAdditionalSearchPath(pybullet_data.getDataPath())

    # terrain
    terrainColId = p.createCollisionShape(p.GEOM_MESH, fileName=TERRAIN_FILE, meshScale=[0.3, 0.3, 0.3])
    terrainVisID = p.createVisualShape(p.GEOM_MESH, fileName=TERRAIN_FILE, meshScale=[0.3, 0.3, 0.3])
    terrainId = p.createMultiBody(baseMass=0,
                                  baseCollisionShapeIndex=terrainColId,
                                  baseVisualShapeIndex=terrainVisID,
                                  basePosition=[0, 0, -0.5],
                                  baseOrientation=[0, 0, 0, 1])

    # tee holder
    teeColId = p.createCollisionShape(p.GEOM_CYLINDER, height=0.1, radius=0.03)
    teeVisID = p.createVisualShape(p.GEOM_CYLINDER, length=0.1, radius=0.03)
    teeId = p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=teeColId,
                              baseVisualShapeIndex=teeVisID,
                              basePosition=[ball_pos[0], ball_pos[1], ball_pos[2] - 0.18],
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
    p.setCollisionFilterPair(iiwaId, terrainId, -1, -1, 0)
    return iiwaId, ballId


if __name__ == "__main__":
    robot_id, ball_id = create_env(ball_pos=[0.3, 0.3, 0.3], GUI=True)

    p.setGravity(0, 0, -9.8)
    while True:
        p.stepSimulation()
