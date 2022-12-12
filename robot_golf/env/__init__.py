import os
import pybullet as p
import numpy as np
import pybullet_data

from .terrain import make_terrain


from .prefix import IIWA_FILE, BALL_FILE, BALL_COLLISION_RADIUS


def create_env(ball_pos):
    cur_path = os.path.dirname(__file__)

    # terrain
    terrainId, hole_pos = make_terrain(hole_radius=0.15)

    # tee holder
    teeColId = p.createCollisionShape(p.GEOM_CYLINDER, height=0.2, radius=0.03)
    teeVisID = p.createVisualShape(p.GEOM_CYLINDER, length=0.2, radius=0.03)
    teeId = p.createMultiBody(baseMass=0,
                              baseCollisionShapeIndex=teeColId,
                              baseVisualShapeIndex=teeVisID,
                              basePosition=[ball_pos[0], ball_pos[1], ball_pos[2] - 0.23],
                              baseOrientation=[0, 0, 0, 1])

    # ball
    ballColId = p.createCollisionShape(p.GEOM_SPHERE, radius=BALL_COLLISION_RADIUS)
    ballVisID = p.createVisualShape(p.GEOM_MESH,
                                    fileName=os.path.join(cur_path, BALL_FILE),
                                    meshScale=[0.03, 0.03, 0.03])
    ballId = p.createMultiBody(baseMass=0.002,
                               baseCollisionShapeIndex=ballColId,
                               baseVisualShapeIndex=ballVisID,
                               basePosition=ball_pos,
                               baseOrientation=[0, 0, 0, 1])

    # arm with club
    iiwaId = p.loadURDF(
        os.path.join(cur_path, IIWA_FILE), [0, 0, 0], [0, 0, 0, 1], useFixedBase=True, flags=p.URDF_USE_SELF_COLLISION
    )

    # collision filter
    p.setCollisionFilterPair(teeId, terrainId, -1, -1, 0)
    for link_index in range(-1, 2):
        p.setCollisionFilterPair(iiwaId, terrainId, link_index, -1, 0)
    for link_index in range(p.getNumJoints(iiwaId)):
        p.setCollisionFilterPair(iiwaId, teeId, link_index, -1, 0)
    return iiwaId, ballId, terrainId, hole_pos
