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
    return iiwaId, ballId


if __name__ == "__main__":
    robot_id, ball_id = create_env(ball_pos=[0.3, 0.3, 0.3], GUI=True)
    p.setGravity(0, 0, -9.8)
    while True:
        p.stepSimulation()

    # def __str__(self):
    #     return self.robot_name_list
    #
    # #
    # # def init_new_problem(self, index=None):
    # #     '''
    # #     Initialize a new eval problem
    # #     '''
    # #     if index is None:
    # #         self.index = self.episode_i
    # #     else:
    # #         self.index = index
    # #
    # #     obstacles, start, goal, path = self.problems[index]
    # #
    # #     self.episode_i += 1
    # #     self.episode_i = (self.episode_i) % len(self.order)
    # #     self.collision_check_count = 0
    # #
    # #     self.reset_env()
    # #
    # #     self.collision_point = None
    # #
    # #     self.obstacles = obstacles
    # #     self.init_state = start
    # #     self.goal_state = goal
    # #     self.path = path
    # #     self.obs_ids = []
    # #
    # #     for halfExtents, basePosition in obstacles:
    # #         self.obs_ids.append(self.create_voxel(halfExtents, basePosition, [0, 0, 0, 1]))
    # #
    # #     return self.get_problem()
    # #
    # # def init_new_problem_with_config(self, start, goal, obstacles):
    # #     '''
    # #     Initialize a new eval problem
    # #     '''
    # #     self.index = 0
    # #
    # #     self.collision_check_count = 0
    # #     self.reset_env()
    # #     self.collision_point = None
    # #
    # #     self.obstacles = obstacles
    # #     self.init_state = start
    # #     self.goal_state = goal
    # #     self.obs_ids = []
    # #
    # #     for halfExtents, basePosition in obstacles:
    # #         self.obs_ids.append(self.create_voxel(halfExtents, basePosition, [0, 0, 0, 1]))
    # #
    # #     return self.get_problem()
    # #
    # def reset_env(self, enable_object=False, fixed=True):
    #     p.resetSimulation()
    #
    #     plane = p.createCollisionShape(p.GEOM_PLANE)
    #     self.plane = p.createMultiBody(0, plane)
    #     for robot_name in self.robot_name_list:
    #         if robot_name == "panda":
    #             self.robot_list.append(FrankaPanda())
    #             # p.setCollisionFilterPair(self.robot_list[-1].robotId, self.plane, 1, -1, 0)
    #         else:
    #             raise NotImplementedError(f"Robot {robot_name} not supported yet.")
    #
    #     # TODO initialize object unattached
    #     if enable_object:
    #         if len(self.robot_list) > 1:
    #             raise NotImplementedError
    #         else:
    #             object = p.createCollisionShape(p.GEOM_CYLINDER, radius=0.01, height=np.random.uniform(0.15, 0.25))
    #             objectPos, objectOrn = self.robot_list[0].get_link_PosOrn(self.robot_list[0].body_joints[-1])
    #             objectOrn = p.multiplyTransforms([0, 0, 0], [1/np.sqrt(2), 1/np.sqrt(2), 0, 0], [0, 0, 0], objectOrn)[1]
    #             self.object = p.createMultiBody(0.01, object, basePosition=objectPos, baseOrientation=objectOrn)
    #             self.robot_list[0].end_effector.activate(self.object)
    #
    #     self._generate_obstacle(fixed=fixed)
    #
    #     p.setGravity(0, 0, -10)
    #
    # # ===================== internal module ===========================
    #
    # def _generate_obstacle(self, num=3, fixed=True):
    #     # TODO random posed obstacle
    #     self.obstacle_ids = [self.plane]
    #     # fixed = False
    #     # num = 4
    #     if fixed:
    #         positions = [[0.3, 0.17, 0.3], [-0.4, 0.23, 0.4], [0.0, -0.23, 0.6]]
    #     else:
    #         positions = np.random.uniform(low=(-0.5, -0.5, 0), high=(0.5, 0.5, 1), size=(num, 3))
    #     for ind in range(num):
    #         halfExtents = [0.05, 0.05, 0.1]
    #         basePosition = positions[ind]
    #         baseOrientation = [0, 0, 0, 1]
    #         self.obstacle_ids.append(self._create_voxel(halfExtents, basePosition, baseOrientation, color='random'))
    #
    # def _create_voxel(self, halfExtents, basePosition, baseOrientation, color='random'):
    #     voxColId = p.createCollisionShape(p.GEOM_BOX, halfExtents=halfExtents)
    #     if color == 'random':
    #         voxVisID = p.createVisualShape(shapeType=p.GEOM_BOX,
    #                                        rgbaColor=np.random.uniform(0, 1, size=3).tolist() + [1],
    #                                        # specularColor=[0.4, .4, 0],
    #                                        halfExtents=halfExtents)
    #     else:
    #         voxVisID = p.createVisualShape(shapeType=p.GEOM_BOX,
    #                                        rgbaColor=color,
    #                                        # specularColor=[0.4, .4, 0],
    #                                        halfExtents=halfExtents)
    #     voxId = p.createMultiBody(baseMass=0,
    #                               baseCollisionShapeIndex=voxColId,
    #                               baseVisualShapeIndex=voxVisID,
    #                               basePosition=basePosition,
    #                               baseOrientation=baseOrientation)
    #     return voxId
