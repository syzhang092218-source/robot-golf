import pybullet as p
import time
import numpy as np

from robot_golf.env import create_env
from robot_golf.env.prefix import GRAVITY, BALL_COLLISION_RADIUS
from robot_golf.env.ball import ball_aerodynamics
from robot_golf.planner.robot import get_club_init_transform
from robot_golf.planner.ball import solve_init_velocity


if __name__ == "__main__":
    ball_pos = np.array([0.0, -0.5, 0.35])
    robot_id, ball_id, terrainId, hole_pos = create_env(ball_pos=ball_pos, GUI=True)

    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1
    p.setGravity(0, 0, GRAVITY)
    p.changeDynamics(ball_id, linkIndex=-1, linearDamping=0, contactDamping=1, contactStiffness=100, restitution=0.98, lateralFriction=0., spinningFriction=0., rollingFriction=0.)
    p.setTimeStep(1 / 100)
    p.changeDynamics(terrainId, linkIndex=-1, contactDamping=1, contactStiffness=100, restitution=0.98)

    # wait for the ball to land on the tee holder
    for _ in range(100):
        p.stepSimulation()

    v_ball_init, dt = solve_init_velocity(
        hole_pos=hole_pos - np.array(p.getBasePositionAndOrientation(ball_id)[0]),
        hole_radius=0.1,
        ball_radius=0.08,
        max_land_angle=np.pi / 2 * 0.15,
        v_constraint=[np.array([-100000, -100000, 0]), np.array([100000, 100000, 100000])]
    )
    p.setTimeStep(0.01)

    # v_ball_init = np.array([1, 0, 0])

    ball_center = np.array(p.getBasePositionAndOrientation(ball_id)[0])
    # ball_bounding_box = np.array(p.getAABB(ball_id))
    # ball_radius = (ball_bounding_box[1, :] - ball_bounding_box[0, :]).mean() / 2
    contact_point = ball_center - BALL_COLLISION_RADIUS * v_ball_init / np.linalg.norm(v_ball_init)
    X_WC = get_club_init_transform(contact_point, v_ball_init, robot_id)
    # p.addUserDebugPoints([X_WC[0]], [[1, 0, 0]], pointSize=5)

    joint_pos = p.calculateInverseKinematics(robot_id, club_link_id, X_WC[0], X_WC[1])
    for i in range(n_joints - 2):
        p.resetJointState(robot_id, i, joint_pos[i])

    a = 0

# i    for i in range(n_joints - 3):
#         p.createConstraint(robot_id, i, robot_id, i + 1, p.JOINT_FIXED, [0, 0, 1], [])


    # p.calculateInverseKinematics(robot_id, club_link_id, club_init_X[0], club_init_X[1])

    # p.setJointMotorControl2(robot_id, club_link_id, p.POSITION_CONTROL, )
    # p.resetBaseVelocity(ball_id, v_ball_init)

    while True:
        p.stepSimulation()
        for i in range(n_joints - 3):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0)
        p.setJointMotorControl2(robot_id, n_joints - 3, p.POSITION_CONTROL, joint_pos[-1], -20)
        p.setJointMotorControl2(robot_id, n_joints - 4, p.POSITION_CONTROL, joint_pos[-2], 50)
        ball_aerodynamics(ball_id)
        p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=5)
        p.addUserDebugPoints([p.getLinkState(robot_id, club_link_id)[0]], [[0, 0, 1]], pointSize=5)
