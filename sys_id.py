import pybullet as p

from robot_golf.simulator import set_simulator


def get_data():
    robot_id, ball_id, joint_pos, v_ball_init = set_simulator()
    n_joints = p.getNumJoints(robot_id)
    club_link_id = n_joints - 1
    for _ in range(5000):
        p.stepSimulation()
        for i in range(1, n_joints - 2):
            p.setJointMotorControl2(robot_id, i, p.POSITION_CONTROL, joint_pos[i], 0, maxVelocity=0.1)
    for _ in range(5000):
        p.stepSimulation()
        p.setJointMotorControl2(robot_id, 0, p.POSITION_CONTROL, joint_pos[0], 0, maxVelocity=0.05)


