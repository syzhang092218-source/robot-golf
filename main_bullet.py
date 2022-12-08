import pybullet as p
import time

from robot_golf.env import create_env


if __name__ == "__main__":
    robot_id, ball_id = create_env(ball_pos=[-0.3, -0.3, 0.35], GUI=True)
    p.setGravity(0, 0, -9.8)
    p.resetBaseVelocity(ball_id, [5, 0, 0])
    while True:
        time.sleep(0.003)
        p.setTimeStep(1/240)
        p.stepSimulation()
        p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=5)
        p.addUserDebugPoints([p.getLinkState(robot_id, 7)[0]], [[0, 0, 1]], pointSize=5)
