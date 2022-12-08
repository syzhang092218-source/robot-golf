import pybullet as p
import time
import numpy as np

from robot_golf.env import create_env
from robot_golf.env.prefix import GRAVITY
from robot_golf.env.ball import ball_aerodynamics
from robot_golf.planner.ball_shooting import solve_init_velocity


if __name__ == "__main__":
    ball_pos = np.array([0.0, -0.5, 0.35])
    robot_id, ball_id, terrainId, hole_pos = create_env(ball_pos=ball_pos, GUI=True)
    p.setGravity(0, 0, GRAVITY)
    p.changeDynamics(ball_id, linkIndex=-1, linearDamping=0, contactDamping=1, contactStiffness=100, restitution=0.98)
    p.setTimeStep(1 / 100)
    p.changeDynamics(terrainId, linkIndex=-1, contactDamping=1, contactStiffness=100, restitution=0.98)

    # wait for the ball to land on the tee holder
    for _ in range(100):
        p.stepSimulation()

    v_init, dt = solve_init_velocity(
        hole_pos=hole_pos - np.array(p.getBasePositionAndOrientation(ball_id)[0]),
        hole_radius=0.1,
        ball_radius=0.08,
        max_land_angle=np.pi / 2 * 0.15,
        v_constraint=[np.array([-100000, -100000, 0]), np.array([100000, 100000, 100000])]
    )
    p.setTimeStep(dt)
    p.resetBaseVelocity(ball_id, v_init)

    while True:
        p.stepSimulation()
        ball_aerodynamics(ball_id)

        p.addUserDebugPoints([p.getBasePositionAndOrientation(ball_id)[0]], [[1, 0, 0]], pointSize=5)
        p.addUserDebugPoints([p.getLinkState(robot_id, 7)[0]], [[0, 0, 1]], pointSize=5)
