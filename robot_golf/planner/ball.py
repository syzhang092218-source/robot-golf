import casadi
import numpy as np

from ..env.ball import ball_flying_dyn


def solve_init_velocity(hole_pos, hole_radius, ball_radius, max_land_angle, v_constraint):
    opti = casadi.Opti()
    v_init = opti.variable(3)
    dt = opti.variable(1)
    opti.minimize(dt)

    # velocity constraint
    opti.subject_to(v_init >= v_constraint[0])
    opti.subject_to(v_init <= v_constraint[1])

    # time step constraint
    opti.subject_to(dt >= 0)

    # construct waypoints
    steps = 300
    x0 = casadi.MX.zeros(6)
    x0[3:] = v_init
    x = [x0]
    for _ in range(steps):
        x.append(x[-1] + ball_flying_dyn(x[-1]) * dt)

    # landing point constraint
    radius_diff = hole_radius - ball_radius
    assert radius_diff > 0
    opti.subject_to(x[-1][:2] >= hole_pos[:2] - radius_diff)
    opti.subject_to(x[-1][:2] <= hole_pos[:2] + radius_diff)
    opti.subject_to(x[-1][5] <= 0)
    opti.subject_to(x[-1][2] <= hole_pos[2] + 0.0001)
    opti.subject_to(x[-1][2] >= hole_pos[2] - 0.0001)

    # landing angle constraint
    opti.subject_to(x[-1][3]**2 + x[-1][4]**2 <= np.tan(max_land_angle)**2 * x[-1][5]**2)

    # solve
    p_opts = {"expand": False}
    s_opts = {"max_iter": 10000}
    opti.solver('ipopt', p_opts, s_opts)
    # opti.debug.value
    sol = opti.solve()

    v_init = sol.value(v_init)
    dt = sol.value(dt)
    print(f'v_init: {v_init}')
    print(f'dt: {dt}')

    return v_init, dt
