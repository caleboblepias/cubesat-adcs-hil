import numpy as np
from rk4 import rk4


# quaternion derivative ------------------------------------------------
# inputs:
#  - q : current quaternion [4x1]
#  - w : angular velocity [3x1]
# outputs:
#  - q_dot : quaternion derivative
def quaternion_dynamics(q, w):

    wx, wy, wz = w

    Omega = np.array([
        [0,   -wx, -wy, -wz],
        [wx,   0,   wz, -wy],
        [wy,  -wz,  0,   wx],
        [wz,   wy, -wx,  0 ]
    ])

    q_dot = 0.5 * Omega @ q

    return q_dot


# quaternion propagation -----------------------------------------------
# uses RK4
# inputs:
#  - q : current quaternion
#  - w : angular velocity
#  - dt : timestep
# outputs:
#  - q_next : propagated quaternion
def propagate_quaternion(q, w, dt):

    q_next = rk4(quaternion_dynamics, q, dt, w)

    # normalize to avoid numerical drift
    q_next = q_next / np.linalg.norm(q_next)

    return q_next