import numpy as np


# wheel allocation matrix formation ------------------------------------------------
# inputs:
#  - wheel_axes [nx3]
# outputs:
#  - A [3xn]
def build_allocation_matrix(wheel_axes):

    n = wheel_axes.shape[0]

    A = np.zeros((3, n))

    for i in range(n):
        A[:, i] = wheel_axes[i]

    return A


# allocate wheel torques ----------------------------------------------------
# inputs:
#  - torque_cmd [3x1]
#  - A_pinv [3xn]
# outputs:
#  - wheel_torque [nx1]
def allocate_wheel_torques(torque_cmd, A_pinv):

    # Moore Penrose pseudoinverse allocation
    wheel_torque = A_pinv @ torque_cmd

    return wheel_torque


# propagate wheel speeds ----------------------------------------------------
# uses Euler integration, linear relationship
# inputs:
#  - w : current wheel speeds [nx1]
#  - wheel_torque : allocated wheel torques [nx1]
#  - Jw : wheel inertia (constant)
# outputs:
#  - w_next : estimated wheel speeds after dt [nx1]
def propagate_wheel_speeds(w, wheel_torque, Jw, dt):

    w_dot = wheel_torque / Jw

    w_next = w + w_dot * dt

    return w_next


def compute_body_torque(w_next, w, Jw, A, dt):

    delta_w = w_next - w

    torque_body = -A @ (Jw * delta_w / dt)

    return torque_body

