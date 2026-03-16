import numpy as np


# quaternion conjugate ------------------------------------------------
def quat_conjugate(q):

    q0, q1, q2, q3 = q
    return np.array([q0, -q1, -q2, -q3])


# quaternion multiplication -------------------------------------------
def quat_multiply(q1, q2):

    w1, x1, y1, z1 = q1
    w2, x2, y2, z2 = q2

    return np.array([
        w1*w2 - x1*x2 - y1*y2 - z1*z2,
        w1*x2 + x1*w2 + y1*z2 - z1*y2,
        w1*y2 - x1*z2 + y1*w2 + z1*x2,
        w1*z2 + x1*y2 - y1*x2 + z1*w2
    ])


# attitude error ------------------------------------------------------
# inputs:
#  - q_des : desired quaternion
#  - q_curr : current quaternion
# outputs:
#  - q_err : error quaternion
#  - e : vector error (used for control)
def quaternion_error(q_des, q_curr):

    q_curr_inv = quat_conjugate(q_curr)

    q_err = quat_multiply(q_des, q_curr_inv)

    

    # vector part is attitude error
    e = q_err[1:4]

    return q_err, e