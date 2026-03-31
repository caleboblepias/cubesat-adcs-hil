import numpy as np
from rk4 import rk4

# rigid body dynamics --------------------------------------------
# calculates Euler's rotation equation each timestep
# inputs: 
#  - wc : current angular velocity, [3x1]
#  - torque : total torque applied, [3x1]
#  - I : inertial tensor
#  - I_inv : inertial tensor inverse)
# outputs:
#  - w : next angular velocity, [3x1]
def rigid_body_dynamics(wc, torque, I, I_inv):

    w_dot = I_inv @ (torque - np.cross(wc, I @ wc))

    return w_dot

# imu emulator ------------------------------------------------
# uses rk4 integrator to estimate state
# inputs: 
#  - wc : current angular velocity, [3x1]
#  - torque : total torque applied, [3x1]
#  - I : inertial tensor
#  - I_inv : inertial tensor inverse
#  - dt : timestep definining control loop frequency, seconds
# outputs:
#  - w : next angular velocity, [3x1]
def imu_emulator(wc, torque, I, I_inv, dt):

    w_next = rk4(rigid_body_dynamics, wc, dt, torque, I, I_inv)

    return w_next
