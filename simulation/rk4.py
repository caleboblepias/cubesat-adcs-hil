import numpy as np

# rk4 ------------------------------------------------
# state estimation using rk4 model
# inputs: 
#  - f : derivative of function
#  - x : current state
#  - dt : timestep
#  - *args : extra params for f
# outputs:
#  - x_next : estimated state after timestep
def rk4(f, x, dt, *args):

    k1 = f(x, *args)
    k2 = f(x + 0.5 * dt * k1, *args)
    k3 = f(x + 0.5 * dt * k2, *args)
    k4 = f(x + dt * k3, *args)

    x_next = x + (dt/6) * (k1 + 2*k2 + 2*k3 + k4)

    return x_next