import numpy as np

class PIDController:

    def __init__(self, Kp, Ki, Kd, integral_limit=0.1):

        self.Kp = Kp
        self.Ki = Ki
        self.Kd = Kd

        self.integral_error = np.zeros(3)

        self.integral_limit = integral_limit

    def compute(self, e, w, dt):

        # integrate error
        self.integral_error += e * dt

        
        # clamp integral (anti-windup)
        self.integral = np.clip(
            self.integral_error,
            -self.integral_limit,
            self.integral_limit
        )
        

        torque = (
            -self.Kp @ e
            -self.Kd @ w
            -self.Ki @ self.integral_error
        )

        return torque