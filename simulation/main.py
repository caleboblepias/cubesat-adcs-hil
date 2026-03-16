import numpy as np
import matplotlib.pyplot as plt

from rotational_dynamics import imu_emulator
from quaternion_formation import propagate_quaternion
from attitude_error import quaternion_error
from controller import PIDController
from reaction_wheels import allocate_wheel_torques
from reaction_wheels import propagate_wheel_speeds
from reaction_wheels import compute_body_torque

# logging
time_log = []
w_log = []
q_log = []
wheel_log = []
error_log = []

# simulation parameters
dt = 0.01
sim_time = 200
steps = int(sim_time / dt)

# spacecraft inertia
I = np.diag([0.02, 0.02, 0.01])
I_inv = np.linalg.inv(I)

# reaction wheel configuration
wheel_axes = np.array([
    [1, 0, 0],
    [0, 1, 0],
    [0, 0, 1],
    [1, 1, 1] / np.sqrt(3)
])

A = wheel_axes.T
A_pinv = np.linalg.pinv(A)

Jw = np.array([1e-5, 1e-5, 1e-5, 1e-5])
wheel_speeds = np.zeros(4)

# max wheel speeds
omega_max = 6000 * 2*np.pi/60 # 6000 RPM = ~628 rad/s

# max torque
tau_max = 0.002 # 2 mN * m

# controller gains
Kp = np.diag([0.005, 0.005, 0.005])
Kd = np.diag([0.003, 0.003, 0.003])
Ki = np.diag([0.0001, 0.0001, 0.0001])
#Ki = np.diag([0, 0, 0])

controller = PIDController(Kp, Ki, Kd)

# initial state
w = np.zeros(3)

q = np.array([1, 0, 0, 0])   # current attitude

#q_des = np.array([0, 0, 0, 1])  # desired attitude 180 deg turn
q_des = np.array([0.9239, 0, 0, 0.3827]) # 45 deg turn

# simulation loop
for k in range(steps):

    # attitude error
    q_err, e = quaternion_error(q_des, q)

    # control torque
    torque_cmd = controller.compute(e, w, dt)

    # wheel allocation
    wheel_torque = -allocate_wheel_torques(torque_cmd, A_pinv)

    # clipping for actuator realism
    wheel_torque = np.clip(wheel_torque, -tau_max, tau_max)

    # saturation masking for each wheel (meaning, can we still apply torque)
    sat_mask = (
    ((wheel_speeds >= omega_max) & (wheel_torque > 0)) |
    ((wheel_speeds <= -omega_max) & (wheel_torque < 0))
    )

    wheel_torque[sat_mask] = 0

    # propagate wheel speeds
    wheel_speeds_next = propagate_wheel_speeds(wheel_speeds, wheel_torque, Jw, dt)

    # clipping for actuator realism
    wheel_speeds_next = np.clip(wheel_speeds_next, -omega_max, omega_max)

    # actual body torque
    torque_body = compute_body_torque(wheel_speeds_next, wheel_speeds, Jw, A, dt)

    # rigid body propagation
    w = imu_emulator(w, torque_body, I, I_inv, dt)

    # quaternion propagation
    q = propagate_quaternion(q, w, dt)

    wheel_speeds = wheel_speeds_next

    # compute true attitude error angle
    dot = abs(np.dot(q_des, q))
    dot = np.clip(dot, -1.0, 1.0)
    theta = 2 * np.arccos(dot)

    # log results
    if k % 100 == 0:
        print("t:", k*dt)
        print("attitude:", q)
        print("angular velocity:", w)
        print("wheel speeds:", wheel_speeds)
        print()

    # logging
    time_log.append(k * dt)
    w_log.append(w.copy())
    q_log.append(q.copy())
    wheel_log.append(wheel_speeds.copy())
    error_log.append(theta)
    


# visualize
time_log = np.array(time_log)
w_log = np.array(w_log)
q_log = np.array(q_log)
wheel_log = np.array(wheel_log)
error_log = np.array(error_log)

fig, axs = plt.subplots(4, 1, figsize=(10, 12), sharex=True)


# angular velocity
axs[0].plot(time_log, w_log[:,0], label="wx")
axs[0].plot(time_log, w_log[:,1], label="wy")
axs[0].plot(time_log, w_log[:,2], label="wz")
axs[0].set_title("Angular Velocity")
axs[0].set_ylabel("rad/s")
axs[0].legend()
axs[0].grid()


# quaternion
axs[1].plot(time_log, q_log[:,0], label="q0")
axs[1].plot(time_log, q_log[:,1], label="q1")
axs[1].plot(time_log, q_log[:,2], label="q2")
axs[1].plot(time_log, q_log[:,3], label="q3")
axs[1].set_title("Quaternion")
axs[1].legend()
axs[1].grid()


# wheel speeds
for i in range(wheel_log.shape[1]):
    axs[2].plot(time_log, wheel_log[:,i], label=f"wheel {i}")
axs[2].set_title("Reaction Wheel Speeds")
axs[2].set_ylabel("rad/s")
axs[2].legend()
axs[2].grid()


# attitude error
axs[3].plot(time_log, error_log)
axs[3].set_title("Attitude Error Magnitude")
axs[3].set_xlabel("Time (s)")
axs[3].set_ylabel("error")
axs[3].grid()


plt.tight_layout()
plt.show()