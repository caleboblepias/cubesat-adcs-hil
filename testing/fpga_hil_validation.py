import numpy as np
import matplotlib.pyplot as plt

from simulation.rotational_dynamics import imu_emulator


# fpga output

wx_fpga_raw = np.array([
    6514, 6474, 6434, 6394, 6354, 6314, 6274, 6234, 6194, 6154,
    6114, 6074, 6034, 5994, 5954, 5914, 5873, 5832, 5791, 5750,
    5709, 5668, 5627, 5586, 5545, 5504, 5463, 5422, 5381, 5340,
    5299, 5258, 5217, 5176, 5135, 5094, 5053, 5012, 4971, 4930,
    4889, 4848, 4807, 4765, 4723, 4681, 4639, 4597, 4555, 4513,
    4471, 4429, 4387, 4345, 4303, 4261, 4219, 4177, 4135, 4093,
    4051, 4009, 3967, 3925, 3883, 3841, 3799, 3757, 3715, 3673,
    3631, 3589, 3547, 3505, 3463, 3421, 3379, 3337, 3295, 3253,
    3210, 3167, 3124, 3081, 3038, 2995, 2952, 2909, 2866, 2823,
    2780, 2737, 2694, 2651, 2608, 2565, 2522, 2479, 2436, 2393
], dtype=np.int32)

wy_fpga_raw = np.array([
    13126, 13145, 13164, 13183, 13202, 13220, 13238, 13256, 13274, 13292,
    13310, 13328, 13346, 13363, 13380, 13397, 13414, 13431, 13448, 13465,
    13482, 13498, 13514, 13530, 13546, 13562, 13578, 13594, 13610, 13625,
    13640, 13655, 13670, 13685, 13700, 13715, 13730, 13744, 13758, 13772,
    13786, 13800, 13814, 13828, 13842, 13855, 13868, 13881, 13894, 13907,
    13920, 13933, 13946, 13958, 13970, 13982, 13994, 14006, 14018, 14030,
    14042, 14053, 14064, 14075, 14086, 14097, 14108, 14119, 14130, 14140,
    14150, 14160, 14170, 14180, 14190, 14200, 14210, 14219, 14228, 14237,
    14246, 14255, 14264, 14273, 14282, 14290, 14298, 14306, 14314, 14322,
    14330, 14338, 14345, 14352, 14359, 14366, 14373, 14380, 14387, 14394
], dtype=np.int32)

wz_fpga_raw = np.array([
    19656, 19651, 19646, 19641, 19636, 19631, 19626, 19621, 19616, 19611,
    19606, 19601, 19596, 19591, 19586, 19581, 19576, 19572, 19568, 19564,
    19560, 19556, 19552, 19548, 19544, 19540, 19536, 19532, 19528, 19524,
    19520, 19516, 19512, 19508, 19504, 19500, 19496, 19492, 19488, 19484,
    19480, 19476, 19472, 19468, 19464, 19460, 19456, 19452, 19448, 19444,
    19440, 19436, 19432, 19428, 19424, 19420, 19416, 19413, 19410, 19407,
    19404, 19401, 19398, 19395, 19392, 19389, 19386, 19383, 19380, 19377,
    19374, 19371, 19368, 19365, 19362, 19359, 19356, 19353, 19350, 19347,
    19344, 19341, 19338, 19335, 19332, 19329, 19326, 19323, 19320, 19317,
    19314, 19311, 19309, 19307, 19305, 19303, 19301, 19299, 19297, 19295
], dtype=np.int32)


# constants

Q = 65536.0
NUM_STEPS = 100

dt_raw = 655

w_initial_raw = np.array([
    6554,
    13107,
    19661
], dtype=np.int64)

I_raw = np.array([
    65536,
    131072,
    196608
], dtype=np.int64)

I_inv_raw = np.array([
    65536,
    32768,
    21845
], dtype=np.int64)

torque_raw = np.array([
    0,
    0,
    0
], dtype=np.int64)

w_fpga_raw = np.column_stack((
    wx_fpga_raw,
    wy_fpga_raw,
    wz_fpga_raw
)).astype(np.int64)

w_fpga = w_fpga_raw / Q


# bit-width helpers

MASK32 = (1 << 32) - 1
MASK35 = (1 << 35) - 1


def s32(x):
    """
    Reproduce assignment to signed [31:0].
    """
    x = int(x) & MASK32

    if x & (1 << 31):
        x -= (1 << 32)

    return x


def s35(x):
    """
    Reproduce assignment to signed [34:0].
    """
    x = int(x) & MASK35

    if x & (1 << 34):
        x -= (1 << 35)

    return x


# bit-accurate qmul_sequential

def qmul(a, b):
    """
    Numerically equivalent to:

        logic signed [63:0] product;
        product = a * b;
        result = product >>> 16;

    The result is then stored in signed [31:0].
    """

    a = s32(a)
    b = s32(b)

    product = a * b

    return s32(product >> 16)


# bit-accurate divide-by-6

def div6_rtl(x):
    """
    RTL divider operates on magnitude and restores the sign.

    Therefore integer division truncates toward zero.
    """

    x = s35(x)

    if x < 0:
        return s32(-((-x) // 6))

    return s32(x // 6)


# bit-accurate rigid-body dynamics

def rigid_body_q16(w, torque, I, I_inv):

    wx = s32(w[0])
    wy = s32(w[1])
    wz = s32(w[2])

    tx = s32(torque[0])
    ty = s32(torque[1])
    tz = s32(torque[2])

    Ixx = s32(I[0])
    Iyy = s32(I[1])
    Izz = s32(I[2])

    Ixx_inv = s32(I_inv[0])
    Iyy_inv = s32(I_inv[1])
    Izz_inv = s32(I_inv[2])

    # cross products

    wy_wz = qmul(wy, wz)
    wz_wx = qmul(wz, wx)
    wx_wy = qmul(wx, wy)

    # inertia differences

    I_diff_x = s32(Izz - Iyy)
    I_diff_y = s32(Ixx - Izz)
    I_diff_z = s32(Iyy - Ixx)

    # cross-term scaling

    cross_x = qmul(I_diff_x, wy_wz)
    cross_y = qmul(I_diff_y, wz_wx)
    cross_z = qmul(I_diff_z, wx_wy)

    # torque - w x Iw

    diff_x = s32(tx - cross_x)
    diff_y = s32(ty - cross_y)
    diff_z = s32(tz - cross_z)

    # I^-1 * (...)

    wx_dot = qmul(Ixx_inv, diff_x)
    wy_dot = qmul(Iyy_inv, diff_y)
    wz_dot = qmul(Izz_inv, diff_z)

    return np.array([
        wx_dot,
        wy_dot,
        wz_dot
    ], dtype=np.int64)


# bit-accurate rk4

def rk4_q16(w, torque, dt, I, I_inv):

    w0 = np.array([
        s32(w[0]),
        s32(w[1]),
        s32(w[2])
    ], dtype=np.int64)

    # k1 = f(w)

    k1 = rigid_body_q16(
        w0,
        torque,
        I,
        I_inv
    )

    # x2 = w + (dt*k1)/2

    dt_k1 = np.array([
        qmul(dt, k1[0]),
        qmul(dt, k1[1]),
        qmul(dt, k1[2])
    ], dtype=np.int64)

    x2 = np.array([
        s32(w0[0] + (int(dt_k1[0]) >> 1)),
        s32(w0[1] + (int(dt_k1[1]) >> 1)),
        s32(w0[2] + (int(dt_k1[2]) >> 1))
    ], dtype=np.int64)

    # k2 = f(x2)

    k2 = rigid_body_q16(
        x2,
        torque,
        I,
        I_inv
    )

    # x3 = w + (dt*k2)/2

    dt_k2 = np.array([
        qmul(dt, k2[0]),
        qmul(dt, k2[1]),
        qmul(dt, k2[2])
    ], dtype=np.int64)

    x3 = np.array([
        s32(w0[0] + (int(dt_k2[0]) >> 1)),
        s32(w0[1] + (int(dt_k2[1]) >> 1)),
        s32(w0[2] + (int(dt_k2[2]) >> 1))
    ], dtype=np.int64)

    # k3 = f(x3)

    k3 = rigid_body_q16(
        x3,
        torque,
        I,
        I_inv
    )

    # x4 = w + dt*k3

    dt_k3 = np.array([
        qmul(dt, k3[0]),
        qmul(dt, k3[1]),
        qmul(dt, k3[2])
    ], dtype=np.int64)

    x4 = np.array([
        s32(w0[0] + dt_k3[0]),
        s32(w0[1] + dt_k3[1]),
        s32(w0[2] + dt_k3[2])
    ], dtype=np.int64)

    # k4 = f(x4)

    k4 = rigid_body_q16(
        x4,
        torque,
        I,
        I_inv
    )

    # weighted = k1 + 2*k2 + 2*k3 + k4
    # rtl uses signed [34:0]

    weighted = np.array([
        s35(
            int(k1[0])
            + 2 * int(k2[0])
            + 2 * int(k3[0])
            + int(k4[0])
        ),
        s35(
            int(k1[1])
            + 2 * int(k2[1])
            + 2 * int(k3[1])
            + int(k4[1])
        ),
        s35(
            int(k1[2])
            + 2 * int(k2[2])
            + 2 * int(k3[2])
            + int(k4[2])
        )
    ], dtype=np.int64)

    # average = weighted / 6

    average = np.array([
        div6_rtl(weighted[0]),
        div6_rtl(weighted[1]),
        div6_rtl(weighted[2])
    ], dtype=np.int64)

    # delta = dt * average

    delta = np.array([
        qmul(dt, average[0]),
        qmul(dt, average[1]),
        qmul(dt, average[2])
    ], dtype=np.int64)

    # w_next = w + delta

    return np.array([
        s32(w0[0] + delta[0]),
        s32(w0[1] + delta[1]),
        s32(w0[2] + delta[2])
    ], dtype=np.int64)


# floating-point python reference

dt = dt_raw / Q

w_python_current = w_initial_raw.astype(np.float64) / Q

I_python = np.diag(
    I_raw.astype(np.float64) / Q
)

I_inv_python = np.diag(
    I_inv_raw.astype(np.float64) / Q
)

torque_python = torque_raw.astype(np.float64) / Q

w_python = []

for _ in range(NUM_STEPS):

    w_python_current = imu_emulator(
        w_python_current,
        torque_python,
        I_python,
        I_inv_python,
        dt
    )

    w_python.append(w_python_current.copy())

w_python = np.array(w_python)


# bit-accurate rtl reference

w_fixed_current = w_initial_raw.copy()

w_fixed_raw = np.zeros(
    (NUM_STEPS, 3),
    dtype=np.int64
)

for k in range(NUM_STEPS):

    w_fixed_current = rk4_q16(
        w_fixed_current,
        torque_raw,
        dt_raw,
        I_raw,
        I_inv_raw
    )

    w_fixed_raw[k] = w_fixed_current

w_fixed = w_fixed_raw / Q


# floating-point comparison

float_error = w_fpga - w_python
float_error_lsb = float_error * Q

print("=" * 60)
print("Q16.16 FPGA vs Floating-Point Python")
print("=" * 60)

print("\nInitial state:")
print(w_initial_raw / Q)

print("\nFinal FPGA state:")
print(w_fpga[-1])

print("\nFinal Python state:")
print(w_python[-1])

print("\nFinal error:")
print(float_error[-1])

print("\nFinal error in Q16.16 LSBs:")
print(float_error_lsb[-1])

print("\nMaximum absolute error:")
print(np.max(np.abs(float_error), axis=0))

print("\nMaximum absolute error in Q16.16 LSBs:")
print(np.max(np.abs(float_error_lsb), axis=0))

print("\n" + "=" * 60)
print("Floating-point error progression")
print("=" * 60)

for step in range(10, NUM_STEPS + 1, 10):

    e = float_error_lsb[step - 1]

    print(
        f"Step {step:3d}: "
        f"wx = {e[0]:8.3f} LSB, "
        f"wy = {e[1]:8.3f} LSB, "
        f"wz = {e[2]:8.3f} LSB"
    )


# bit-accurate comparison

rtl_error = (
    w_fpga_raw.astype(np.int64)
    - w_fixed_raw
)

print("\n" + "=" * 60)
print("Q16.16 FPGA vs Q16.16 Python")
print("=" * 60)

print("\nFinal Q16.16 FPGA:")
print(w_fpga_raw[-1])

print("\nFinal Q16.16 Python:")
print(w_fixed_raw[-1])

print("\nFinal difference:")
print(rtl_error[-1])

print("\nMaximum absolute difference:")
print(np.max(np.abs(rtl_error), axis=0))

exact_samples = np.sum(rtl_error == 0)
total_samples = rtl_error.size

print(
    f"\nExact samples: "
    f"{exact_samples}/{total_samples}"
)

if np.all(rtl_error == 0):

    print("\nRESULT: bit-exact match")

else:

    print("\nRESULT: mismatch")

    mismatch = np.argwhere(rtl_error != 0)

    print("\nFirst mismatches:")

    for row in mismatch[:10]:

        step = row[0]
        axis = row[1]

        axis_name = ["wx", "wy", "wz"][axis]

        print(
            f"Step {step + 1:3d}, "
            f"{axis_name}: "
            f"FPGA={w_fpga_raw[step, axis]}, "
            f"model={w_fixed_raw[step, axis]}, "
            f"diff={rtl_error[step, axis]}"
        )


# plot trajectories

time = np.arange(
    1,
    NUM_STEPS + 1
) * dt

plt.figure(figsize=(10, 6))

plt.plot(
    time,
    w_python[:, 0],
    label="Python wx"
)

plt.plot(
    time,
    w_python[:, 1],
    label="Python wy"
)

plt.plot(
    time,
    w_python[:, 2],
    label="Python wz"
)

plt.plot(
    time,
    w_fpga[:, 0],
    "--",
    label="FPGA wx"
)

plt.plot(
    time,
    w_fpga[:, 1],
    "--",
    label="FPGA wy"
)

plt.plot(
    time,
    w_fpga[:, 2],
    "--",
    label="FPGA wz"
)

plt.xlabel("Time (s)")
plt.ylabel("Angular velocity (rad/s)")
plt.title("Floating-Point Python vs Q16.16 FPGA")
plt.legend()
plt.grid()

plt.tight_layout()


# floating-point numerical error

plt.figure(figsize=(10, 5))

plt.plot(
    time,
    float_error_lsb[:, 0],
    label="wx error"
)

plt.plot(
    time,
    float_error_lsb[:, 1],
    label="wy error"
)

plt.plot(
    time,
    float_error_lsb[:, 2],
    label="wz error"
)

plt.xlabel("Time (s)")
plt.ylabel("Error (Q16.16 LSBs)")
plt.title("Q16.16 FPGA vs Floating-Point Python Error")
plt.legend()
plt.grid()

plt.tight_layout()


# fpga vs bit-accurate rtl model

plt.figure(figsize=(10, 5))

plt.plot(
    time,
    rtl_error[:, 0],
    label="wx"
)

plt.plot(
    time,
    rtl_error[:, 1],
    label="wy"
)

plt.plot(
    time,
    rtl_error[:, 2],
    label="wz"
)

plt.axhline(0)

plt.xlabel("Time (s)")
plt.ylabel("Difference (Q16.16 LSBs)")
plt.title("Q16.16 FPGA vs Q16.16 Python")
plt.legend()
plt.grid()

plt.tight_layout()

plt.show()