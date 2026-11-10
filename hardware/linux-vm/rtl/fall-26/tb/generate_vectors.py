FRAC_BITS = 16
SCALE = 1 << FRAC_BITS

def to_fixed(x):
    return round(x * SCALE)

def from_fixed(x):
    return x / SCALE

def qmul(a, b):
    return (a * b) >> FRAC_BITS


def dynamics(wx, wy, wz, tx, ty, tz):
    # Match the FPGA's Q16.16 values exactly
    wx = to_fixed(wx)
    wy = to_fixed(wy)
    wz = to_fixed(wz)

    tx = to_fixed(tx)
    ty = to_fixed(ty)
    tz = to_fixed(tz)

    Ixx = to_fixed(0.02)
    Iyy = to_fixed(0.02)
    Izz = to_fixed(0.01)

    Ixx_inv = to_fixed(50.0)
    Iyy_inv = to_fixed(50.0)
    Izz_inv = to_fixed(100.0)

    cross_x = qmul(Izz - Iyy, qmul(wy, wz))
    cross_y = qmul(Ixx - Izz, qmul(wz, wx))
    cross_z = qmul(Iyy - Ixx, qmul(wx, wy))

    wx_dot = qmul(Ixx_inv, tx - cross_x)
    wy_dot = qmul(Iyy_inv, ty - cross_y)
    wz_dot = qmul(Izz_inv, tz - cross_z)

    return wx, wy, wz, tx, ty, tz, wx_dot, wy_dot, wz_dot


tests = [
    # torque only
    (0.0, 0.0, 0.0,
     0.001, -0.0015, 0.002),

    # rotational coupling only
    (0.1, -0.2, 0.3,
     0.0, 0.0, 0.0),

    # complete dynamics
    (0.1, -0.2, 0.3,
     0.001, -0.0015, 0.002),

    # different signs/magnitudes
    (-0.4, 0.15, -0.25,
     -0.001, 0.0005, -0.0015),
]

for i, test in enumerate(tests, 1):
    result = dynamics(*test)

    print(f"TEST {i}")
    print("inputs:  ", result[:6])
    print("outputs: ", result[6:])
    print("outputs float:",
          tuple(from_fixed(x) for x in result[6:]))
    print()
