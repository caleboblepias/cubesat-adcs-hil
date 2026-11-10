FRAC_BITS = 16
SCALE = 1 << FRAC_BITS


# ================================================================
# Fixed-point helpers
# ================================================================

def to_fixed(x):
    return int(round(x * SCALE))


def from_fixed(x):
    return x / SCALE


def qmul(a, b):
    """
    Match RTL:

        product = a * b
        result  = product >>> 16

    Python right shift on negative integers is arithmetic.
    """
    return (a * b) >> FRAC_BITS


def trunc_div(a, b):
    """
    Match SystemVerilog signed integer division:
    truncate toward zero.

    Do NOT use Python // here because // rounds toward -infinity.
    """
    if a >= 0:
        return a // b
    else:
        return -((-a) // b)


# ================================================================
# Rigid-body dynamics
#
# This mirrors rigid_body_dynamics.sv exactly.
# ================================================================

def rigid_body_dynamics(
    wx, wy, wz,
    tx, ty, tz,
    Ixx, Iyy, Izz,
    Ixx_inv, Iyy_inv, Izz_inv
):

    cross_x = qmul(
        Izz - Iyy,
        qmul(wy, wz)
    )

    cross_y = qmul(
        Ixx - Izz,
        qmul(wz, wx)
    )

    cross_z = qmul(
        Iyy - Ixx,
        qmul(wx, wy)
    )

    diff_x = tx - cross_x
    diff_y = ty - cross_y
    diff_z = tz - cross_z

    wx_dot = qmul(Ixx_inv, diff_x)
    wy_dot = qmul(Iyy_inv, diff_y)
    wz_dot = qmul(Izz_inv, diff_z)

    return wx_dot, wy_dot, wz_dot


# ================================================================
# Bit-accurate RK4
#
# Mirrors rk4.sv exactly.
# ================================================================

def rk4(
    wx, wy, wz,
    tx, ty, tz,
    dt,
    Ixx, Iyy, Izz,
    Ixx_inv, Iyy_inv, Izz_inv
):

    # ------------------------------------------------------------
    # K1
    # ------------------------------------------------------------

    k1 = rigid_body_dynamics(
        wx, wy, wz,
        tx, ty, tz,
        Ixx, Iyy, Izz,
        Ixx_inv, Iyy_inv, Izz_inv
    )


    # ------------------------------------------------------------
    # x2 = w + (dt*k1)/2
    #
    # RTL:
    #     w + (qmul(dt, k1) >>> 1)
    # ------------------------------------------------------------

    x2 = (
        wx + (qmul(dt, k1[0]) >> 1),
        wy + (qmul(dt, k1[1]) >> 1),
        wz + (qmul(dt, k1[2]) >> 1)
    )


    # ------------------------------------------------------------
    # K2
    # ------------------------------------------------------------

    k2 = rigid_body_dynamics(
        x2[0], x2[1], x2[2],
        tx, ty, tz,
        Ixx, Iyy, Izz,
        Ixx_inv, Iyy_inv, Izz_inv
    )


    # ------------------------------------------------------------
    # x3 = w + (dt*k2)/2
    # ------------------------------------------------------------

    x3 = (
        wx + (qmul(dt, k2[0]) >> 1),
        wy + (qmul(dt, k2[1]) >> 1),
        wz + (qmul(dt, k2[2]) >> 1)
    )


    # ------------------------------------------------------------
    # K3
    # ------------------------------------------------------------

    k3 = rigid_body_dynamics(
        x3[0], x3[1], x3[2],
        tx, ty, tz,
        Ixx, Iyy, Izz,
        Ixx_inv, Iyy_inv, Izz_inv
    )


    # ------------------------------------------------------------
    # x4 = w + dt*k3
    # ------------------------------------------------------------

    x4 = (
        wx + qmul(dt, k3[0]),
        wy + qmul(dt, k3[1]),
        wz + qmul(dt, k3[2])
    )


    # ------------------------------------------------------------
    # K4
    # ------------------------------------------------------------

    k4 = rigid_body_dynamics(
        x4[0], x4[1], x4[2],
        tx, ty, tz,
        Ixx, Iyy, Izz,
        Ixx_inv, Iyy_inv, Izz_inv
    )


    # ------------------------------------------------------------
    # Weighted derivative sums
    #
    # k1 + 2*k2 + 2*k3 + k4
    # ------------------------------------------------------------

    sum_x = k1[0] + 2*k2[0] + 2*k3[0] + k4[0]
    sum_y = k1[1] + 2*k2[1] + 2*k3[1]+ k4[1]
    sum_z = k1[2] + 2*k2[2] + 2*k3[2] + k4[2]


    # ------------------------------------------------------------
    # RTL divides weighted sum by 6 BEFORE qmul(dt,...).
    #
    # SystemVerilog signed division truncates toward zero.
    # ------------------------------------------------------------

    avg_x = trunc_div(sum_x, 6)
    avg_y = trunc_div(sum_y, 6)
    avg_z = trunc_div(sum_z, 6)


    # ------------------------------------------------------------
    # Final state
    # ------------------------------------------------------------

    wx_next = wx + qmul(dt, avg_x)
    wy_next = wy + qmul(dt, avg_y)
    wz_next = wz + qmul(dt, avg_z)


    return {
        "k1": k1,
        "x2": x2,
        "k2": k2,
        "x3": x3,
        "k3": k3,
        "x4": x4,
        "k4": k4,

        "weighted_sum": (
            sum_x,
            sum_y,
            sum_z
        ),

        "average": (
            avg_x,
            avg_y,
            avg_z
        ),

        "next": (
            wx_next,
            wy_next,
            wz_next
        )
    }


# ================================================================
# Spacecraft constants
# ================================================================

Ixx = to_fixed(0.02)
Iyy = to_fixed(0.02)
Izz = to_fixed(0.01)

Ixx_inv = to_fixed(50.0)
Iyy_inv = to_fixed(50.0)
Izz_inv = to_fixed(100.0)

dt = to_fixed(0.01)


# ================================================================
# Test vectors
# ================================================================

tests = [

    # ------------------------------------------------------------
    # TEST 0
    # Zero state, zero torque
    # ------------------------------------------------------------

    {
        "name": "zero",
        "w": (
            to_fixed(0.0),
            to_fixed(0.0),
            to_fixed(0.0)
        ),
        "torque": (
            to_fixed(0.0),
            to_fixed(0.0),
            to_fixed(0.0)
        )
    },


    # ------------------------------------------------------------
    # TEST 1
    # Torque only from zero angular velocity
    # ------------------------------------------------------------

    {
        "name": "torque_only",
        "w": (
            to_fixed(0.0),
            to_fixed(0.0),
            to_fixed(0.0)
        ),
        "torque": (
            to_fixed(0.001),
            to_fixed(-0.0015),
            to_fixed(0.002)
        )
    },


    # ------------------------------------------------------------
    # TEST 2
    # Rotational coupling only
    # ------------------------------------------------------------

    {
        "name": "coupling_only",
        "w": (
            to_fixed(0.1),
            to_fixed(-0.2),
            to_fixed(0.3)
        ),
        "torque": (
            to_fixed(0.0),
            to_fixed(0.0),
            to_fixed(0.0)
        )
    },


    # ------------------------------------------------------------
    # TEST 3
    # Full dynamics
    # ------------------------------------------------------------

    {
        "name": "full_dynamics",
        "w": (
            to_fixed(0.1),
            to_fixed(-0.2),
            to_fixed(0.3)
        ),
        "torque": (
            to_fixed(0.001),
            to_fixed(-0.0015),
            to_fixed(0.002)
        )
    },


    # ------------------------------------------------------------
    # TEST 4
    # Mixed signs
    # ------------------------------------------------------------

    {
        "name": "mixed_signs",
        "w": (
            to_fixed(-0.4),
            to_fixed(0.15),
            to_fixed(-0.25)
        ),
        "torque": (
            to_fixed(-0.001),
            to_fixed(0.0005),
            to_fixed(-0.0015)
        )
    }

]


# ================================================================
# Print vectors
# ================================================================

print("==============================================")
print("Q16.16 RK4 GOLDEN VECTORS")
print("==============================================")

print()
print("CONSTANTS")
print(f"dt       = {dt}")
print(f"Ixx      = {Ixx}")
print(f"Iyy      = {Iyy}")
print(f"Izz      = {Izz}")
print(f"Ixx_inv  = {Ixx_inv}")
print(f"Iyy_inv  = {Iyy_inv}")
print(f"Izz_inv  = {Izz_inv}")


for index, test in enumerate(tests):

    wx, wy, wz = test["w"]
    tx, ty, tz = test["torque"]

    result = rk4(
        wx, wy, wz,
        tx, ty, tz,
        dt,
        Ixx, Iyy, Izz,
        Ixx_inv, Iyy_inv, Izz_inv
    )

    print()
    print("==============================================")
    print(f"TEST {index}: {test['name']}")
    print("==============================================")

    print(f"w      = {test['w']}")
    print(f"torque = {test['torque']}")

    print()
    print(f"k1 = {result['k1']}")
    print(f"x2 = {result['x2']}")

    print(f"k2 = {result['k2']}")
    print(f"x3 = {result['x3']}")

    print(f"k3 = {result['k3']}")
    print(f"x4 = {result['x4']}")

    print(f"k4 = {result['k4']}")

    print()
    print(f"weighted sum = {result['weighted_sum']}")
    print(f"average      = {result['average']}")

    print()
    print(f"next fixed = {result['next']}")

    print(
        "next float = "
        f"({from_fixed(result['next'][0]):.10f}, "
        f"{from_fixed(result['next'][1]):.10f}, "
        f"{from_fixed(result['next'][2]):.10f})"
    ) 
