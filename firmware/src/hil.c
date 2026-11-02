
#include "hil.h"
#include "spi.h"

#define HIL_CMD_WRITE  0x01
#define HIL_CMD_READ   0x02

#define HIL_REG_STATUS  0x00
#define HIL_REG_CONTROL 0x01

#define HIL_REG_TX      0x10
#define HIL_REG_TY      0x14
#define HIL_REG_TZ      0x18

#define HIL_REG_WX      0x20
#define HIL_REG_WY      0x24
#define HIL_REG_WZ      0x28

#define HIL_REG_DT      0x30

#define HIL_REG_IXX     0x40
#define HIL_REG_IYY     0x44
#define HIL_REG_IZZ     0x48

#define HIL_REG_INV_IXX 0x50
#define HIL_REG_INV_IYY 0x54
#define HIL_REG_INV_IZZ 0x58

#define HIL_STATUS_READY (1U << 1)
#define HIL_CONTROL_START (1U << 0)

#define HIL_TIMEOUT_POLLS 1000U

// convert float to signed Q16.16
static int32_t HIL_ToFixed(float value)
{
    return (int32_t)(value * 65536.0f);
}

// convert signed Q16.16 to float
static float HIL_ToFloat(int32_t value)
{
    return (float)value / 65536.0f;
}

// write one 32-bit register
static void HIL_WriteRegister(uint8_t address, uint32_t value)
{
    CS_LOW();

    spi_transfer(HIL_CMD_WRITE);
    spi_transfer(address);

    spi_transfer((uint8_t)(value >> 24));
    spi_transfer((uint8_t)(value >> 16));
    spi_transfer((uint8_t)(value >> 8));
    spi_transfer((uint8_t)value);

    spi_wait_idle();
    CS_HIGH();
}

// read one 32-bit register
static uint32_t HIL_ReadRegister(uint8_t address)
{
    uint32_t value = 0;

    CS_LOW();

    spi_transfer(HIL_CMD_READ);
    spi_transfer(address);

    value |= (uint32_t)spi_transfer(0x00) << 24;
    value |= (uint32_t)spi_transfer(0x00) << 16;
    value |= (uint32_t)spi_transfer(0x00) << 8;
    value |= (uint32_t)spi_transfer(0x00);

    spi_wait_idle();
    CS_HIGH();

    return value;
}

void HIL_Init(void)
{
    spi_init();

    // 200 Hz integration step
    HIL_WriteRegister(HIL_REG_DT,
                      (uint32_t)HIL_ToFixed(0.005f));

    // spacecraft inertia (kg m^2)
    HIL_WriteRegister(HIL_REG_IXX,
                      (uint32_t)HIL_ToFixed(0.02f));
    HIL_WriteRegister(HIL_REG_IYY,
                      (uint32_t)HIL_ToFixed(0.02f));
    HIL_WriteRegister(HIL_REG_IZZ,
                      (uint32_t)HIL_ToFixed(0.01f));

    // inverse inertia
    HIL_WriteRegister(HIL_REG_INV_IXX,
                      (uint32_t)HIL_ToFixed(50.0f));
    HIL_WriteRegister(HIL_REG_INV_IYY,
                      (uint32_t)HIL_ToFixed(50.0f));
    HIL_WriteRegister(HIL_REG_INV_IZZ,
                      (uint32_t)HIL_ToFixed(100.0f));

    // initial angular velocity
    HIL_WriteRegister(HIL_REG_WX, 0);
    HIL_WriteRegister(HIL_REG_WY, 0);
    HIL_WriteRegister(HIL_REG_WZ, 0);
}

int HIL_Step(float tx, float ty, float tz,
             float* wx, float* wy, float* wz)
{
    // write body torque
    HIL_WriteRegister(HIL_REG_TX,
                      (uint32_t)HIL_ToFixed(tx));
    HIL_WriteRegister(HIL_REG_TY,
                      (uint32_t)HIL_ToFixed(ty));
    HIL_WriteRegister(HIL_REG_TZ,
                      (uint32_t)HIL_ToFixed(tz));

    // start RK4 integration
    HIL_WriteRegister(HIL_REG_CONTROL, HIL_CONTROL_START);

    // wait for completion
    uint32_t status = 0;

    for (uint32_t i = 0; i < HIL_TIMEOUT_POLLS; i++)
    {
        status = HIL_ReadRegister(HIL_REG_STATUS);

        if (status & HIL_STATUS_READY)
        {
            break;
        }
    }

    if (!(status & HIL_STATUS_READY))
    {
        return -1;
    }

    // read angular velocity
    *wx = HIL_ToFloat((int32_t)HIL_ReadRegister(HIL_REG_WX));
    *wy = HIL_ToFloat((int32_t)HIL_ReadRegister(HIL_REG_WY));
    *wz = HIL_ToFloat((int32_t)HIL_ReadRegister(HIL_REG_WZ));

    return 0;
}
