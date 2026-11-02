
#include "state.h"
#include "hil.h"
#include "uart.h"

static angular_velocity_t angular_velocity = {0};

static uint32_t state_update_count = 0;
static uint32_t state_error_count = 0;

void State_Init(void)
{
    HIL_Init();

    angular_velocity.wx = 0.0f;
    angular_velocity.wy = 0.0f;
    angular_velocity.wz = 0.0f;
}

int State_Update(void)
{
    float wx, wy, wz;

    // zero torque until control loop is integrated
    int status = HIL_Step(0.0f, 0.0f, 0.0f,
                          &wx, &wy, &wz);

    if (status != 0)
    {
        state_error_count++;
        return -1;
    }

    // publish updated angular velocity
    angular_velocity.wx = wx;
    angular_velocity.wy = wy;
    angular_velocity.wz = wz;

    state_update_count++;

    return 0;
}

angular_velocity_t State_GetAngularVelocity(void)
{
    return angular_velocity;
}
