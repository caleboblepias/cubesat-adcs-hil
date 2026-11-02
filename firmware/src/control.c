
#include "control.h"
#include "state.h"

volatile uint32_t control_update_count = 0;

void Control_Init(void)
{
    control_update_count = 0;
}

void Control_Update(void)
{
    // read latest angular velocity
    angular_velocity_t omega = State_GetAngularVelocity();

    // TODO: quaternion propagation
    // TODO: quaternion attitude error
    // TODO: PID controller
    // TODO: reaction wheel allocation

    (void)omega;

    control_update_count++;
}
