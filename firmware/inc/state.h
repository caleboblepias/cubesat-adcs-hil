
#ifndef STATE_H
#define STATE_H

#include <stdint.h>

typedef struct
{
    float wx;
    float wy;
    float wz;
} angular_velocity_t;

// initialize state update module
void State_Init(void);

// advance dynamics and update state
// returns 0 on success
int State_Update(void);

// read current angular velocity
angular_velocity_t State_GetAngularVelocity(void);

#endif // STATE_H
