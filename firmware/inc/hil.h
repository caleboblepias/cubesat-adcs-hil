
#ifndef HIL_H
#define HIL_H

#include <stdint.h>

// initialize FPGA simulation
void HIL_Init(void);

// execute one simulation step
// returns 0 on success, -1 on timeout
int HIL_Step(float tx, float ty, float tz,
             float* wx, float* wy, float* wz);

#endif // HIL_H
