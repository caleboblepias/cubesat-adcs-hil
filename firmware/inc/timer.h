
#ifndef TIMER_H
#define TIMER_H

#include <stdint.h>

// initialize TIM2 for 200 Hz
void timer_init(void);

// start TIM2
void timer_start(void);

// stop TIM2
void timer_stop(void);

#endif // TIMER_H
