#ifndef GPIO_H
#define GPIO_H

#include <stdint.h>

#define TIMER_PIN   8 // PA8
#define STATE_PIN   5 // PB5
#define CONTROL_PIN 6 // PB6

// initialize debug GPIO
void gpio_init(void);

// timer
void gpio_timer_toggle(void);

// state update
void gpio_state_high(void);
void gpio_state_low(void);

// control loop
void gpio_control_high(void);
void gpio_control_low(void);

#endif // GPIO_H