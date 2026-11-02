#include "gpio.h"
#include "stm32f103xb.h"

void gpio_init(void)
{
    // enable GPIOA and GPIOB clocks
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN |
                    RCC_APB2ENR_IOPBEN;

    // PA8: output push-pull, 2 MHz
    GPIOA->CRH &= ~(0xFU << 0);
    GPIOA->CRH |=  (0x2U << 0);

    // PB5: output push-pull, 2 MHz
    GPIOB->CRL &= ~(0xFU << 20);
    GPIOB->CRL |=  (0x2U << 20);

    // PB6: output push-pull, 2 MHz
    GPIOB->CRL &= ~(0xFU << 24);
    GPIOB->CRL |=  (0x2U << 24);

    // initialize all outputs low
    GPIOA->BSRR = (1U << (TIMER_PIN + 16));
    GPIOB->BSRR = (1U << (STATE_PIN + 16)) |
                  (1U << (CONTROL_PIN + 16));
}

void gpio_timer_toggle(void)
{
    GPIOA->ODR ^= (1U << TIMER_PIN);
}

void gpio_state_high(void)
{
    GPIOB->BSRR = (1U << STATE_PIN);
}

void gpio_state_low(void)
{
    GPIOB->BSRR = (1U << (STATE_PIN + 16));
}

void gpio_control_high(void)
{
    GPIOB->BSRR = (1U << CONTROL_PIN);
}

void gpio_control_low(void)
{
    GPIOB->BSRR = (1U << (CONTROL_PIN + 16));
}