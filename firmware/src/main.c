#ifndef STM32F103xB
#define STM32F103xB
#endif

#include "stm32f1xx.h"
#include <stdint.h>

static void delay(void) {
    for (volatile uint32_t i = 0; i < 300000; i++) {}
}

int main(void) {
    RCC->APB2ENR |= RCC_APB2ENR_IOPAEN;

    GPIOA->CRL &= ~(GPIO_CRL_MODE5 | GPIO_CRL_CNF5);
    GPIOA->CRL |= GPIO_CRL_MODE5_1; // PA5 output, 2 MHz, push-pull

    while (1) {
        GPIOA->ODR ^= (1U << 5);
        delay();
    }
}