
#include "timer.h"
#include "stm32f103xb.h"
#include "system_stm32f1xx.h"

void timer_init(void)
{
    // enable TIM2 peripheral clock
    RCC->APB1ENR |= RCC_APB1ENR_TIM2EN;

    // determine APB1 prescaler
    uint32_t ppre1 = (RCC->CFGR >> 8) & 0x7U;
    uint32_t apb1_div = (ppre1 < 4U) ? 1U : (1U << (ppre1 - 3U));

    // TIM2 clock is doubled when APB1 is prescaled
    uint32_t pclk1 = SystemCoreClock / apb1_div;
    uint32_t tim_clk = (apb1_div == 1U) ? pclk1 : (2U * pclk1);

    // 10 kHz timer counter frequency
    uint32_t prescaler = tim_clk / 10000U;

    TIM2->CR1 = 0;
    TIM2->PSC = prescaler - 1U;

    // 10 kHz / 50 = 200 Hz
    TIM2->ARR = 49U;

    // load prescaler immediately
    TIM2->EGR = TIM_EGR_UG;

    // clear pending update flag
    TIM2->SR = 0;

    // enable update interrupt
    TIM2->DIER |= TIM_DIER_UIE;

    // FreeRTOS-compatible interrupt priority
    NVIC_SetPriority(TIM2_IRQn, 5);
    NVIC_EnableIRQ(TIM2_IRQn);
}

void timer_start(void)
{
    TIM2->CNT = 0;
    TIM2->SR = 0;
    TIM2->CR1 |= TIM_CR1_CEN;
}

void timer_stop(void)
{
    TIM2->CR1 &= ~TIM_CR1_CEN;
}
