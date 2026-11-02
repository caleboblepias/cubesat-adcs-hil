
#include "tasks.h"
#include "timer.h"
#include "uart.h"
#include "state.h"
#include "control.h"
#include "stm32f103xb.h"
#include "gpio.h"

#define NOTIFY_TIMER    0
#define NOTIFY_COMPLETE 1

static TaskHandle_t stateTaskHandle = NULL;
static TaskHandle_t controlTaskHandle = NULL;

static void StateUpdateTask(void *argument)
{
    (void)argument;

    uint32_t count = 0;

    timer_start();

    while (1)
    {
        // wait for 200 Hz timer
        // wait for timer
        ulTaskNotifyTakeIndexed(
            NOTIFY_TIMER,
            pdTRUE,
            portMAX_DELAY
        );

        gpio_state_high();

        // advance FPGA dynamics and update state
        if (State_Update() != 0)
        {
            uart_print("HIL step timeout\r\n");
            continue;
        }

        gpio_state_low();

        // release control task
        xTaskNotifyGive(controlTaskHandle);

        // wait for control task to finish
        ulTaskNotifyTakeIndexed(
            NOTIFY_COMPLETE,
            pdTRUE,
            portMAX_DELAY
        );

        count++;

        if (count % 200 == 0)
        {
            uart_print("200 control cycles completed\r\n");
        }
    }
}

static void ControlTask(void *argument)
{
    (void)argument;

    while (1)
    {
        // wait for new state
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);

        gpio_control_high();

        // execute control iteration
        Control_Update();

        gpio_control_low();

        // notify state task of completion
        xTaskNotifyGiveIndexed(
            stateTaskHandle,
            NOTIFY_COMPLETE
        );
    }
}

BaseType_t Tasks_Init(void)
{
    State_Init();
    Control_Init();

    if (xTaskCreate(
        StateUpdateTask,
        "State",
        256,
        NULL,
        3,
        &stateTaskHandle
    ) != pdPASS)
    {
        return pdFAIL;
    }

    if (xTaskCreate(
        ControlTask,
        "Control",
        256,
        NULL,
        2,
        &controlTaskHandle
    ) != pdPASS)
    {
        return pdFAIL;
    }

    return pdPASS;
}

// TIM2 interrupt handler
void TIM2_IRQHandler(void)
{
    if (TIM2->SR & TIM_SR_UIF)
    {
        TIM2->SR = 0;

        gpio_timer_toggle();

        BaseType_t higherPriorityTaskWoken = pdFALSE;

        vTaskNotifyGiveIndexedFromISR(
            stateTaskHandle,
            NOTIFY_TIMER,
            &higherPriorityTaskWoken
        );

        portYIELD_FROM_ISR(higherPriorityTaskWoken);
    }
}

void vApplicationMallocFailedHook(void)
{
    taskDISABLE_INTERRUPTS();
    while (1);
}

void vApplicationStackOverflowHook(
    TaskHandle_t xTask,
    char *pcTaskName
)
{
    (void)xTask;
    (void)pcTaskName;

    taskDISABLE_INTERRUPTS();
    while (1);
}
