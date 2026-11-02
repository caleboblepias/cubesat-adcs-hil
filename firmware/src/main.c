#include "stm32f1xx.h"
#include "FreeRTOS.h"
#include "task.h"

#include "tasks.h"
#include "timer.h"
#include "uart.h"
#include "gpio.h"

int main(void)
{
    SystemCoreClockUpdate();

    uart_init();
    gpio_init();
    uart_print("Starting FreeRTOS...\r\n");

    if (Tasks_Init() != pdPASS)
    {
        uart_print("Task creation failed!\r\n");
        while (1);
    }

    timer_init();

    vTaskStartScheduler();

    while (1);
}
