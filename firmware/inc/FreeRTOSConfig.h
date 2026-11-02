#pragma once

#include <stdint.h>
#include "stm32f1xx.h"

#define vPortSVCHandler    SVC_Handler
#define xPortPendSVHandler PendSV_Handler
#define xPortSysTickHandler SysTick_Handler

#define INCLUDE_xTaskGetSchedulerState  1

#define configUSE_PREEMPTION            1
#define configUSE_TIME_SLICING          1
#define configCPU_CLOCK_HZ              ( SystemCoreClock )
#define configTICK_RATE_HZ              ( 1000 )
#define configMAX_PRIORITIES            5
#define configMINIMAL_STACK_SIZE        128
#define configSUPPORT_DYNAMIC_ALLOCATION 1
#define configTOTAL_HEAP_SIZE           ( 8 * 1024 )

#define configTICK_TYPE_WIDTH_IN_BITS   TICK_TYPE_WIDTH_32_BITS
#define configIDLE_SHOULD_YIELD         1

#define configUSE_MUTEXES               1
#define configUSE_SEMAPHORES            1

#define configCHECK_FOR_STACK_OVERFLOW  2
#define configUSE_MALLOC_FAILED_HOOK    1

#define configUSE_IDLE_HOOK             0
#define configUSE_TICK_HOOK             0

#define configASSERT(x) if ((x) == 0) { taskDISABLE_INTERRUPTS(); for(;;); }

#define INCLUDE_vTaskDelay              1

/* Cortex-M interrupt priorities */
#define configPRIO_BITS                 4
#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY   15
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 5

#define configKERNEL_INTERRUPT_PRIORITY \
    ( configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    ( configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS) )

#define configTASK_NOTIFICATION_ARRAY_ENTRIES 2

