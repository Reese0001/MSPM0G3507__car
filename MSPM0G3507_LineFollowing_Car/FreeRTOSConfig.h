#ifndef FREERTOS_CONFIG_H
#define FREERTOS_CONFIG_H

#define configCPU_CLOCK_HZ 80000000UL
#define configTICK_RATE_HZ 1000
#define configENABLE_MPU 0
#define configMAX_PRIORITIES 5
#define configMINIMAL_STACK_SIZE 96U
#define configMAX_TASK_NAME_LEN 16
#define configUSE_PREEMPTION 1
#define configUSE_TIME_SLICING 0
#define configUSE_TICKLESS_IDLE 0
#define configUSE_TIMERS 0
#define configUSE_MUTEXES 0
#define configUSE_RECURSIVE_MUTEXES 0
#define configUSE_COUNTING_SEMAPHORES 0
#define configUSE_TASK_NOTIFICATIONS 1
#define configSUPPORT_STATIC_ALLOCATION 1
#define configSUPPORT_DYNAMIC_ALLOCATION 0
#define configCHECK_FOR_STACK_OVERFLOW 2
#define configUSE_16_BIT_TICKS 0
#define configIDLE_SHOULD_YIELD 1
#define configQUEUE_REGISTRY_SIZE 0
#define configUSE_TRACE_FACILITY 0
#define configGENERATE_RUN_TIME_STATS 0
#define configUSE_CO_ROUTINES 0
#define configUSE_IDLE_HOOK 0
#define configUSE_TICK_HOOK 0
#define configUSE_MALLOC_FAILED_HOOK 0

/* Keep kernel assertions observable on the target; do not silently discard
 * a failed first-task/vector/critical-section invariant. */
void App_FreeRTOS_Assert(void);
#define configASSERT(x) ((x) ? (void)0 : App_FreeRTOS_Assert())

/* FreeRTOS 11.2 passes the idle-task handle array to this trace hook just
 * before xPortStartScheduler().  The bridge keeps boot_trace types out of the
 * kernel build. */
void BootTrace_PortStart(void);
#define traceSTARTING_SCHEDULER(xIdleTaskHandles) \
    do {                                           \
        (void)(xIdleTaskHandles);                  \
        BootTrace_PortStart();                     \
    } while (0)

#define INCLUDE_vTaskDelayUntil 1
#define INCLUDE_vTaskDelay 1
#define INCLUDE_vTaskDelete 0
#define INCLUDE_vTaskSuspend 0

/* MSPM0G3507 CM0+ interrupt priorities and handler bindings. */
#ifdef __NVIC_PRIO_BITS
#define configPRIO_BITS __NVIC_PRIO_BITS
#else
#define configPRIO_BITS 2
#endif

#define configLIBRARY_LOWEST_INTERRUPT_PRIORITY 0x03
#define configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY 1
#define configKERNEL_INTERRUPT_PRIORITY \
    (configLIBRARY_LOWEST_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))
#define configMAX_SYSCALL_INTERRUPT_PRIORITY \
    (configLIBRARY_MAX_SYSCALL_INTERRUPT_PRIORITY << (8 - configPRIO_BITS))

/* The TI startup source provides the CMSIS exception names. */
#define xPortPendSVHandler PendSV_Handler
#define vPortSVCHandler SVC_Handler
#define xPortSysTickHandler SysTick_Handler

#endif
