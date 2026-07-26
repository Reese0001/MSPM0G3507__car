#include "app_tasks.h"

#include <stdint.h>

#include "FreeRTOS.h"
#include "task.h"

#include "motor_safety.h"

static StaticTask_t bootstrap_tcb;
static StackType_t bootstrap_stack[configMINIMAL_STACK_SIZE];
static TaskHandle_t bootstrap_task_handle;

static void BootstrapTask(void *argument)
{
    (void)argument;
    Motor_Safety_Disarm();
    for (;;) {
        vTaskDelay(pdMS_TO_TICKS(1000U));
    }
}

bool AppTasks_Create(void)
{
    bootstrap_task_handle = xTaskCreateStatic(BootstrapTask,
                                              "Bootstrap",
                                              configMINIMAL_STACK_SIZE,
                                              NULL,
                                              tskIDLE_PRIORITY + 1U,
                                              bootstrap_stack,
                                              &bootstrap_tcb);
    return bootstrap_task_handle != NULL;
}

void vApplicationGetIdleTaskMemory(StaticTask_t **tcb,
                                   StackType_t **stack,
                                   uint32_t *depth)
{
    static StaticTask_t idle_tcb;
    static StackType_t idle_stack[configMINIMAL_STACK_SIZE];

    *tcb = &idle_tcb;
    *stack = idle_stack;
    *depth = configMINIMAL_STACK_SIZE;
}

void vApplicationStackOverflowHook(TaskHandle_t task,
                                   char *task_name)
{
    (void)task;
    (void)task_name;
    Motor_Safety_Disarm();
    for (;;) {
    }
}
