#include "app_scheduler.h"

#include "app_motor.h"
#include "questions.h"

static void AppScheduler_RunKey(uint32_t now_ms)
{
    (void)now_ms;
    Legacy_Questions_HandleKey();
}

static void AppScheduler_RunOdometry(uint32_t now_ms)
{
    (void)now_ms;
    Get_Odometry();
}

static AppTask app_tasks[] = {
    {100U, 0U, AppScheduler_RunKey},
    {15U, 0U, AppScheduler_RunOdometry},
};

void AppScheduler_Init(uint32_t now_ms)
{
    uint32_t index;

    for (index = 0U; index < (uint32_t)(sizeof(app_tasks) / sizeof(app_tasks[0])); index++) {
        app_tasks[index].last_ms = now_ms;
    }
}

void AppScheduler_Run(uint32_t now_ms)
{
    uint32_t index;

    for (index = 0U; index < (uint32_t)(sizeof(app_tasks) / sizeof(app_tasks[0])); index++) {
        AppTask *task = &app_tasks[index];

        if ((uint32_t)(now_ms - task->last_ms) >= task->period_ms) {
            task->run(now_ms);
            task->last_ms = now_ms;
        }
    }
}
