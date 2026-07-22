#include "task.h"

Task tasks[] = {
    {100, 0, Legacy_Questions_HandleKey},
    {5, 0, Get_EulerAngles},
    {30, 0, Get_CalibratedAngles},
    {15, 0, Get_Odometry},
};

void Scheduler_Run(void)
{
    uint32_t now = Get_Time();

    for (int i = 0; i < sizeof(tasks) / sizeof(Task); i++) {
        if (now - tasks[i].last_call >= tasks[i].interval) {
            tasks[i].task();
            tasks[i].last_call = now;
        }
    }
}
