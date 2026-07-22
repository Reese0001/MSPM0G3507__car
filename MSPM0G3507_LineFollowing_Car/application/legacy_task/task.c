#include "task.h"

/* Compatibility entry point; application/app_main.c owns the active loop. */
void Scheduler_Run(void)
{
    AppScheduler_Run(Get_Time());
}
