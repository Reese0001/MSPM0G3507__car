#ifndef	__TASK_H__
#define __TASK_H__

#include "ti_msp_dl_config.h"
#include "app_motor.h"
#include "bsp_mpu6050.h"
#include "app_mpu6050.h"
#include "inv_mpu.h"
#include "app_irtracking.h"
#include "key.h"
#include "led.h"
#include "buzzer.h"


typedef struct {
    uint32_t interval;    // 任务执行间隔（单位：ms）   Task execution interval (unit: ms)
    uint32_t last_call;    // 上次执行时间戳    Last execution time stamp
    void (*task)(void);    // 任务函数指针  Task function pointer
} Task;

void Scheduler_Run(void);

#endif
