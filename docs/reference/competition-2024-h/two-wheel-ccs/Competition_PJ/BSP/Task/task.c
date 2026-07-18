#include "task.h"

//在此创建任务  Create a task here
Task tasks[] = {
    {100, 0, Key_Handle},           // 每100ms执行按键检测  Perform key detection every 100ms
    {10, 0, Get_EulerAngles},        // 每5ms获取角度数据   Perform key detection every 5ms
    {30, 0, Get_CalibratedAngles},  //每30ms获取校准后的角度  Get the calibrated angle every 30ms
    {1000, 0, Get_Odometry},          //每1s获取一次里程计数值    Get the calibrated angle every 30ms
};

void Scheduler_Run(void) {
    uint32_t now = Get_Time();  // 获取当前时间戳   Get the current timestamp
    for(int i=0; i<sizeof(tasks)/sizeof(Task); i++) {
        // 计算时间差（自动处理32位溢出）   Calculate the time difference (automatically handle 32-bit overflow)
        if(now - tasks[i].last_call >= tasks[i].interval) {
            tasks[i].task();          // 执行任务   Perform tasks
            tasks[i].last_call = now; // 更新时间戳 Update time stamp
        }
    }
}












