# Task Scheduler

## Purpose

裸机协作式时间片轮询调度器，通过毫秒定时器触发周期任务，替代 RTOS。

## Requirements

### Requirement: 时间片轮询
主循环中通过 `Get_Time()` 时间差判断是否触发各任务。

#### Scenario: 注册周期任务
- **GIVEN** `tasks[]` 数组定义了任务列表
- **WHEN** 添加 `{interval_ms, last_call, 函数指针}` 条目
- **THEN** 该任务按 interval_ms 周期被调度

#### Scheduler_Run 执行
- **GIVEN** 主循环调用 `Scheduler_Run()`
- **WHEN** 当前时间 - task.last_call >= task.interval_ms
- **THEN** 执行 task 函数，更新 last_call

### Requirement: 毫秒计时基
TIMA0 提供全局毫秒计数，作为调度器时间基准。

#### Scenario: 获取当前时间
- **GIVEN** TIMA0 中断已使能
- **WHEN** 调用 `Get_Time()`
- **THEN** 返回系统启动以来的毫秒数

### Requirement: 任务间隔独立
每个任务有独立的执行间隔，互不影响。

#### Scenario: 不同频率任务共存
- **GIVEN** 任务A interval=10ms, 任务B interval=50ms
- **WHEN** 调度器运行
- **THEN** 任务A 每10ms执行一次，任务B 每50ms执行一次

### Requirement: 可扩展任务列表
新增周期任务只需在 `tasks[]` 数组中加一条记录。

#### Scenario: 添加新任务
- **GIVEN** 需要新增一个 20ms 周期的传感器读取任务
- **WHEN** 在 `tasks[]` 中添加 `{20, 0, Read_Sensor}`
- **THEN** 调度器自动调度该任务
