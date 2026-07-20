# Competition State Machine

## Purpose

比赛题目流程的状态机管理，支持按键切换题目 (Q1~Q4) 和启动/停止控制。

## Requirements

### Requirement: 题目切换
通过按键 K1 (PA2) 短按/长按实现题目切换和启动。

#### Scenario: 长按切题
- **GIVEN** 当前处于某题目或 STOP_STATE
- **WHEN** 长按 K1 (>1s)
- **THEN** 切换到下一题目 (STOP → Q1 → Q2 → Q3 → Q4 → STOP)

#### Scenario: 短按启动
- **GIVEN** 当前处于 STOP_STATE，已选定题目
- **WHEN** 短按 K1 (<1s)
- **THEN** 启动当前题目任务，蜂鸣器响 500ms

### Requirement: 题目状态机
每个题目内部有独立的子状态机，管理具体执行流程。

#### Scenario: Q1 执行流程
- **GIVEN** Q1 已启动
- **WHEN** 子状态机执行
- **THEN** 按 Q1 定义的步骤依次执行 (循迹→转弯→循迹...)

#### Scenario: Q2 执行流程
- **GIVEN** Q2 已启动
- **WHEN** 子状态机执行
- **THEN** 按 Q2 定义的步骤依次执行

#### Scenario: 任务完成
- **GIVEN** 某题目所有子步骤执行完毕
- **WHEN** 最后一个子状态完成
- **THEN** 返回 STOP_STATE，蜂鸣器提示

### Requirement: 状态机嵌套结构
主状态机 (Main_State) 包含子状态机 (Q1~Q4_State)。

#### Scenario: 状态机结构
- **GIVEN** `struct state_machine` 包含 Main_State + Q1_State~Q4_State
- **WHEN** 主循环调用 `Question_Task_N()`
- **THEN** 根据 Main_State 分发到对应子状态机执行

### Requirement: 蜂鸣器反馈
操作按键时蜂鸣器给出声音反馈。

#### Scenario: 启动提示音
- **GIVEN** 短按启动题目
- **WHEN** 进入运行状态
- **THEN** 蜂鸣器响 500ms

#### Scenario: 切题提示音
- **GIVEN** 长按切题
- **WHEN** 题目切换完成
- **THEN** 蜂鸣器短促响一声
