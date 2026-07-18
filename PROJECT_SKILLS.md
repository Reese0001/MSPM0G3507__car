## 工程规范与工具技能
- [Skill: MSPM0-DriverLib] 遵循 TI SDK 的 API 风格，使用 `DL_` 前缀函数。
- [Skill: Motor-Safety-Layer] 任何电机驱动代码必须包含：
    - 启动占空比限制 (0-30% 初始 ramp)。
    - 定时器中断监控（防失控）。
    - 本项目由 `Motor_Safety_RequestSpeed()` 统一接收速度请求；禁止业务代码直接调用 `Contrl_Speed()`。
    - 上电后 1000 ms 分 10 级从 0 提升到 30%，之后每 100 ms 放宽 10%，直至目标速度上限。
    - 200 ms 未收到新速度请求时锁存故障，并由定时器中断发送固定零速帧；故障只能通过重新初始化解除。
- [Skill: Debug-Protocol] 遇到编译问题时，按以下逻辑排查：
    1. SysConfig 配置引脚是否冲突。
    2. 电源/地连接 (GND) 是否共地。
    3. PWM 频率与电机驱动时序匹配。
