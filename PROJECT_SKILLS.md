## 工程规范与工具技能
- [Skill: MSPM0-DriverLib] 遵循 TI SDK 的 API 风格，使用 `DL_` 前缀函数。
- [Skill: Motor-Safety-Layer] 任何电机驱动代码必须包含：
    - 启动占空比限制 (0-30% 初始 ramp)。
    - 定时器中断监控（防失控）。
- [Skill: Debug-Protocol] 遇到编译问题时，按以下逻辑排查：
    1. SysConfig 配置引脚是否冲突。
    2. 电源/地连接 (GND) 是否共地。
    3. PWM 频率与电机驱动时序匹配。