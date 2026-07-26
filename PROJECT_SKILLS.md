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
- [Skill: SysConfig-Duplicate-Pin-Name] `error: XXX(/ti/driverlib/GPIO) associatedPins[n].$name: Duplicate name` 表示**引脚名在整个配置里必须全局唯一**，不只是组内唯一。两个软件 I2C 不能都叫 `SCL`/`SDA`。
    - 现象：`gmake` 在 “SysConfig - building file: ../empty.syscfg” 阶段就报错，根本没进编译。
    - 处理：给后加的组换名（本项目 OLED 用 `CLK`/`DAT`，MPU6050 保留 `SCL`/`SDA`），生成的宏是 `<组名>_<引脚名>_PIN`，必要时在 BSP 里 `#define` 别名保持可读性。
    - 别改引脚分配来绕开——引脚号和接线不变，只改名字。
- [Skill: Headless-TI-Build] CCS Theia 没有无头构建入口，`Debug/*.mk` 是 IDE 产物且会过期（引用早已删除的 `BSP/Motor` 一类目录）。
    - 离线验证走 `MSPM0G3507_LineFollowing_Car/Makefile`（`gmake -C MSPM0G3507_LineFollowing_Car all`），它复刻 CCS Debug 的编译/链接命令。
    - 该 Makefile 用 `SHELL := cmd.exe`，否则从 Git Bash 调 `gmake` 会拿 `sh` 解析 `if not exist` 而语法报错。
    - 链接成功后用 map 自查：四个静态任务栈存在、无 `ucHeap`、PendSV/SVC/SysTick 来自 `port.o`。
- [Skill: Firmware-Image-Freshness] 声称“固件已构建”前，先核对产物时间戳和 obj 列表是否包含新模块。
    - 本项目踩过：`firmware/*.hex` 比新代码早一天，obj 里根本没有 `app_tasks`/`ssd1306`，等于拿旧固件当验收依据。
    - `firmware/` 不入库；每次验收都用 `gmake images` 重新生成并记 SHA-256。
