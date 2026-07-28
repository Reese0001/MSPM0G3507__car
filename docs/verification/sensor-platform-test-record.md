# 传感器小车分级验收记录

固件分支：`codex/sensor-safety-integration`

原则：前一级未 PASS，不得进入后一级。任何异常立即断开电机动力，软件保持 BOOT_SAFE。

## A. 离线验证

| 项目 | 判据 | 当前结果 | 证据 |
|---|---|---|---|
| 单元/契约测试 | 全部通过 | PASS | 92/92 |
| SysConfig | 无 pinmux/外设冲突并可生成 | PASS | SysConfig CLI 1.26.2 |
| 新调度代码语法 | TI Arm Clang 4.0.4 无错误 | PASS | `app_scheduler.c`、`timer.c` |
| 1MHz 微秒时基 | TIMG12、装载 0xFFFFFFFF | PASS | SysConfig 生成宏 |
| 电机启动授权 | 默认不调用 `Motor_Safety_Arm()` | PASS | BOOT_SAFE 契约测试 |
| CCS 完整链接 | Build Project 成功 | 待测 | 需在 CCS Theia 生成并构建 |

## B. 断开电机动力的模块验收

| 项目 | 判据 | 结果 | 实测/备注 |
|---|---|---|---|
| 调试 UART | 115200 稳定输出 | 待测 | |
| 灰度完整一帧 | 周期≤5ms，无阻塞 | 待测 | |
| 九轴 IMU 启动 | 上电≥5s 后 100Hz 数据，磁航向健康 | 待测 | |
| 超声波 | 60ms 测量周期、30ms 超时，静态距离误差可接受 | 待测 | |
| K230 | 115200 事件正确，断线≤300ms 标 stale | 待测 | |
| 安全任务 | 1ms 任务无预算超限 | 待测 | 读取 `AppScheduler_GetDiagnostics()` |

## C. 电机前 Checklist

- [ ] 轮子已架空，旁边无人、无线缠绕物。
- [ ] M2/M4 左右轮对应关系已拍照并确认。
- [ ] PB6→驱动 RX、PB7←驱动 TX，所有逻辑地共地。
- [ ] 上电后零速帧有效。
- [ ] 仅允许 0→10%→20%→30%，禁止直接 100%。
- [ ] 0→30% soft-start 已观察确认。
- [ ] 200ms watchdog 可使 M2/M4 归零。
- [ ] 超声波≤200mm 可锁存急停。
- [ ] 可立即拔掉电机动力电源。
- [ ] `power_qualified` 的实测放行条件全部 PASS。

检查人：__________　日期：__________

## D. 架空轮验收

| 步骤 | 判据 | 结果 | 备注 |
|---|---|---|---|
| 零速 | M2/M4 不转 | 待测 | |
| 10% 前进 | 两轮方向正确、无突跳 | 待测 | |
| 20% 前进 | 平滑升速 | 待测 | |
| 30% 前进 | 不超过软件上限 | 待测 | |
| 左/右转 | 差速方向正确 | 待测 | |
| 单轮反转 | 仅在恢复/转向状态允许 | 待测 | |
| 障碍急停 | ≤200mm 立即归零并锁存 | 待测 | |
| 障碍恢复 | >400mm 连续5帧且人工复位 | 待测 | |
| IMU/K230/超声波断线 | 必需传感器 stale 后归零 | 待测 | |

## E. 低速落地路线

| 场景 | 判据 | 结果 | 备注 |
|---|---|---|---|
| 直线 | 稳定居中 | 待测 | |
| 缓弯/连续弯 | IMU 趋势辅助，无蛇形放大 | 待测 | |
| 直角 | 可恢复到黑线 | 待测 | |
| 短时丢线 | 按最近趋势搜索并重新捕获 | 待测 | |
| 完全丢线 | 最大45°或800ms后停车 | 待测 | |
| 前向障碍 | 350mm限速、200mm急停 | 待测 | |
| 定距/定角 | 误差记录完整 | 待测 | |
| 视觉掉线 | 任务按安全策略停止/降级 | 待测 | |

## F. 30分钟压力测试

记录开始/结束电压、扩展板稳压器温度、K230温度、任务最大运行时间、deadline miss、传感器错误、复位次数和急停次数。

最终等级：

- [ ] 可编译
- [ ] 可烧录
- [ ] 可断电机运行
- [ ] 可架空轮运行
- [ ] 可低速落地
- [ ] 可稳健档运行

最终结论：**HOLD - 等待 CCS 完整构建、实物照片、电源带载和分级上车测试**

---

## G. FreeRTOS 查表控制验收（2026-07-26 迁移，Task 8）

固件：`main` 分支 FreeRTOS 四任务 + 15 位置查表控制（取代 PID）。

### G.1 软件验证

| 项目 | 判据 | 结果 | 证据 |
|---|---|---|---|
| 离线测试 | 全部通过 | PASS（190/190） | `python -m unittest discover -s tests -p "test_*.py"`；本次目录整理后复核通过 |
| TI clean build | 无错误链接 | PASS | `gmake -C MSPM0G3507_LineFollowing_Car all`；text 29144 / bss 6381 字节 |
| 静态内核 | 四个静态任务栈，无应用堆 | PASS | map 中 control 0x300 / display 0x280 / safety 0x280 / sensor 0x280 + idle 0x180，无 `ucHeap` |
| 中断归属 | PendSV/SVC/SysTick 由 FreeRTOS port 提供 | PASS | map：三个 handler 均来自 `freertos_kernel_ticlang.lib : port.o` |
| HEX/TI-TXT 一致性 | 两种格式地址/数据字节一致 | PASS | 各 29144 字节，逐地址比对完全相同 |

构建方式：CCS Theia 无无头构建命令，且 `Debug/*.mk` 由 IDE 重新生成，因此新增
`MSPM0G3507_LineFollowing_Car/Makefile` 复刻 CCS Debug 配置（同编译器、同参数、
同 SysConfig 产物、同源码排除表），CLI 产物写入 `build/cli/`。
**烧录前仍以 CCS GUI 构建为准。**

### G.2 当前查表参数（待实测调整）

15 位置表（存 0..7 非负半区，负位置镜像差速；`line_lookup_config.h`）：

| \|位置\| | base | diff |
|---|---|---|
| 0 | 420 | 0 |
| 1 | 400 | 30 |
| 2 | 370 | 65 |
| 3 | 330 | 100 |
| 4 | 285 | 135 |
| 5 | 235 | 165 |
| 6 | 175 | 195 |
| 7 | 120 | 220 |

阈值：`LINE_LOOKUP_HIGH_YAW_DPS = 95.0`（|位置|≥5 时差速×3/4）；
IMU 过期限速 `LINE_LOOKUP_IMU_DEGRADED_LIMIT = 280`；
心跳超时 SENS 20ms / CTRL 30ms（超时锁存 D1）；
电机帧限频 `MOTOR_UART_MIN_PERIOD_MS = 5`（零速命令立即发）。

### G.3 分级实测（需实车，逐项填写）

| 阶段 | 判据 | 结果 | 备注 |
|---|---|---|---|
| 1. 仅 USB | OLED 仪表页出现、D2 心跳、8 位灰度实时、MPU 状态 READY | 待测 | 12.6V 断开 |
| 2. 架空轮 | 0→30% soft-start；断请求 200ms 内零速 | 待测 | |
| 3. 低速落地 | 15 个位置转向方向全部正确 | 待测 | |
| 4. 丢线 | 按最近稳定方向盲走+同向旋转；不锁存 D1（STOPPED 可恢复） | 待测 | |
| 5. 直角/急弯 | 两帧稳定重捕获，无过弯后振荡 | 待测 | |
| 6. 全程 | 只上调中心区 base，不超过 450 | 待测 | |

### G.4 固件指纹（2026-07-26 生成，`gmake images`）

`build/` 和 `dist/` 不入库，需要时用 `gmake -C MSPM0G3507_LineFollowing_Car clean all images` 重新生成。

| 文件 | SHA-256 |
|---|---|
| dist/firmware/MSPM0G3507_LineFollowing_Car.hex | 待重新生成 |
| dist/firmware/MSPM0G3507_LineFollowing_Car.txt | 待重新生成 |

两者携带的地址/数据字节完全一致（各 29144 字节），仅容器格式不同。

最大任务延迟（实测）：待测——需要上电后读 OLED 仪表页。

