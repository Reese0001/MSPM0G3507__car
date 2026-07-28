# FreeRTOS 启动链临时诊断设计

## 目标

定位固件显示 `SCHED START` 后、SafetyTask 入口未执行的问题。诊断完成后删除本设计引入的运行时检查点，恢复 D1/D2 原用途，避免正常循迹增加开销。

## 诊断边界

只检查以下启动链：

`main → vTaskStartScheduler → xPortStartScheduler → SVC → 首次上下文恢复 → Safety/Sensor/Control/Display 任务入口`

不修改循迹算法、PID、电机协议、引脚配置或任务周期。四个任务全部确认上线前，电机保持禁止。

## 状态记录

新增一个最小的 `boot_trace` 模块，只保存最后阶段码和四任务上线位图。普通上下文可把阶段文字写入 OLED；异常上下文只写 GPIO，不调用 OLED、格式化、UART 或 FreeRTOS API。

阶段码：

- `01`：进入 main
- `02`：任务创建完成
- `03`：准备启动调度器
- `04`：执行 FreeRTOS port 启动前 trace hook
- `05`：进入 SVC
- `06`：首次任务上下文开始恢复
- `10`：SafetyTask 入口
- `11`：SensorTask 入口
- `12`：ControlTask 入口
- `13`：DisplayTask 入口
- `14`：四任务全部上线

致命状态：

- `E1`：FreeRTOS 断言
- `E2`：HardFault
- `E3`：未处理的默认中断
- `E4`：任务栈溢出
- `E5`：调度器意外返回

## 现场显示规则

- OLED 显示最后一个安全可打印的阶段及任务位图。
- 阶段 `03/04/05/06` 是静态 D1:D2 码 `00/01/10/11`；`00` 表示 port 启动前 trace hook 未到，断言则使用 `E1`。
- 非零且未达到 `0x0F` 的任务位图使 D1 熄灭，D2 按已上线任务数重复 1～3 个 100 ms 短脉冲，并留明显长间隔。
- D1 常亮表示致命错误，D2 重复 1～5 个脉冲分别表示 `E1-E5`。
- 四任务达到 `0x0F` 时只清除一次启动灯状态并恢复原 D2 250 ms 心跳；`TASK MASK 0F` 只记录一次，可能从流动 OLED 日志中滚走，此后以心跳为准。
- 如果 OLED 停在 `SCHED START`，以 D1/D2 故障码为准。

## 实现方式

应用层检查点放在 `empty.c` 和四个任务入口。FreeRTOS 端口检查点通过仓库内的极小诊断回调接入当前内核构建；CLI Makefile 与 CCS projectspec 编译同一个本地 `portasm_diagnostic.c`，由它局部改名并包含官方 `portasm.c`，不修改 SDK 安装目录。已有 1 ms 回调以无阻塞、无除法、仅 GPIO 的状态机驱动部分任务脉冲。异常处理集中调用同一个 GPIO-only 故障锁存函数，并在第一步关闭可屏蔽中断。

## 验证与撤销

- 主机契约测试检查阶段码唯一、四任务都有入口登记、异常路径不调用 OLED/UART。
- 每次先清理 FreeRTOS 内核和应用构建缓存，再生成 TI-TXT。
- 根据现场阶段码定位并修复根因后，删除 `boot_trace`、端口诊断回调和临时任务入口标记；保留最终根因修复及原 D2 心跳。
