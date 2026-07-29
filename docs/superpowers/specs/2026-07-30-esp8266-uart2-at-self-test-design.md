# ESP8266 UART2 AT 自检设计

## 目标

在不改动电机 UART1、循迹控制或 PID 参数的前提下，验证 MSPM0G3507 与野火小智 ESP8266-12F 的串口 AT 通信。

## 硬件边界

- ESP8266 TXD 连接 MSPM0 PA22（UART2_RX）。
- ESP8266 RXD 连接 MSPM0 PA21（UART2_TX）。
- ESP8266 和 MSPM0 共用稳定的 3.3V 与 GND。
- USB-TTL/CH340 不参与本方案。
- 电机 12.6V 电源在本自检阶段保持断开。

## 软件设计

1. 在 `empty.syscfg` 新增 UART2，使用 PA21/PA22、115200 baud、RX 中断。
2. 新增独立 `wifi_uart` 模块：维护有限长度接收缓冲区、发送 AT 命令并等待响应。
3. 初始化完成后，模块只发送一次 `AT\r\n`；不会发送电机命令或 PID 修改命令。
4. 在限定超时内接收到完整 `OK`，将状态标记为 `ESP_OK`；超时、缓冲区溢出或非预期响应标记为 `ESP_FAULT`。
5. OLED 只显示简短状态：`ESP OK` 或 `ESP TIMEOUT`。故障不会影响既有安全门控，也不会改变电机输出。

## 验收方法

1. 断开 12.6V 电机电源，仅为 MCU、ESP8266、OLED 供电。
2. 烧录后观察 OLED：ESP8266 返回 `OK` 时显示 `ESP OK`；否则显示 `ESP TIMEOUT`。
3. 确认 PB6/PB7 的 UART1 电机通信未被改动，且整个测试没有任何电机启动请求。

## 范围外内容

- Wi-Fi 配网、TCP/UDP 服务器和串口透传。
- 远程 PID 参数读写、参数持久化和 AI 调参程序。
- 电机协议、安全层、循迹和 PID 算法的任何改动。
