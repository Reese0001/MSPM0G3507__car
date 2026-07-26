# README 现状同步设计

## 目标

原地刷新根目录 `README.md`，使首次接触项目的人能按当前 `main` 分支完成构建、UniFlash 烧录、OLED 接线与基础验收，不再被旧裸机、UART0 或 PD 控制说明误导。

## 内容范围

- 项目概述改为 FreeRTOS 四静态任务与 15 位置查表循迹。
- 接线表改为 PA10/PA11 SSD1306 软件 I2C，并保留已确认的电机、灰度和 MPU6050 引脚。
- 同时说明 CCS/XDS110 与 UniFlash TI-TXT 两种烧录路径；提示 PA18 与 BSL Invoke 复用。
- 说明 OLED 地址 `0x3C`、诊断页字段、正常现象及最短排查顺序。
- 保留电机 soft-start、看门狗和分阶段上车 Checklist。
- 链接现有详细验收记录，不在 README 重复完整测试表。

## 明确不做

- 不修改固件、SysConfig、控制参数或接线定义。
- 不新增文档层级、脚本、依赖或第二份快速入门。
- 不把尚未完成的实车验收写成已通过。

## 验证

- README 为有效 UTF-8。
- 所有仓库内相对链接存在。
- UniFlash 文件名、OLED 引脚、地址和诊断字段与当前源码一致。
- `git diff --check` 无错误。
