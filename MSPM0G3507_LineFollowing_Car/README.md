# MSPM0G3507_LineFollowing_Car CCS 工程

这是仓库的唯一活动固件工程，可作为 Existing Project 直接导入 CCS Theia。

## 入口文件

- `empty.c`：初始化、L 型 520 电机选择、循迹循环和安全服务。
- `empty.syscfg`：引脚与外设配置的唯一真实来源。
- `modules/line_tracking/`：PA15～PA18 灰度采样和循迹决策。
- `modules/motor/`：两轮运动学、UART 驱动协议和电机安全层。
- `bsp/time/`：1 ms 时间基和 200 ms 电机看门狗计时。

## 构建

要求：

- CCS Theia
- TI Arm Clang 4.0.4 LTS
- MSPM0 SDK 2.10.00.04
- SysConfig 1.26 或兼容版本

在 CCS 中执行 **Project → Clean**，再执行 **Build Project**。输出文件名应为 `MSPM0G3507_LineFollowing_Car.out`。

如果 CCS 仍显示旧工程名，请从工作区移除旧项目但不要删除磁盘文件，然后重新导入本目录。`.project`、`.cproject` 和 `.theia/launch.json` 已使用新名称。

## 灰度输入约定

PA15=AD0、PA16=AD1、PA17=AD2 选择通道，PA18=OUT 读取数字电平。代码将低电平转换为黑线有效位，再用八路对称权重计算位置误差。

首次上车前必须实测确认：

1. X1 是否为车体最左侧、X8 是否为最右侧。
2. 白底是否确实为高电平，黑线是否确实为低电平。
3. M2/M4 正速度是否都对应车辆前进。

未确认这三项前，只允许离线构建和断电/架空轮检查。
