# 循迹小车离线检查记录

日期：2026-07-18  
分支：`verify/line-following-car`  
基线：`9a3f7b0c9d369c2a9364d4f6db08d461bbee8b2c`

## 安全范围

本轮未烧录、未连接 12.6 V 电机电源、未发送非零电机命令，仅执行离线生成、编译、链接和源代码合同测试。

## 已通过

- SysConfig 1.27.0 能从 `empty.syscfg` 生成配置文件。
- TI Arm Clang 4.0.4 成功编译 20 个源文件并完成链接。
- 生成 `_integration_staging/line-following-audit/build/line_following_audit.out`，大小 158436 字节。
- 符号表包含 `main`、`LineWalking`、`Motor_Safety_Init`、`Motor_Safety_Service`、`Motor_Safety_Tick1ms`。
- 主循环按 `LineWalking()` → `Motor_Safety_Service()` 执行。
- 循迹层没有直接调用 `Contrl_Speed()`，速度经 `Motion_Car_Control()` 和安全层发送。
- M1/M3 固定为零，M2/M4 分别作为左右驱动轮。
- SysConfig 与用户确认的灰度接口一致：PA15=AD0、PA16=AD1、PA17=AD2、PA18=OUT。
- UART1 电机接口为 PB6 TX / PB7 RX，115200；定时器周期为 1 ms。

## 修复前失败证据

1. `APP_ELE_PID_Calc()` 使用 `error_last` 计算 D 项，但函数返回前没有执行 `error_last = error;`。
2. 八路全部为 1（按当前注释表示全白/丢线）时，没有请求零速；`trun_flag != 0` 时会保持上一次误差。
3. 两个直角分支各调用 `delay_ms(100)`，阻塞传感器更新和主循环安全服务。
4. 当前 `KP=400` 与 `Car_APB=188` 的组合过强。基础速度 120 时：
   - `err=2` → PID 约 801 → M2≈270.6、M4≈-30.6，一侧已经反转；
   - `err=4` → PID 约 1602 → M2≈421.2、M4≈-181.2；
   - `|err|>=8` → PID 饱和到 3000 → 一侧约 684、另一侧约 -444。

这意味着当前算法对最小偏差也不是平滑修正，而是原地/近原地扭转。即使编译和链接成功，也不能据此判定可稳定循迹。

## 尚需实物确认

- X1 到 X8 在车体上是否按左到右排列。
- OUT 高电平是否确实表示白色、低电平是否表示黑线。
- M2/M4 正速度对应的实际车轮前进方向。

在这三项确认前，不调整转向正负号。

## 2026-07-18 控制修复后复验

- CCS 工程目录和工程名已改为 `MSPM0G3507_LineFollowing_Car`。
- 八路误差改为 `-7,-5,-3,-1,+1,+3,+5,+7` 对称加权平均。
- PD 每周期更新上一误差，普通循迹修正量受基础速度 80% 限制。
- 丢线前两周期低速恢复，第三周期仍丢线时请求 `Motion_Car_Control(0, 0, 0)`。
- `LineWalking()` 的活动路径不再包含 `delay_ms(100)`。
- Python 离线合同共 21 项通过。
- TI Arm Clang 4.0.4 重新编译 20 个生产源文件并成功链接：
  `_integration_staging/line-following-audit/build/MSPM0G3507_LineFollowing_Car.out`，149376 字节。
- 符号表已确认包含 `main`、`LineWalking`、`Tracking_ComputeWeightedError`、`Motor_Safety_Service` 和 `Motor_Safety_Tick1ms`。

以上仍只证明离线可编译、可链接，不代表已完成烧录或实车循迹验证。
