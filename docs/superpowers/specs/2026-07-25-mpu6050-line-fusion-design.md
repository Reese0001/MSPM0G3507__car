# MPU6050 与八路循迹融合设计

## 目标

在不改变现有八路灰度路线判断、直角状态机和电机安全层的前提下，
用 MPU6050 的 Z 轴角速度降低直线摆动与转弯过冲。代码直接进入
`main`，每个可验证功能独立提交并立即推送。

## 已确认硬件

- MCU：MSPM0G3507。
- MPU6050：3.3V 供电并与 MCU 共地。
- SCL：PA12；SDA：PA13。
- AD0：接 GND，7 位地址为 `0x68`。
- INT、XDA、XCL：不接。
- 模块水平安装，模块 `+Y` 指向车头，`+Z` 朝上。
- 左转对应正 Z 轴角速度，方向符号保留为可调参数。
- 上电后小车可以完全静止 2 秒。

## 控制边界

八路灰度始终是路线位置和转弯方向的唯一主传感器。MPU6050 不产生
绝对航向，不使用 DMP/eMPL，也不单独决定直角是否完成。

MPU6050 只提供以下辅助：

1. 正常循迹：根据灰度控制量生成目标角速度，以陀螺仪实测角速度做
   有界修正，抑制转向不足和过冲。
2. 高角速度：降低前进速度，给灰度传感器留下反应时间。
3. 直角 `COMMIT/SEEK`：旋转速度超过阈值时使用较缓的转角命令，
   黑线重新捕获仍由现有连续帧逻辑确认。
4. 找线后的 `SETTLE`：继续使用正常循迹闭环，让角速度反馈抑制余振。

不改变现有电机命令范围、软启动、方向互锁、通信超时和失控监控。

## 模块结构

```text
MSPM0G3507_LineFollowing_Car/
├─ bsp/
│  └─ bsp_i2c.c/.h                 # 复用现有非阻塞开漏软件 I2C
├─ modules/
│  └─ mpu6050/
│     ├─ mpu6050.c                 # 初始化、非阻塞采样、校准、发布快照
│     ├─ mpu6050.h                 # 状态与快照接口
│     └─ mpu6050_config.h          # 安装方向、滤波和失效参数
└─ application/
   ├─ app_scheduler.c              # 服务传感器并把 yaw rate 送入控制链
   └─ config/
      ├─ line_control_config.h      # 角速度闭环参数
      └─ line_following_profile.h   # 启用硬件、保持 fail-soft
```

不复用 `modules/legacy_mpu6050` 的阻塞式驱动和 DMP，不复用
`modules/ybimu` 的磁力计、四元数与厂商寄存器协议。

## MPU6050 数据链

驱动复用现有 `BSP_I2C_BeginRead/BeginWrite/Service` 状态机。SysConfig
为 PA12/PA13 生成专用 GPIO 宏，GPIO 通过“输出低/释放输入”模拟开漏，
总线服务不得包含阻塞循环或延时。

初始化采用最少寄存器：

- `WHO_AM_I (0x75)` 必须返回 `0x68`。
- `PWR_MGMT_1 (0x6B) = 0x01`，唤醒并选择稳定时钟。
- `CONFIG (0x1A) = 0x03`，启用陀螺仪低通。
- `GYRO_CONFIG (0x1B) = 0x08`，量程 ±500 dps。
- `SMPLRT_DIV (0x19) = 0x09`，目标采样率 100 Hz。
- 从 `GYRO_ZOUT_H (0x47)` 连续读取两个字节。

上电校准持续 2000 ms，至少收集 150 个有效样本。校准期间不允许电机
启动；完成后减去平均零偏，再经过方向映射、限幅、死区和一阶低通，
发布带时间戳与序号的 `Mpu6050Snapshot`。

## 控制算法

基础灰度 PD 先产生 `base_turn`。IMU 新鲜时：

```text
target_yaw_rate = base_turn × yaw_rate_per_command
assist = clamp(
    (target_yaw_rate - measured_yaw_rate) × yaw_rate_kp,
    -yaw_assist_limit,
    +yaw_assist_limit)
turn = clamp(base_turn + assist, -turn_limit, +turn_limit)
```

辅助量必须有独立上限，不能越过现有转向、电机和安全速度限制。参数
集中放置，首版采用保守值，实车只需修改配置文件。

## 启动与失效策略

- 启动校准期间：使命请求无效，车轮保持停止。
- 初始化或校准超过限定时间仍失败：进入纯灰度循迹并限制前进速度。
- 运行中样本超时、I2C 错误或数据越界：取消角速度辅助，限速继续跑。
- 数据恢复并重新达到新鲜条件：自动恢复辅助，不要求复位。
- IMU 失效不能触发 D1 常亮的控制故障锁存；电机故障和现有线路状态机
  故障仍按原策略闭锁。

## 验证

1. 主机单元测试验证寄存器解码、校准、滤波、超时和符号映射。
2. 控制器测试验证角速度不足时增加转向、过快时减少转向、失效时与
   纯灰度结果一致。
3. 调度器契约测试验证启动静止、快照新鲜度和 fail-soft 限速。
4. 运行全量 Python/C harness，并使用 TI Arm Clang 完整编译链接。
5. 生成 UniFlash 可用的 HEX 与 TXT，首次上车架空驱动轮验证方向。
