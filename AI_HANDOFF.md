# MSPM0G3507 循迹小车项目 AI 交接说明

更新时间：2026-07-18  
仓库：`D:\DevProject\MSPM0G3507__car`

## 1. 接手后先做什么

1. 完整阅读根目录 `AGENTS.md`，严格遵守电机安全和编码要求。
2. 执行 `git status --short --branch` 和 `git diff`，不要覆盖工作区现有修改。
3. 当前存在一份尚未提交、尚未验证的循迹算法改动，必须先审查，不能直接烧录或合并。
4. 涉及电机启动、串口电机协议、引脚或 SysConfig 的修改，先向用户给出 Checklist 并获得确认。

## 2. 当前 Git 状态

- 当前分支：`fix/line-tracking-turn`
- 当前 HEAD：`8fb789f fix: reverse line-following steering polarity`
- `main` 也指向 `8fb789f`。
- `origin/main` 指向 `be2f771`，所以本地 `main` 比远端领先 1 个提交，尚未推送。
- `fix/line-tracking-turn` 是后来出现的本地分支。
- `MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c` 有未提交修改。这些修改不是已验证提交 `8fb789f` 的内容，不得丢弃，也不得未经检查直接提交。
- 在当前未提交代码上于 2026-07-18 重新运行 22 项测试，结果为 17 项通过、5 项失败；失败均来自 `test_tracking_decision_table.py`，说明该改动已经破坏已验证的循迹合同。

查看现场：

```powershell
git status --short --branch
git branch -vv
git diff -- MSPM0G3507_LineFollowing_Car/BSP/Eight_Tracking/app_irtracking.c
```

## 3. 已完成并验证的工作

提交 `8fb789f` 修复了“本来应该向左转，实车却向右转”的循迹方向问题：

- 保留 X1～X8 权重 `{-7,-5,-3,-1,1,3,5,7}`。
- 保留 PA15/PA16/PA17 选择通道、PA18 读取 OUT。
- 保留 M1/M3 停止以及 M2/M4 驱动轮映射。
- 在控制器输出边界定义 `TRACKING_STEERING_POLARITY (-1)`。
- 正常循迹和丢线恢复使用同一转向极性。
- 未绕过电机安全层，软启动和看门狗仍有效。
- 当时完整离线测试为 22 项全部通过，CCS clean build 成功。

对应计划：

`docs/superpowers/plans/2026-07-18-fix-tracking-steering-direction.md`

## 4. 当前未提交改动的风险

当前 `app_irtracking.c` 的未提交内容大幅改变了已验证算法，包括：

- KP 从 40 提高到 250，并启用 KI。
- 基础速度从 120 提高到 300。
- 丢线恢复速度从 36 提高到 200。
- 丢线停止周期从 3 提高到 5。
- 禁用了 `Tracking_ComputeWeightedError()` 的原实现。
- 改成所谓“2024 H 题”离散模式/PID策略。

这些变化超出“只修正方向”的范围，并且高速度、高增益可能带来实车安全风险。下一位 AI 必须先运行测试、完整审查文件，并询问用户这是否是用户主动修改以及是否确实要保留。不要自行执行 `git checkout --`、`git reset` 或覆盖文件。

## 5. 工程和硬件概要

- MCU：TI MSPM0G3507，裸机 no-RTOS。
- IDE：CCS Theia；工具链 TI Arm Clang 4.0.4。
- CCS 工程：`MSPM0G3507_LineFollowing_Car/`
- SysConfig 唯一真实来源：`MSPM0G3507_LineFollowing_Car/empty.syscfg`
- 八路灰度：PA15/PA16/PA17 为通道选择，PA18 为 OUT。
- UART0 调试：PA10/PA11，115200。
- UART1 电机：PB6/PB7。
- MPU6050：PA12/PA13 软件 I2C。
- 电机命令必须经过安全层；禁止直接 100% PWM，保留软启动和失控看门狗。

详细架构、引脚和规则见 `AGENTS.md`、`README.md`、`SETUP_GUIDE.md` 和 `PROJECT_SKILLS.md`。

## 6. 构建与测试

离线测试：

```powershell
python -m unittest discover -s tests -v
```

CCS clean build（工作目录必须是 Debug）：

```powershell
Set-Location MSPM0G3507_LineFollowing_Car\Debug
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' clean
& 'D:\DevTools\ti\ccs2050\ccs\utils\bin\gmake.exe' -j4 all
```

不要手改 `Debug/ti_msp_dl_config.c`，它是 SysConfig 生成文件。

## 7. 烧录文件与 UniFlash

最近一次已验证构建产物：

- CCS/XDS110：`MSPM0G3507_LineFollowing_Car/Debug/MSPM0G3507_LineFollowing_Car.out`
- UniFlash 串口 BSL：`MSPM0G3507_LineFollowing_Car/Debug/MSPM0G3507_LineFollowing_Car.txt`

TI-TXT 最近一次大小约 48,860 字节，以 `@0000` 开始、以 `q` 结束。修改源码后必须重新 clean build 和重新生成 TI-TXT，不能继续烧录旧文件。

串口 BSL 注意事项：

- COM10 曾被 CCS/ccs-server 占用；烧录前关闭 CCS 串口监视器或占用进程。
- UniFlash 使用 `Serial Connection > MSPM0G3507(BOOTLOADER)`。
- PA18 同时用于当前灰度 OUT 和 BSL Invoke；进入 BSL/烧录时建议临时断开灰度模块 PA18，避免电平冲突。
- 自动进入 BSL 失败时，需要按板卡说明手动进入 BSL，再点击 Load Image。
- 可在 UniFlash 菜单 `Session → Save Session As` 保存烧录会话。

## 8. 推荐的下一步

1. 先询问用户：当前 `fix/line-tracking-turn` 中未提交的大幅 PID/速度改动是否由用户主动要求保留。
2. 在不改文件的情况下运行完整测试，记录具体失败项。
3. 若用户只需要已验证的方向修复，应保留现场并让用户明确授权如何处理额外改动。
4. 若用户要继续新算法，应另写计划，恢复合同测试，限制速度，并先进行架空轮低速测试。
5. 实车验证顺序：电机主电源关闭 → 架空驱动轮 → 烧录 → 低速检查左偏产生左转修正 → 再落地测试。
6. 经用户确认后再决定是否把本地 `main` 推送到 GitHub；当前未推送。

## 9. 禁止事项

- 不要在未确认来源前丢弃当前未提交修改。
- 不要把未验证的高速度/高增益代码直接烧录到落地小车。
- 不要交换 M2/M4 来掩盖循迹符号问题。
- 不要绕过 `Motor_Safety_RequestSpeed()`。
- 不要把 C 文件随意转成 UTF-8；现有 C 源文件可能保留 GBK 中文注释。Markdown 文档使用 UTF-8 无 BOM。
- 不要擅自推送、强制重置或删除分支。

## 10. 给下一位 AI 的一句话摘要

已验证的方向修复在提交 `8fb789f`；当前另有一份未提交、风险较高的 PID/速度重写，先保护现场、确认来源、运行测试，再决定保留或回退。
