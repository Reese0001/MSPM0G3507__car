# Handoff Notes

更新时间：2026-07-03

## 我们在做什么

当前任务是执行 `/init`：分析 `D:\DevProject\workspace_ccstheia` 代码库，并创建/改进根目录 `CLAUDE.md`，供后续 Claude Code 实例在该仓库内工作时快速理解项目。

Spec / 需求来源：用户的 `/init` 指令，要求：
- 记录常用构建、测试、开发命令；
- 总结高层代码架构；
- 如果已有 `CLAUDE.md`，提出改进；
- 吸收 README、Cursor/Copilot rules 等重要信息；
- 不编造不存在的任务/命令/测试；
- 文件必须以指定标题和说明开头。

项目背景：MSPM0G3507 + CCS Theia + TI Arm Clang 的 2026 电赛小车平台工程，主体目录为 `empty_LP_MSPM0G3507_nortos_ticlang/`。

## 已经改了哪些文件

已修改：
- `CLAUDE.md`
  - 原文件内容严重编码损坏（乱码），无法可靠保留；已重写为 UTF-8 无 BOM 的清晰版本。
  - 新内容包含：项目概述、行为准则、构建/烧录方式、工具链版本、路径注意事项、代码架构、BSP 模块说明、引脚表、文档与编码约定。

已创建：
- `docs/notes/handoff.md`
  - 本交接文件。

临时文件：
- 曾创建 `claude_decoded_tmp.txt` 用于尝试解码旧 `CLAUDE.md`，已删除。

## 哪些测试通过、哪些没跑

已做的验证 / 检查：
- 探查了根目录结构：确认不是 git 仓库，主要工程目录为 `empty_LP_MSPM0G3507_nortos_ticlang/`。
- 读取并整理了：
  - `empty_LP_MSPM0G3507_nortos_ticlang/empty.c`
  - `empty_LP_MSPM0G3507_nortos_ticlang/SETUP_GUIDE.md`
  - `empty_LP_MSPM0G3507_nortos_ticlang/README.md`
  - `PROJECT_SKILLS.md`（GBK 解码后可读）
  - `BSP/Task/task.c/.h`
  - `BSP/Questions/questions.h`
  - `BSP/Motor/app_motor.h`
  - `BSP/usart.h`
  - `Debug/empty_LP_MSPM0G3507_nortos_ticlang_build.log`
  - `.ccsproject`, `.clangd`, `.mcp.json`
- 确认最近一次构建日志显示工程曾成功构建：`**** Build finished ****`。

未运行：
- 未重新执行 CCS Theia 构建。
- 未烧录 MSPM0G3507 板卡。
- 未运行串口/硬件验证。
- 未运行代码 coverage 或 profiling。
- 未验证新 `CLAUDE.md` 在 IDE 中是否按预期渲染，虽然 Write 工具已确认写入成功。

原因：当前任务只是 `/init` 文档初始化，且工程主要通过 CCS Theia GUI 构建/烧录；没有直接运行硬件测试。

## 下一步计划

1. 向用户汇报已完成 `CLAUDE.md` 重写和本 handoff 写入。
2. 如用户要求继续完善，可检查 `CLAUDE.md` 内容是否需要更贴合用户偏好或补充 `hardware_spec.txt`（当前仓库未发现该文件）。
3. 若进入代码开发阶段，按 `CLAUDE.md` 中的流程执行：
   - 先确认硬件/引脚/供电；
   - 写修改前 Checklist；
   - 再改模块；
   - 最后通过 CCS Theia 构建、烧录、串口观测逐步验证。
4. 如要做真实构建，优先处理路径问题：项目从 `E:\workspace_ccstheia` 移到了 `D:\DevProject\workspace_ccstheia`，构建配置/日志里仍有旧路径。

## 还没说出来的重要发现

- 旧 `CLAUDE.md` 不是普通 GBK 文件，而是已经发生二次/多次编码损坏，解码后仍是大量 `锟斤拷` / `閿熸枻鎷` 乱码；从内容碎片只能还原大意，不能逐字保留。
- `PROJECT_SKILLS.md` 是 GBK 编码，但内容可以成功解码，核心规则是：使用 TI `DL_` DriverLib 风格、电机代码必须带软启动/失控监控、编译问题按 SysConfig 引脚冲突 → 共地 → PWM 时序顺序排查。
- `SETUP_GUIDE.md` 描述的是更完整的小车主程序框架，但当前 `empty.c` 实际上只是 MPU6050 I2C 地址扫描测试，不是指南中提到的完整主循环版本；后续不能假设当前固件已经实现全部比赛任务。
- `empty_LP_MSPM0G3507_nortos_ticlang/docs` 目录不存在；实际资料在根目录 `docs_backup/`。
- `.mcp.json` 中的 `ccs-serial` 仍指向 `E:\Software\ti\css2051\...`，如果本机实际安装路径不同，串口 MCP 会失败。
- 构建日志显示 SDK/CCS/编译器路径也都在 `E:\Software\ti\...`，而当前工作目录在 `D:\DevProject\...`；迁移后 `.cproject` 可能还需要刷新或重新导入工程。
- `.clangd` 明确抑制所有 diagnostics，不能依赖 clangd 报错来判断 C 工程是否健康。
- C 源文件里的中文注释显示为乱码，推断源文件多为 GBK/Windows 中文编码；改 C 文件时要谨慎保持编码，否则注释会进一步损坏。
