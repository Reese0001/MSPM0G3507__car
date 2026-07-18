# 项目目录与历史资料整理设计

日期：2026-07-18

## 目标

在不改变 MSPM0G3507 固件功能、不修改外设配置和电机控制逻辑的前提下，整理仓库根目录和约 3.8 GB 的历史资料。整理后，`empty_LP_MSPM0G3507_nortos_ticlang/` 仍可作为独立 CCS Theia 工程直接导入和编译。

## 硬约束

- 保持当前 CCS 工程名和工程目录不变。
- 保持 `empty.c`、`empty.syscfg`、`BSP/`、`targetConfigs/` 原位。
- 不修改固件运行逻辑、引脚、外设、电机协议、速度参数或 PID。
- 不手工维护 `Debug/` 中由 CCS 生成的 makefile。
- 历史压缩包不保留；仅保留筛选后的关键资料与总结文档。
- 删除旧资料前必须完成保留项核验和当前工程完整性核验。

## 目标目录

```text
MSPM0G3507__car/
├─ AGENTS.md
├─ CLAUDE.md
├─ PROJECT_SKILLS.md
├─ .mcp.json
├─ .agents/
├─ .claude/
├─ .codex/
├─ .theia/
├─ docs/
│  ├─ README.md
│  ├─ archive-manifest.md
│  ├─ setup/
│  │  └─ SETUP_GUIDE.md
│  ├─ hardware/
│  │  ├─ images/
│  │  └─ manuals/
│  ├─ reference/
│  │  ├─ README.md
│  │  ├─ motor-controller/
│  │  ├─ mspm0-two-wheel/
│  │  ├─ eight-tracking/
│  │  ├─ mpu6050/
│  │  └─ competition-2024-h/
│  ├─ notes/
│  └─ superpowers/specs/
└─ empty_LP_MSPM0G3507_nortos_ticlang/
   ├─ .project
   ├─ .cproject
   ├─ .ccsproject
   ├─ .settings/
   ├─ empty.c
   ├─ empty.syscfg
   ├─ BSP/
   ├─ targetConfigs/
   ├─ README.md
   └─ README.html
```

## 当前工程处理

### 保持原位

- `.project`、`.cproject`、`.ccsproject`、`.settings/`
- `empty.c`、`empty.syscfg`
- `BSP/`
- `targetConfigs/`
- 工程内 `README.md` 和 `README.html`

`.cproject` 直接使用 `${PROJECT_ROOT}/BSP/...` 作为包含路径，因此本次不把 BSP 改成 `src/bsp`。`.ccsproject` 指定导入后打开工程内 `README.md`，所以该文件也保留原位。

### 移动或清理

- 工程内 `SETUP_GUIDE.md` 移至 `docs/setup/SETUP_GUIDE.md`，同步修正文档引用。
- `.vscode/` 只包含失效的旧 `E:` 调试路径和宽泛的头文件搜索配置，删除前先记录到清单；若发现仍有有效配置则保留并修复。
- `.theia/launch.json` 保留，因为它引用当前 CCS 工程资源 ID。
- 根目录 `.mcp.json` 保留；旧 CCS 安装路径作为环境待配置项记录，不臆测新的本机路径。
- `.env` 仅在确认不含有效项目变量后删除；检查结果不得泄露变量值。

## Debug 目录处理

当前工程 `Debug/` 含有大量已经不属于当前源码树的 `Debug/docs/...` 残留。处理方式：

1. 将最近一次成功构建日志移至 `docs/notes/build-history/`。
2. 在清单中记录现有 `.out`、`.bin` 的名称、大小、修改时间和 SHA-256。
3. 删除整个 `Debug/` 生成目录。
4. 由 CCS Theia 下一次 Build Project 重新生成干净的 `Debug/`。

旧日志只作为历史证据，不作为整理后的构建验证结果。

## 历史资料筛选

### 保留

- `硬件信息/` 中的全部图片。
- `images/` 中与接线、主控、电机和传感器相关的图片。
- PDF 手册。
- MSPM0 双驱八路巡线 CCS 示例中的人工源码和必要工程配置。
- 四路电机驱动板 MSPM0 CCS 示例中的 UART/IIC 协议人工源码和必要工程配置。
- 2024 H 题两驱 CCS 示例中的核心源码和必要工程配置。
- MPU6050 资料中当前工程未包含的说明、寄存器资料和必要示例。
- 驱动板现成固件，放入 `docs/reference/motor-controller/firmware/`，明确标注目标设备不是 MSPM0 主控板。

### 从保留示例中排除

- `Debug/`、`OBJ/`、`build/`、`Objects/`、`Listings/`。
- `.o`、`.obj`、`.d`、`.crf`、`.map`、`.axf`、`.elf`、`.a` 等可再生成文件。
- IDE 用户状态文件、缓存、日志和临时文件。
- 与已筛选资料重复的压缩包。

### 总结后删除

- STM32 底盘完整示例。
- ESP32、Arduino、PICO、树莓派、Jetson、RDK 的电机示例。
- K210/K230 视觉演示。
- 当前硬件未确认采用的 MSPM0 四驱、PS2、舵机、CCD、电磁循迹完整工程。
- 2024 H 题四驱工程。
- 所有 `.rar` 和 `.zip` 原始压缩包。
- 旧 `docs/docs_backup/` 容器目录。

删除以已分类的目录为边界，不在整个工作区内按扩展名盲删。

## 执行阶段

### 阶段一：清点和暂存

创建 `_organize_staging/`：

```text
_organize_staging/
├─ keep/
├─ inventory-before.csv
├─ keep-manifest.csv
└─ deletion-summary.md
```

- `inventory-before.csv` 记录原路径、类型、大小和修改时间。
- `keep-manifest.csv` 记录最终保留文件的来源路径、目标路径和 SHA-256。
- `deletion-summary.md` 记录被排除的资料类别、容量和理由。
- `keep/` 只存放最终候选资料。

### 阶段二：核验并落位

- 确认保留目录中没有构建输出目录和目标文件。
- 确认图片和 PDF 可读取，关键参考源码不是空目录。
- 确认每个保留文件均可追溯到原路径。
- 确认当前 CCS 工程受保护文件的 SHA-256 未改变。
- 将 `keep/` 合并进最终 `docs/` 结构。
- 生成 `docs/archive-manifest.md` 和 `docs/reference/README.md`。

### 阶段三：删除旧资料和暂存

只有阶段二全部通过后才删除：

- `docs/docs_backup/`
- `_organize_staging/` 中已落位的临时副本
- 已确认无效的 `.vscode/`
- 已确认无项目用途的 `.env`
- 当前工程旧 `Debug/`

删除后重新统计文件数量、容量并写入归档清单。

## 验证标准

### CCS 工程结构

1. `.project`、`.cproject`、`.ccsproject`、`empty.syscfg`、`empty.c` 均存在。
2. `.cproject` 中所有 `${PROJECT_ROOT}/BSP/...` 路径均存在。
3. 受保护 C/H/SysConfig 文件整理前后 SHA-256 完全一致。
4. `.project` 和 `.cproject` XML 可解析。
5. `empty.syscfg` 及目标配置文件没有被移动。
6. CCS 工程目录内不再混入历史资料。

### 构建验证

- 若本机 CCS、SDK 和 TI Arm Clang 路径有效，执行一次完整重建。
- 若构建失败，先判断是缺失源文件还是旧 `E:` 工具链路径造成。
- 目录整理导致的缺失必须恢复后才能结束。
- 外部工具链路径无效则作为环境阻塞如实报告，不能用旧构建日志替代新验证。

### 文档验证

- `AGENTS.md`、`CLAUDE.md` 和目录索引不再引用 `docs_backup/`。
- `SETUP_GUIDE.md` 的新路径可访问。
- `archive-manifest.md` 包含整理前后数量、容量、保留类别和删除类别。

## 失败与恢复

- 保留项核验失败：停止，不删除旧资料。
- 当前工程受保护文件哈希变化：停止，恢复原文件。
- 文档出现断链：修复后再继续删除。
- CCS 元数据缺少路径：恢复相关目录或元数据修改。
- CCS 构建因旧 SDK/编译器绝对路径失败：保留已完成的目录整理，记录准确阻塞路径，不修改未确认的本机安装配置。

## 完成条件

- 历史资料从 3.8 GB 原始集合缩减为与当前 MSPM0G3507 两驱平台直接相关的资料。
- 原始压缩包、异平台完整工程和构建产物已删除。
- 当前固件源码和 SysConfig 哈希不变。
- CCS 工程仍具有完整、可导入的元数据和源文件结构。
- 所有保留和删除决策均能通过归档清单追溯。
