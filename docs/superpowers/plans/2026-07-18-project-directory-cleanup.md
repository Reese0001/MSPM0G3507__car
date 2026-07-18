# Project Directory Cleanup Implementation Plan

> **For agentic workers:** REQUIRED SUB-SKILL: Use superpowers:subagent-driven-development (recommended) or superpowers:executing-plans to implement this plan task-by-task. Steps use checkbox (`- [ ]`) syntax for tracking.

**Goal:** 将 3.8 GB 历史资料精简为与 MSPM0G3507 两驱平台直接相关、可追溯的参考资料，同时保持 CCS Theia 工程源码和元数据完整、可直接导入。

**Architecture:** 当前 CCS 工程目录保持原位，整理工作限定在 `docs/`、根目录辅助配置和工程生成目录。先建立受保护文件哈希与历史资料清单，再把候选保留项复制到暂存区、过滤生成物、验证完整性，最后落位并删除旧历史树；任何验证失败都在删除前停止。

**Tech Stack:** PowerShell 7/Windows PowerShell、Git/GitHub CLI、TI CCS Theia、TI Arm Clang、MSPM0 SysConfig、SHA-256。

## Global Constraints

- 保持工程目录 `empty_LP_MSPM0G3507_nortos_ticlang/` 和工程名不变。
- 不修改 `empty.c`、`empty.syscfg`、`BSP/`、`targetConfigs/` 的内容或位置。
- 不修改引脚、外设、电机协议、速度、PWM 或 PID；本计划不启动硬件。
- 不手工编辑 `Debug/` 中 CCS 生成的 makefile。
- 不保留 `.rar`、`.zip` 原始压缩包。
- 删除操作必须使用已解析并校验的绝对路径，禁止对工作区根目录或通配路径递归删除。
- C/H 文件保持现有编码和字节内容；以 SHA-256 验证未改变。
- 每个任务完成后提交到 `main`；完成验证后使用 `git push origin main` 上传私有仓库。

---

## File Map

**Create:**

- `_organize_staging/inventory-before.csv`：原始历史资料清单。
- `_organize_staging/protected-before.csv`：当前 CCS 工程受保护文件哈希。
- `_organize_staging/keep/`：最终资料候选区。
- `_organize_staging/keep-manifest.csv`：保留文件来源、目标和哈希。
- `_organize_staging/deletion-summary.md`：删除分类、数量、容量和理由。
- `docs/README.md`：整理后文档入口。
- `docs/archive-manifest.md`：最终整理审计记录。
- `docs/reference/README.md`：参考资料适用范围与风险说明。
- `docs/notes/build-history/`：最近一次构建日志和旧产物元数据。

**Move:**

- `empty_LP_MSPM0G3507_nortos_ticlang/SETUP_GUIDE.md` → `docs/setup/SETUP_GUIDE.md`。
- 筛选后的图片、PDF、MSPM0示例、驱动板协议、MPU6050资料和2024 H题两驱源码 → `docs/hardware/` 与 `docs/reference/`。

**Modify:**

- `AGENTS.md`：修正文档路径与历史资料说明。
- `CLAUDE.md`：修正文档路径与历史资料说明。
- `.gitignore`：整理完成后移除已不存在的暂存规则可选；保留 `Debug/`、`.env` 和构建产物规则。

**Delete only after all gates pass:**

- `docs/docs_backup/`
- `_organize_staging/`
- `empty_LP_MSPM0G3507_nortos_ticlang/Debug/`
- `.vscode/`（已确认仅含无效旧路径）
- `.env`（只在确认无有效变量后）

---

### Task 1: Capture Baseline and Protected Hashes

**Files:**
- Create: `_organize_staging/inventory-before.csv`
- Create: `_organize_staging/protected-before.csv`

**Interfaces:**
- Consumes: `docs/docs_backup/` 和当前 CCS 工程。
- Produces: 后续任务必须使用的原始清单与受保护哈希基线。

- [ ] **Step 1: Confirm clean Git baseline**

Run:

```powershell
git status --short --branch
```

Expected: `## main...origin/main`，其后没有修改或未跟踪文件。

- [ ] **Step 2: Create staging directory without touching source data**

Run:

```powershell
New-Item -ItemType Directory -Force -Path '_organize_staging\keep' | Out-Null
```

Expected: `_organize_staging/keep/` exists and `docs/docs_backup/` remains unchanged.

- [ ] **Step 3: Record the complete historical inventory**

Run:

```powershell
$archiveRoot = (Resolve-Path -LiteralPath 'docs\docs_backup').Path
Get-ChildItem -LiteralPath $archiveRoot -Recurse -File | ForEach-Object {
    [pscustomobject]@{
        RelativePath = $_.FullName.Substring($archiveRoot.Length + 1)
        Length       = $_.Length
        LastWriteUtc = $_.LastWriteTimeUtc.ToString('o')
        Extension    = $_.Extension.ToLowerInvariant()
    }
} | Export-Csv -LiteralPath '_organize_staging\inventory-before.csv' -NoTypeInformation -Encoding UTF8
```

Expected: CSV contains approximately 12,374 data rows and total `Length` is approximately 3.8 GB.

- [ ] **Step 4: Hash all protected CCS inputs**

Run:

```powershell
$projectRoot = (Resolve-Path -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang').Path
$protected = @(
    '.project', '.cproject', '.ccsproject', 'empty.c', 'empty.syscfg'
)
$files = foreach ($name in $protected) { Get-Item -LiteralPath (Join-Path $projectRoot $name) }
$files += Get-ChildItem -LiteralPath (Join-Path $projectRoot 'BSP') -Recurse -File
$files += Get-ChildItem -LiteralPath (Join-Path $projectRoot 'targetConfigs') -Recurse -File
$files | Sort-Object FullName | ForEach-Object {
    [pscustomobject]@{
        RelativePath = $_.FullName.Substring($projectRoot.Length + 1)
        Length       = $_.Length
        SHA256       = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
    }
} | Export-Csv -LiteralPath '_organize_staging\protected-before.csv' -NoTypeInformation -Encoding UTF8
```

Expected: every protected file has a non-empty 64-character SHA-256 value.

- [ ] **Step 5: Validate baseline files**

Run:

```powershell
$inventory = Import-Csv -LiteralPath '_organize_staging\inventory-before.csv'
$protected = Import-Csv -LiteralPath '_organize_staging\protected-before.csv'
if ($inventory.Count -lt 10000) { throw 'Historical inventory is unexpectedly small' }
if ($protected.Count -lt 30) { throw 'Protected file inventory is unexpectedly small' }
if ($protected.Where({ $_.SHA256.Length -ne 64 }).Count -ne 0) { throw 'Invalid protected hash' }
'BASELINE_OK'
```

Expected: `BASELINE_OK`.

---

### Task 2: Build the Curated Hardware and Manual Set

**Files:**
- Create: `_organize_staging/keep/hardware/images/`
- Create: `_organize_staging/keep/hardware/manuals/`

**Interfaces:**
- Consumes: baseline archive tree from Task 1.
- Produces: hardware images and readable PDF manuals for final `docs/hardware/`.

- [ ] **Step 1: Copy hardware images**

Run:

```powershell
$sourceRoot = (Resolve-Path -LiteralPath 'docs\docs_backup').Path
$imageDest = (New-Item -ItemType Directory -Force -Path '_organize_staging\keep\hardware\images').FullName
@('硬件信息', 'images') | ForEach-Object {
    $source = Join-Path $sourceRoot $_
    if (Test-Path -LiteralPath $source) {
        Get-ChildItem -LiteralPath $source -File | Where-Object Extension -In '.png','.jpg','.jpeg' | ForEach-Object {
            Copy-Item -LiteralPath $_.FullName -Destination (Join-Path $imageDest $_.Name)
        }
    }
}
```

Expected: the destination contains the controller, motor, chassis, expansion-board, wiring and grayscale-sensor images.

- [ ] **Step 2: Copy readable PDFs**

Run:

```powershell
$manualDest = (New-Item -ItemType Directory -Force -Path '_organize_staging\keep\hardware\manuals').FullName
Get-ChildItem -LiteralPath 'docs\docs_backup' -Recurse -File -Filter '*.pdf' | ForEach-Object {
    $target = Join-Path $manualDest $_.Name
    if (-not (Test-Path -LiteralPath $target)) { Copy-Item -LiteralPath $_.FullName -Destination $target }
}
```

Expected: five or fewer unique PDF files; no zero-byte files.

- [ ] **Step 3: Validate media integrity**

Run:

```powershell
$media = Get-ChildItem -LiteralPath '_organize_staging\keep\hardware' -Recurse -File
if ($media.Count -lt 10) { throw 'Too few hardware/manual files were retained' }
if ($media.Where({ $_.Length -eq 0 }).Count -ne 0) { throw 'Zero-byte media file found' }
if ($media.Where({ $_.Extension -notin '.png','.jpg','.jpeg','.pdf' }).Count -ne 0) { throw 'Unexpected media type' }
'MEDIA_OK'
```

Expected: `MEDIA_OK`.

---

### Task 3: Curate Relevant Reference Source

**Files:**
- Create: `_organize_staging/keep/reference/motor-controller/`
- Create: `_organize_staging/keep/reference/mspm0-two-wheel/`
- Create: `_organize_staging/keep/reference/eight-tracking/`
- Create: `_organize_staging/keep/reference/mpu6050/`
- Create: `_organize_staging/keep/reference/mspm0-two-wheel/README.md`
- Create: `_organize_staging/keep/reference/competition-2024-h/`
- Create: `_organize_staging/keep-manifest.csv`

**Interfaces:**
- Consumes: exact historical subtrees described below.
- Produces: source-only reference set with traceable origins.

- [ ] **Step 1: Define the allowlist and exclusion rules**

Use these exact source-to-destination mappings:

```powershell
$archiveCode = (Resolve-Path -LiteralPath 'docs\docs_backup\8.程序源码汇总').Path
$mappings = @(
    @{ Source = Join-Path $archiveCode '四路电机驱动板源码\MSPM0\CCS'; Destination = '_organize_staging\keep\reference\motor-controller\mspm0-ccs' },
    @{ Source = Join-Path $archiveCode 'MSPM0底盘传感器扩展源码\双驱\八路巡线模块\CCS'; Destination = '_organize_staging\keep\reference\eight-tracking\two-wheel-ccs' },
    @{ Source = Join-Path $archiveCode '2024年国赛赛题H题\两驱\CCS'; Destination = '_organize_staging\keep\reference\competition-2024-h\two-wheel-ccs' }
)
$excludedDirectories = @('Debug','Release','OBJ','Objects','Listings','build','.metadata','.cache')
$excludedExtensions = @('.o','.obj','.d','.crf','.map','.axf','.elf','.a','.log','.tmp','.bak')
```

Expected: every `Source` exists. If any does not exist, stop and inspect the archive instead of substituting another platform.

- [ ] **Step 2: Copy only human-authored source and necessary project metadata**

Run in the same PowerShell session as Step 1:

```powershell
foreach ($mapping in $mappings) {
    if (-not (Test-Path -LiteralPath $mapping.Source)) { throw "Missing source: $($mapping.Source)" }
    Get-ChildItem -LiteralPath $mapping.Source -Recurse -File | Where-Object {
        $relative = $_.FullName.Substring($mapping.Source.Length + 1)
        $segments = $relative -split '[\\/]'
        ($segments | Where-Object { $_ -in $excludedDirectories }).Count -eq 0 -and
        $_.Extension.ToLowerInvariant() -notin $excludedExtensions -and
        $_.Name -notmatch '\.uvguix\.'
    } | ForEach-Object {
        $relative = $_.FullName.Substring($mapping.Source.Length + 1)
        $target = Join-Path $mapping.Destination $relative
        New-Item -ItemType Directory -Force -Path (Split-Path -Parent $target) | Out-Null
        Copy-Item -LiteralPath $_.FullName -Destination $target
    }
}
```

Expected: each destination contains C/H files and contains no excluded directory.

- [ ] **Step 3: Extract only module-level MPU6050 documentation without retaining the ZIP**

Run:

The ZIP expands to approximately 1.63 GB and contains more than 18,000 mostly STM32 example files. Do not call `Expand-Archive`. Open the ZIP and extract only:

- files under `1-硬件资料_参考资料_模块手册/` with `.pdf`, `.jpg` or `.txt`, excluding `参考资料/` and `例程工具上位机/`;
- `2-配套例程_接线说明/野火小智MPU6050六轴姿态模块例程说明_20240620.pdf`.

Expected: eight module-level files totaling approximately 1.46 MB. The ZIP remains untouched; STM32 board examples, third-party bundles and host applications are not retained.

- [ ] **Step 4: Retain motor-controller firmware separately**

Run:

```powershell
$firmwareDest = '_organize_staging\keep\reference\motor-controller\firmware'
New-Item -ItemType Directory -Force -Path $firmwareDest | Out-Null
Get-ChildItem -LiteralPath 'docs\docs_backup\8.程序源码汇总\四路电机驱动板源码\固件' -File | Where-Object Extension -In '.hex','.bin' | Copy-Item -Destination $firmwareDest
```

Expected: only driver-board `.hex`/`.bin` firmware is retained here.

- [ ] **Step 5: Generate the keep manifest**

Run:

```powershell
$keepRoot = (Resolve-Path -LiteralPath '_organize_staging\keep').Path
$archiveRoot = (Resolve-Path -LiteralPath 'docs\docs_backup').Path
Get-ChildItem -LiteralPath $keepRoot -Recurse -File | ForEach-Object {
    [pscustomobject]@{
        Destination = $_.FullName.Substring($keepRoot.Length + 1)
        Length      = $_.Length
        SHA256      = (Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash
    }
} | Export-Csv -LiteralPath '_organize_staging\keep-manifest.csv' -NoTypeInformation -Encoding UTF8
```

Expected: all records have a 64-character SHA-256 and no destination begins with a build-output directory.

- [ ] **Step 6: Verify no generated artifacts leaked into keep**

Run:

```powershell
$badDir = Get-ChildItem -LiteralPath '_organize_staging\keep' -Recurse -Directory | Where-Object Name -In 'Debug','Release','OBJ','Objects','Listings','build'
$badFile = Get-ChildItem -LiteralPath '_organize_staging\keep' -Recurse -File | Where-Object Extension -In '.o','.obj','.d','.crf','.map','.axf','.elf','.a','.zip','.rar'
if ($badDir -or $badFile) { throw 'Generated artifact leaked into curated keep set' }
'CURATION_OK'
```

Expected: `CURATION_OK`.

---

### Task 4: Write Documentation and Deletion Audit

**Files:**
- Create: `_organize_staging/deletion-summary.md`
- Create: `_organize_staging/keep/README.md`
- Create: `_organize_staging/keep/reference/README.md`
- Create: `_organize_staging/keep/archive-manifest.md`

**Interfaces:**
- Consumes: `inventory-before.csv` and `keep-manifest.csv`.
- Produces: human-readable audit that ships with final `docs/`.

- [ ] **Step 1: Calculate before/after statistics**

Run:

```powershell
$before = Import-Csv -LiteralPath '_organize_staging\inventory-before.csv'
$keep = Import-Csv -LiteralPath '_organize_staging\keep-manifest.csv'
$stats = [pscustomobject]@{
    BeforeFiles = $before.Count
    BeforeBytes = [int64](($before | Measure-Object Length -Sum).Sum)
    KeepFiles   = $keep.Count
    KeepBytes   = [int64](($keep | Measure-Object Length -Sum).Sum)
}
$stats | Format-List
```

Expected: `KeepBytes` is substantially smaller than `BeforeBytes` and `KeepFiles` is smaller than `BeforeFiles`.

- [ ] **Step 2: Write `deletion-summary.md` with exact categories**

The document must state that the following will be deleted after verification: original ZIP/RAR packages; STM32 examples; ESP32/Arduino/PICO/Raspberry Pi/Jetson/RDK examples; K210/K230 demos; MSPM0 four-wheel, PS2, servo, CCD and electromagnetic-tracking projects; 2024 H four-wheel projects; every historical build-output directory and generated object/library/map file.

Expected: the document contains no claim that current CCS source or motor-controller firmware will be deleted.

- [ ] **Step 3: Write the final documentation indexes**

`_organize_staging/keep/README.md` must link `hardware/`, `reference/`, `setup/`, `notes/` and `archive-manifest.md`.

`_organize_staging/keep/reference/README.md` must explain:

- reference source is not compiled into the current project;
- `motor-controller/firmware/` targets the external motor board;
- 2024 H code is historical algorithm reference, not the 2026 final solution;
- current truth remains `empty.syscfg`, `empty.c` and `BSP/`.

`_organize_staging/keep/reference/mspm0-two-wheel/README.md` must link to the retained two-wheel examples in `../eight-tracking/two-wheel-ccs/` and `../competition-2024-h/two-wheel-ccs/`, explaining that this directory is an index rather than a duplicate source copy.

- [ ] **Step 4: Write `archive-manifest.md`**

Include exact counts and bytes from Step 1, retained categories, deleted categories, original archive location, completion date `2026-07-18`, and a link to the approved design document.

- [ ] **Step 5: Review documentation for placeholders and stale paths**

Run:

```powershell
rg -n 'TBD|TODO|待定|稍后' '_organize_staging\keep' '_organize_staging\deletion-summary.md' -g '*.md'
```

Expected: no placeholder matches in Markdown files. Historical source comments are outside this documentation check; `docs_backup` may appear in the audit documents as the former source location.

---

### Task 5: Move Project Documentation and Update References

**Files:**
- Move: `empty_LP_MSPM0G3507_nortos_ticlang/SETUP_GUIDE.md` → `_organize_staging/keep/setup/SETUP_GUIDE.md`
- Preserve copy under Git until final docs commit.
- Modify: `AGENTS.md`
- Modify: `CLAUDE.md`

**Interfaces:**
- Consumes: curated keep tree.
- Produces: final document paths without altering CCS source/include paths.

- [ ] **Step 1: Copy setup guide into staging**

Run:

```powershell
New-Item -ItemType Directory -Force -Path '_organize_staging\keep\setup' | Out-Null
Copy-Item -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang\SETUP_GUIDE.md' -Destination '_organize_staging\keep\setup\SETUP_GUIDE.md'
```

Expected: source and staging copy hashes are equal.

- [ ] **Step 2: Update root documentation references**

Modify both `AGENTS.md` and `CLAUDE.md` so that:

- `SETUP_GUIDE.md` points to `docs/setup/SETUP_GUIDE.md`;
- `docs_backup/` descriptions point to `docs/reference/` and `docs/archive-manifest.md`;
- no statement claims the old raw archive remains after cleanup.

- [ ] **Step 3: Verify references before deleting the old guide**

Run:

```powershell
rg -n 'SETUP_GUIDE\.md|docs_backup|docs/reference|archive-manifest' AGENTS.md CLAUDE.md
```

Expected: new setup/reference paths are present; `docs_backup` appears only in historical context if necessary.

- [ ] **Step 4: Commit root documentation changes before destructive work**

Run:

```powershell
git add AGENTS.md CLAUDE.md docs/superpowers/specs docs/superpowers/plans
git commit -m 'docs: plan project directory cleanup'
git push origin main
```

Expected: commit succeeds and `main` remains synchronized with `origin/main`.

---

### Task 6: Run the Pre-Deletion Safety Gate

**Files:**
- Read: `_organize_staging/protected-before.csv`
- Read: `_organize_staging/keep-manifest.csv`
- Read: CCS project metadata.

**Interfaces:**
- Consumes: every artifact created by Tasks 1–5.
- Produces: explicit `PRE_DELETE_GATE_OK`; Task 7 is forbidden without it.

- [ ] **Step 1: Recalculate protected hashes and compare exactly**

Run:

```powershell
$projectRoot = (Resolve-Path -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang').Path
$before = Import-Csv -LiteralPath '_organize_staging\protected-before.csv'
foreach ($entry in $before) {
    $path = Join-Path $projectRoot $entry.RelativePath
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Protected file missing: $($entry.RelativePath)" }
    $hash = (Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash
    if ($hash -ne $entry.SHA256) { throw "Protected file changed: $($entry.RelativePath)" }
}
'PROTECTED_HASHES_OK'
```

Expected: `PROTECTED_HASHES_OK`.

- [ ] **Step 2: Parse CCS XML and check every BSP include directory**

Run:

```powershell
[xml](Get-Content -Raw -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang\.project') | Out-Null
[xml](Get-Content -Raw -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang\.cproject') | Out-Null
$required = @('BSP','BSP\Motor','BSP\MPU6050','BSP\eMPL','BSP\Eight_Tracking','BSP\Task','BSP\Questions','BSP\LED','BSP\Key','BSP\Buzzer','BSP\Timer')
foreach ($dir in $required) {
    $path = Join-Path 'empty_LP_MSPM0G3507_nortos_ticlang' $dir
    if (-not (Test-Path -LiteralPath $path -PathType Container)) { throw "Missing CCS include directory: $dir" }
}
'CCS_STRUCTURE_OK'
```

Expected: `CCS_STRUCTURE_OK`.

- [ ] **Step 3: Validate keep-manifest hashes**

Run:

```powershell
$keepRoot = (Resolve-Path -LiteralPath '_organize_staging\keep').Path
$manifest = Import-Csv -LiteralPath '_organize_staging\keep-manifest.csv'
foreach ($entry in $manifest) {
    $path = Join-Path $keepRoot $entry.Destination
    if (-not (Test-Path -LiteralPath $path -PathType Leaf)) { throw "Keep file missing: $($entry.Destination)" }
    if ((Get-FileHash -LiteralPath $path -Algorithm SHA256).Hash -ne $entry.SHA256) { throw "Keep hash mismatch: $($entry.Destination)" }
}
'KEEP_HASHES_OK'
```

Expected: `KEEP_HASHES_OK`.

- [ ] **Step 4: Emit the only valid deletion authorization marker**

Run all three previous checks in one fresh PowerShell session. Only after they all pass:

```powershell
Set-Content -LiteralPath '_organize_staging\PRE_DELETE_GATE_OK' -Value '2026-07-18 verified' -Encoding UTF8
```

Expected: marker exists. Absence of this exact marker blocks Task 7.

---

### Task 7: Install Curated Docs and Remove Obsolete Data

**Files:**
- Create final: `docs/hardware/`, `docs/reference/`, `docs/setup/`, `docs/archive-manifest.md`, `docs/README.md`
- Delete: old raw archive, old project `Debug/`, stale `.vscode/`, temporary staging after verification.

**Interfaces:**
- Consumes: `PRE_DELETE_GATE_OK` and curated keep tree.
- Produces: final organized repository.

- [ ] **Step 1: Refuse to proceed without the marker**

Run:

```powershell
if (-not (Test-Path -LiteralPath '_organize_staging\PRE_DELETE_GATE_OK' -PathType Leaf)) { throw 'Pre-delete gate not passed' }
```

Expected: no output and exit code 0.

- [ ] **Step 2: Preserve build history metadata**

Run:

```powershell
$debug = 'empty_LP_MSPM0G3507_nortos_ticlang\Debug'
$history = '_organize_staging\keep\notes\build-history'
New-Item -ItemType Directory -Force -Path $history | Out-Null
Get-ChildItem -LiteralPath $debug -File -Filter '*_build.log' | Copy-Item -Destination $history
Get-ChildItem -LiteralPath $debug -File | Where-Object Extension -In '.out','.bin' | ForEach-Object {
    [pscustomobject]@{ Name=$_.Name; Length=$_.Length; LastWriteUtc=$_.LastWriteTimeUtc.ToString('o'); SHA256=(Get-FileHash -LiteralPath $_.FullName -Algorithm SHA256).Hash }
} | Export-Csv -LiteralPath (Join-Path $history 'previous-artifacts.csv') -NoTypeInformation -Encoding UTF8
```

Expected: historical log is copied and artifact metadata CSV exists; binaries themselves are not retained.

- [ ] **Step 3: Merge keep tree into final docs**

Run:

```powershell
Get-ChildItem -LiteralPath '_organize_staging\keep' -Force | Copy-Item -Destination 'docs' -Recurse -Force
```

Expected: final docs indexes, hardware, reference and setup directories exist.

- [ ] **Step 4: Remove the old tracked setup guide after final copy exists**

Run:

```powershell
if (-not (Test-Path -LiteralPath 'docs\setup\SETUP_GUIDE.md' -PathType Leaf)) { throw 'Final setup guide missing' }
Remove-Item -LiteralPath 'empty_LP_MSPM0G3507_nortos_ticlang\SETUP_GUIDE.md'
```

Expected: only the final guide remains.

- [ ] **Step 5: Delete exact obsolete directories after resolving targets**

Run:

```powershell
$workspace = (Resolve-Path -LiteralPath '.').Path
$targets = @(
    'docs\docs_backup',
    'empty_LP_MSPM0G3507_nortos_ticlang\Debug',
    '.vscode'
)
foreach ($relative in $targets) {
    $candidate = Join-Path $workspace $relative
    if (Test-Path -LiteralPath $candidate) {
        $resolved = (Resolve-Path -LiteralPath $candidate).Path
        if (-not $resolved.StartsWith($workspace + [IO.Path]::DirectorySeparatorChar)) { throw "Unsafe delete target: $resolved" }
        if ($resolved -eq $workspace) { throw 'Refusing to delete workspace root' }
        Remove-Item -LiteralPath $resolved -Recurse -Force
    }
}
```

Expected: only the three exact targets are removed. This step is destructive and not recoverable from Git for ignored raw archives.

- [ ] **Step 6: Inspect `.env` names without exposing values, then decide**

Run:

```powershell
if (Test-Path -LiteralPath '.env') {
    $names = Get-Content -LiteralPath '.env' | Where-Object { $_ -match '^\s*[A-Za-z_][A-Za-z0-9_]*\s*=' } | ForEach-Object { ($_ -split '=',2)[0].Trim() }
    $names
    if ($names.Count -eq 0) { Remove-Item -LiteralPath '.env' }
}
```

Expected: values never print. If any variable names exist, preserve `.env` and report it instead of deleting.

- [ ] **Step 7: Remove extraction temp and staging only after final docs verification**

Run:

```powershell
$requiredFinal = @('docs\README.md','docs\archive-manifest.md','docs\setup\SETUP_GUIDE.md','docs\hardware','docs\reference\README.md')
foreach ($path in $requiredFinal) { if (-not (Test-Path -LiteralPath $path)) { throw "Final docs missing: $path" } }
$staging = (Resolve-Path -LiteralPath '_organize_staging').Path
$workspace = (Resolve-Path -LiteralPath '.').Path
if (-not $staging.StartsWith($workspace + [IO.Path]::DirectorySeparatorChar)) { throw 'Unsafe staging path' }
Remove-Item -LiteralPath $staging -Recurse -Force
```

Expected: `_organize_staging/` is gone; final docs remain.

- [ ] **Step 8: Commit the organized tree**

Run:

```powershell
git add --all
git status --short
git commit -m 'chore: organize project documentation and references'
```

Expected: commit contains curated docs, guide move, stale VS Code removal and documentation updates; it contains no `docs/docs_backup`, `Debug`, archive or object files.

---

### Task 8: Final Structural, Git and CCS Verification

**Files:**
- Verify: complete repository and CCS project.
- Modify only if required: `docs/archive-manifest.md` with final counts/build result.

**Interfaces:**
- Consumes: organized repository from Task 7.
- Produces: evidence-backed completion and synchronized private remote.

- [ ] **Step 1: Verify forbidden data is absent and required data is present**

Run:

```powershell
$forbidden = @('docs\docs_backup','_organize_staging','.vscode','empty_LP_MSPM0G3507_nortos_ticlang\Debug')
foreach ($path in $forbidden) { if (Test-Path -LiteralPath $path) { throw "Obsolete path remains: $path" } }
$required = @('docs\README.md','docs\archive-manifest.md','docs\setup\SETUP_GUIDE.md','docs\reference\README.md','empty_LP_MSPM0G3507_nortos_ticlang\empty.c','empty_LP_MSPM0G3507_nortos_ticlang\empty.syscfg','empty_LP_MSPM0G3507_nortos_ticlang\BSP')
foreach ($path in $required) { if (-not (Test-Path -LiteralPath $path)) { throw "Required path missing: $path" } }
'FINAL_TREE_OK'
```

Expected: `FINAL_TREE_OK`.

- [ ] **Step 2: Verify Git contains no generated or secret files**

Run:

```powershell
$tracked = git ls-files
$bad = $tracked | Where-Object { $_ -match '(^|/)(Debug|Release|OBJ|Objects|Listings|build)/|\.(o|obj|crf|axf|elf|a|zip|rar)$|^\.env$' }
if ($bad) { $bad; throw 'Forbidden tracked files found' }
'GIT_CONTENT_OK'
```

Expected: `GIT_CONTENT_OK`.

- [ ] **Step 3: Verify CCS metadata and include paths again**

Run the XML and required-directory check from Task 6 Step 2.

Expected: `CCS_STRUCTURE_OK`.

- [ ] **Step 4: Attempt a clean CCS build when toolchain paths exist**

Check whether the compiler path recorded in `.cproject`/old build log exists. If CCS Theia is available, run **Build Project** for `empty_LP_MSPM0G3507_nortos_ticlang` and record the new build result in `docs/archive-manifest.md`.

Expected success: a newly generated `Debug/empty_LP_MSPM0G3507_nortos_ticlang.out` and build log with no compiler/linker errors.

Expected environment-blocked result: exact missing CCS/SDK/compiler path is recorded; no claim of successful rebuild is made.

- [ ] **Step 5: Confirm the generated Debug directory remains ignored**

Run:

```powershell
git status --short
git check-ignore -v 'empty_LP_MSPM0G3507_nortos_ticlang/Debug/empty_LP_MSPM0G3507_nortos_ticlang.out'
```

Expected: generated build files do not appear in Git status and `.gitignore` provides the matching rule.

- [ ] **Step 6: Commit any final audit update and push**

Run:

```powershell
git add docs/archive-manifest.md
git diff --cached --quiet; if ($LASTEXITCODE -ne 0) { git commit -m 'docs: record directory cleanup verification' }
git push origin main
git status --short --branch
gh repo view Reese0001/MSPM0G3507__car --json visibility,defaultBranchRef,url
```

Expected:

- local `main` tracks and matches `origin/main`;
- working tree is clean;
- GitHub visibility is `PRIVATE`;
- default branch is `main`.

---

## Completion Report

Report:

- historical file count and bytes before cleanup;
- retained file count and bytes after cleanup;
- exact categories retained and removed;
- CCS protected-file/hash verification result;
- CCS build result or exact external environment blocker;
- final commit SHA and private GitHub repository URL;
- whether `.env` was retained and why, without exposing values.
