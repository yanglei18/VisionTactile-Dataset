# VIVE Ultimate Tracker 三设备 Windows USBPcap 抓包教程

更新日期：2026-07-30

## 1. 目的

在 Windows 下记录 VIVE Streaming Hub、Wireless Dongle 和三只 VIVE
Ultimate Tracker 的一次完整冷启动过程，供 Ubuntu/PyVUT 提取新的、与当前
配对状态一致的 53 条初始化命令和 7 个事件边界。

本教程描述旧抓包中的 Tracker RF 地址已与当前配对状态不一致时的重新抓包流程。
遇到该情况，旧抓包和旧 `live-bootstrap.json` 不得再次用于真机写入。

## 2. 安全边界

- 抓包文件可能包含设备标识或无线配置，只能存放在私有目录。
- 不得将 `.pcapng`、`capture_notes.txt` 或生成的 bundle 提交到 Git 或上传
  GitHub。
- 不要重置已有地图；仅当 Windows 确认配对已失效时才重新配对。
- 抓包结束后不要再次配对、换 Dongle、重置地图或运行其他 RF 初始化命令，
  否则新抓包可能立即失效。
- Wireshark 不设置 capture filter。需要减少显示内容时只使用 display filter，
  不要因此丢掉初始化阶段的数据包。

## 3. Windows 准备

### 3.1 安装工具

安装 Wireshark，并在安装向导中同时安装 USBPcap。完成后重启 Windows，使用
管理员身份运行 Wireshark。

### 3.2 建立私有目录

在 PowerShell 中执行：

```powershell
New-Item -ItemType Directory -Force D:\VUT_Captures
```

如果电脑没有 `D:` 盘，可改为：

```powershell
New-Item -ItemType Directory -Force C:\VUT_Captures
```

### 3.3 验证 Windows 端状态

1. 将三只 Tracker 配对到本次实验使用的同一个 Wireless Dongle。
2. 保留已经验证可用的地图，不主动重新建图。
3. 确认三只 Tracker 在 Windows 下均能变为绿色并稳定定位。
4. 给三只 Tracker 贴上固定标签，并确定本次开机顺序：
   - Tracker 1：`host`；
   - Tracker 2：`client0`；
   - Tracker 3：`client1`。

标签代表本次抓包中的固定开机顺序，后续 Ubuntu 真机测试也必须保持一致。

## 4. 确认正确的 USBPcap 接口

1. 临时插入 Wireless Dongle。
2. 打开 Wireshark 首页，查看 `USBPcap1`、`USBPcap2` 等接口的 Attached USB
   Devices。
3. 找到包含 HTC/VIVE Dongle 的接口。已知设备标识为
   `VID:PID = 0bb4:0350`。
4. 记住对应接口，例如 `USBPcap2`。
5. 拔下 Dongle。

尽量让 Dongle 所在 Root Hub 不连接键盘、U 盘等其他设备，以减少无关数据和
潜在敏感信息。

## 5. 录制完整冷启动包

### 5.1 初始状态

开始前逐项确认：

- 三只 Tracker 全部关机；
- Wireless Dongle 已拔出；
- VIVE Streaming Hub 已安装且可以正常运行；
- Wireshark 以管理员身份运行；
- 已确认正确的 `USBPcapN` 接口；
- 不进入建图、重置地图或固件升级流程。

### 5.2 严格操作顺序

1. 在 Wireshark 中双击正确的 `USBPcapN`，先开始抓包。
2. 确认列表中已经有 USB 数据流动。
3. 插入 Wireless Dongle。
4. 等待 5～10 秒，让 VIVE Streaming Hub 完成 Dongle 初始化。
5. 开启 Tracker 1（`host`）。
6. 等待它稳定定位，然后缓慢移动 10～15 秒。
7. 开启 Tracker 2（`client0`）。
8. 等待它稳定定位，然后缓慢移动 10～15 秒。
9. 开启 Tracker 3（`client1`）。
10. 等待它稳定定位，然后缓慢移动 10～15 秒。
11. 确认三只 Tracker 全部为绿色。
12. 按 Tracker 1、2、3 的顺序分别再次移动每只设备。
13. 三只均稳定后继续抓包至少 30 秒。
14. 点击 Wireshark 红色停止按钮。

如果某只 Tracker 没有稳定定位，不要把这次文件作为最终包。排除 Windows 端
配对或定位问题后，从“三只关机、Dongle 拔出”重新录制完整流程。

## 6. 保存文件

推荐文件名：

```text
D:\VUT_Captures\vive_three_trackers_startup_2026-07-30.pcapng
```

如果同一天重新录制，不覆盖旧文件，依次使用：

```text
vive_three_trackers_startup_2026-07-30_v2.pcapng
vive_three_trackers_startup_2026-07-30_v3.pcapng
```

同时创建：

```text
D:\VUT_Captures\capture_notes_2026-07-30.txt
```

建议记录：

```text
Capture started:
Dongle inserted:
Tracker 1 (host) powered on / stable:
Tracker 2 (client0) powered on / stable:
Tracker 3 (client1) powered on / stable:
All three green:
Capture stopped:
Map reset/rebuild: no
Firmware update: no
```

## 7. Windows 端验收

在 PowerShell 中执行：

```powershell
$Capture = "D:\VUT_Captures\vive_three_trackers_startup_2026-07-30.pcapng"
Get-Item $Capture | Select-Object FullName, Length, LastWriteTime
Get-FileHash $Capture -Algorithm SHA256
```

如已安装 `tshark`，再检查文件能否正常读取：

```powershell
& "C:\Program Files\Wireshark\tshark.exe" -r $Capture -q -z io,stat,1
```

合格条件：

- 文件大小不为 0；
- Wireshark/tshark 可以重新打开；
- 包含从 Dongle 插入开始的完整数据；
- 包含三只 Tracker 严格依次上线、稳定定位和移动过程；
- 已记录 SHA256；
- 三只 Tracker 在结束前均为绿色。

## 8. 切换回 Ubuntu 前

抓包完成后：

1. 不再重新配对或重新建图。
2. 不更换 Wireless Dongle。
3. 关闭三只 Tracker。
4. 正常关闭 Windows。
5. 拔下 Dongle。
6. 启动 Ubuntu。

## 9. Ubuntu 私有存放与校验

把新文件复制到 Git 仓库之外。以下使用当前用户的私有数据目录，文件名日期应按
实际抓包日期调整：

```text
${HOME}/visiontactile-data/private/vut/vive_three_trackers_startup_2026-07-30.pcapng
${HOME}/visiontactile-data/private/vut/capture_notes_2026-07-30.txt
```

计算 Ubuntu 端 SHA256：

```bash
export VT_VUT_PRIVATE="${HOME}/visiontactile-data/private/vut"
sha256sum "${VT_VUT_PRIVATE}/vive_three_trackers_startup_2026-07-30.pcapng"
```

结果必须与 Windows PowerShell 的 SHA256 完全相同。不同则说明复制损坏，不得
继续生成 bundle。

## 10. 提取新的私有 bootstrap bundle

以下步骤需要单独准备包含 `extract_live_bootstrap_bundle.py` 的获准扩展 PyVUT
checkout 和它的运行环境；公开 submodule 并不保证含有该扩展脚本。
将 `VT_PYVUT_PYTHON`、`VT_PYVUT_SOURCE` 改为实际路径，并将 `VT_CAPTURE_SHA256`
改为刚刚核对一致的摘要：

```bash
export VT_PYVUT_PYTHON="/path/to/approved-pyvut-venv/bin/python"
export VT_PYVUT_SOURCE="/path/to/approved-pyvut-checkout"
export VT_CAPTURE_SHA256="REPLACE_WITH_VERIFIED_SHA256"
"${VT_PYVUT_PYTHON}" \
  "${VT_PYVUT_SOURCE}/scripts/extract_live_bootstrap_bundle.py" \
  --capture "${VT_VUT_PRIVATE}/vive_three_trackers_startup_2026-07-30.pcapng" \
  --expected-sha256 "${VT_CAPTURE_SHA256}" \
  --bundle-output "${VT_VUT_PRIVATE}/live-bootstrap-2026-07-30.json"
```

预期摘要信息应包含：

```text
capture_verified=yes
commands=53
boundaries=7
tracker_count=3
bundle=written
mode=0600
```

检查私有 bundle 权限：

```bash
stat -c '%a %n' "${VT_VUT_PRIVATE}/live-bootstrap-2026-07-30.json"
```

期望权限为 `600`。如果命令数不是 53、事件边界不是 7、设备数不是 3，立即
停止，不进行 Linux 真机写入。

## 11. 最终交接清单

回到 Ubuntu 后需准备以下三项：

- 新 `.pcapng` 的绝对路径；
- Windows 与 Ubuntu 两端一致的 SHA256；
- `capture_notes_2026-07-30.txt` 的绝对路径。

在新抓包完成校验、新 bundle 成功提取且软件测试通过之前，不再执行旧 bundle
或旧 53 条命令。
