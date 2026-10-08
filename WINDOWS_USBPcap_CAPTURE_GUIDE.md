# VIVE Ultimate Tracker 三设备 Windows USB 抓包指南

## 目标

在 Windows 上捕获 VIVE Hub 启动三只 HTC VIVE Ultimate Tracker 时与
Wireless Dongle 之间的 USB 通信，用于在 Ubuntu/PyVUT 中复现正确的
三 Tracker 初始化流程。

本流程只记录现有地图的正常加载和设备启动过程：

- 不重新建图；
- 不重置地图；
- 不升级固件；
- 不向设备发送自定义 USB 命令；
- 不将抓包上传 GitHub 或任何公共服务。

## 已知硬件

- HTC VIVE Wireless Dongle；
- Dongle USB VID:PID：`0bb4:0350`；
- 三只 VIVE Ultimate Tracker；
- 三只 Tracker 已在 Windows 完成建图；
- 三只 Tracker 已在 Windows 验证可以同时稳定定位。

## 一、准备 Windows 环境

### 1. 初始状态

1. 三只 Tracker 全部关机。
2. Wireless Dongle 暂时不要连接。
3. 重启进入 Windows。

### 2. 创建私有抓包目录

用 PowerShell 执行：

```powershell
New-Item -ItemType Directory -Force C:\VUT-Capture
```

### 3. 安装 Wireshark 和 USBPcap

1. 从 Wireshark 官方网站下载安装程序：
   <https://www.wireshark.org/download.html>
2. 安装时确保勾选：
   - Wireshark；
   - USBPcap。
3. 安装完成后重启 Windows。

USBPcap官方说明：
<https://desowin.org/usbpcap/tour.html>

### 4. 检查 USBPcap

用管理员身份启动 PowerShell，执行：

```powershell
Get-ChildItem "C:\Program Files\Wireshark" -Recurse -Filter "*USBPcap*"
Get-ChildItem "C:\Program Files\USBPcap" -ErrorAction SilentlyContinue
```

用管理员身份打开 Wireshark。首页应出现类似接口：

```text
USBPcap1
USBPcap2
USBPcap3
```

如果没有出现 USBPcap接口，请重新安装 USBPcap并重启 Windows。

## 二、识别 Dongle 所在 USB Root Hub

1. 暂时插入 Wireless Dongle。
2. 在 Wireshark 首页点击每个 `USBPcapN` 旁边的齿轮。
3. 查看 `Attached USB Devices`。
4. 查找下列任一标识：
   - VID：`0bb4`；
   - PID：`0350`；
   - HTC、VIVE 或 Wireless Dongle。
5. 记下对应的 USBPcap接口，例如 `USBPcap2`。
6. 再拔下 Dongle。

尽量选择没有同时连接键盘、U盘等设备的 Root Hub，以减少无关或敏感
USB 数据。

## 三、捕获完整启动过程

### 1. 抓包前状态

- Dongle：未插入；
- 三只 Tracker：关机；
- VIVE Hub：可以打开，但不要进入建图流程；
- Wireshark：以管理员身份运行。

不要点击以下功能：

- `Start setup`；
- `Create tracking map`；
- `Reset map`；
- 固件升级。

### 2. 开始抓包

1. 在 Wireshark 双击已确认的 `USBPcapN` 接口。
2. 确认 Wireshark 已经开始显示数据。
3. 插入 Wireless Dongle。
4. 等待约 5 秒。
5. 打开或切回 VIVE Hub。
6. 开启第一只 Tracker，等待约 5 秒。
7. 开启第二只 Tracker，等待约 5 秒。
8. 开启第三只 Tracker。
9. 等三只全部显示稳定定位。
10. 分别缓慢移动三只 Tracker，每只约 5 至 10 秒。
11. 三只都稳定后继续录制约 30 秒。
12. 点击 Wireshark 红色停止按钮结束抓包。

依次开机有助于后续区分每只 Tracker 的槽位和初始化命令。

## 四、保存抓包与操作记录

### 1. 保存抓包

在 Wireshark 中选择：

```text
File → Save As
```

保存为：

```text
C:\VUT-Capture\vive_three_trackers_startup.pcapng
```

### 2. 创建操作时间记录

创建文件：

```text
C:\VUT-Capture\capture_notes.txt
```

建议记录：

```text
Dongle inserted:
VIVE Hub started:
Tracker 1 powered on:
Tracker 2 powered on:
Tracker 3 powered on:
All three stable:
Capture stopped:
No map creation/reset was performed.
```

时间不需要精确到毫秒，记录大致时刻即可。

## 五、验证抓包

在 PowerShell 中执行：

```powershell
Get-Item C:\VUT-Capture\vive_three_trackers_startup.pcapng |
    Select-Object FullName, Length, LastWriteTime

Get-FileHash C:\VUT-Capture\vive_three_trackers_startup.pcapng `
    -Algorithm SHA256
```

如果安装了 `tshark`，再执行：

```powershell
& "C:\Program Files\Wireshark\tshark.exe" `
    -r C:\VUT-Capture\vive_three_trackers_startup.pcapng `
    -q -z io,stat,1
```

验收要求：

- 文件不是 0 字节；
- Wireshark可以重新打开；
- 能看到 USB/USBPcap 数据；
- 已获得 SHA256；
- 抓包覆盖 Dongle 插入、VIVE Hub 初始化、三只 Tracker 依次上线和稳定定位。

## 六、Windows Codex 交接提示词

在 Windows 中启动 Codex 后，将下面整段文字粘贴给它：

```text
目标：在 Windows 上捕获 VIVE Hub 启动三只 HTC VIVE Ultimate Tracker 的
USB 通信，用于在 Ubuntu/PyVUT 中复现正确的三 Tracker 初始化流程。

硬件：
- HTC VIVE Wireless Dongle，USB VID:PID = 0bb4:0350
- 三只 VIVE Ultimate Tracker
- 三只 Tracker 已在 Windows 完成建图，并验证可以同时稳定定位
- 不需要、也不允许重新建图

请协助完成：
1. 检查 Windows 是否安装 Wireshark 和 USBPcap。
2. 如未安装，指导我从官方 Wireshark 安装程序安装 USBPcap；任何管理员
   操作先让我确认。
3. 创建 C:\VUT-Capture。
4. 帮助识别包含 0bb4:0350 Dongle 的 USBPcap Root Hub。
5. 指导我在 Dongle 未插入、三只 Tracker 关机的状态下开始抓包。
6. 抓包开始后插入 Dongle，启动 VIVE Hub，然后三只 Tracker 依次开机，
   等待全部稳定定位，再录制 30 秒。
7. 不点击 Start setup、Create tracking map、Reset map，不升级固件，
   不发送任何自定义 USB 命令。
8. 保存为
   C:\VUT-Capture\vive_three_trackers_startup.pcapng。
9. 创建 capture_notes.txt 记录每个操作的大致时间。
10. 使用 tshark 验证文件可读取，并计算 SHA256。
11. 不打印、提取或上传其中可能存在的 Wi-Fi凭据、序列号或其他敏感数据。
12. 不把抓包上传 GitHub或任何公共服务。

请一次只指导一个阶段。涉及安装驱动、管理员权限或操作硬件时暂停并
等待我确认。
```

## 七、返回 Ubuntu

将以下文件复制到 Ubuntu：

```text
/home/yanglei/visiontactile-data/private/vut/vive_three_trackers_startup.pcapng
/home/yanglei/visiontactile-data/private/vut/capture_notes.txt
```

然后提供：

- `.pcapng` 的完整路径；
- `capture_notes.txt` 的完整路径；
- Windows 计算出的 SHA256。

抓包内容应始终保存在私有数据目录，不应加入 Git、上传 GitHub或公开分享。
