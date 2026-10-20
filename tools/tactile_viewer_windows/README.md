# 触觉三维预览 Windows 安装包

[English](README.en.md) · [新电脑部署指南](../../docs/deployment.zh-CN.md) · [中英文用户说明（源码）](USER_GUIDE.txt)

**本分支公开提供源码，不提供可公开下载的 `.exe` 安装包或静态手模型。** 只使用小工具的新用户，请先向交付方取得获准分发的安装包；从源码制作安装包的维护人员按下文准备构建环境和模型输入。完整采集系统与独立 Windows 小工具的部署步骤分别见[新电脑部署指南](../../docs/deployment.zh-CN.md)。

该目录维护独立触觉工具的 Windows x64 构建流程。最终用户只需运行安装程序，不需要安装 Python、ROS、PyTorch 或配置模型路径。安装包完全离线，默认安装到当前用户目录，安装向导和应用均支持简体中文、English，提供桌面/开始菜单入口、覆盖升级和卸载。

安装后的应用只有串口选择、连接/断开、语言选择和固定右手触觉三维显示。共享预览组件的主程序设置、相机预览、手势解算、采集核心模块不进入冻结包；资源按明确清单收集，不包含其他手势工具的配置或启动入口。构建会检查实际模块归档和资源目录，发现额外业务模块就停止发布。

目标系统为 Windows 10 1809 及以上、Windows 11 的 x64 版本，且具有可用的桌面 OpenGL。最低版本与 [Qt 6.11 支持范围](https://doc.qt.io/qt-6.11/supported-platforms.html) 一致；这不代替本应用的 Windows 实机验收。构建脚本必须由 Windows x64 Python 执行。Linux 维护者可以在隔离 Wine 环境调用相同的 Windows 工具链；Wine 检查与真正 Windows 硬件验收应分别记录。

## 使用与语言选择

安装时可选择简体中文或 English；默认跟随 Windows 界面语言，覆盖安装默认使用上次安装语言。安装向导语言用于安装页面、快捷方式和卸载入口。切换安装语言会更新本产品的已知快捷方式，不产生中英两套桌面入口。

应用顶部的语言选项可即时切换界面并保存选择，切换时保持当前连接和数据状态。首次启动采用系统语言，后续优先使用已保存的选择。安装程序不会覆盖应用语言设置；安装向导和应用的语言可以不同。命令行 `--language zh_CN` / `--language en` 仅覆盖本次运行，适用于检查或临时使用。

安装后启动 **触觉三维预览**，选择触觉板的 COM 串口，确认波特率（默认 115200），点击 **连接**。只刷新端口列表不会连接设备。同一串口不要同时交给主采集程序或其他工具使用。拖动旋转模型，滚轮缩放；结束时断开或关闭窗口即可释放串口。拔插或读取错误后需手动重新连接。

固定右手上的颜色表示原始 ADC，不是已标定的压力，也不跟随手指姿态。首次有效数据确定色标范围（最小跨度 100 ADC），本次连接中保持固定；重新连接清除旧画面和范围。状态行的 Hz 是不同源快照进入显示的频率，不能用作硬件扫描频率或 400 点同步采样频率。单块数据有效期为 100 ms；断流后保留的画面会标记为历史。完整显示含义和排障方法见[中英文用户说明](USER_GUIDE.txt)。

升级前关闭应用。卸载时在 Windows 设置中操作，用户偏好保留。用户说明在源码中名为 `USER_GUIDE.txt`，安装后位于程序目录的 **`使用说明.txt`**，同一文件包含中英文内容。

## 构建机准备

只在维护人员的 Windows 构建机安装：

1. CPython 3.12 x64，包含 Python Launcher（`py`）。
2. [Inno Setup 6.7.3](https://jrsoftware.org/isdl.php)。简体中文安装文本已固定保存在本目录 `vendor/`，不依赖编译器安装目录的语言包，来源和许可见该目录说明。
3. 当前仓库源码，以及明确允许用于本次构建/分发的静态右手 JSON 和许可文本。

`requirements-build.lock` 固定全部 Python 构建依赖及 Windows wheel SHA-256。脚本为每次构建创建独立虚拟环境；不激活开发环境，不复制开发机的虚拟环境，不改系统 PATH，也不连接硬件。PySide6 的动态库由 PyInstaller 收集，不采用静态链接。安装脚本采用无 BOM 的 UTF-8，Inno Setup 从 6.3 起已[支持该编码](https://jrsoftware.org/ishelp/topic_unicode.htm)。

编译器版本通过无输出的探测脚本读取官方 `VER` 预处理变量，避免依赖可能为 `0.0.0.0` 的 PE 文件版本。中文安装文本和上游许可均校验固定 SHA-256；`vendor/.gitattributes` 保留上游原始字节，避免不同平台换行转换影响校验。

## 模型资源

构建入口必须明确接收 `--model-bundle` 和 `--model-license`。模型不随源码提交，不从开发机目录自动搜集，也不从网络下载。许可文本只是可追溯记录，**不会自动授予模型的商业使用或再分发权**。MANO 导出的网格仍需按实际许可处理；发布者应提供适用于其分发范围的资源。

模型沿用共享预览的 schema version 1（`neutral_geometry`、`model_axes`），固定 778 顶点、21 个关节。任意其他拓扑不能直接替换，需要先扩展共享映射和验证接口。

已有获准使用的 MANO 环境可导出静态模型，例如在当前 Linux 开发机：

```bash
.venv-fsglove-120hz/bin/python -I -m vt_fsglove12.tactile_preview_export \
  --mano-assets /path/to/authorized/mano-assets --hand right \
  --output /path/to/private-build-inputs/right_hand.json
```

该导出命令只供维护人员执行。用户安装后的程序直接加载 JSON，不需要第二个 Python 环境，也不包含 PyTorch 或原始 `.pkl`。

## 一条命令构建

在 Windows PowerShell 的仓库根目录执行：

```powershell
powershell -NoProfile -ExecutionPolicy Bypass -File .\tools\tactile_viewer_windows\build.ps1 `
  -ModelBundle C:\private-build-inputs\right_hand.json `
  -ModelLicense C:\private-build-inputs\MODEL_LICENSE.txt
```

这里的 `ExecutionPolicy Bypass` 只作用于本次 PowerShell 进程，不更改系统执行策略。若已允许本地脚本，可直接运行 `build.ps1`。也可使用 Python 入口：

```powershell
py -3.12 tools\tactile_viewer_windows\build.py `
  --model-bundle C:\private-build-inputs\right_hand.json `
  --model-license C:\private-build-inputs\MODEL_LICENSE.txt
```

非默认安装的 Inno Setup 可通过 `-Iscc` 或 `--iscc` 指定。`-Output` / `--output` 改变输出目录，路径允许空格和中文。参数始终以数组传递，不拼接 shell 命令。

构建依次执行：输入检查、隔离依赖安装、本地四个 Python 包安装、`pip check`、源程序自检、PyInstaller `onedir`、冻结包审计和自检、许可收集、Inno Setup 编译。任何检查失败即停止，不把不完整目录包装成安装程序。构建日志保存在独立 `build-*` 目录。

输出默认在被 Git 忽略的 `dist/tactile-viewer-windows/release/`：

- `VisionTactile3D-<版本>-windows-x64-setup.exe`：交付给用户的离线安装包。
- 同名 `.exe.sha256`：安装包 SHA-256。
- 同名 `.manifest.json`：模型/许可、依赖锁文件、安装文件的校验值和冻结自检结果。

用户只需要 `.exe` 安装包。源码、构建脚本、wheel 缓存和构建工程包是维护资料，不作为可直接安装的产品交付。

独立工具版本仅从本目录 `product.toml` 的 `version` 字段读取，当前为 `0.5.1`，不随主采集程序的版本联动。稳定 AppId 用于识别同一应用的升级，配置保存在用户目录；卸载不会递归删除用户数据目录。重复构建同一版本会更新该版本的发布文件，历史中间目录保留供诊断，可由维护人员按需清理。

升级依赖 Inno 的安装文件记录，新版本撤销的旧依赖文件可能保留到卸载时；本版不递归清空安装目录。后续可通过版本化运行目录进一步管理升级残留。

## 无网络构建

最终用户安装和使用始终不需要联网。构建机默认从 PyPI 安装固定依赖，也支持预先准备 Windows wheel 缓存：

```powershell
py -3.12 -m pip download --only-binary=:all: `
  -r tools\tactile_viewer_windows\requirements-build.lock -d C:\build-wheels

py -3.12 tools\tactile_viewer_windows\build.py `
  --model-bundle C:\private-build-inputs\right_hand.json `
  --model-license C:\private-build-inputs\MODEL_LICENSE.txt `
  --wheelhouse C:\build-wheels
```

`--wheelhouse` 强制 `--no-index`，本地包构建关闭 build isolation，避免隐式下载其他构建依赖。跨平台准备 wheel 时，增加 `--platform win_amd64 --python-version 312 --implementation cp --abi cp312`。升级依赖时须整体更新锁文件并重新完成冻结包与硬件验收。

## 验证边界与发布验收

平台无关的构建逻辑测试：

```bash
.venv-capture-console/bin/python -m pytest -q tools/tactile_viewer_windows/tests
```

Linux 上验证真实模型输入（不生成安装包）：

```bash
.venv-capture-console/bin/python tools/tactile_viewer_windows/build.py \
  --model-bundle /path/to/private-build-inputs/right_hand.json \
  --model-license /path/to/private-build-inputs/MODEL_LICENSE.txt --check-inputs
```

冻结程序 `VisionTactile3D.exe --self-check C:\temp\tactile-check.json` 检查模型、触觉映射、Qt/OpenGL 导入和主题/语言资源，不打开串口，也不创建图形上下文。它不能代替桌面显示、串口硬件测试。正式交付前在没有 Python/ROS 的干净 Windows 机器完成：

- 离线安装、中文路径、普通用户启动，显示固定手模型且无需配置模型环境。
- 中文/英文安装向导和应用界面、语言即时切换、重启后偏好保留，覆盖升级不覆盖应用语言；切换安装语言不残留重复快捷方式。
- 真实 COM 口连接、触觉变化、拔插/断线历史提示、重新连接及关闭释放串口。
- 本地桌面 OpenGL 绘制、旋转、缩放；确认显卡驱动兼容。
- 同 AppId 升级、再次启动、卸载和快捷方式清理，用户设置仍可保留。

安装包不自动安装 USB 驱动；设备是否使用系统串口驱动需以目标硬件为准。构建默认生成未签名安装包；正式发布可在组织的签名流程中签署程序和安装包，并重新生成最终签名文件的 SHA-256。

维护依据：[PyInstaller 使用参数](https://pyinstaller.org/en/stable/usage.html)、[非管理员安装](https://jrsoftware.org/ishelp/topic_admininstallmode.htm)、[Inno 官方中文翻译](https://jrsoftware.org/files/istrans/)。
