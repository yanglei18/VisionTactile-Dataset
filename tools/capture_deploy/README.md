# 自动安装器维护说明

普通用户只阅读[快速上手](../../docs/quickstart.zh-CN.md)（[English](../../docs/quickstart.md)）。本文件说明交付构建与维护边界，不要求用户分别下载软件或申请 GitHub 权限。

## 支持范围

自动安装目标为原生 Ubuntu 24.04 x86_64 桌面、普通用户、联网安装、至少 12 GiB 软件可用空间。录制容量另算。安装器包含自身的 Python / Tcl / Tk，先显示中英界面，再通过一次系统管理员认证配置基础库、ROS 2 Jazzy、RealSense SDK `2.58.1`、wrapper `4.58.1`、ROS 工作区、Qt 工作台及对应设备的 Tracker 包。

相机与 Tracker 的设备权限、串口组权限和重启后 USB 缓冲设置也由安装流程处理。安装不启动硬件、配对、建图或录制。实体左右手和相机身份仍由操作员确认；独立手势求解、许可手模型、专用标定环境不在此安装器的自动准备范围内。Windows 独立触觉工具沿用[自己的安装器](../tactile_viewer_windows/README.md)。

## 构建完整交付

在可信、已检查的源码目录和 Ubuntu 24.04 构建机上准备隔离环境；以下命令仅由维护者执行。构建机预先需要系统 Python 3.12、venv、Tcl/Tk 和 `dpkg-query`。

```bash
/usr/bin/python3 -m venv .venv-linux-installer-build
.venv-linux-installer-build/bin/python -m pip install \
  -r tools/release/linux-installer-requirements.txt
python3 tools/release/build_linux_installer.py \
  --python "${PWD}/.venv-linux-installer-build/bin/python" \
  --output "/仓库外的已有父目录/bootstrap-新版本"
python3 tools/release/build_tracker_delivery.py \
  --bootstrap-directory "/仓库外的已有父目录/bootstrap-新版本" \
  --tracker-package "/仓库外的绝对路径/对应设备.vttracker" \
  --output "/仓库外的已有父目录/VisionTactile-新版本.tar.gz"
```

两个输出路径都必须不存在。先冻结安装器源码，再构建二进制和外层交付；外层工具会核对二进制构建时记录的源码摘要，拒绝与当前交付不匹配的旧二进制。完整包包括 `Install-VisionTactile`、程序快照、Tracker 包、中英说明、运行库许可及交付清单。构建不会安装系统软件、访问设备或自动上传。

维护者应核验自身及第三方软件分发权限、保存归档摘要，并在新系统验收后交付。原始归档包含对应设备的私有身份与连接资料，不能以明文公开上传。

当前使用 [Vision Tactile 免登录下载站](https://yanglei18.github.io/VisionTactile-Downloads/)托管加密归档，完整链接单独交给配套设备使用者；浏览器自动校验、解密和保存，用户无需 GitHub 账号、下载密码或手工解密。构建、发布及浏览器验收见[下载托管维护说明](../../docs/download-hosting.md)。完整链接可被转发，不是一次性或自动过期链接，不应放进公开文档或仓库。

如改用官网或私有对象存储，可另行提供限时链接。私有 GitHub Release 仍属于维护端存储，其原始链接需要 GitHub 权限。安装器不包含维护者长期令牌。

不传 `--bootstrap-directory` 时保留旧源码交付方式：外层 `install.sh` 只安装工作台及 Tracker，系统和 ROS 仍由维护者准备。包含图形安装器的新交付中，外层 `install.sh` 则调用同一完整安装流程的无界面入口。

## 安装状态与更新

安装器校验所有源码及设备包摘要，并复制到 `~/.local/share/visiontactile/application/versions/<摘要>/`。下载目录不作为运行目录，完成后可以删除。依赖源码和构建位于 `~/.local/share/visiontactile/dependencies/`，成功记录位于 `~/.local/share/visiontactile/installation/last-success.json`。既有工作台设备配置仍由原配置模块管理。

同一版本可重试，校验发现内容变化会停止。驱动版本冲突或其他程序的 USB 规则由维护者处理，不自动卸载。所有构建和 Python 包安装使用普通用户；系统代理只处理固定的系统动作。取消时普通构建进程退出，已开始的 apt/dpkg 事务会安全结束，之后停止后续步骤。已经安装的依赖保留，不承诺全系统回滚。运行库校验及 Tracker 安装成功后才登记桌面入口。

## 模块职责

| 文件 | 职责 |
| --- | --- |
| `install_full.py` | 图形、无界面、仅校验入口 |
| `oneclick/gui.py` | 中英界面、进度及取消，不直接安装依赖 |
| `oneclick/delivery.py` | 交付内容校验及私有版本目录 |
| `oneclick/engine.py` | 安装顺序、进程管理、重试和最终桌面登记 |
| `oneclick/plan.py`、`contract.py` | 固定版本、目录和构建参数 |
| `install_system.py`、`oneclick/system.py` | 一次授权后的固定系统动作与权限配置 |
| `../release/build_linux_installer.py` | 冻结图形安装器，记录源码及依赖许可 |
| `../release/build_tracker_delivery.py` | 校验并组合完整交付 |

升级 ROS、SDK 或 wrapper 时，统一更新版本与固定提交、系统安装动作及运行时断言，再完成空白系统安装和硬件验收。不要通过界面临时拼接命令或放开任意 root 命令协议。

## 验证

单元测试用合成包、临时文件和伪系统命令，不修改系统或连接设备：

```bash
PYTHONPATH=tools/capture_deploy:tools/tracker_deploy/src \
  xvfb-run -a python -m pytest -q tools/capture_deploy/tests
python3 -m unittest discover -s tools/release/tests -v
```

完整交付的 `Install-VisionTactile --check-only` 仅核验包内容，`--smoke-test` 仅显示并关闭安装窗口。两者都不能替代一次真实安装。干净 Ubuntu 用户态容器可验证软件安装及导入，但不等同于桌面 polkit、udev、systemd、USB 和传感器验收；发布记录必须分别说明实际覆盖范围。
