# VisionTactile-Dataset

![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)
![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)

[English](README.md) · [桌面工作台](tools/capture_console/README.md) ·
[用户手册](docs/user-manual.md) · [接口参考](docs/interface-reference.md)

VisionTactile-Dataset 提供 ROS 2 采集、桌面工作台和离线对齐，面向两台 D405、
一台 D436、三个 VIVE Ultimate Tracker，以及可选的左右手 IMU 手套和触觉阵列。
公开工作流保留显式设备身份、来源时钟和录制 Topic 清单，为每个 session 管理
一个不压缩的 rosbag Recorder 进程。

这是采集与处理软件，不是可下载的数据集，也不是已完成整机验收的交钥匙产品。
无硬件演示不需要 ROS、设备、私有 Tracker 输入或 MANO 模型；真实采集需要按模块
准备依赖。录包、抓包、凭据、bootstrap、role map、设备外参和许可模型不随仓库分发。

## 状态

- [集成采集工作台](tools/capture_console/README.md)处于 v0.5 开发预览阶段：支持桌面启动、
  图形化配置、显式设备启停、四类预览与放大、Tracker 初始化输入持久化、标定／离线对齐入口、
  试次管理与录后质检；真机闭环验收与标定服务化仍待完成。
- 软件目标环境：Ubuntu 24.04、ROS 2 Jazzy、RealSense ROS 4.58.1。
- 手势与触觉支持左右手独立启用，保留单手、仅手势、仅触觉与混合模式；操作见
  [双手启动说明](ros2_ws/src/vt_glove_bridge/README.md#22-单手双手和仅触觉统一入口)。
  左手硬件映射/精度仍待实测，未配置左触觉布局时默认显示原始矩阵。
- FSGlove 解算结果可通过 `vt_glove_bridge` 发布，工作台可显示动态 MANO 手形或骨架。
  标定仍在原手套窗口完成；旧 `glove_hand_pose` 研究原型保持独立，不属于这条 ROS 桥接链路。
- 触觉工作台支持二维手形、20×20 原始矩阵和静态 MANO 三维手形。色标为原始 ADC，
  不是压力或基线差值；静态触觉与动态手势是独立视图，尚未在工作台同步叠加。
- 运行时录制使用 ROS 2 Jazzy 的 `ros2bag` CLI。
- 参考配置不代表硬件曝光同步，也不代表已录数据的质量。
- 已提供相机采集和三 Tracker 只读 ROS 2 发布节点；选定的数据流写入同一个统一
  正式 bag。
- 已提供独立离线 Tracker–RealSense 手眼标定工具，以及使用三组身份绑定外参对齐
  统一 bag 的离线工具；专用标定 bag 和设备外参不进入正式 bag。
- 相机流程可从公开仓库独立复现；Tracker Publisher 还需要经批准的扩展 PyVUT
  checkout 和私有 bootstrap bundle，本仓库不提供这些私有输入。

## 默认 bag 契约

默认 bag = **6 Image + 3 CameraInfo + 3 CameraFrameTiming + 3
TrackerSample = 15 topics**。每台相机贡献彩色、深度、彩色内参和成组时序，
三个物理 Tracker role 各贡献一路 sample；allowlist 明确且不含通配符。后续数据
流必须通过配置显式加入。

全部 15 个录制 topic 均使用 **keep-last depth 30、best-effort、volatile**
QoS overrides。

`COMPLETE = Recorder process lifecycle complete; it is not a data-quality claim.`
也就是说，`COMPLETE` 只表示 Recorder 进程生命周期完整，不是数据质量声明。

Recorder 以 MCAP 写入，no real-time rosbag or MCAP compression.

标定结果、TF、RealSense 原始 metadata、Tracker status/pose 便利 Topic 和
session 描述不在 bag 中。

## 参考硬件

已测设备身份和 USB 拓扑建议见[硬件参考](docs/hardware-reference.md)。

| 相机 | 型号 | 序列号 | 固件 | ASIC 序列号 |
| --- | --- | --- | --- | --- |
| `d405_1` | D405 | `260322278433` | `5.15.1.55` | `255323071625` |
| `d405_2` | D405 | `260322276463` | `5.15.1.55` | `255323071742` |
| `d436` | D436 | `408322071716` | `5.17.0.213` | `343123151280` |

## 数据速率规划

`1280×720@30 raw estimate = 414.72 MB/s, about 124.4 GB per 300 s.`

这是六路图像流的算术规划值，不是运行时容量门槛。请针对实际目标文件系统进行
测量并预留合适的运行余量。

## 快速开始

### 1. 获取源码和固定版本的 submodule

```bash
git clone --recurse-submodules \
  https://github.com/yanglei18/VisionTactile-Dataset.git
cd VisionTactile-Dataset
export VT_REPO="$(pwd -P)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
mkdir -p "${VT_DATA_ROOT}"
```

已有 clone 可在根目录运行 `git submodule update --init --recursive`；保留仓库固定的
revision，不要随意切到上游最新分支。GitHub 源码 ZIP 不会初始化 submodule。
公开 `pyvut` 不包含 Tracker 生产流程所需的全部扩展模块，仍需另行准备经批准的
checkout；许可边界见[第三方说明](THIRD_PARTY_NOTICES.md)。数据和本机输入放在仓库外。

### 2. 安装依赖并构建 ROS 工作区

使用 Ubuntu 24.04，先完成 ROS 2 Jazzy 安装，再运行：

```bash
sudo apt update
sudo apt install -y git python3-colcon-common-extensions python3-venv \
  python3-yaml python3-tk libegl1 libgl1 libxkbcommon0 libxcb-cursor0 \
  fonts-noto-cjk ros-jazzy-realsense2-camera ros-jazzy-ros2bag \
  librealsense2-utils fio
source /opt/ros/jazzy/setup.bash
# 若本机尚未初始化 rosdep，先执行一次 sudo rosdep init。
rosdep update
rosdep install --from-paths "${VT_WS}/src" --ignore-src -r -y --rosdistro jazzy
cd "${VT_WS}"
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

ROS 使用系统 Python，不在 Conda 中构建或运行。桌面 Qt 和手势模型各自使用独立
环境，不要把 PySide6、PyQt5、PyTorch 混装到 ROS Python。软件测试通过后，仍需针对
自己的硬件完成采集和数据质量验收。

### 3. 安装桌面入口并先运行无硬件演示

```bash
cd "${VT_REPO}"
bash tools/capture_deploy/install.sh
bash tools/capture_deploy/start_demo.sh
```

安装器建立 `.venv-capture-console` 和当前用户的应用菜单／桌面入口，不连接传感器、
不下载许可模型，也不注册开机服务。仅看演示时可先不安装或构建 ROS，只需上一步的
Python venv 与 Qt 系统依赖；演示数据是合成数据，不录制 bag。

真实工作台需要已构建的 ROS 工作区。关闭演示后运行：

```bash
bash tools/capture_deploy/start_workbench.sh
```

工作台启动或连接本机后台，默认不会启动设备、Tracker 初始化、手套标定或录制。
在“配置设备”中选择实际需要的模块并填写设备身份、串口和路径，再显式启动。
关闭大图或工作台不会停止后台设备；需要停机时先使用录制／设备停止按钮。

左右手姿态和触觉可独立启用，仅触觉采集与二维显示不需要 MANO 或 PyTorch。
三维视图需要相应的 `MANO_RIGHT.pkl`／`MANO_LEFT.pkl` 和独立模型环境，均由用户
自行准备；缺失模型不以中立手或合成姿态冒充真实解算。具体见
[工作台手册](tools/capture_console/README.md)和[双手配置](ros2_ws/src/vt_glove_bridge/README.md#22-单手双手和仅触觉统一入口)。

Tracker 向导仅保存本机输入路径与抓包指纹，不保存批准勾选或“已初始化”状态。
每次运行经批准的脚本前仍需显式同意；初始化完成后另行启动只读读取。Windows 配对、
建图、扩展 PyVUT、bootstrap 和 role map 都不是向导或安装器自动提供的内容。

### 4. 可选的命令行采集与离线处理

未从 GUI 启动相机时，可单独启动相机组，不会自动录包：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  output_root:="${VT_DATA_ROOT}"
```

不要同时从 GUI 和 CLI 启动同一设备。三 Tracker、手动录包及离线对齐的完整命令见
[英文快速开始](README.md#6-prepare-the-tracker-publisher)与下列操作手册。
勾选设备或显示预览不会自动修改录制清单；加入手套、触觉 Topic 时，必须同时显式配置
Recorder allowlist 和对齐输入。离线工具已有手套／触觉专用校验，但不进行手部关节插值。

## 测试与公开发布准备

按 [CONTRIBUTING.md](CONTRIBUTING.md) 建立隔离环境，分别运行 ROS、工作台和离线
pytest 测试；Tk GUI 测试需要桌面或 Xvfb，Qt 无硬件测试使用 `offscreen`。
合成数据测试和界面测试不是整套真实硬件已经验收的证据。

```bash
cd "${VT_REPO}"
python3 tools/test_check_public_tree.py
python3 tools/release/prepare_release.py check
```

发布工具检查当前工作区可发布候选，不替你暂存文件；原 `tools/check_public_tree.py`
检查 Git 暂存区。当前树检查通过不表示旧 Git 历史没有抓包、私有数据或第三方许可风险，
首次公开上传前请遵循[源码检查与导出说明](tools/release/README.md)，不要直接推送未经审查的历史。

## 文档

- [桌面安装、设备配置、预览、试次与质检](tools/capture_console/README.md)
- [最终用户手册](docs/user-manual.md)
- [启动参数、Topic、QoS 与命令行接口参考](docs/interface-reference.md)
- [架构与统一 15 topic 契约](docs/architecture.md)
- [采集指南](docs/capture-guide.md)
- [硬件参考](docs/hardware-reference.md)
- [故障排查](docs/troubleshooting.md)
- [VIVE Tracker ROS 2 发布与验收](docs/tracker-ros2-publisher.md)
- [Tracker 与 RealSense 离线外参标定](docs/tracker-camera-calibration.md)
- [Tracker–RealSense 外参标定一本式产品操作手册](tools/tracker_camera_calibration/README.md)
- [统一 bag 离线对齐一本式产品操作手册](tools/multisensor_alignment/README.md)
- [12 路 IMU 手套标定与实体三维手模型（独立原型，尚未接入 ROS）](tools/glove_hand_pose/README.md)
- [官方 FSGlove 核心与 12 路适配（三姿势标定、可微手形优化，需外部源码与 MANO 模型）](tools/fsglove12/README.md)
- [手套 ROS 桥接及左右手独立运行](ros2_ws/src/vt_glove_bridge/README.md)
- [触觉 ROS 2 采集、热力图、录包回放与离线重建](ros2_ws/src/vt_tactile_capture/README.md)：有效矩阵、二维/三维显示和离线导出统一为 20×20；原始包协议保持兼容。
- [维护者发布检查表](docs/release-checklist.md)
- [源码检查与导出](tools/release/README.md)
- [开发与测试](CONTRIBUTING.md) · [变更记录](CHANGELOG.md) · [安全报告](SECURITY.md)

## 范围与限制

本版本不宣称跨相机硬件曝光同步，也未实现点云生成、动捕手套专用关节插值或在线
外参 TF 发布。已提供相机/Tracker 离线手眼标定与统一 bag 离线对齐，但三组真实
外参仍需使用最终刚性硬件逐组标定和验收。Tracker 节点只发布原生 `vive_map`
坐标且不发布 TF。`CameraFrameTiming` 只描述单台相机内部的软件分组；离线工具
基于 host time 匹配并报告覆盖率，不等同于同时曝光。

另提供独立的 12 路 IMU 手套姿态研究原型，支持手动标定、原始 CSV 回放和
实体三维手模型、手掌朝向跟随及局部骨架对照。它不属于已验收的 ROS 采集链路；四指末端关节为推算，
六轴航向仍会漂移，真实手部姿态精度尚未验收。

FSGlove 的 ROS 桥接和工作台显示已接入，但不能据此宣称原型手套已经完成精度验收。
左手真实硬件映射、长时间稳定性及相机／Tracker／手套／触觉整机闭环仍需独立验证。
手部姿态位于每只手的局部参考系，不自动与 Tracker 合成为统一世界坐标。

## 贡献

软件测试和文档要求见 [CONTRIBUTING.md](CONTRIBUTING.md)。敏感缺陷请按
[SECURITY.md](SECURITY.md) 中的私密流程报告。

## 许可证

本项目自有文件采用 Apache-2.0，详见 [LICENSE](LICENSE) 与
[第三方说明](THIRD_PARTY_NOTICES.md)；其中 `pyvut` submodule 不因本项目而被
重新许可。

MANO 模型须由用户依其许可自行获取，本仓库不分发模型或派生网格缓存。
原始 FSGlove 源码、批准的扩展 Tracker checkout、本机抓包、bootstrap、role map
及个人配置也不随公开源码包提供；详见 [FSGlove 第三方说明](tools/fsglove12/THIRD_PARTY_NOTICES.md)。
