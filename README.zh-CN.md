# VisionTactile-Dataset

![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)
![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)

[English](README.md) · [新电脑部署](docs/deployment.zh-CN.md) · [桌面工作台](tools/capture_console/README.md) ·
[用户手册](docs/user-manual.md) · [接口参考](docs/interface-reference.md)

VisionTactile-Dataset 提供 ROS 2 采集、桌面工作台和离线对齐，面向两台 D405、
一台 D436、三个 VIVE Ultimate Tracker，以及可选的左右手 IMU 手套和触觉阵列。
公开工作流保留显式设备身份、来源时钟和录制 Topic 清单，为每个 session 管理
一个不压缩的 rosbag Recorder 进程。

这是采集与处理软件，不是可下载的数据集，也不是已完成整机验收的交钥匙产品。
无硬件演示不需要 ROS、设备、私有 Tracker 输入或 MANO 模型；真实采集需要按模块
准备依赖。录包、抓包、凭据、bootstrap、role map、设备外参和许可模型不随仓库分发。

当前以**仅含源码的开发预览版**准备发布。工作台包版本 `0.5.0` 是该组件的版本，
不代表这些变更已发布为同名项目 tag；本次候选范围见
[Unreleased 变更记录](CHANGELOG.md#unreleased)。
独立触觉可视化工具采用单独的 `0.5.1` 版本，支持中文／English。
本仓库提供其 [Windows 安装包源码构建流程](tools/tactile_viewer_windows/README.md)，
本次公开源码准备不包含已获再分发授权的模型，也不提供公开 Windows 安装包下载。

| 工作流 | 本仓库源码之外的前提 |
| --- | --- |
| 合成桌面演示和公开依赖路径的软件测试 | 公开 Python／Qt 依赖；ROS 测试另需 ROS 依赖 |
| RealSense 采集与触觉采集 | 支持的硬件、驱动、设备权限和本机配置 |
| Tracker 生产采集 | 单独获取的扩展 PyVUT、Windows 建图和私有 bootstrap／role 输入；公开 submodule 本身不足以运行 |
| FSGlove／MANO 手形重建 | 单独获取的 FSGlove 源码、对应许可 MANO 模型、标定和可选模型环境 |
| 独立固定手形触觉预览与 Windows 打包 | 串口触觉硬件；获准使用的静态模型包，或许可 MANO 模型及独立模型环境；Windows 构建须显式提供模型及许可文本 |
| 标定后的相机／Tracker 联合视图 | 当前硬件身份绑定且已验收的外参，以及匹配的地图配置 |

固定的公开 PyVUT 提交未包含许可证文件，本项目许可证不授予该代码的使用或再分发权利。
依赖与模型边界见[第三方说明](THIRD_PARTY_NOTICES.md)。合成测试通过不代表整套设备
已可复现、精度已验收或可持续 30 Hz 录制。

## 状态

- [集成采集工作台](tools/capture_console/README.md)处于 v0.5 开发预览阶段：支持桌面启动、
  图形化配置、显式设备启停、四类预览与放大、Tracker 初始化输入持久化、标定／离线对齐入口、
  试次管理与录后质检；已实现后台管理的在线引导标定，真机闭环验收仍待完成。
- 软件目标环境：Ubuntu 24.04、ROS 2 Jazzy、RealSense ROS 4.58.1。
- 手势与触觉支持左右手独立启用，保留单手、仅手势、仅触觉与混合模式；操作见
  [双手启动说明](ros2_ws/src/vt_glove_bridge/README.md#22-单手双手和仅触觉统一入口)。
  左手硬件映射/精度仍待实测，未配置左触觉布局时默认显示原始矩阵。
- 新增联合空间页面：三个 Tracker 在原生公共坐标系中显示；相机身份和外参核验通过后显示三维机身与视锥，支持轨迹、图层、冻结与独立窗口。见[空间配置与操作](tools/capture_console/README.md#三维联合可视化)。
- [档案兼容回放](tools/capture_console/README.md#档案兼容回放)直接嵌入档案页，默认进入“空间”，不设独立“预览”页；在“话题详情”按包内实际 Topic 浏览图像、位姿、触觉或消息字段。缺失数据与质量告警保留提示，不阻断已有数据；切换话题保留播放时间。
- [采集计划](tools/capture_console/README.md#采集计划与录前核对)通过 IPC v5 展示当前实际录制清单，
  默认按可用数据录制，未连接 Tracker 等设备不阻止开始；启动时冻结已接收流供质检检查缺失。
  未接入的可选流只提示缺失信息，稍后上线的数据仍会录入和检查。历史计划与报告不改写。
  不动态修改 Recorder 清单；CameraInfo、机内外参、时间元数据和运行时消息类型尚未实时验证，触觉录制可用性直接观察原始样本到达，不以状态心跳替代。
  实际 bag 完整性仍需录后检查。旧后台保留原流程并明确提示未启用计划校验；历史包与不完整包仍可在档案内回放。
- 工作台按请求编号保留操作回执，设备与录制操作互不覆盖；回放核对响应身份，
  加载失败也释放解码进程。两种 bag 读取器拒绝符号链接路径与非常规存储文件，
  元数据最多 4 MiB、清单最多 4096 个文件，读取期间须保持包已关闭且不被修改。
  不完整包仍可播放已有 Topic。质检可从临时 SQLite 故障恢复，关闭窗口后后台
  日志限制继续工作；已运行后台需在空闲时明确重启后使用新代码。
  详见[恢复与回放约定](docs/architecture.md#durable-recovery-and-background-ownership)和[操作指南](tools/capture_console/README.md)。
- FSGlove 解算结果可通过 `vt_glove_bridge` 发布，工作台可显示动态 MANO 手形或骨架。
  标定仍在原手套窗口完成；旧 `glove_hand_pose` 研究原型保持独立，不属于这条 ROS 桥接链路。
- 触觉工作台支持二维手形、20×20 原始矩阵和静态 MANO 三维手形。色标为原始 ADC，
  不是压力或基线差值；静态触觉与动态手势是独立视图，尚未在工作台同步叠加。
- [独立触觉可视化工具](tools/tactile_viewer_windows/README.md)为 `0.5.1` 中英文双语版，
  支持串口选择、连接／断开和固定手形三维显示。切换语言保留连接与当前数据，独立保存语言偏好。
  Windows x64 安装包构建流程随源码提供，模型须单独授权；Windows 真机串口与显卡验收仍待完成。
- 运行时录制使用 ROS 2 Jazzy 的 `ros2bag` CLI。
- 参考配置不代表硬件曝光同步，也不代表已录数据的质量。
- 已提供相机采集和三 Tracker 只读 ROS 2 发布节点；选定的数据流写入同一个统一
  正式 bag。
- 已提供嵌入工作台的在线引导及独立离线 Tracker–RealSense 手眼标定工具，后台管理
  三轮独立采集、求解、验证、版本化应用和标定 Topic 发布。实际精度、重复性与长期运行仍需真机验收。
  另有使用三组身份绑定外参对齐统一 bag 的离线工具；专用标定 bag 和 Tracker–相机安装外参不进入正式 bag。
- 相机软件可使用公开依赖构建，真实采集仍需对应硬件与本机配置；Tracker Publisher 还需要经批准的扩展 PyVUT
  checkout 和私有 bootstrap bundle，本仓库不提供这些私有输入。

## 默认 bag 契约

默认订阅清单 = **6 Image + 6 CameraInfo + 3 Extrinsics + 3 CameraFrameTiming + 3
TrackerSample = 21 个候选 topics**；实际 bag 按可用数据写入，不要求 Tracker 必须在线。每台相机贡献彩色、深度、彩色与原始深度内参、深度→彩色机内外参和成组时序，
三个物理 Tracker role 各贡献一路 sample；allowlist 明确且不含通配符。后续数据
流必须通过配置显式加入。

机内外参使用 **reliable、transient_local、keep-last depth 1**，相机先启动、
Recorder 后启动也能接收驱动保留的标定消息。其余核心 topic 使用
**keep-last depth 30、volatile**；六路受控相机图像默认请求 **reliable**，
CameraInfo、时序、Tracker 及可选数据源保留 **best_effort** 默认值；逐 topic 的显式配置优先。
受控 RealSense 图像发布端也显式使用可靠 QoS，避免依赖系统默认策略。

`color/camera_info`、`depth/camera_info` 和 `extrinsics/depth_to_color` 直接
保存官方驱动的出厂标定，不依赖 Tracker 或空间页面。机内外参只描述同一相机
深度与彩色之间的关系；相机与 Tracker、其他相机或世界之间的安装外参仍需另行
标定。新 Recipe v2 使用 `unified-dataset-v2`；历史 Recipe v1、
`unified-dataset-v1` 冻结计划保留原四流相机合同和指纹，不回写旧包。共享的
`/tf_static` 不加入录制清单。

工作台与直接启动三相机共用包内 Fast DDS 传输配置，每个 participant 使用有界的
64 MiB 共享内存段；用户显式指定的中间件和传输配置优先。更新后需要停止设备、
重载后台再启动设备，已运行进程不会自动重载 DDS 配置。可靠传输和缓冲不能代替
实际图像完整性验收，见[采集指南](docs/capture-guide.zh-CN.md#相机图像传输)。

`COMPLETE = Recorder process lifecycle complete; it is not a data-quality claim.`
也就是说，`COMPLETE` 只表示 Recorder 进程生命周期完整，不是数据质量声明。

录制后质检逐条检查 Image 消息头，不解码像素，也不以 CameraFrameTiming 的
频率代替图像帧率。有冻结采集计划的工作台试次按 30 Hz 检查配置中的实际图像流，平均
频率容差为 1%，超过 50 ms 的缺口单独告警；有效源时间区间不足一秒时报告样本
不足。旧版无计划试次只显示观测值，不擅自设置目标频率。此检查不认证像素内容
或 RGB-D 配对完整性。

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

这是六路图像流的算术规划值。控制器启动录制前另按计划时长、10% 消息／容器
余量与停止预留检查可用容量；录制中空间过低或查询失败时提前停止并收尾。
默认 300 秒需要约 129.73 GiB，追加 Topic 的容量估计需显式配置。
容量保护不代表写入吞吐量或录制质量已通过验证，详见[采集指南](docs/capture-guide.zh-CN.md)。

## 快速开始

### 1. 获取源码和固定版本的 submodule

```bash
git clone --branch source-preview-20261019 --recurse-submodules \
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

新电脑请先完成[部署指南第 2–6 节](docs/deployment.zh-CN.md#ubuntu)：包含 ROS 官方软件源、Jazzy、固定的 RealSense wrapper `4.58.1` / SDK `2.58.1`、驱动工作区、主系统构建和验证。不要用未锁版本的 apt 包替代固定驱动。本文克隆命令明确选择当前预览分支，保留的 `main` 不代表本次源码。

已有匹配环境时，按[构建步骤](docs/deployment.zh-CN.md#build)重新构建。ROS 使用系统 Python，桌面 Qt 与手势模型分别使用独立环境。首次安装、权限检查和迁移以部署指南为统一入口，避免拼接不同版本的安装命令。

### 3. 安装桌面入口并先运行无硬件演示

```bash
cd "${VT_REPO}"
bash tools/capture_deploy/install.sh
bash tools/capture_deploy/start_demo.sh
```

安装器建立 `.venv-capture-console` 和当前用户的应用菜单／桌面入口，不连接传感器、
不下载许可模型，也不注册开机服务。仅看演示时可先不安装或构建 ROS，只需上一步的
[基础 Python／Qt 系统依赖](docs/deployment.zh-CN.md#ubuntu)；演示数据是合成数据，不录制 bag。

真实工作台需要已构建的 ROS 工作区。关闭演示后运行：

```bash
bash tools/capture_deploy/start_workbench.sh
```

工作台启动或连接本机后台，默认不会启动设备、Tracker 初始化、手套标定或录制。
在“配置设备”中选择实际需要的模块并填写设备身份、串口和路径，再显式启动。
关闭大图或工作台不会停止后台设备；需要停机时先使用录制／设备停止按钮。

从“标定与离线工具 → 相机–追踪器标定”进入在线引导或离线流程。固定实测尺寸的
ChArUco 标定板，移动刚性连接的相机–Tracker 整体；在线任务管理三轮独立采集，
每轮默认至少 40 个求解姿态与 10 个独立验证姿态，重复性通过后才自动应用。
单个离线 bag 只能生成候选结果，三个独立 bag 可用于申请应用。
需按[标定手册](tools/tracker_camera_calibration/README.md)单独安装标定环境。
后台发布 `/calibration/<camera>/tracker_from_camera`，包含设备身份、版本与有效期；
任务占用录制槽期间阻止配置重载和正式录制，界面可断开后重新连接同一后台任务。
上述功能已实现，但最终刚性硬件上的精度、重复性与长期运行尚需独立证据。

新建标定默认采用 `CC200-15-11.25` 参考板：12 × 9 方格、15 mm 方格边长、
11.25 mm 黑色标记边长和兼容的 `DICT_5X5_1000` 字典。有效图案为 180 × 135 mm，
外板标称 200 × 150 mm 不参与求解尺度；使用前须核对实物。导入配置保留原实测参数，
新默认值不改写已有配置和结果。

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
  config_path:="${VT_DATA_ROOT}/config/cameras.yaml" \
  output_root:="${VT_DATA_ROOT}"
```

不要同时从 GUI 和 CLI 启动同一设备。三 Tracker、手动录包及离线对齐的完整命令见
[英文快速开始](README.md#6-prepare-the-tracker-publisher)与下列操作手册。
勾选设备或显示预览不会自动修改录制清单；加入手套、触觉 Topic 时，必须同时显式配置
Recorder allowlist 和对齐输入。离线工具已有手套／触觉专用校验，但不进行手部关节插值。

## 测试与公开发布准备

按 [中文开发指南](CONTRIBUTING.zh-CN.md) 建立隔离环境，分别运行 ROS、工作台和离线
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

- [新电脑部署指南（Ubuntu 主系统／Windows 独立工具）](docs/deployment.zh-CN.md) · [English](docs/deployment.md)
- [Tracker 私有部署包：新机安装与界面导入](docs/tracker-deployment.zh-CN.md) · [English](docs/tracker-deployment.md)
- [相机 USB 权限、缓冲检查与撤销](tools/capture_deploy/REALSENSE_USB.md)

- [桌面安装、设备配置、预览、试次与质检](tools/capture_console/README.md)
- [最终用户手册](docs/user-manual.md)
- [启动参数、Topic、QoS 与命令行接口参考](docs/interface-reference.md)
- [架构与统一 21 topic 契约](docs/architecture.md)
- [采集指南](docs/capture-guide.zh-CN.md)
- [硬件参考](docs/hardware-reference.md)
- [故障排查](docs/troubleshooting.zh-CN.md)
- [VIVE Tracker ROS 2 发布与验收](docs/tracker-ros2-publisher.md)
- [Tracker 与 RealSense 离线外参标定](docs/tracker-camera-calibration.md)
- [Tracker–RealSense 外参标定一本式产品操作手册](tools/tracker_camera_calibration/README.md)
- [统一 bag 离线对齐一本式产品操作手册](tools/multisensor_alignment/README.md)
- [12 路 IMU 手套标定与实体三维手模型（独立原型，尚未接入 ROS）](tools/glove_hand_pose/README.md)
- [官方 FSGlove 核心与 12 路适配（三姿势标定、可微手形优化，需外部源码与 MANO 模型）](tools/fsglove12/README.md)
- [手套 ROS 桥接及左右手独立运行](ros2_ws/src/vt_glove_bridge/README.md)
- [触觉 ROS 2 采集、热力图、录包回放与离线重建](ros2_ws/src/vt_tactile_capture/README.md)：有效矩阵、二维/三维显示和离线导出统一为 20×20；原始包协议保持兼容。
- [独立双语触觉可视化工具与 Windows 源码构建](tools/tactile_viewer_windows/README.md)
- [维护者发布检查表](docs/release-checklist.md)
- [源码检查与导出](tools/release/README.md)
- [开发与测试](CONTRIBUTING.zh-CN.md) · [变更记录](CHANGELOG.md) · [安全报告](SECURITY.md)

## 范围与限制

本版本不宣称跨相机硬件曝光同步，也未实现点云融合、动捕手套专用关节插值或在线
外参 TF 发布。已提供用于可视化的 RGB-D 点云生成，以及相机/Tracker 在线引导与离线手眼标定
与统一 bag 离线对齐，但三组真实
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

软件测试和文档要求见 [中文开发指南](CONTRIBUTING.zh-CN.md)。敏感缺陷请按
[SECURITY.md](SECURITY.md) 中的私密流程报告。

## 许可证

本项目自有文件采用 Apache-2.0，详见 [LICENSE](LICENSE) 与
[第三方说明](THIRD_PARTY_NOTICES.md)；其中 `pyvut` submodule 不因本项目而被
重新许可。

MANO 模型须由用户依其许可自行获取，本次公开源码不分发模型或派生网格缓存。
将静态手模型打入 Windows 安装包不会自动授予该模型的再分发权，公开二进制前须核实适用授权。
原始 FSGlove 源码、批准的扩展 Tracker checkout、本机抓包、bootstrap、role map
及个人配置也不随公开源码包提供；详见 [FSGlove 第三方说明](tools/fsglove12/THIRD_PARTY_NOTICES.md)。
