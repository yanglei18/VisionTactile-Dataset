# VisionTactile-Dataset 最终用户手册

配套设备用户从[用户快速上手](quickstart.zh-CN.md)开始：下载完整包，打开图形安装器，安装后从桌面使用。日常界面操作见[工作台说明](../tools/capture_console/README.md)。无需执行本手册中的源码构建和系统维护命令。

本文档保留详细运行、验收和维护操作。维护者从源码部署的标准流程为：

```text
准备环境 -> 验证硬件 -> 构建 -> 基础运行 -> 录制/发布 -> 可视化 -> 验收 -> 安全停机
```

公开仓库提供源码；包含图形安装器的设备专属完整包由维护者构建与交付。
相机采集可从公开源码独立构建。完整包携带对应设备的扩展 PyVUT、私有启动
bundle 与角色映射，并自动准备运行环境；仅下载公开源码不包含这些设备材料。
Tracker 原有配对、建图和实物角色关系仍须有效。

## 1. 产品范围

当前版本提供：

- Ubuntu 24.04、ROS 2 Jazzy 下的两台 D405 与一台 D436 采集；
- 每台相机的软件时序分组信息；
- 手动 START/STOP 控制的 21 个候选 topic、单一无压缩 MCAP 统一录制；
- 三只已配对、已建图 VIVE Ultimate Tracker 的 Linux 只读 ROS 2 发布；
- Tracker 的 RViz2 与独立桌面监视器；
- Tracker 与 RealSense 彩色光学坐标系的主程序引导标定及独立离线手眼标定工具；
- 三相机、三 Tracker 与配置式新增数据流的独立离线对齐工具；
- 独立[触觉 ROS 2 采集与热力图](../ros2_ws/src/vt_tactile_capture/README.md)，
  支持解码 ADC 子包与现有数据流共同录制、离线重建；真实触点布局/扫描速度仍需验收；
- 软件测试、真机验收命令和公开仓库检查工具。

当前版本不提供：

- 三相机硬件曝光同步；
- 三组设备专属外参成品或在线外参 TF 发布；
- Tracker 配对、建图、固件更新或自动恢复写操作；
- 点云融合、动捕手套专用关节插值或在线外参 TF 发布；
- 对录制数据质量、无丢帧或实验适用性的自动保证。

系统模块与数据流见[架构说明](architecture.md)，所有公开参数与 topic 见
[接口参考](interface-reference.md)。

从“标定与离线工具 → 相机–Tracker 标定”进入主程序内的标定工作区。
引导模式要求固定实测尺寸的标定板，移动相机–Tracker 刚性整体。程序自动管理
三轮专用录包，每轮至少 40 个求解姿态和 10 个独立验证姿态，全部通过并满足
重复性要求后保存、应用并发布标定。具体进度以有效姿态和运动充分性为准。
离线模式选择一个 bag 可检查候选结果，选择三个独立 bag 可完成正式验证与应用。
结果详情提供恢复上一版本操作；后台仍会检查设备身份和录制状态。

新会话默认标定板对应 `CC200-15-11.25`：12 × 9 方格、方格边长 15 mm、
标记黑色外边长 11.25 mm，使用与参考图兼容的 `DICT_5X5_1000`。有效图案尺寸
为 180 × 135 mm；商品标称的 200 × 150 mm 是外板尺寸，不参与求解尺度。
使用前核对实物尺寸，不能只依据商品标称或打印设置。导入已有配置会保留其中的
实测板参数，新默认值不改写历史配置或标定结果。

使用前按标定手册安装独立 Python 环境。标定任务由后台拥有，切换页面或关闭
工作台不会自动停止任务；需要停止时使用标定页面的“取消”。后台重启会取消未完成
采集并保留证据，不把未完成的包标为有效。正式采集与标定任务互斥，已有有效结果
在新候选失败时保持不变。配置已提交但发布失败时，页面显示待恢复状态。

独立离线外参标定仍使用单独的 calibration bag 逐组生成外参；现场操作人员可执行
[Tracker–RealSense 外参标定一本式产品操作手册](../tools/tracker_camera_calibration/README.md)，
无需再拼接其他标定文档。该手册以相机和 Tracker ROS Topic 已经稳定发布为明确
输入边界，不负责 Tracker 配对、建图或私有 bootstrap。

正式实验默认候选包括 18 个相机 Topic 与 3 个 TrackerSample Topic。完成
三组外参后，按
[统一 bag 离线对齐一本式产品操作手册](../tools/multisensor_alignment/README.md)
生成可审计的对齐索引；该工具不修改原始 bag。

## 2. 参考环境与硬件

| 项目 | 参考配置 |
| --- | --- |
| 操作系统 | Ubuntu 24.04 |
| ROS | ROS 2 Jazzy |
| RealSense ROS | 4.58.1 |
| 相机 | D405 + D405 + D436，RGB8/Z16，1280x720@30 |
| Tracker | 3 × VIVE Ultimate Tracker + 1 × Wireless Dongle |
| Tracker 前置系统 | Windows 10/11，完成配对、建图和重定位验证 |

测试过的相机固件、序列号和 USB 建议见[硬件参考](hardware-reference.md)。替换
相机、固件、ROS 版本、PyVUT revision 或 USB 控制器后，应视为新硬件组合重新
验收。

三相机原始图像的算术规划速率约为 414.72 MB/s，300 秒约 124.4 GB；这不是
实际写入保证。录制盘必须位于源码仓库之外，并为操作系统保留额外空间。

## 3. 首次安装

新电脑按[部署指南](deployment.zh-CN.md)连续完成以下步骤；该指南集中维护环境安装命令，本手册从第 4 节开始负责日常运行。

1. [准备 Ubuntu 与获取当前分支](deployment.zh-CN.md#ubuntu)：确认 Ubuntu 24.04，克隆 `source-preview-20261019`，建立仓库外的数据目录。
2. [安装 ROS 2 Jazzy](deployment.zh-CN.md#ros)：配置官方软件源，安装系统包、MCAP 与 rosdep。
3. [固定 RealSense 版本](deployment.zh-CN.md#realsense)：wrapper `4.58.1` + SDK `2.58.1`，检查实际加载的版本与路径。
4. [构建并测试](deployment.zh-CN.md#build)：先加载驱动工作区，再构建主工作区；测试无失败后继续。
5. [安装工作台并运行演示](deployment.zh-CN.md#desktop)。
6. [配置权限与设备、完成短包验收](deployment.zh-CN.md#permissions)。

每个新终端重新设置实际的 `VT_REPO`、`VT_WS`、`VT_DATA_ROOT`，先 source ROS，再 source 主工作区的 `install/setup.bash`。同次运行使用相同 `ROS_DOMAIN_ID`；不要在 Conda 中运行 ROS。

如果只用 Windows 独立触觉工具，直接执行[安装包使用流程](deployment.zh-CN.md#windows)，不部署本节的 ROS 系统。

## 4. 每次运行前检查

1. 退出 Conda，关闭可能占用相机、Dongle 或 HID 的其他程序。
2. 确认三台相机与 Tracker Dongle 不竞争同一条受限 USB 上行链路。
3. 检查相机均以 `5000M` 枚举，并核对设备序列号。
4. 检查录制目录空间、写权限和路径。
5. Tracker 运行前确认三只设备已充电、Windows 地图仍有效，且私有 bundle、
   角色映射与本次硬件身份一致。

```bash
lsusb -t
rs-enumerate-devices -s
df -h "${VT_DATA_ROOT}"
test -w "${VT_DATA_ROOT}"
```

正式录制前可选执行存储测试；它会在目标目录写入并清理专用测试文件，应避开
已有采集进程：

```bash
ros2 run vt_realsense_capture storage_bench \
  --output-root "${VT_DATA_ROOT}"
```

## 5. 三相机基础运行（不录制）

基础测试只启动相机、`timing_normalizer` 和 `capture_controller`，不发送 START，
因此不会创建 bag。

终端 A：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  config_path:="${VT_DATA_ROOT}/config/cameras.yaml" \
  output_root:="${VT_DATA_ROOT}"
```

终端 B：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 node list
ros2 topic list
timeout 10 ros2 topic hz /d405_1/color/image_raw
timeout 10 ros2 topic hz /d405_2/color/image_raw
timeout 10 ros2 topic hz /d436/color/image_raw
timeout 10 ros2 topic hz /d405_1/frame_timing
timeout 10 ros2 topic hz /d405_2/frame_timing
timeout 10 ros2 topic hz /d436/frame_timing
```

基础运行通过条件：三台序列号绑定正确、六路图像和三路 `/frame_timing` 均有
输出、没有持续 USB 重连。`Incomplete timing group` 是软件时序诊断，不会阻止
录制；频繁出现时仍需先处理带宽、QoS 或源流缺失问题。

### 5.1 D436 高级实时可视化（不录制）

相机已启动后，可以打开独立的 RViz2 工作台，在同一窗口查看彩色图、深度图、
带 RGB 颜色的三维点云、相机坐标轴和显示状态，并旋转、缩放、测距：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 param set /d436 pointcloud.enable true
ros2 launch vt_realsense_capture camera_visualization.launch.py
```

第一条参数操作只启用驱动的派生点云计算，不改变原始图像的分辨率、深度对齐、
滤波或录制配置；点云增加 CPU/GPU 与数据传输负载，正式采集前应重新观察帧率。
其余部分是 ROS 订阅端，不重新打开相机，也不会发出录包 START。
固定坐标系为 `d436_link`，网格只作相机相对参考，不是已标定的地面、Tracker
地图或世界坐标；测距精度取决于原始深度质量，空洞不会被显示端填补。

左侧 Displays 可查看各显示项状态及开关；鼠标拖动旋转、滚轮缩放，工具栏
Measure 用于两点测距。若性能不足，可取消点云显示，并执行
`ros2 param set /d436 pointcloud.enable false` 关闭派生计算。关闭 RViz 本身不会
停止相机，也不会自动回写相机参数。默认配置针对 D436；其他命名空间可另存
RViz 配置并通过 `rviz_config:=/absolute/path/to/config.rviz` 加载。

## 6. 三相机与三 Tracker 统一录制

以下命令说明完整参考配置。工作台默认按可用数据录制，不要求连接 Tracker；
启动时正在接收的流纳入缺失检查，未接入的可选设备只显示覆盖信息。稍后上线且
在配置清单内的数据仍可录入，实际丢帧和无效数据仍告警。历史档案的检查依据不改写。

先按第 7 节启动并验收三 Tracker Publisher，再保持第 5 节终端 A 运行，在
终端 B 确认完整配置的 21 个核心 Topic 存在并观察状态：

```bash
ros2 topic echo /capture/status
```

终端 C 发送 START。每个不同命令使用新的非空 `request_id`：

```bash
ros2 topic pub --once /capture/command \
  vt_camera_msgs/msg/CaptureCommand \
  "{request_id: run-001, command: 1, session_label: trial, planned_duration_sec: 300}"
```

看到 `RECORDING` 后才算 Recorder 已启动。达到计划时长会自动停止；也可发送：

```bash
ros2 topic pub --once /capture/command \
  vt_camera_msgs/msg/CaptureCommand \
  "{request_id: stop-001, command: 2, session_label: '', planned_duration_sec: 0}"
```

等待状态进入：

```text
RECORDING -> FINALIZING -> COMPLETE
```

不要在 `FINALIZING` 时关闭 launch。`COMPLETE` 只表示 Recorder 进程终止已确认，
不表示数据完整。之后使用实际 `session_id` 检查：

```bash
ros2 bag info "${VT_DATA_ROOT}/<session-id>/bag"
```

默认订阅清单包含 6 个 `sensor_msgs/msg/Image`、6 个
`sensor_msgs/msg/CameraInfo`、3 个 `realsense2_camera_msgs/msg/Extrinsics`、3 个
`vt_camera_msgs/msg/CameraFrameTiming` 和 3 个
`vt_tracker_msgs/msg/TrackerSample`；实际录入取决于可用数据，不要求可选 Tracker
必须在线。每台相机的彩色/原始深度内参和深度→彩色机内外参直接来自官方驱动，
不依赖 Tracker 或空间配置，也不表示获得了相机与外界的安装外参。
新 Recipe v2 使用 `unified-dataset-v2`，历史 v1 Recipe 与冻结合同保持原样。完整操作细节见
[相机采集指南](capture-guide.zh-CN.md)。

录制后必须按[离线对齐手册](../tools/multisensor_alignment/README.md)依次执行
`inspect`、`align` 和 `validate`。Recorder 的 `COMPLETE` 与离线质量
`ACCEPTED` 是两个独立结论。

只想先了解一次录制的消息数、频率、时间间断和触觉覆盖情况时，无需外参，
可先运行[录制后质量报告](../tools/multisensor_alignment/README.md#116-录制结束后的统一质量报告)：

```bash
vt-multisensor-align report-capture \
  --bag "${VT_DATA_ROOT}/<session-id>/bag" \
  --output "${VT_DATA_ROOT}/<session-id>/capture_quality.json"
```

先按离线工具手册安装并 source ROS / workspace；可用 `--config` 指定对齐配置
检查预期流。此报告只描述已录数据，不修改 bag，不给在线 Recorder 增加停止条件。
联合录制按数据源设置缓冲：机内静态外参使用 reliable/transient_local/depth 1，
Recorder 晚于相机启动时仍能获取保留消息；其他相机/Tracker depth 30、
手势 depth 256、触觉原始批次 depth 512。六路受控图像默认请求 reliable，
其他数据源保留 best-effort/volatile；实际值以会话 `qos_overrides.yaml`
为准。这是有限缓冲，不承诺零丢失。

## 7. 三 Tracker 运行

Tracker 流程具有明确的写入/只读边界：Windows 负责配对与建图；经批准的手动
bootstrap 独占 Dongle 并执行已锁定的启动写入；ROS 2 Publisher 随后独占 Dongle
并只读发布。二者不得同时运行。

### 7.1 首次或地图变更后

1. 在 Windows 完成三只 Tracker 配对、房间建图和重启后重定位，见
   [Windows 建图](tracker-windows-map.md)。
2. 在 Linux 完成 USB、单 Tracker 和三 Tracker 门禁，见
   [Linux 验证](tracker-linux-validation.md)。
3. 在仓库外生成并保存私有 bundle；核对批准的抓包摘要。
4. 将 Host、Client 0、Client 1 映射为 `torso`、`left_wrist`、
   `right_wrist`。角色映射只在物理角色确实改变或身份不匹配时重建。

Windows 重新配对可能改变无线槽位，但不应自动改变实验角色。任何配对、建图、
清图、固件更新或未知命令都不由 ROS Publisher 执行。

### 7.2 每次 Linux 启动

按 [Tracker ROS 2 Publisher](tracker-ros2-publisher.md) 执行批准的手动
bootstrap，确认其到达文档规定的 `COMPLETE` 后退出；然后设置：

```bash
: "${VT_BUNDLE:?export VT_BUNDLE as the approved bundle absolute path}"
: "${VT_ROLE_MAP:?export VT_ROLE_MAP as the private role-map absolute path}"
test -r "${VT_BUNDLE}"
test -r "${VT_ROLE_MAP}"
```

启动只读 Publisher：

```bash
ros2 launch vt_vive_tracker triple_tracker.launch.py \
  bundle_path:="${VT_BUNDLE}" \
  role_map_path:="${VT_ROLE_MAP}"
```

启动这里的只读 Publisher 前，三只 Tracker 可以都已开机且常绿；Publisher
本身不要求重复配对。但这不替代手动 bootstrap 的开机顺序：使用
`host-then-clients` 时，必须按交互提示先确认三只关机，再开 Host，最后同时
开启两个 Client。以程序的完整位姿观察结果判断是否成功，不能只凭绿灯。
若缺少一路，程序允许其他角色继续发布状态和位姿，验收工具仍会把三路不完整
判为失败。启动写入中途退出后，不可从已完成序号自行续发命令。

## 8. Tracker 可视化与验收

保持 Publisher 运行，在新的 sourced 终端选择一个或同时启动两个只读消费者：

```bash
ros2 launch vt_vive_tracker tracker_visualization.launch.py
ros2 launch vt_vive_tracker_gui tracker_gui.launch.py
```

颜色固定为 `left_wrist` 青色、`right_wrist` 洋红、`torso` 橙色。健康状态为：

- `FRESH`/绿色：最近 250 ms 内收到有效跟踪位姿；
- `DELAYED`/黄色：位姿年龄为 250–1000 ms，或已连接但当前无完整跟踪；
- `OFFLINE`/红色：从未收到、超过 1000 ms 或角色已断开。

关闭 GUI 或 RViz 不会关闭 Publisher、Dongle 或 Tracker。只出现界面卡顿时，只
重启可视化消费者，不要复位硬件。

移动每只 Tracker，在另一个终端运行 30 秒验收：

```bash
mkdir -p "${VT_DATA_ROOT}/tracker-validation"
ros2 run vt_vive_tracker vt-vive-validate-topics \
  --duration 30 \
  --output "${VT_DATA_ROOT}/tracker-validation/tracker-ros2-30s.json"
```

参考通过输出为：

```text
status=PASS roles=3 identity_swaps=0 dropped=0
```

正式长时实验还应完成 300 秒验证。验收报告只保存在仓库外。

## 9. 交付验收矩阵

| 对象 | 最小通过条件 | 不代表 |
| --- | --- | --- |
| 软件构建 | `colcon test-result` 零失败 | 真机可用 |
| 相机与 Tracker 发现 | 三相机节点、三个 Tracker role 和 21 个核心契约 Topic 持续可见 | 帧完整或跨相机同步 |
| Recorder | START 后进入 `RECORDING`，最终进入 `COMPLETE` | 数据质量合格 |
| Bag 内容 | `ros2 bag info` 完整参考采集包含 21 个核心 Topic 及已声明扩展 Topic，时长和计数经人工复核 | 实验语义正确 |
| Tracker Publisher | 三角色身份稳定、持续发布 | 与相机坐标已对齐 |
| Tracker 30 秒 | 验收工具输出 PASS | 300 秒长期稳定 |
| 可视化 | 三角色移动对应且健康状态正确 | GUI FPS 等于 Tracker 数据 Hz |
| Tracker–相机外参 | 留出验证不超过 10 mm、1 度，身份绑定正确 | 在线动态定位始终达到同等误差 |
| 离线对齐 | `align` 与 `validate` 均退出 0，质量为 `ACCEPTED`，三相机/Tracker 覆盖率达阈值 | 硬件同时曝光 |
| 发布仓库 | 公开树检查、CI、链接和敏感文件检查通过 | 私有 bundle 已随仓库提供 |

交付某次数据时，至少记录 Git commit、配置摘要、OS/ROS/驱动/固件版本、USB
拓扑、session ID、开始与结束时间、bag info、验收 JSON 和已知异常。原始数据与
上述私有证据不要提交到公开仓库。

## 10. 安全停机与恢复

相机录制时：发送 STOP，等待 `COMPLETE`，再以 `Ctrl-C` 结束 launch。没有录制
时可直接结束 launch。

Tracker 运行时：先关闭 GUI/RViz，再以 `Ctrl-C` 停止 Publisher，确认进程释放
Dongle 后再关闭 Tracker 或拔出 Dongle。不要同时启动第二个 PyVUT、验证器或
Publisher 争抢同一 HID。

断电或崩溃后，不要把残留 session 标成成功；保留目录与日志，重新启动一个新
session，并单独检查旧 bag 是否可读。

## 11. 故障处理

优先按下面顺序定位：

1. 核对 shell 的 `ROS_DISTRO`、`ROS_DOMAIN_ID`、source 路径和 Conda 状态；
2. 查看 launch 终端的第一条错误，而不是只看最终退出信息；
3. 检查进程、节点、topic、QoS 和唯一硬件所有者；
4. 检查 `lsusb -t`、供电、线材、设备序列号与内核 USB 日志；
5. 区分“Recorder 生命周期”“数据质量”“可视化显示”三个不同问题；
6. 按[故障排查](troubleshooting.zh-CN.md)中的症状表执行。

公开 issue 应包含版本/commit、环境、硬件/固件、USB 拓扑、执行命令、发生时间、
预期和实际结果以及脱敏日志。安全问题按 [SECURITY.md](../SECURITY.md) 私密提交。

## 12. 数据与开源边界

不得公开提交：

- bag、MCAP、`pcapng`、视频、相机画面和现场数据；
- Tracker bundle、角色映射、设备凭据或可恢复凭据的摘要；
- 包含用户名、主机名、绝对开发路径或私有 USB 身份的原始日志；
- `build/`、`install/`、`log/`、工作树、缓存和打包产物。

硬件参考页列出的测试相机身份是项目主动公开的复现清单；除此之外的私有设备
身份不应进入 Git。维护者发布前必须执行[发布检查表](release-checklist.md)。

## 13. 已知限制与支持

- `vive_map` 是 Windows 建图产生的原生坐标，Publisher 不发布 TF；
- `/frame_timing` 是单相机 color/depth 软件分组，不是三相机曝光同步；
- D436 的设备时间可能重置，壁钟关联应使用明确的 host realtime 字段；
- 三路 TrackerSample 已进入统一正式 bag；外参标定仍使用独立专用 bag；
- 离线对齐采用 host realtime，并分别用相机 `CLOCK_MONOTONIC_RAW` 和 Tracker
  `CLOCK_MONOTONIC` 审计 realtime 跳变，不直接比较两种 monotonic 值；
- 通用扩展流当前支持整条消息的 nearest/previous 引用选择；真实动捕手套的关节
  插值需在消息合同确定后新增 typed adapter；
- 离线工具已通过合成真值测试，但三组真实设备外参仍需完成真机标定与重复性
  验收；
- 当前兼容矩阵仅覆盖本手册列出的参考环境，其他组合按“未验证”处理。

版本变更、行为变更与迁移信息见 [CHANGELOG](../CHANGELOG.md)。接口细节见
[接口参考](interface-reference.md)，第三方许可边界见
[THIRD_PARTY_NOTICES](../THIRD_PARTY_NOTICES.md)，贡献规范见
[中文开发指南](../CONTRIBUTING.zh-CN.md)。
