# VisionTactile-Dataset 最终用户手册

本文档面向安装、运行、验收和维护人员，是当前开源版本的主操作入口。标准流程为：

```text
准备环境 -> 验证硬件 -> 构建 -> 基础运行 -> 录制/发布 -> 可视化 -> 验收 -> 安全停机
```

本项目是源码交付，不是预装软件包。相机采集与 Tracker 驱动源码均可从公开
仓库构建，其中 PyVUT 通过固定提交的 Git submodule 交付。Tracker 真机运行仍
需要 Windows 已完成建图，以及用户在仓库外保存的私有启动抓包、bundle 和角色
映射；公开仓库不会包含这些设备专属数据。

## 1. 产品范围

当前版本提供：

- Ubuntu 24.04、ROS 2 Jazzy 下的两台 D405 与一台 D436 采集；
- 每台相机的软件时序分组信息；
- 手动 START/STOP 控制的九 topic、无压缩 MCAP 录制；
- 三只已配对、已建图 VIVE Ultimate Tracker 的 Linux 只读 ROS 2 发布；
- Tracker 的 RViz2 与独立桌面监视器；
- Tracker 与 RealSense 彩色光学坐标系的独立离线手眼标定工具；
- 软件测试、真机验收命令和公开仓库检查工具。

当前版本不提供：

- 三相机硬件曝光同步或帧级跨相机对齐；
- 三组设备专属外参成品、在线外参 TF 发布或自动标定数据录制；
- Tracker 配对、建图、固件更新或自动恢复写操作；
- 相机与 Tracker 的联合录制；
- 对录制数据质量、无丢帧或实验适用性的自动保证。

系统模块与数据流见[架构说明](architecture.md)，所有公开参数与 topic 见
[接口参考](interface-reference.md)。

离线外参标定不改变默认九 Topic Recorder。安装最终刚性支架后，使用单独的
calibration bag 逐组生成外参；现场操作人员可直接从头到尾执行
[Tracker–RealSense 外参标定一本式产品操作手册](../tools/tracker_camera_calibration/README.md)，
无需再拼接其他标定文档。该手册以相机和 Tracker ROS Topic 已经稳定发布为明确
输入边界，不负责 Tracker 配对、建图或私有 bootstrap。

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

### 3.1 获取源码

```bash
git clone --recurse-submodules \
  https://github.com/yanglei18/VisionTactile-Dataset.git
cd VisionTactile-Dataset
git submodule update --init --recursive
export VT_REPO="$(pwd -P)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
mkdir -p "${VT_DATA_ROOT}"
```

`VT_DATA_ROOT` 必须是已存在、可写、非符号链接的绝对路径。不要把 bag、图像、
日志、抓包、bundle 或角色映射写入 `VT_REPO`。

### 3.2 安装依赖

```bash
sudo apt update
sudo apt install -y \
  git \
  python3-colcon-common-extensions \
  ros-jazzy-realsense2-camera \
  ros-jazzy-ros2bag \
  librealsense2-utils \
  fio \
  python3-yaml \
  python3-venv \
  python3-tk
```

为 Tracker 创建隔离环境并安装仓库固定的 PyVUT：

```bash
export VT_PYVUT_ROOT="${VT_REPO}/third_party/pyvut"
export VT_PYVUT_VENV="${HOME}/.local/share/visiontactile/pyvut-venv"
python3 -m venv "${VT_PYVUT_VENV}"
"${VT_PYVUT_VENV}/bin/pip" install -e "${VT_PYVUT_ROOT}[pcap]"
git -C "${VT_PYVUT_ROOT}" rev-parse HEAD
```

最后一条命令必须输出本版本文档中固定的 PyVUT commit；不得让 submodule
自动漂移到其他 revision。

构建和运行 shell 中不得激活 Conda：

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
test "${ROS_DISTRO}" = jazzy
```

安装过 ROS 2 并初始化过 `rosdep` 的主机还应同步解析 package dependencies：

```bash
cd "${VT_REPO}"
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y \
  --rosdistro jazzy
```

### 3.3 构建并测试

```bash
cd "${VT_WS}"
colcon build --event-handlers console_direct+
source install/setup.bash
colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

只有零失败时才进入真机流程。每个新终端都要依次 source ROS 2 和同一个
workspace；同一次运行的所有终端必须使用同一个 `ROS_DOMAIN_ID`。

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

## 6. 相机录制

保持第 5 节终端 A 运行，在终端 B 观察状态：

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

默认 bag 必须只有 6 个 `sensor_msgs/msg/Image` 和 3 个
`vt_camera_msgs/msg/CameraFrameTiming`。完整操作细节见
[相机采集指南](capture-guide.md)。

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

三只 Tracker 可以在启动程序前同时开机并恢复常绿；Publisher 不要求用户人为
重复配对。若缺少一路，程序允许其他角色继续发布状态和位姿，验收工具仍会把
三路不完整判为失败。

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
| 相机发现 | 三节点、九个契约 topic 持续可见 | 帧完整或跨相机同步 |
| Recorder | START 后进入 `RECORDING`，最终进入 `COMPLETE` | 数据质量合格 |
| Bag 内容 | `ros2 bag info` 仅含准确九 topic，时长和计数经人工复核 | 实验语义正确 |
| Tracker Publisher | 三角色身份稳定、持续发布 | 与相机坐标已对齐 |
| Tracker 30 秒 | 验收工具输出 PASS | 300 秒长期稳定 |
| 可视化 | 三角色移动对应且健康状态正确 | GUI FPS 等于 Tracker 数据 Hz |
| Tracker–相机外参 | 留出验证不超过 10 mm、1 度，身份绑定正确 | 在线动态定位始终达到同等误差 |
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
6. 按[故障排查](troubleshooting.md)中的症状表执行。

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
- Tracker 不在默认相机 bag 中；外参标定使用独立专用 bag，联合正式采集仍需
  单独设计和验收；
- 离线工具已通过合成真值测试，但三组真实设备外参仍需完成真机标定与重复性
  验收；
- 当前兼容矩阵仅覆盖本手册列出的参考环境，其他组合按“未验证”处理。

版本变更、行为变更与迁移信息见 [CHANGELOG](../CHANGELOG.md)。接口细节见
[接口参考](interface-reference.md)，第三方许可边界见
[THIRD_PARTY_NOTICES](../THIRD_PARTY_NOTICES.md)，贡献规范见
[CONTRIBUTING](../CONTRIBUTING.md)。
