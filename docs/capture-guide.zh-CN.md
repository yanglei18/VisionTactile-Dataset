# 统一采集指南

[English](capture-guide.md) · [新电脑部署指南](deployment.zh-CN.md)

本指南将三台相机和三个 Tracker 的采样流录制到同一个 bag，录制 Topic 使用明确的允许列表。
命令以 Ubuntu 24.04、ROS 2 Jazzy、RealSense ROS 4.58.1、正常运行的只读 Tracker
Publisher，以及位于源码仓库外的输出目录为前提，描述完整的参考设备组合。
工作台也支持录制已经配置且可用的数据源，无须连接 Tracker；缺少可选数据源表示数据覆盖不完整，
不表示录制流程失败。

在仓库根目录执行一次，定义后续使用的路径：

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
```

## 1. 安装并检查依赖

新电脑请先完成[部署指南](deployment.zh-CN.md)。其中包含 Ubuntu 与
[ROS 2 Jazzy 配置](deployment.zh-CN.md#ros)、`rosdep` 初始化、
[RealSense ROS 4.58.1 安装](deployment.zh-CN.md#realsense)及 APT 候选版本不一致时的处理、
USB 访问权限和工作台环境。完成环境验证后，再按本文执行命令行采集流程。

部署指南安装 Jazzy 的 `ros2bag` 命令和包括 MCAP 在内的必要存储插件。
不需要 rosbag 或 MCAP 压缩插件。

以下命令采用部署指南的源码驱动工作空间。
如果使用指南中另行核验的 APT 路线，省略驱动工作空间的 `source` 命令，保留版本检查。

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
source "${HOME}/visiontactile-drivers_ws/install/setup.bash"
test "$ROS_DISTRO" = jazzy
ros2 pkg xml realsense2_camera |
  sed -n 's:.*<version>\(.*\)</version>.*:\1:p'
ros2 pkg prefix ros2bag
ros2 bag --help
rs-enumerate-devices -s
```

RealSense 包版本命令应输出 `4.58.1`。如果不同，请按[部署指南](deployment.zh-CN.md#realsense)
选择版本，不要直接用更新的 wrapper 替代。项目构建、测试和运行命令使用系统 Python 环境，
不要激活 Conda。

在已经初始化 `rosdep` 的电脑上解析 ROS 包声明的依赖：

```bash
cd "${VT_REPO}"
rosdep update
rosdep install --from-paths ros2_ws/src --ignore-src -r -y \
  --rosdistro jazzy \
  --skip-keys "librealsense2 realsense2_camera realsense2_camera_msgs"
```

跳过的依赖已经由核验过的驱动安装提供；该列表不表示这些依赖可以缺少。

## 2. 构建与测试

正常构建工作空间，不使用符号链接安装：

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
source "${HOME}/visiontactile-drivers_ws/install/setup.bash"
cd "${VT_WS}"
colcon build --event-handlers console_direct+
source install/setup.bash
PYTHONPATH="${VT_REPO}/tools/multisensor_alignment/src:${VT_REPO}/ros2_ws/src/vt_realsense_capture${PYTHONPATH:+:$PYTHONPATH}" \
  colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

软件测试报告零失败后再继续。不要将 `build/`、`install/` 和 `log/` 提交到 Git。
主工作空间的 `install/setup.bash` 会保留构建时选定的驱动 underlay；
后续启动时需要继续保留该驱动工作空间。

## 3. 准备存储与 USB 连接

在仓库外创建一个已存在、可写的绝对路径输出根目录：

```bash
mkdir -p "${VT_DATA_ROOT}"
test "$(realpath "${VT_DATA_ROOT}")" = "${VT_DATA_ROOT}"
test -w "${VT_DATA_ROOT}"
df -h "${VT_DATA_ROOT}"
lsusb -t
```

原始图像的规划数据量为 414.72 MB/s，按配置的最长 300 秒试次估算约为 124.4 GB。
控制器会在创建试次前、启动 Recorder 前分别检查剩余容量，预算包含计划时长、
10% 消息和容器开销，以及包含 Recorder 缓存的停止预留空间。
默认情况下，30 秒、60 秒、300 秒分别需要约 15.02 GiB、27.77 GiB、129.73 GiB。
该估算不依赖压缩或丢帧节省空间。容量检查通过后，仍需验证持续写入吞吐和实际录制质量。

录制期间每 0.5 秒检查一次剩余空间。达到停止预留量或无法查询容量时，
系统进入既有的停止和收尾流程。提前退出得到确认时会保留原因；
退出尚未确认时继续保持 `FINALIZING`。如果其他进程突然耗尽文件系统，
或 Recorder 无法正常停止，预留空间不能保证恢复。

可选设置位于所选相机 YAML 的 `recording.disk_guard`，与其他录制设置并列：

```yaml
disk_guard:
  min_free_bytes: 2147483648
  check_interval_sec: 0.5
  additional_stream_bytes_per_second: 0
```

实际停止预留空间不小于 `min_free_bytes`，可能进一步增加，以覆盖缓存和正常停止延迟。
附加数据流估算是额外 Topic 的合计字节速率。零表示未配置估算；
存在额外 Topic 时，控制器会明确标记该预算只覆盖已知图像流。
运行时剩余空间检查仍然生效。设置在下一次启动控制器时生效。

如果 `lsusb -t` 显示 D436 与两台 D405 共用同一条受限根路径，
应将 D436 连接到独立的 USB 根总线。不同物理插孔不一定属于不同主机控制器。

以下命令只检查 USB 访问权限和 RSUSB 缓冲区预算，不打开相机：

```bash
python3 tools/capture_deploy/configure_realsense_usb.py check
```

需要修正时，按 [RealSense USB 检查、应用与回滚指南](../tools/capture_deploy/REALSENSE_USB.md)
操作。应用修改前先停止相机进程。辅助工具的缓冲区设置仅在本次开机期间有效；
重启电脑后再次运行 `check`。RSUSB 缓冲区要求不能用于判断 V4L2 丢帧原因。

正式录制前，可以选择测试目标文件系统。以下命令写入专用的 16 GiB 测试文件，
结束后删除该数据文件并保留 JSON 报告；它不属于 Recorder 生命周期的准入条件：

```bash
ros2 run vt_realsense_capture storage_bench \
  --output-root "${VT_DATA_ROOT}"
```

## 4. 启动 Tracker Publisher

完成经批准的手动初始化后，按 [Tracker 操作手册](tracker-ros2-publisher.md)，
在独立终端启动只读 Publisher：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_vive_tracker triple_tracker.launch.py \
  bundle_path:="${VT_BUNDLE}" \
  role_map_path:="${VT_ROLE_MAP}"
```

Recorder 不负责启动或配置 Tracker 硬件。需要三个定位姿态的实验，
应在 START 前检查三路 `/vive/<role>/sample`。
工作台录制本身不要求 Tracker：新建计划会固定当时正在接收的数据源，
未出现的候选数据源保持可选。配置允许列表中的数据源稍后上线时仍会被录制。

## 5. 启动相机与控制器

先按[部署指南](deployment.zh-CN.md#permissions)建立 `config/cameras.yaml` 本机副本并填写实际设备身份。

让启动进程保持在终端前台：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  config_path:="${VT_DATA_ROOT}/config/cameras.yaml" \
  output_root:="${VT_DATA_ROOT}"
```

该启动入口运行三个按序列号绑定的 RealSense 节点、`timing_normalizer`
和 `capture_controller`。只有接受 `START` 命令后才开始录制。

## 6. 检查实时 Topic

另开终端，加载相同 ROS 环境并使用相同 `ROS_DOMAIN_ID`，然后检查实时数据：

```bash
ros2 topic list
for topic in \
  /d405_1/color/image_raw /d405_1/depth/image_rect_raw \
  /d405_1/color/camera_info /d405_1/frame_timing \
  /d405_1/depth/camera_info /d405_1/extrinsics/depth_to_color \
  /d405_2/color/image_raw /d405_2/depth/image_rect_raw \
  /d405_2/color/camera_info /d405_2/frame_timing \
  /d405_2/depth/camera_info /d405_2/extrinsics/depth_to_color \
  /d436/color/image_raw /d436/depth/image_rect_raw \
  /d436/color/camera_info /d436/frame_timing \
  /d436/depth/camera_info /d436/extrinsics/depth_to_color \
  /vive/left_wrist/sample /vive/right_wrist/sample /vive/torso/sample
do
  ros2 topic info -v "$topic"
done
```

时间信息 Topic 的类型为 `vt_camera_msgs/msg/CameraFrameTiming`；
Tracker 采样 Topic 的类型为 `vt_tracker_msgs/msg/TrackerSample`。
实时图中还可能存在 TF、RealSense 原始元数据、Tracker pose/status 辅助 Topic 和控制 Topic，
但这些不写入 bag。当前允许列表有 21 个候选 Topic，
包含每台相机的彩色/深度 CameraInfo 和出厂 `extrinsics/depth_to_color`。
外参订阅采用 reliable、transient-local、keep-last 深度 1，
用于接收此前启动的相机保留的标定消息。其余核心 Topic 使用 keep-last 深度 30、volatile：
六路受管图像订阅请求 reliable，CameraInfo、时间信息和 Tracker 则保持 best-effort。

出厂标定信息的录制不依赖 Tracker 或空间视图配置。它描述单台相机的内参、
以及内部深度到彩色的变换，不是相机外部安装变换或世界坐标变换。
新 Recipe v2 使用 `unified-dataset-v2`；已保存的 Recipe v1 计划继续保留原来的
相机四 Topic 组合及指纹。检查已发布的出厂外参时，使用 transient-local 读取：

```bash
ros2 topic echo --once --qos-reliability reliable --qos-durability transient_local \
  /d405_1/extrinsics/depth_to_color
```

### 相机图像传输

受管 RealSense 发布端明确使用 wrapper 的 `DEFAULT` 彩色和深度 QoS：
reliable、volatile、keep-last 深度 10，并非未指定的系统默认值。
这些节点参数位于 `realsense_options.yaml`，Recorder 使用兼容的 reliable 图像订阅。
外部图像发布端和可选数据源不默认视为提供 reliable 传输。

工作台管理后台和直接调用 `triple_realsense.launch.py` 的入口使用同一份打包的 Fast DDS
配置，除非操作者提供了自己的中间件或传输配置。
该配置为每个 participant 分配 64 MiB 共享内存段和 4096 项端口队列，
同时保留 UDPv4 发现和跨主机通信。传输消息上限仍为 65500 字节，
因此大图像仍然需要分片。每台相机约产生 138.24 MB/s 的原始 RGB8/Z16 载荷；
共享内存段提供有上限的突发缓冲，不会无限保存图像历史。

配置在 ROS 初始化前应用，由相机、控制器和 Recorder 进程继承。
它独立于 1 GiB rosbag 写入缓存；写入缓存无法恢复到达 Recorder 前已经丢失的消息。
修改传输配置后，需要停止并重新启动已有进程。显式 DDS 配置及非 Fast DDS
中间件仍由操作者控制。这些设置不保证无丢帧：应检查录制后各路图像的时间戳、
帧率、间隔，以及 Recorder 的传输丢失日志。

### 高速手套与触觉录制缓冲区

可选的 `cameras.with_right_glove_tactile.yaml` 配置在 21 个核心 Topic 之外，
额外录制已解算的 `/gloves/right/state` 和原始 `/gloves/right/tactile/samples`。
Recorder 按**配置的消息类型**选择订阅历史深度，不根据 Topic 名称猜测：

| 录制的消息类型 | 默认 keep-last 深度 |
| --- | ---: |
| `vt_tactile_msgs/msg/TactileSampleArray` | 512 |
| `vt_glove_msgs/msg/GloveState` | 256 |
| 受管出厂外参 | 1（reliable、transient-local） |
| 其他相机、Tracker 及其他类型 | 30 |

可选数据源保持 `best_effort` / `volatile`；发布端设置、传感器采样率和 20×20
触觉状态格式不变。按每秒 500 批触觉消息计算，512 项约提供一秒的订阅历史，
**不保证无丢帧**。DDS 资源上限、发布端历史、持续存储吞吐和既有 rosbag2
缓存仍然影响结果。增加历史深度有助于吸收短时调度波动，不能解决持续写入速度不足。

如需针对具体部署调整，可在相机配置已有的 `recording` 部分加入精确 Topic 的覆盖值，
不要替换该部分的其他键：

```yaml
recording:
  # 保留已有文件中的 max_bag_duration_seconds、max_bag_size_bytes、
  # max_cache_size_bytes 和 additional_streams。
  qos_overrides:
    /gloves/right/tactile/samples:
      depth: 1024
    /gloves/right/state:
      depth: 256
      reliability: best_effort
```

仅接受 `depth`（1–4096 的整数）和 `reliability`（`best_effort` 或 `reliable`）。
Topic 必须已经属于显式录制列表；通配符、未知键和未录制的 Topic 会在加载配置时被拒绝。
省略的字段继承数据源或消息类型的默认值，不带 `qos_overrides` 的现有配置仍可使用。
仅在发布端提供 reliable QoS 且已验证部署时使用 `reliable`：
它无法连接只支持 best-effort 的发布端，还可能产生重传或背压成本。

供 `ros2 bag record` 使用的合并后配置会保存在每个试次的 `qos_overrides.yaml`，
Recorder 通过 `--qos-profile-overrides-path` 读取该文件。
修改在下次启动控制器和新建录制试次时生效，不会改变正在录制的 bag。
这些是缓冲配置，不是实时质量准入条件；单个传感器质量下降不会停止其他数据流。

## 7. 发送 START

以下命令请求创建一个计划时长为配置上限 300 秒的试次：

```bash
ros2 topic pub --once /capture/command vt_camera_msgs/msg/CaptureCommand \
  "{request_id: run-001, command: 1, session_label: trial, planned_duration_sec: 300}"
```

每个不同的命令使用新的非空 `request_id`。重复同一请求 ID 和同一命令是幂等的；
同一 ID 改用于另一种命令会产生冲突。

## 8. 观察状态

```bash
ros2 topic echo /capture/status
```

成功的状态流转为：

```text
IDLE -> RECORDING -> FINALIZING -> COMPLETE
```

状态详情 `Recorder process lifecycle complete` 表示 Recorder 已启动，
且其退出已得到确认。`COMPLETE` 不代表录制数据质量合格。

排查丢帧时，应区分以下三个测量阶段：

- 工作台的**接收频率（Receive Hz）**统计管理后台最近一秒收到的消息数。
  它使用独立于 Recorder 的订阅，既不是相机硬件计数，也不是写入 bag 的消息计数。
- `CaptureStatus.streams` 和 `camera_groups` 当前为空；
  `observed_write_mb_s` 是保留字段，目前返回零。
  零表示测量不可用，不表示已确认写入吞吐为零。
  单凭文件增长也不能证明录制完整或数据已持久写入。
- 录制结束后，打开**档案 → 试次详情 → 质量报告**。
  只有已完成的报告才能说明实际执行的检查结果。
  未执行、排队中、被中断或失败的检查都不提供质量结论；
  没有告警的报告也不能证明检查范围之外的性质。

Recorder 按 Topic 的实时丢失计数，以及完整设备组合持续 30 Hz 的验收，仍需单独开展。
预览帧率和软件测试不能代替这些测量。

受管录制在启动 Recorder 前先分配试次目录，目录存在本身不证明已经录制。
当控制器实例及 START 请求匹配、控制器处于空闲状态，
且已确认不存在活动 Recorder 时，试次会判定为 `FAILED`；
诊断目录保留，并允许下一次显式 START。先前试次已经完成或无效时也采用该规则。

如果启动时可能已经创建了子进程，随后发生异常，控制器会继续持有进程，
并保持 `FINALIZING`，直到退出得到确认。
超时、控制器实例变化和试次身份冲突都属于未解决状态，不会自动触发重试。
试次生命周期日志分别保存启动错误、收尾原因、返回码和退出确认，
这些记录独立于数据质量结论。

事件和当前试次身份可以单独查看：

```bash
ros2 topic echo /capture/event
ros2 topic echo --once /capture/session_info \
  --qos-durability transient_local
```

`/capture/status`、`/capture/event` 和 `/capture/session_info` 是控制平面 Topic，
不写入录制数据。

## 9. 发送 STOP

试次达到计划时长后自动收尾。如需提前停止，发布：

```bash
ros2 topic pub --once /capture/command vt_camera_msgs/msg/CaptureCommand \
  "{request_id: stop-001, command: 2, session_label: '', planned_duration_sec: 0}"
```

等到 `COMPLETE` 后再停止前台启动进程。
`FINALIZING` 期间，控制器仍在确认 Recorder 进程是否已经退出。

## 10. 离线检查与对齐

达到 `COMPLETE` 后，可以手动查看试次：

```bash
ros2 bag info "${VT_DATA_ROOT}/<session-id>/bag"
```

`ros2 bag info` 供操作者按需查看，不会改变 `COMPLETE` 状态，
控制器也不会解析其输出。默认允许列表包含六路 `sensor_msgs/msg/Image`、
六路 `sensor_msgs/msg/CameraInfo`、三路 `realsense2_camera_msgs/msg/Extrinsics`、
三路 `vt_camera_msgs/msg/CameraFrameTiming` 和三路 `vt_tracker_msgs/msg/TrackerSample`。
实际录制覆盖范围取决于可用数据源；可选 Tracker 缺失不会阻止录制。

bag 有意不包含外部 Tracker–相机标定文件、TF、RealSense 原始元数据、
试次说明、采集命令、状态和事件。不进行实时 rosbag 或 MCAP 压缩；
如需复制或压缩已完成的试次，应作为独立的离线操作进行。

使用数据前，执行独立的离线审查与对齐流程：

```bash
vt-multisensor-align inspect --bag "${VT_DATA_ROOT}/<session-id>/bag" \
  --config "${ALIGN_CONFIG}"
```

安装、外部标定、对齐、输出和质量判定的完整流程见
[离线对齐操作手册](../tools/multisensor_alignment/README.md)。
全部启动参数、Topic QoS 和时间戳语义见[接口参考](interface-reference.md)。
