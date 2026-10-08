# 三相机、三 Tracker 离线数据对齐工具

本文是 `vt-multisensor-alignment 0.1.0` 的唯一操作入口。按照本文可以完成：

```text
一个统一 MCAP bag
  -> Topic/类型/身份/时钟审计
  -> 三相机一对一帧匹配
  -> 三 Tracker 线性 + SLERP 位姿插值
  -> 融合三组 tracker_from_camera 外参
  -> 可扩展 JSONL 对齐索引与质量报告
```

参考环境为 Ubuntu 24.04、ROS 2 Jazzy、Python 3.12。工具严格离线、只读 bag，
不会打开相机或 Dongle，不会配对或建图，不会修改 Tracker，也不会把图像重新
编码。原始 bag、Tracker ID、角色映射、外参和对齐结果必须保存在 Git 仓库外。

## 1. 产品边界

### 1.1 当前版本提供

- 从一个 rosbag2/MCAP 目录读取三台 RealSense 与三只 Tracker 的统一数据；
- 检查必需 Topic、ROS 类型、相机型号/序列号/frame、Tracker role/ID/world frame；
- 使用 `CameraFrameTiming` 将 color、depth 与 host realtime 精确关联；
- 以一台参考相机为行频，对另外两台相机执行有界、按时间有序、不可复用的最近
  帧匹配；匹配先最大化覆盖行数，再最小化总绝对时间残差；
- 对三个 `TrackerSample` 位姿做不外推的平移线性插值与最短弧四元数 SLERP；
- 使用三份有效外参计算每帧的 `vive_map_from_camera`；
- 为未来动捕手套等 Topic 提供配置式 `nearest` / `previous` 通用适配；
- 对 `vt_glove_msgs/msg/GloveState` 额外检查 live/有效姿态/源时间，并隔离
  无效区间、标定和滤波 epoch 切换；仍只选择消息，不插值手势；
- 对 `vt_tactile_msgs/msg/TactileSampleArray` 检查接收时间、软件序号和连接
  连续性；用 `export-tactile` 在参考时刻逐区域重建 **20×20** ADC/有效 mask/年龄，
  见[触觉接入与重建手册](../../ros2_ws/src/vt_tactile_capture/README.md#6-回放与离线重建)；
- 原子、拒绝覆盖地输出 JSON/JSONL/CSV/SVG，并支持哈希复验。

### 1.2 当前版本不提供

- 三相机硬件触发、曝光同步或在线同步；
- 对图像像素插值、深度重投影、点云融合或触觉力值/接触语义推断；
- 外参求解；外参应先由
  [Tracker–RealSense 标定工具](../tracker_camera_calibration/README.md)生成；
- 对未知动捕手套关节数组做插值。取得真实消息定义后可新增专用 adapter；当前
  通用 adapter 只选择整条消息的引用；
- 对低质量数据“自动修好”。门禁拒绝时仍保留诊断产物，但不能把结果标成通过。

## 2. 统一 bag 合同

默认核心 allowlist 是 15 个 Topic，而且全部写入同一个 bag：

| 来源 | 每个来源的 Topic | 数量 |
| --- | --- | --- |
| `d405_1`、`d405_2`、`d436` | `color/image_raw`、`depth/image_rect_raw`、`color/camera_info`、`frame_timing` | 12 |
| `left_wrist`、`right_wrist`、`torso` | `/vive/<role>/sample` | 3 |

完整默认名称为：

```text
/d405_1/color/image_raw
/d405_1/depth/image_rect_raw
/d405_1/color/camera_info
/d405_1/frame_timing
/d405_2/color/image_raw
/d405_2/depth/image_rect_raw
/d405_2/color/camera_info
/d405_2/frame_timing
/d436/color/image_raw
/d436/depth/image_rect_raw
/d436/color/camera_info
/d436/frame_timing
/vive/left_wrist/sample
/vive/right_wrist/sample
/vive/torso/sample
```

Recorder 使用显式 allowlist，不使用 `ros2 bag record -a`。配置中的扩展 Topic 会
在核心 15 Topic 后按名称排序，以避免后台出现的无关 ROS Topic 污染数据合同。
所有录制 Topic 使用 keep-last depth 30、best-effort、volatile QoS override。
离线检查也会拒绝配置中未声明的 bag Topic，避免扩展数据被静默忽略。

## 3. 时间语义

对齐时间轴是 Linux host realtime 纳秒：

1. color 与 depth 的 `header.stamp` 必须等于
   `CameraFrameTiming.shared_ros_timestamp_ns`；
2. 对应帧的对齐时刻取
   `CameraFrameTiming.group_host_realtime_ns`；
3. Tracker 位姿时刻取 `TrackerSample.host_realtime_ns`；
4. 通用扩展流配置的字段也必须表达 host realtime，不得把相机 device clock 或
   未换算的手套设备时钟直接填入。

相机的 `group_host_monotonic_raw_ns` 来自 `CLOCK_MONOTONIC_RAW`；当前 Tracker 的
`host_monotonic_ns` 来自 `CLOCK_MONOTONIC`。二者只分别用于检查 realtime 是否
发生跳变，绝不互相相减，也不被当成跨设备共同时间轴。D436 device clock 即使
重置，也不会直接参与跨设备匹配。

## 4. 开始前检查

必须满足：

- 三相机六路图像、三路 `CameraInfo` 和三路 `frame_timing` 稳定发布；
- 三只 Tracker 已完成 Windows 配对/建图，Linux Publisher 持续输出有效 6DoF；
- 相机与对应 Tracker 已刚性固定，并已完成三组最终外参标定；
- 三份外参均为 `status: VALID`，且与本次 bag 中的真实 Tracker ID 完全一致；
- 数据盘可容纳约 124.4 GB/300 秒的六路原始图像以及余量；
- bag 和所有结果目录位于源码仓库之外。

只启动相机、不启动 Tracker 就开始正式录制，会得到缺少三路核心数据的 bag；
Recorder 生命周期仍可能进入 `COMPLETE`，但 `inspect` 会明确拒绝该 bag。

## 5. 安装

从仓库根目录执行：

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"

test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
cd "${VT_WS}"
colcon build --event-handlers console_direct+
source install/setup.bash

python3 -m venv --system-site-packages "${VT_REPO}/.venv-alignment"
source "${VT_REPO}/.venv-alignment/bin/activate"
python -m pip install --upgrade pip
python -m pip install "${VT_REPO}/tools/multisensor_alignment"
```

验证：

```bash
vt-multisensor-align --version
vt-multisensor-align --help
python -c 'import rosbag2_py,rclpy,numpy,yaml; print("alignment_dependencies=OK")'
python -c 'from vt_camera_msgs.msg import CameraFrameTiming; from vt_tracker_msgs.msg import TrackerSample; print("workspace_messages=OK")'
```

预期版本为 `vt-multisensor-alignment 0.1.0`。

## 6. 录制一个统一 bag

### 6.1 启动 Tracker Publisher

按项目的 [Tracker ROS 2 操作手册](../../docs/tracker-ros2-publisher.md)完成手动
bootstrap，然后保持 Publisher 运行：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_vive_tracker triple_tracker.launch.py \
  bundle_path:="${VT_BUNDLE}" \
  role_map_path:="${VT_ROLE_MAP}"
```

三路验收至少应有有效位姿：

```bash
for role in left_wrist right_wrist torso; do
  ros2 topic echo --once "/vive/${role}/sample"
done
```

### 6.2 启动相机与 Recorder 控制器

在第二个 sourced 终端：

```bash
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  output_root:="${VT_DATA_ROOT}"
```

在第三个 sourced 终端确认 15 个核心 Topic 均存在：

```bash
for topic in \
  /d405_1/color/image_raw /d405_1/depth/image_rect_raw \
  /d405_1/color/camera_info /d405_1/frame_timing \
  /d405_2/color/image_raw /d405_2/depth/image_rect_raw \
  /d405_2/color/camera_info /d405_2/frame_timing \
  /d436/color/image_raw /d436/depth/image_rect_raw \
  /d436/color/camera_info /d436/frame_timing \
  /vive/left_wrist/sample /vive/right_wrist/sample /vive/torso/sample
do
  ros2 topic type "${topic}"
done
```

开始最多 300 秒的录制：

```bash
ros2 topic pub --once /capture/command vt_camera_msgs/msg/CaptureCommand \
  "{request_id: unified-001, command: 1, session_label: unified, planned_duration_sec: 300}"
```

需要提前结束时：

```bash
ros2 topic pub --once /capture/command vt_camera_msgs/msg/CaptureCommand \
  "{request_id: unified-stop-001, command: 2, session_label: '', planned_duration_sec: 0}"
```

等待 `/capture/status` 进入 `COMPLETE`。假定本次生成：

```bash
export BAG="${VT_DATA_ROOT}/<session-id>/bag"
ros2 bag info "${BAG}"
```

`COMPLETE` 只表示 Recorder 已停止；它不替代下面的离线质量门禁。

## 7. 准备配置和三份外参

复制配置到本次私有数据目录，不要直接修改仓库示例：

```bash
export RUN_ROOT="${VT_DATA_ROOT}/alignment/<run-id>"
mkdir -p "${RUN_ROOT}/config" "${RUN_ROOT}/extrinsics" "${RUN_ROOT}/results"
chmod 700 "${RUN_ROOT}" "${RUN_ROOT}/config" \
  "${RUN_ROOT}/extrinsics" "${RUN_ROOT}/results"
cp "${VT_REPO}/tools/multisensor_alignment/config/alignment.example.yaml" \
  "${RUN_ROOT}/config/alignment.yaml"
chmod 600 "${RUN_ROOT}/config/alignment.yaml"
export ALIGN_CONFIG="${RUN_ROOT}/config/alignment.yaml"
export EXTRINSICS_DIR="${RUN_ROOT}/extrinsics"
```

逐项核对 `alignment.yaml`：

- 三台相机的 name、model、serial、color optical frame 和四个 Topic；
- 每台相机实际刚性安装的 `tracker_role`；
- 三只 Tracker 的 sample Topic 和 body frame；
- `reference_camera`。默认 `d405_1` 只是参考选择，不代表硬件同步主机；
- `frames.world` 必须与 `TrackerSample.header.frame_id` 一致；
- 阈值是否仍采用已验收的参考值。

将三次重复标定中最终选定的 `extrinsics.yaml` 分别复制并重命名为：

```text
${EXTRINSICS_DIR}/d405_1.yaml
${EXTRINSICS_DIR}/d405_2.yaml
${EXTRINSICS_DIR}/d436.yaml
```

不得手工修改 Tracker ID 来“适配”新 bag。ID 不一致表示硬件/角色/标定产物选错，
必须回到物理身份核对。

## 8. 检查、对齐、复验

### 8.1 只读检查

```bash
vt-multisensor-align inspect \
  --bag "${BAG}" \
  --config "${ALIGN_CONFIG}"
```

只有三台相机均有完整 frame、三角色均至少两个有效位姿、身份稳定且必需扩展流
存在时才继续。

### 8.2 生成结果

结果目录必须不存在：

```bash
export ALIGN_OUTPUT="${RUN_ROOT}/results/aligned-v01"
test ! -e "${ALIGN_OUTPUT}"
vt-multisensor-align align \
  --bag "${BAG}" \
  --config "${ALIGN_CONFIG}" \
  --extrinsics "${EXTRINSICS_DIR}" \
  --output "${ALIGN_OUTPUT}"
```

退出码含义：

| 退出码 | 含义 |
| --- | --- |
| `0` | 结果生成且质量为 `ACCEPTED` |
| `1` | 输入、合同、身份、格式、I/O 或完整性错误 |
| `2` | 诊断结果已生成，但质量阈值为 `REJECTED` |

### 8.3 独立复验

```bash
vt-multisensor-align validate --output "${ALIGN_OUTPUT}"
```

该命令复算五个非 manifest 文件的 SHA-256、检查 JSONL 行数与 verdict。预期：

```json
{
  "verdict": "ACCEPTED",
  "aligned_frame_count": 12345,
  "tool_version": "0.1.0"
}
```

其中 `aligned_frame_count` 应以本次实际输出为准。

## 9. 输出文件说明

| 文件 | 作用 |
| --- | --- |
| `manifest.json` | 工具/合同版本、bag metadata/config/外参哈希、硬件身份、文件清单、总行数与 verdict |
| `stream_catalog.json` | 每个配置 Topic 的类型、必需性、bag 消息数、有效消息数和扩展流时间规则 |
| `aligned_frames.jsonl` | 每个参考相机帧一行；包含三相机消息引用、三 Tracker 位姿、附着 Tracker 位姿、`world_from_camera` 和扩展流引用 |
| `timing_residuals.csv` | 每行/每相机的相机时间差与附着 Tracker 插值 bracket gap |
| `quality_report.json` | 覆盖率、时钟审计、阈值、拒绝原因和最终 verdict |
| `diagnostics.svg` | 覆盖率与阈值的人工可视化 |

`aligned_frames.jsonl` 只保存消息引用，不复制图像像素。每个引用包含 Topic、该
Topic 内从 0 开始的 sequence、rosbag 写入时间和消息源时间。下游读取像素时，应
按 manifest 固定的原 bag 和引用定位消息。

变换语义为：

```text
vive_map_from_camera
  = vive_map_from_tracker × tracker_from_camera
```

平移单位为米，四元数顺序为 `x,y,z,w`。

## 10. 阈值与门禁

默认值：

| 配置 | 默认 | 含义 |
| --- | --- | --- |
| `max_camera_delta_ms` | 20 ms | 非参考相机与参考帧的最大绝对时间差 |
| `max_tracker_gap_ms` | 50 ms | Tracker 插值两端样本的最大间隔 |
| `max_clock_step_ms` | 5 ms | 相邻 realtime 增量与对应 monotonic 增量的最大差 |
| `min_camera_match_ratio` | 0.99 | 每台相机匹配覆盖率下限 |
| `min_tracker_coverage_ratio` | 0.99 | 三角色参考时刻及三台相机附着时刻的位姿覆盖率下限 |
| `min_required_stream_coverage_ratio` | 0.99 | 必需扩展流覆盖率下限 |

不能为了让失败数据变绿而事后放宽阈值。需要不同阈值时，应在采集前形成新配置、
记录理由并单独验收。

## 11. 新增动捕手套等 Topic

### 11.1 加入统一 Recorder allowlist

参考硬件应复制 `ros2_ws/src/vt_realsense_capture/config/cameras.yaml` 到仓库外；
其他硬件使用 `cameras.example.yaml` 后必须同时填写全部真实身份。然后在
`recording.additional_streams` 中加入真实 ROS 类型：

```yaml
recording:
  max_bag_duration_seconds: 300
  max_bag_size_bytes: 137438953472
  max_cache_size_bytes: 1073741824
  additional_streams:
    - topic: /gloves/left/state
      type: glove_msgs/msg/GloveState
    - topic: /gloves/right/state
      type: glove_msgs/msg/GloveState
```

使用 `config_path:=<私有配置绝对路径>` 启动相机 launch。

### 11.2 加入离线对齐配置

在 `alignment.yaml` 中配置：

```yaml
additional_streams:
  - name: left_glove
    topic: /gloves/left/state
    type: glove_msgs/msg/GloveState
    time_source: header_stamp
    timestamp_field: header.stamp
    strategy: nearest
    max_delta_ms: 20.0
    required: true
```

- `header_stamp` 要求字段严格为 `header.stamp`；该 stamp 必须是 host realtime；
- 如果消息另有 `uint64 host_realtime_ns`，使用 `time_source: field` 和
  `timestamp_field: host_realtime_ns`；嵌套字段使用点号；
- `nearest` 可选择前后最近消息，时间相同时优先较早消息；
- `previous` 只选择不晚于参考时刻的最近消息；
- `required: false` 允许 Topic 缺失且不影响 verdict；`true` 则执行覆盖率门禁。

当真实手套消息定义确定后，如需对每个关节角、位置或四元数插值，应新增并测试
专用 typed adapter；不能假设所有手套消息都可以用同一种插值规则。

### 11.3 当前 FSGlove12 解算结果

`vt_glove_msgs/msg/GloveState` 已有专用读取分支（不需要配置新的 strategy）。
先构建并 source 包含 `vt_glove_msgs` 的 ROS workspace；保留原手套标定/可视化
窗口，通过手套 bridge 发布 `/gloves/right/state`。此工具本身不打开手套串口，
也不运行或修改 FSGlove/DiffHCal 算法。消息与 bridge 的使用见
[`vt_glove_bridge/README.md`](../../ros2_ws/src/vt_glove_bridge/README.md)。

录制端配置的最小附加流为：

```yaml
recording:
  # 保留现有的其他 recording 配置。
  additional_streams:
    - topic: /gloves/right/state
      type: vt_glove_msgs/msg/GloveState
```

对应本工具的配置为：

```yaml
additional_streams:
  - name: right_glove
    topic: /gloves/right/state
    type: vt_glove_msgs/msg/GloveState
    time_source: field
    timestamp_field: host_realtime_ns
    strategy: nearest
    max_delta_ms: 20.0
    required: false
```

`required: false` 适合先验证多模态接入：手套没有有效结果时仍可处理相机和
Tracker，输出该行的手套为 `null`。正式数据要求手套覆盖率时改为 `true`。
20 ms 是匹配容差，不代表测量同步精度。

专用分支遵循以下规则：

- 只接受 `source_mode: live`、`pose_valid: true`、`time_valid: true` 且存在
  正数源时钟的消息；零时间或无效状态只形成区间边界，不使整个 bag 读取失败。
- `host_realtime_ns` 必须等于 `header.stamp`，来源为手掌参考节点 12 的
  `host_receive`；单调钟须为 `CLOCK_MONOTONIC`。不使用 bridge 发布时间或 bag
  写入时间代替源时间。这是主机接收时间对齐，不是 MCU 采样硬同步。
- 同一源时间产生多次正常解算时保留最后一条有效消息引用；不加 1 ns 伪造新样本。
  `accepted_counts` 因而计数的是有效、去重后的输入时间，不是所有解算发布次数。
- 无效/回放状态、session、标定版本或源节点滤波 epoch 变化会切断有效段。
  段间不做 nearest/previous 填补；不知道失效事件源时刻时，保守地从上一有效源
  时刻之后到下一有效源时刻之前留空，而不是根据 bag 时间虚构失效采样时刻。
- 每个 session 的 realtime 与 monotonic 独立加入 clock audit；同源时间的
  完全相同钟对只记录一次。不会与相机的 `CLOCK_MONOTONIC_RAW` 直接相减。
- `units_verified: false` 不自动否定解算结果，仍保留原消息中的质量标记；接入
  ROS 不会验证硬件量程、修复原始 IMU 截顶或证明手势精度。
- 输出仍为原始消息引用与时间残差，不会把 MANO 参数当 URDF 单轴关节角，
  不做关节/四元数插值，也不自动给手掌添加 Tracker 世界系平移。

默认只需录制这一路解算消息。若还录入 raw/status/calibration 等 Topic，必须
在离线配置中明确声明；不能假设未配置的 Topic 会被自动忽略。

### 11.4 已提供的右手正式采集配置

无需手工拼接 YAML，仓库提供配套的完整配置：

| 配置 | 用途 |
| --- | --- |
| [`cameras.with_right_glove.yaml`](../../ros2_ws/src/vt_realsense_capture/config/cameras.with_right_glove.yaml) | 保留 `cameras.yaml` 的全部硬件、图像和录制参数，仅在 allowlist 增加 `/gloves/right/state`。 |
| [`alignment.right_glove.yaml`](config/alignment.right_glove.yaml) | 保留 `alignment.example.yaml` 的全部相机、Tracker、外参与阈值约定，增加必须存在的右手解算流。 |

这是一套 **16 Topic** 的正式数据合同：原 15 路加一路手套解算结果，不增加
手套原始 IMU、状态或标定 Topic。手套要求为 `required: true`，采用
`host_realtime_ns`、`nearest`、20 ms 最大匹配差与原有 99% 必需流覆盖率。
**20 ms 是匹配容差，不是测得的同步精度或硬同步保证。**

旧默认配置没有改变；不选择新 profile 的启动方式继续保持原行为。两份新配置
包含当前参考硬件身份，只适用于身份相符的设备；其他设备请在私有副本中核对
相机序列号及实际 Tracker 安装关系。以下在完成第 5 节安装后，从仓库根目录执行：

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export RUN_ROOT="${VT_DATA_ROOT}/alignment/right-glove-001"
mkdir -p "${RUN_ROOT}/config" "${RUN_ROOT}/extrinsics" "${RUN_ROOT}/results"
cp "${VT_REPO}/ros2_ws/src/vt_realsense_capture/config/cameras.with_right_glove.yaml" \
  "${RUN_ROOT}/config/cameras.with_right_glove.yaml"
cp "${VT_REPO}/tools/multisensor_alignment/config/alignment.right_glove.yaml" \
  "${RUN_ROOT}/config/alignment.right_glove.yaml"
export CAPTURE_CONFIG="${RUN_ROOT}/config/cameras.with_right_glove.yaml"
export ALIGN_CONFIG="${RUN_ROOT}/config/alignment.right_glove.yaml"
export EXTRINSICS_DIR="${RUN_ROOT}/extrinsics"
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
```

每次采集使用新的 `RUN_ROOT`，避免覆盖旧配置。先按第 6.1 节保持三 Tracker
Publisher 运行，再按 [手套 bridge 手册](../../ros2_ws/src/vt_glove_bridge/README.md)
启动原 GUI、连接手套并取得有效解算。以下相机命令**不会自动启动手套或 Tracker**。
若旧相机 launch 已运行，应正常退出旧实例后再启动，不能同时让两个实例争用相机。

```bash
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  config_path:="${CAPTURE_CONFIG}" \
  output_root:="${VT_DATA_ROOT}"
```

在另一个已 source 的终端确认手套消息类型为 `vt_glove_msgs/msg/GloveState`，
且实际输出 `source_mode: live`、`pose_valid: true`、`time_valid: true`：

```bash
ros2 topic type /gloves/right/state
ros2 topic echo --once /gloves/right/state
```

随后使用第 6.2 节的 `/capture/command` 开始/停止命令录制同一个 bag。等待
`COMPLETE`，按实际 session 目录设置 `BAG`，并按第 7 节将三份真实外参放入
`${EXTRINSICS_DIR}`。在已安装本工具的环境中执行：

```bash
source "${VT_REPO}/.venv-alignment/bin/activate"
vt-multisensor-align inspect --bag "${BAG}" --config "${ALIGN_CONFIG}"
export ALIGN_OUTPUT="${RUN_ROOT}/results/aligned-right-glove-v01"
vt-multisensor-align align \
  --bag "${BAG}" --config "${ALIGN_CONFIG}" \
  --extrinsics "${EXTRINSICS_DIR}" --output "${ALIGN_OUTPUT}"
vt-multisensor-align validate --output "${ALIGN_OUTPUT}"
```

`ALIGN_OUTPUT` 必须是尚不存在的新目录。缺少手套 Topic 或全部手套解算无效时，
正式配置的 `inspect` 会拒绝；部分缺失由 `align` 的覆盖率判断，不会拿旧姿态填满
无效区间。只想排查接入而暂时不要求手套覆盖率时，在私有副本将 `required` 改为
`false`，或按第 11.3 节使用调试配置，不要修改仓库默认配置。

### 11.5 触觉统一为 20×20 的离线导出

手套的物理有效区域为原协议地址 `row=0..19, column=0..19`，共 400 个触点。
新的离线 NPZ、在线 `/gloves/right/tactile/state`、二维触觉窗口与手套三维表面
均使用这个 **20×20** 区域；不缩放、不插值，也不将外围无效地址算作有效触点。

串口与 bag 的 `/gloves/right/tactile/samples` **继续保留原始 schema 1 协议**：
每包 10 个 ADC 值，`column = column_group + 8*k`，协议地址范围仍为 80×80。
这不是一张整掌矩阵，不能将一包或一个 batch 的频率当作整掌刷新率。
旧 bag 无需转换即可使用本工具；外围包仍参与序号/丢失检查，但不更新物理区域
的 ADC、有效性或源时钟。输出矩阵使用行主序，`index = row*20 + column`。

已有统一对齐结果后执行（沿用前文的 `BAG`、`ALIGN_OUTPUT`）：

```bash
vt-multisensor-align export-tactile \
  --bag "${BAG}" \
  --aligned-frames "${ALIGN_OUTPUT}/aligned_frames.jsonl" \
  --output "${ALIGN_OUTPUT}/tactile-20x20" \
  --max-age-ms 100
```

输出目录必须尚不存在。扩展统一 bag 的配置示例见
[`alignment.right_glove_tactile.yaml`](config/alignment.right_glove_tactile.yaml)；
录制与回放操作见[触觉手册](../../ros2_ws/src/vt_tactile_capture/README.md)。

| 产物 / 字段 | 内容 |
| --- | --- |
| `manifest.json` | 导出 `schema_version: 2`，`rows: 20`、`cols: 20`，并明确记录原始 wire 80×80 与物理 ROI。 |
| `frames/*.npz`: `adc`, `valid_mask`, `age_ns` | 均为 20×20；ADC 零也是有效读数，必须结合 `valid_mask` 判断。 |
| `block_realtime_ns`, `block_monotonic_ns` | 均为 20×8，每个物理行/列组保留原始 host-read 完成时钟。 |
| `block_message_sequence`, `block_packet_index` | 均为 20×8；保留完整原始包流中的索引，不因裁剪而重新编号。 |
| `source_metadata.jsonl` | 每个参考帧的有效点数、源事件和连续性边界；`last_source_event` 是最近原始事件，`last_contributing_event` 是最近实际更新 20×20 区域的事件。 |
| `quality_report.json` | 完整、部分、无有效触点的帧数，有效覆盖率及有效点年龄统计；生成文件不等于数据质量通过。 |

这里仍是“参考时刻前最近一次有效分区更新”的滚动重建，不是硬同步的同时采样。
超龄/缺失点 `valid_mask=false`，不以最近收到的外围包刷新旧点年龄。
历史已导出的 80×80 NPZ 不会原地改写；需要 20×20 产物时从原 bag 导出到新目录。

导出会自动读取 `aligned_frames.jsonl` 同目录的 `manifest.json`，核对原 bag
metadata 和对齐索引的散列；新对齐结果还记录分卷文件名称及大小。bag 移动到
另一目录仍可核对，不要求原绝对路径。发现来源或索引不匹配会在生成产物前
明确报错，不把另一场实验误当作当前采集。这里的分卷清单不是分卷内容的全量
散列校验，不承诺检测所有文件损坏。

历史手工索引没有 manifest 时，默认继续导出，但终端和新 manifest 明确标为
`unverified_missing_manifest`。正式交付可加 `--require-provenance`，要求提供
来源 manifest；这个选项只约束离线导出，不控制任何在线采集节点。全部帧无有效
触点时仍保留诊断产物，但质量摘要明确告警，不以“文件生成成功”冒充有效数据。
专用导出在 rosbag reader 层只选择触觉 Topic，不再逐条读取其他模态的消息载荷。

### 11.6 录制结束后的统一质量报告

不需要外参，也不需要先完成对齐。确认 Recorder 已正常停止后，在已安装本工具
并 source ROS / workspace 的终端执行：

```bash
vt-multisensor-align report-capture \
  --bag "${BAG}" \
  --output "$(dirname "${BAG}")/capture_quality.json" \
  --config "${VT_REPO}/tools/multisensor_alignment/config/alignment.right_glove_tactile.yaml"
```

`--config` 可省略：提供时额外报告预期但缺失的数据流；这里使用的是**对齐配置**，
不是相机启动配置。输出必须是尚不存在的新 JSON 文件，不覆盖旧报告或 bag。
此命令是手动录制后处理，不随 START 自动运行，也不会给 Recorder 增加质量门禁。

报告汇总各 Topic 消息数、频率统计依据、可观测的时间间断/无效状态，以及触觉
批次与有效分区的更新情况。图像等大消息不逐条解码：基于 metadata 的消息数
除以全 bag 时长得到的频率会明确标识，不能等同于每路实际运行区间的源帧率。
触觉批次 Hz、逐分区更新率与 400 点覆盖情况分开表示，不将约 500 批/秒解释为
500 Hz 整掌扫描；也不把源软件序号缺口当作固件端全部丢失的证明。

终端和文件的 `status` 为 `ok` 或 `warnings`。缺流、部分无效或质量告警不会
改变已录数据，也不会要求重新录制：成功写出报告时退出 0，路径/读写等执行错误
退出 1。`ok` 只表示已检查项未发现告警，不代表未检查的数据、物理精度或硬件同步
获得保证；未支持/无法解码的消息会在报告中说明检查范围。

### 11.7 单/双手独立选择与配套配置

手势与触觉可以分别选择 `none/left/right/both`，原右手默认配置不变。
统一 bag 的三个相机、三个 Tracker 合同仍保留；这些 profile 不是纯手套 bag 的对齐配置。
运行端选择方法见 [hands.launch.py](../../ros2_ws/src/vt_glove_bridge/README.md#22-单手双手和仅触觉统一入口)。

录包与对齐应使用同一组合。例如双手姿态＋右手触觉：

```bash
vt-multisensor-align make-profiles \
  --capture-template "$VT_REPO/ros2_ws/src/vt_realsense_capture/config/cameras.yaml" \
  --alignment-template "$VT_REPO/tools/multisensor_alignment/config/alignment.example.yaml" \
  --pose-hands both --tactile-hands right \
  --output /absolute/path/new_hand_profiles
```

输出 `cameras.yaml`（用于相机采集控制器）和 `alignment.yaml`（用于离线工具），
只替换附加流清单，保留模板内设备与时序配置；不会改写原模板或覆盖已有输出。
手势选中的流默认 `required: true`，触觉默认可选；实验要求触觉完整时增加
`--require-tactile`。这是离线验收契约，不增加在线质量停录条件。

对应预设也已提供：`alignment.left_glove.yaml`、`alignment.both_gloves.yaml`、
`alignment.left_glove_tactile.yaml`、`alignment.both_gloves_tactile.yaml`、
`alignment.left_tactile.yaml`、`alignment.right_tactile.yaml`、`alignment.both_tactile.yaml`，
采集配置目录有对应 `cameras.with_*.yaml`。硬件 serial 等仍应使用本机已确认配置。

标准 `/gloves/{hand}/state` 和 `/gloves/{hand}/tactile/samples` 会校验消息手别与对应
`glove_{hand}_reference` / `{hand}_tactile` frame，防止左右路由对调。自定义 Topic 可在
additional stream 设置 `expected_hand`、`expected_frame_id`。手势默认按消息声明的
`reference_node_id`（1–12）核对源数组时间，不强制节点 12；需要固定具体掌节点时，
配置 `reference_node_id`，或生成 profile 时增加 `--left-reference-node-id 11` 等参数。

双手触觉按 Topic 分别导出，使用同一条相机参考时间轴：

```bash
vt-multisensor-align export-tactile --bag "$BAG" \
  --aligned-frames "$ALIGN_OUTPUT/aligned_frames.jsonl" \
  --topic /gloves/left/tactile/samples --output /absolute/path/new_tactile_left
vt-multisensor-align export-tactile --bag "$BAG" \
  --aligned-frames "$ALIGN_OUTPUT/aligned_frames.jsonl" \
  --topic /gloves/right/tactile/samples --output /absolute/path/new_tactile_right
```

两份输出分别保留 20×20 网格、session/epoch、有效性、来源与质量报告；不把左右手
ADC 合并或互相填充。自定义 Topic 可增加 `--expected-hand left/right` 与
`--expected-frame-id ...`。仅手套/触觉 bag 不调用要求三相机的 `align`，可以独立回放并
运行不带统一配置的 `report-capture`；未启用的手不当作缺流。

## 12. 故障排查

### `required topic is absent from bag`

先执行 `ros2 bag info "${BAG}"`。确认录制使用了新的统一合同，且 Tracker/扩展
Publisher 在 START 前已经运行。旧的相机专用 bag 不能伪装成统一 bag。

### `topic type mismatch`

配置中的 `type` 必须与 bag metadata 完全相同，包括 package、`msg` 和类型名。

### `CameraFrameTiming identity/frame mismatch`

配置选错相机、序列号或 optical frame，或 bag 混入了其他设备。不要绕过身份
门禁；重新绑定配置或重新采集。

### `Tracker identity changed` / 外参 `tracker_id mismatch`

角色映射、Tracker 实物或外参文件不属于同一套硬件。重新核对，不要编辑 ID。

### `clock_audit_failed`

host realtime 相对该流 monotonic 时钟发生大于阈值的跳变。检查运行中是否执行了
NTP/手动校时、系统休眠/恢复或虚拟化迁移。应重新采集；device clock 重置本身
不应触发该项。

### 相机覆盖率低

检查 USB 拓扑、hub 上行带宽、供电、图像丢失和 `Incomplete timing group`。匹配
不会复用同一个候选帧，也不会用超过 20 ms 的帧填空。

### Tracker 覆盖率低

检查蓝闪/绿闪、遮挡、地图重定位、Publisher 丢包和 6DoF status。工具不会外推
第一个样本之前或最后一个样本之后的位姿。

### `refusing to overwrite output directory`

每次结果使用新目录，例如 `aligned-v02`。保留旧结果以维持审计链，不要覆盖。

## 13. 最终检查表

- [ ] 原始数据和结果位于 Git 仓库外，目录权限为 `0700`、文件为 `0600`。
- [ ] `ros2 bag info` 显示一个 bag 内至少包含全部 15 个核心 Topic。
- [ ] `inspect` 成功，三相机均有完整帧，三 Tracker 身份稳定且有有效位姿。
- [ ] 三份外参均为最终支架的 `status: VALID` 结果，身份与 bag 完全一致。
- [ ] `align` 退出码为 0，`quality_report.json` 为 `ACCEPTED`。
- [ ] `validate` 退出码为 0，哈希和 JSONL 行数通过。
- [ ] `camera_match_ratio`、Tracker 两类覆盖率和必需扩展流覆盖率均达到阈值。
- [ ] 所有 clock audit 为 valid，无 host realtime 跳变。
- [ ] 人工查看 `diagnostics.svg` 和最大时间残差，没有异常尖峰。
- [ ] 下游使用前记录 Git commit、工具版本、配置/外参哈希和 source bag 名称。

上述检查全部通过，才可把该目录作为离线对齐产品交付。
