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
- 原子、拒绝覆盖地输出 JSON/JSONL/CSV/SVG，并支持哈希复验。

### 1.2 当前版本不提供

- 三相机硬件触发、曝光同步或在线同步；
- 对图像像素插值、深度重投影、点云融合或触觉语义处理；
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
