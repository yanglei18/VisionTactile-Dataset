# Tracker–RealSense 离线外参标定工具

本文是本工具的唯一操作入口。只要相机和 Tracker 已经能够稳定发布本文规定的
ROS 2 Topic，操作人员即可按照本文从头到尾完成：标定板准备、硬件身份绑定、
专用 bag 录制、离线求解、三次重复性验证和最终外参归档。

参考环境：Ubuntu 24.04、ROS 2 Jazzy、RealSense ROS 4.58.1、Python 3.12。
工具版本：`0.3.0`。

## 1. 产品边界

### 1.1 本工具负责什么

每组刚性固定的 Tracker–相机组合最终输出一份：

```text
Tracker body frame <- RealSense color optical frame
```

输出变换记作 `^T T_C`，它把相机坐标系中的点转换到 Tracker body 坐标系：

```text
p_T = ^T T_C * p_C
```

固定标定板时，每个采样姿态满足：

```text
^V T_B(i) = ^V T_T(i) * ^T T_C * ^C T_B(i)
```

- `V`：`vive_map`
- `T`：Tracker body frame
- `C`：RealSense color optical frame
- `B`：固定 ChArUco board
- 输出：`^T T_C`，YAML 语义为 `parent_from_child`

平移单位为米，四元数顺序为 `x,y,z,w`。RealSense optical frame 为 `x` 向右、
`y` 向下、`z` 向前。

### 1.2 本工具不负责什么

本工具严格离线，不会：

- 打开相机、Dongle 或 Tracker；
- 执行 Tracker 配对、Windows 建图、bootstrap、固件更新或任何设备写入；
- 启动、停止或修改正式统一 Recorder；
- 发布静态 TF，或在运行时自动加载标定结果；
- 估计相机与 Tracker 的动态时间偏移；
- 生成三组真实硬件的现成外参。

因此，“一本 README 独立完成”指的是独立完成外参标定交付，不是从公开仓库
重新实现 Tracker 私有启动。开始录制前必须已经存在稳定的相机和 Tracker ROS
输入；这些输入如何建立，不改变本工具的算法和操作流程。

### 1.3 输入与输出

每个专用 calibration bag 只需要四个 Topic：

| 数据 | ROS 类型 | 参考 Topic |
| --- | --- | --- |
| 彩色图像 | `sensor_msgs/msg/Image` | `/d405_1/color/image_raw` |
| 彩色内参 | `sensor_msgs/msg/CameraInfo` | `/d405_1/color/camera_info` |
| 相机软件时序 | `vt_camera_msgs/msg/CameraFrameTiming` | `/d405_1/frame_timing` |
| Tracker 位姿样本 | `vt_tracker_msgs/msg/TrackerSample` | `/vive/torso/sample` |

时间关联统一在 host realtime 时间域完成：工具使用图像 header stamp 查找
`CameraFrameTiming.shared_ros_timestamp_ns`，取得对应的
`group_host_realtime_ns`，再插值 `TrackerSample.host_realtime_ns`。D436 可能
重置的 device clock、ROS wall-clock 和 `CLOCK_MONOTONIC_RAW` 不会被直接互相
比较。当前版本通过静止窗口抑制回调和曝光时刻差异，不估计动态时间偏移。

单次求解输出：

| 文件 | 含义 |
| --- | --- |
| `extrinsics.yaml` | 外参、硬件身份、坐标 frame、质量等级和来源信息 |
| `report.json` | 输入计数、留出验证指标和五种手眼方法评分 |
| `residuals.csv` | 每个入选姿态的固定板闭环误差 |
| `diagnostics.svg` | 供人工查看的平移和旋转残差曲线 |

所有真实 bag、图像、Tracker ID、角色映射和标定产物必须保存在 Git 仓库外。

## 2. 现场标定总流程

```text
相机与 Tracker 安装到最终刚性支架
        |
        v
确认三组物理对应关系
        |
        v
生成、打印并实测 ChArUco 板
        |
        v
为每组刚体生成身份绑定配置
        |
        v
确认四个输入 Topic 与物理身份
        |
        v
每组录制 run01、run02、run03
        |
        v
离线求解三个 bag
        |
        v
比较三次外参并通过重复性门禁
        |
        v
归档最接近共识的有效外参
```

参考系统有三台相机，因此完整交付为 `3 组刚体 × 3 次运行 = 9 个 bag`。如果
当前只标定一组刚体，只执行该组的三个运行即可。

## 3. 开始前的硬件条件

以下条件必须全部满足：

- Tracker 已完成配对、Windows 建图，并在 Linux 下持续输出有效 6DoF。
- 相机与 Tracker 已安装到最终刚性支架，二者之间不存在任何相对运动。
- 支架不会遮挡 Tracker 定位相机，线缆不会拉动相机或 Tracker。
- 三台相机和三只 Tracker 均有清晰、永久的物理标签。
- 标定板保持固定；移动的是整套“相机+Tracker”刚体。
- 光照稳定，无阳光直射、强反光和明显运动模糊。
- 数据目录位于 Git 仓库外，磁盘空间充足。

标定后发生下列任一事件，对应外参立即失效：

- 松动、拆卸或重新安装支架；
- 相机或 Tracker 受到撞击；
- 调整螺钉、安装角度或线缆走向；
- 更换相机、Tracker 或物理角色。

## 4. 一次性安装

### 4.1 创建本次 campaign

从仓库根目录执行。`CAMPAIGN` 必须是本次标定的唯一名称：

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export CAMPAIGN="20260819-final-r01"  # 每次标定必须修改
export CAL_ROOT="${VT_DATA_ROOT}/calibration/${CAMPAIGN}"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-0}"

test ! -e "${CAL_ROOT}"
mkdir -p \
  "${CAL_ROOT}/config" \
  "${CAL_ROOT}/boards" \
  "${CAL_ROOT}/bags" \
  "${CAL_ROOT}/results" \
  "${CAL_ROOT}/final"
chmod 700 "${CAL_ROOT}"
```

不要把 `CAL_ROOT` 放进 `VT_REPO`。后续所有新终端都要重新设置相同的
`VT_REPO`、`VT_WS`、`VT_DATA_ROOT`、`CAMPAIGN`、`CAL_ROOT` 和
`ROS_DOMAIN_ID`。

### 4.2 安装依赖并构建消息包

```bash
sudo apt update
sudo apt install -y \
  python3-colcon-common-extensions \
  python3-venv \
  python3-yaml \
  ros-jazzy-realsense2-camera \
  ros-jazzy-ros2bag \
  ros-jazzy-rosbag2-storage-mcap

test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
cd "${VT_WS}"
colcon build --event-handlers console_direct+
source "${VT_WS}/install/setup.bash"
colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

只有零失败才继续。

### 4.3 安装离线工具

```bash
python3 -m venv --system-site-packages "${VT_REPO}/.venv-calibration"
source "${VT_REPO}/.venv-calibration/bin/activate"
python -m pip install --upgrade pip
python -m pip install "${VT_REPO}/tools/tracker_camera_calibration"
```

验证安装和 ROS 消息环境：

```bash
vt-tracker-camera-calibrate --version
vt-tracker-camera-calibrate --help
python -c 'import cv2,numpy,rosbag2_py,rclpy; print("opencv=" + cv2.__version__ + " numpy=" + numpy.__version__)'
python -c 'from vt_camera_msgs.msg import CameraFrameTiming; from vt_tracker_msgs.msg import TrackerSample; print("messages=OK")'
```

预期版本为 `0.3.0`，子命令包含 `configure`、`board`、`calibrate` 和 `compare`，
OpenCV 为 `4.9–4.11`、NumPy 为 `1.26.x`，最后一条输出 `messages=OK`。

## 5. 确认三组物理对应关系

参考相机身份为：

| 相机名 | 型号 | 序列号 | 彩色 optical frame |
| --- | --- | --- | --- |
| `d405_1` | D405 | `260322278433` | `d405_1_color_optical_frame` |
| `d405_2` | D405 | `260322276463` | `d405_2_color_optical_frame` |
| `d436` | D436 | `408322071716` | `d436_color_optical_frame` |

现场观察每个最终支架，填写真实关系：

| 相机 | 同一支架上的 Tracker 物理角色 | 操作员 | 日期 |
| --- | --- | --- | --- |
| `d405_1` | __________ | __________ | __________ |
| `d405_2` | __________ | __________ | __________ |
| `d436` | __________ | __________ | __________ |

然后设置变量。以下值只是格式示例，必须替换成现场结果：

```bash
export ROLE_D405_1="torso"
export ROLE_D405_2="left_wrist"
export ROLE_D436="right_wrist"

printf '%s\n' "${ROLE_D405_1}" "${ROLE_D405_2}" "${ROLE_D436}" \
  | grep -Ex '(torso|left_wrist|right_wrist)'
test "$(printf '%s\n' "${ROLE_D405_1}" "${ROLE_D405_2}" "${ROLE_D436}" \
  | sort -u | wc -l)" -eq 3
```

第一条必须输出三行，第二条必须无错误返回。不得根据 Host、Client 0 或 Client 1
无线槽位名称猜测物理角色。

## 6. 生成并实测 ChArUco 板

### 6.1 生成 300 DPI 打印文件

```bash
vt-tracker-camera-calibrate board \
  --config "${VT_REPO}/tools/tracker_camera_calibration/config/calibration.example.yaml" \
  --output "${CAL_ROOT}/boards/charuco-9x6-300dpi.png" \
  --dpi 300
```

图案为 `9×6` 方格、`DICT_5X5_1000`，名义尺寸为 `360×240 mm`。使用 A3
纸张，以 100% / Actual Size 打印，关闭 Fit、Shrink 和 Scale to page。将纸张
粘贴到平整、刚性的哑光板材上，不得折叠、翘曲或覆盖反光膜。

### 6.2 测量成品

使用卡尺完成以下测量：

1. 水平方向测量连续 5 个方格的总长度并除以 5。
2. 垂直方向测量连续 5 个方格的总长度并除以 5。
3. 在左、中、右三个区域测量 Marker 黑色外边长并取平均。
4. 水平与垂直方格边长相差超过 `0.20 mm` 时重新打印。
5. 使用实测平均值，不使用名义打印尺寸。

```bash
export SQUARE_LENGTH_MM="39.92"  # 替换为实测值
export MARKER_LENGTH_MM="29.94"  # 替换为实测值

python3 -c 'import os; s=float(os.environ["SQUARE_LENGTH_MM"]); m=float(os.environ["MARKER_LENGTH_MM"]); assert 0 < m < s; print("board_measurement=OK")'
```

## 7. 生成三组身份绑定配置

`configure` 会写入参考相机的型号、序列号、frame 和 Topic，并拒绝覆盖已有文件：

```bash
vt-tracker-camera-calibrate configure \
  --camera d405_1 \
  --tracker-role "${ROLE_D405_1}" \
  --square-length-mm "${SQUARE_LENGTH_MM}" \
  --marker-length-mm "${MARKER_LENGTH_MM}" \
  --output "${CAL_ROOT}/config/d405_1.yaml"

vt-tracker-camera-calibrate configure \
  --camera d405_2 \
  --tracker-role "${ROLE_D405_2}" \
  --square-length-mm "${SQUARE_LENGTH_MM}" \
  --marker-length-mm "${MARKER_LENGTH_MM}" \
  --output "${CAL_ROOT}/config/d405_2.yaml"

vt-tracker-camera-calibrate configure \
  --camera d436 \
  --tracker-role "${ROLE_D436}" \
  --square-length-mm "${SQUARE_LENGTH_MM}" \
  --marker-length-mm "${MARKER_LENGTH_MM}" \
  --output "${CAL_ROOT}/config/d436.yaml"
```

每个文件权限应为 `0600`：

```bash
stat -c '%a %n' "${CAL_ROOT}"/config/*.yaml
```

人工核对三个文件中的相机序列号、Tracker role、四个 Topic、四个 frame 和板材
实测尺寸。三个配置必须使用三个不同的 Tracker role。

`configure` 只覆盖本仓库的三台参考相机。其他硬件可复制
`config/calibration.example.yaml`，准确修改身份、frame 和 Topic 后再使用。

## 8. 录制前输入门禁

### 8.1 启动在线输入

相机可以使用本仓库的三相机 launch：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  output_root:="${VT_DATA_ROOT}"
```

不要向 `/capture/command` 发送 START。标定使用第 9 节的独立四 Topic Recorder。

Tracker Publisher 必须通过当前项目已经验收的只读启动方式单独运行。标定工具
不需要也不读取抓包、bootstrap bundle 或角色映射；它只订阅 Publisher 已经提供
的 `/vive/<role>/sample`。不得让两个程序同时占用 Dongle。

### 8.2 三 Tracker 30 秒门禁

在另一个已经 source ROS 和 workspace 的终端逐只移动三只 Tracker：

```bash
mkdir -p "${VT_DATA_ROOT}/tracker-validation"
ros2 run vt_vive_tracker vt-vive-validate-topics \
  --duration 30 \
  --output "${VT_DATA_ROOT}/tracker-validation/${CAMPAIGN}-preflight-30s.json"
```

必须看到：

```text
status=PASS roles=3 identity_swaps=0 dropped=0
```

绿灯只代表硬件状态，不能替代 ROS Topic 门禁。

### 8.3 检查一个刚体的四个 Topic

下面以 `d405_1` 为例。每次从配置读取 role，避免在新终端中手工重复输入：

```bash
export CAMERA="d405_1"
export CONFIG="${CAL_ROOT}/config/${CAMERA}.yaml"
export ROLE="$(python3 -c \
  'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["tracker"]["role"])' \
  "${CONFIG}")"

ros2 topic type "/${CAMERA}/color/image_raw"
ros2 topic type "/${CAMERA}/color/camera_info"
ros2 topic type "/${CAMERA}/frame_timing"
ros2 topic type "/vive/${ROLE}/sample"

ros2 topic echo --once "/${CAMERA}/color/camera_info"
ros2 topic echo --once "/${CAMERA}/frame_timing"
ros2 topic echo --once "/vive/${ROLE}/sample"
```

必须满足：

- 四个 Topic 类型与第 1.3 节完全一致。
- CameraInfo `frame_id` 等于配置中的 color optical frame。
- CameraFrameTiming 的 name、model 和 serial 与配置完全一致。
- TrackerSample 的 `role` 与配置一致，header `frame_id` 为 `vive_map`。
- TrackerSample 为 `pose_valid: true`，且 `tracking_status` 低四位为 2。

分别观察图像和频率：

```bash
ros2 topic hz "/${CAMERA}/color/image_raw"
ros2 topic hz "/${CAMERA}/frame_timing"
ros2 topic hz "/vive/${ROLE}/sample"
```

每条命令观察约 10 秒后按 `Ctrl-C`。参考值约为 30 Hz，不应持续断流。使用 RViz2
的 Image display 查看 `/${CAMERA}/color/image_raw`，确认 ChArUco 完整、清晰且
占画面约 30%–80%。

### 8.4 连续确认物理身份

```bash
ros2 topic echo "/vive/${ROLE}/pose"
```

只移动与 `${CAMERA}` 固定在同一支架上的 Tracker–相机刚体，观察位姿连续变化，
然后按 `Ctrl-C`。同时确认另外两个物理 Tracker 未被错误映射到该 role。

将 `CAMERA` 改为 `d405_2` 和 `d436`，重新执行第 8.3、8.4 节。任何身份、frame、
序列号或 Topic 不一致都必须先修复，禁止通过事后重命名结果绕过。

## 9. 录制专用 calibration bag

### 9.1 单次录制命令

以下录制 `d405_1` 的 `run01`：

```bash
export CAMERA="d405_1"
export CONFIG="${CAL_ROOT}/config/${CAMERA}.yaml"
export ROLE="$(python3 -c \
  'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["tracker"]["role"])' \
  "${CONFIG}")"
export RUN="01"
export BAG="${CAL_ROOT}/bags/${CAMERA}-${ROLE}-run${RUN}"

test ! -e "${BAG}"
ros2 bag record \
  --storage mcap \
  --output "${BAG}" \
  --max-cache-size 268435456 \
  --disable-keyboard-controls \
  --topics \
  "/${CAMERA}/color/image_raw" \
  "/${CAMERA}/color/camera_info" \
  "/${CAMERA}/frame_timing" \
  "/vive/${ROLE}/sample"
```

录制期间不要启动正式统一 Recorder，也不要同时录制其他相机。动作完成后按
一次 `Ctrl-C`，等待 rosbag2 正常退出并写出 `metadata.yaml`。

```bash
test -f "${BAG}/metadata.yaml"
ros2 bag info "${BAG}"
```

bag 必须恰好包含上述四个 Topic，四个消息计数均大于零。失败或中断的 bag 保留
为现场证据，但不得进入正式求解；使用新的 run 或新的 campaign 重新录制。

### 9.2 每个 bag 的人工动作

标定板固定不动，移动整个“相机+Tracker”刚体。建议采集约 50 个姿态：

1. 覆盖近、中、远三个距离。
2. 覆盖画面中心、左、右、上、下区域。
3. 绕至少两个不共线轴旋转，推荐三个轴均包含正、负方向变化。
4. 包含多组 `20°–30°` 旋转和至少 `50 mm` 的整体平移范围。
5. 到达每个姿态后完全静止 `0.8–1.0 s`。
6. 姿态之间正常移动；运动帧由稳定性门禁自动拒绝。
7. 始终保持标定板完整可见、角点清晰、无严重反光。

不要只在一个平面平移、只绕一个轴旋转，或持续缓慢移动而没有静止窗口。

### 9.3 每组必须录制三个真实独立运行

同一刚体依次录制 `run01`、`run02`、`run03`。每个运行都要停止前一个 Recorder、
创建新 bag，并重新执行一套姿态；不能复制同一 bag 充当三次运行。三次之间不要
拆卸支架。

然后对 `d405_2` 和 `d436` 重复。最终应有九个不同 bag：

```text
d405_1-<role>-run01  d405_1-<role>-run02  d405_1-<role>-run03
d405_2-<role>-run01  d405_2-<role>-run02  d405_2-<role>-run03
d436-<role>-run01    d436-<role>-run02    d436-<role>-run03
```

## 10. 离线求解

求解时可以停止相机和 Tracker，以降低笔记本负载。打开新终端：

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export CAMPAIGN="20260819-final-r01"  # 必须与录制时一致
export CAL_ROOT="${VT_DATA_ROOT}/calibration/${CAMPAIGN}"

source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
source "${VT_REPO}/.venv-calibration/bin/activate"
```

以 `d405_1` 为例：

```bash
export CAMERA="d405_1"
export CONFIG="${CAL_ROOT}/config/${CAMERA}.yaml"
export ROLE="$(python3 -c \
  'import sys,yaml; print(yaml.safe_load(open(sys.argv[1]))["tracker"]["role"])' \
  "${CONFIG}")"

for RUN in 01 02 03; do
  vt-tracker-camera-calibrate calibrate \
    --bag "${CAL_ROOT}/bags/${CAMERA}-${ROLE}-run${RUN}" \
    --config "${CONFIG}" \
    --output "${CAL_ROOT}/results/${CAMERA}-${ROLE}-run${RUN}" || break
done
```

每次先输出输入统计：

```text
images=... timed_images=... board_observations=... tracker_samples=... selected_static_pairs=...
```

成功示例：

```text
quality=TARGET method=HORAUD pairs=50 validation_translation_mm=... validation_rotation_deg=... output=...
```

退出码：

- `0`：结果为 `TARGET` 或 `ACCEPTABLE`，`extrinsics.yaml` 为 `VALID`。
- `2`：结果为 `REJECTED`，保留诊断但禁止使用。
- `1`：输入、身份、frame、样本、依赖或求解错误。

只要出现非零退出码，就停止该组流程并查看第 13 节。不要继续执行重复性比较。
将 `CAMERA` 改为 `d405_2` 和 `d436`，分别重复本节。

## 11. 三次重复性验证与归档

三个单次结果均为 `status: VALID` 后，执行：

```bash
export RESULT_ROOT="${CAL_ROOT}/results"

vt-tracker-camera-calibrate compare \
  --inputs \
  "${RESULT_ROOT}/${CAMERA}-${ROLE}-run01/extrinsics.yaml" \
  "${RESULT_ROOT}/${CAMERA}-${ROLE}-run02/extrinsics.yaml" \
  "${RESULT_ROOT}/${CAMERA}-${ROLE}-run03/extrinsics.yaml" \
  --max-translation-mm 5.0 \
  --max-rotation-deg 0.5 \
  --output "${RESULT_ROOT}/${CAMERA}-${ROLE}-repeatability.json"
```

`compare` 首先确认三个文件描述同一相机序列号、Tracker ID、role、parent frame、
child frame 和变换语义，然后计算所有两两差异。

通过示例：

```text
status=PASS runs=3 maximum_translation_mm=... maximum_rotation_deg=... recommended_input=... output=...
```

- `PASS` / 退出码 0：最大两两差异不超过 `5 mm、0.5°`。
- `FAIL` / 退出码 2：重复性不合格，三个结果都不得归档为正式外参。
- `recommended_input`：三个真实结果中最接近共识的一次，不是新生成的平均外参。

只有 `PASS` 后才归档推荐结果：

```bash
export REPEATABILITY_REPORT="${RESULT_ROOT}/${CAMERA}-${ROLE}-repeatability.json"
export RECOMMENDED="$(python3 -c \
  'import json,sys; d=json.load(open(sys.argv[1])); assert d["status"] == "PASS"; print(d["recommended_input"])' \
  "${REPEATABILITY_REPORT}")"

case "${RECOMMENDED}" in
  "${CAMERA}-${ROLE}-run01/extrinsics.yaml"|\
  "${CAMERA}-${ROLE}-run02/extrinsics.yaml"|\
  "${CAMERA}-${ROLE}-run03/extrinsics.yaml") ;;
  *) printf 'unexpected recommended_input: %s\n' "${RECOMMENDED}" >&2; exit 1 ;;
esac

export FINAL="${CAL_ROOT}/final/${CAMERA}-${ROLE}-extrinsics.yaml"
test ! -e "${FINAL}"
install -m 600 "${RESULT_ROOT}/${RECOMMENDED}" "${FINAL}"
sha256sum "${FINAL}"
```

对三组刚体分别执行。最终 `final/` 应恰好包含三个与真实物理身份对应的 YAML。

## 12. 产品验收标准

每组刚体必须同时满足：

| 门禁 | 通过条件 |
| --- | --- |
| 输入身份 | camera name/model/serial、Tracker role/ID 和 frame 与物理支架一致 |
| 有效姿态 | 每个运行 `selected_static_pairs >= 40` |
| 单帧检测 | 默认每帧 ChArUco 重投影 RMS 不超过 `2 px` |
| 目标质量 | 留出闭环不超过 `5 mm、0.5°`，等级 `TARGET` |
| 最低有效质量 | 留出闭环不超过 `10 mm、1°`，等级 `ACCEPTABLE` |
| 重复性 | 三次最大两两差异不超过 `5 mm、0.5°`，状态 `PASS` |
| 机械状态 | 标定与正式使用之间无拆卸、松动、撞击或线缆拉动 |

只有“三个单次结果均为 `VALID`”并且“重复性报告为 `PASS`”，才允许产生正式
归档文件。低重投影误差、硬件绿灯、收到位姿或求解器返回矩阵，都不能单独替代
上述验收。

本工具的留出验证检查固定标定板闭环一致性；它不是独立测量仪器的绝对精度验证。
对于需要可追溯绝对精度的实验，还应使用外部测量系统验证最终外参。

## 13. 故障排查

| 症状 | 原因 | 处理 |
| --- | --- | --- |
| `ros2 topic list` 没有输入 Topic | 在线相机或 Tracker Publisher 未运行、ROS domain 不一致 | 修复输入系统；不要修改离线算法。 |
| Tracker 绿灯但无 sample | Publisher、Dongle 独占或 ROS domain 问题 | 保持硬件状态，检查 Publisher 和唯一 Dongle owner。 |
| `required topic is absent from bag` | 录制命令或配置错误 | 用 `ros2 bag info` 核对四个 Topic，录制新 bag。 |
| `no usable CameraInfo` | 未录 CameraInfo | 确认 Topic 后录制新 bag，不手填未知内参。 |
| `CameraInfo/Image frame mismatch` | 使用了错误相机配置 | 核对 camera name、serial 和 optical frame。 |
| `CameraFrameTiming ... does not match config` | bag 与配置不是同一相机 | 使用正确配置重新录制，禁止事后交换文件名。 |
| `no valid CameraFrameTiming mappings` | timing 不存在或有效标志长期不完整 | 先修复 `/frame_timing`，再录制新 bag。 |
| `Incomplete timing group` 频繁出现 | 相机流、USB、QoS 或回调长期不完整 | 先解决在线采集问题；无 timing 图像不会进入标定。 |
| `expected one stable tracker_id` | 同一 role Topic 出现身份变化 | 停止并核对角色映射，该 bag 作废。 |
| `selected_static_pairs < 40` | 板不可见、模糊、未静止或姿态重复 | 改善光照，每姿态静止约 1 秒并增加姿态多样性。 |
| `needs ... span/excitation` | 平移或多轴旋转范围不足 | 增加至少两个不共线旋转轴和近中远位置。 |
| `all hand-eye methods failed` | 数据退化、身份/frame 错误或异常值过多 | 核对刚性、坐标和动作设计后录制新 bag。 |
| `quality=REJECTED` | 留出闭环超过 `10 mm、1°` | 查看 SVG/CSV，检查板尺寸、支架和定位后重新录制。 |
| compare 报身份不一致 | 三个结果不是同一物理刚体 | 找回正确结果，禁止删除身份字段或手改 YAML。 |
| compare 为 `FAIL` | 机械、定位或采集重复性不足 | 检查支架和环境，为该组重新录制三个新 bag。 |
| `output ... exists` | 防覆盖保护生效 | 使用新的 campaign；不要删除或覆盖已有证据。 |

不得通过放宽到超过 `10 mm、1°` 的单次门槛，或超过 `5 mm、0.5°` 的重复性
门槛，使不合格结果“通过”。

## 14. 中断与恢复

正常结束 Recorder：

1. 按一次 `Ctrl-C`。
2. 等待 rosbag2 完全退出。
3. 确认存在 `metadata.yaml`。
4. 执行 `ros2 bag info`。

断电、进程崩溃、metadata 缺失或 Topic 计数为零时，该 bag 不进入正式求解。
不要覆盖、拼接或重命名失败数据；创建新的 campaign 或新 run。

停止在线系统时，先停止 Recorder，再停止 Tracker Publisher，最后停止相机。
离线求解不需要任何硬件保持开机。

## 15. 验收记录模板

在 `CAL_ROOT` 对应的私有实验记录中保存：

```text
campaign：
日期 / 操作员 / 复核人：
Git commit：
OS / ROS / RealSense ROS / 工具版本：
camera name / model / serial：
Tracker physical role / tracker_id：
parent frame / child frame：
支架编号与防拆标记：
ChArUco square length mm：
ChArUco marker length mm：
run01 bag / quality / validation error：
run02 bag / quality / validation error：
run03 bag / quality / validation error：
maximum pairwise translation mm：
maximum pairwise rotation deg：
repeatability status：
recommended_input：
final YAML / SHA-256：
已知异常：
```

## 16. 最终检查表

- [ ] 相机与 Tracker 使用最终刚性支架，且物理对应关系已确认。
- [ ] 在线系统稳定发布四类输入 Topic，三 Tracker 30 秒门禁通过。
- [ ] ChArUco 按 100% 打印、刚性固定并完成尺寸实测。
- [ ] 三个配置由实测尺寸生成，camera serial 和 Tracker role 正确。
- [ ] 每组完成三个真实独立 bag，每个 bag 恰好包含四个 Topic。
- [ ] 九个单次结果全部为 `status: VALID`，每次至少 40 个有效姿态。
- [ ] 三个重复性报告全部为 `PASS`。
- [ ] 三个推荐外参以 `0600` 权限归档并记录 SHA-256。
- [ ] bag、图像、Tracker ID、角色映射和外参均位于 Git 仓库外。
- [ ] 标定后支架未拆卸、松动、撞击或改变线缆受力。

完成以上全部项目后，三组离线外参标定才算产品验收完成。统一正式录制与离线
对齐由 `tools/multisensor_alignment/` 负责；在线加载 YAML 和发布静态 TF 不在
本标定工具范围内。

## 17. 软件自检

无需硬件即可运行：

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
PYTHONPATH="${VT_REPO}/tools/tracker_camera_calibration/src" \
  "${VT_REPO}/.venv-calibration/bin/python" -m unittest discover \
  -s "${VT_REPO}/tools/tracker_camera_calibration/tests" -v
```

全部测试必须通过。软件自检和合成数据只证明实现行为，不替代第 12、16 节的真实
硬件验收。
