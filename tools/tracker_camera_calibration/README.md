# Tracker–RealSense 外参标定工具（引导标定与离线 CLI）

本文是本工具的唯一操作入口。只要相机和 Tracker 已经能够稳定发布本文规定的
ROS 2 Topic，操作人员即可按照本文从头到尾完成：标定板准备、硬件身份绑定、
专用 bag 录制、离线求解、三次重复性验证和最终外参归档。

参考环境：Ubuntu 24.04、ROS 2 Jazzy、RealSense ROS 4.58.1、Python 3.12。
工具版本：`0.1.0`。

## 主程序引导标定 / Guided calibration in the workbench

从主程序 **「标定与离线工具 → 相机–Tracker 标定」** 进入内嵌工作区。
选择相机、Tracker 和实测标定板参数后，保持板固定，移动相机–Tracker 刚性整体。
新会话默认板为 `CC200-15-11.25`：12 × 9 方格，方格边长 15 mm，标记黑色外边长
11.25 mm，`DICT_5X5_1000`。已有配置导入后保留其原有实测参数。标称尺寸须核对
实物，图案与外板尺寸的区别及打印方法见第 6 节。
实时页面显示同帧板角点投影、当前动作提示、轮次与有效姿态进度；详细指标默认折叠。
在线检测限频约 5 Hz，录包保存原始发布频率，不使用预览帧率代替录包质量。

每轮默认收集至少 **50 个不同静态姿态**：40 个用于求解，10 个预留作最终独立验证。
每第五个有效姿态在调用原求解器之前划入独立验证集，不参与算法选择、拟合或板参考位姿估计。
原离线 CLI 的 40 姿态门槛和已有文件格式保留。三轮采集使用独立数据、互不重叠的主机时间区间；
图像、内参、Timing 和 Tracker 的身份、时钟、稳定窗口、运动充分性均须通过检查。
达到数量但运动不足时继续引导；Recorder 关闭、实际落盘数据验证完成后才能求解和应用。

每轮自动保存 bag、配置快照、`extrinsics.yaml`、`report.json` 和
`independent-validation.json`。三轮通过后写入 `repeatability.json` 与 `manifest.json`。
单包离线求解得到候选结果，不能触发自动应用；三包须分别通过独立验证和重复性检查。
后台独立核验 manifest 后管理配置应用与 Topic 发布；标定 worker 本身没有活动配置写入或发布权限。
身份/内参改变、主机时钟跳变为当前轮终止原因，后续好帧不能抹去这些异常。
取消时先通知本任务的 Recorder 停止，并等待关闭完成。已有正式录制不由本工具接管。

### 中断后继续与指定单轮重拍

每轮求解及独立验证通过后，立即将结果登记到会话目录的 `round-progress.json`，
不必等三轮全部结束。第 2 轮失败不会作废第 1 轮；原始 bag 和结果始终保留。
在失败、取消或中断页面确认 **「相机与追踪器的固定关系、标定板尺寸未改变」**，
然后点击 **「保留已完成轮次，继续标定」**。程序沿用本会话的冻结配置，核对当前
设备身份，从未完成的轮次继续；不要点「新建标定」，后者是另开独立会话。

失败轮重新采集到 `round-N-attempt-M/`，旧 bag 不覆盖。旧工作进程日志和状态移至
`attempt-history/`。即使关闭工作台或后台重启，已完成轮次也可恢复。
本功能也能登记旧版同一会话已导出的 VALID 外参及 PASS 独立验证结果，不重新拟合。
部分采集但未完成验证的轮次不当成通过；恢复仍需三轮独立验证及最终重复性通过，
不是关闭质量检查。三轮均已通过单轮检查但最终重复性不通过时，默认
**「重新比较三轮结果（不采集）」**只复核已有结果，不会自动重拍或降低阈值。

如需只替换第 1 轮，在结果页的 **「选择重拍轮次」** 下拉框选择
**「仅重拍第 1 轮，保留其余轮次」**，确认装配与板尺寸未改变后点击同名按钮。
第 2、3 轮原样复用；原第 1 轮 bag、外参与验证报告也保留，新数据和结果写入
新的 attempt 目录。旧轮次选择与本次重拍请求保存在 `attempt-history/`，
只有通过新一轮独立验证后才加入当前三轮比较。重拍再次失败仍可继续未完成轮次，
不需要重做其余轮次。界面的“已保存并通过单轮验证”不等于三轮重复性已通过。

每次比较使用独立报告文件：首次为 `repeatability.json`，后续为
`repeatability-attempt-N.json`，不覆盖旧报告，也不会因同名文件阻断重试。
当前结果页和最终 `manifest.json` 指向本次报告；不要只查看首次报告判断最新状态。
只有当前三轮独立验证及 5 mm / 0.5° 重复性检查全部通过，后台才应用并发布结果。

Completed rounds are checkpointed immediately. On failure/cancellation/interruption,
confirm unchanged mounting and board dimensions and choose **Resume calibration,
keeping completed rounds**. Only unfinished rounds are captured again, in new attempt
directories. Existing bags/results remain untouched, and the original three-round
acceptance requirements still apply. New calibration starts a separate session.
To replace just one completed round, select **Recapture round N only; retain other
rounds** and confirm unchanged mounting/board dimensions. Retired artifacts remain
untouched and the selection is archived. Each comparison writes a new report;
retakes never silently weaken the repeatability thresholds.

The workbench keeps **Calibration and offline tools** as its entry point and embeds
camera–Tracker calibration in the main theme. Keep the ChArUco board fixed and move
the rigid camera–Tracker assembly. Guidance and previews run separately from the
full-rate Recorder. Each round defaults to 40 fitting poses plus at least 10
independent validation poses; every fifth accepted pose is reserved before solver
selection or fitting. The original offline CLI and its result format remain supported.

New sessions default to the `CC200-15-11.25` reference board: 12 × 9 squares,
15 mm square side, 11.25 mm black marker side, and `DICT_5X5_1000`.
Importing an existing configuration preserves its measured board parameters.
Check nominal dimensions on the physical board; see section 6 for pattern dimensions
and printing instructions.

Three rounds must pass independent validation and repeatability, use different source
content, and have non-overlapping host-time intervals. A single offline bag creates a
candidate only. The backend independently verifies the result manifest before activation
and publication. Cancelling waits for this task's Recorder to close. Synthetic ROS/MCAP
tests exercise message decoding, recording, actual ChArUco detection, solving and
validation; they do not certify physical mounting, real-device accuracy or endurance.

### 无界面进程接口 / Headless worker protocol

标定环境通过以下模块运行，GUI 不加载 ROS 或 OpenCV：

```bash
python -m vt_tracker_camera_calib.guided_worker \
  --request /absolute/session/request.json \
  --status /absolute/session/status.json \
  --cancel /absolute/session/cancel
```

请求为 JSON schema 1，字段为 `session_id`、`mode`（`guided`/`offline`）、
`config_path`、`output_root`、`bag_paths` 和可选 `expected_tracker_id`。
所有路径为绝对路径，数据目录须在 Git 仓库外；配置由后台冻结。
`output_root` 为已创建的会话根，配置、请求、状态与取消文件位于该根目录内。
在线模式自行管理三轮 bag；离线模式接受一个候选 bag 或三个独立 bag。
状态文件原子替换且不超过 1 MiB，包含轮次、计数、提示码、指标、同帧 JPEG 预览和结果路径。
`READY` 只表示 worker 产物已准备好，**不表示配置已应用或 Topic 已发布**；
`manifest.activation_eligible` 决定是否满足自动应用门禁，后台仍须复核。
取消文件或 SIGINT/SIGTERM 请求取消；不会清除已落盘证据。
检测运行于独立、只保留最新任务的线程，取消无需等待 OpenCV 完成本次检测。
Recorder 启动器先持久保存 `recorder-lease.json`（PID、启动时间标识及本次系统启动 ID），
再使用同一 PID 执行 ros2。后台重启只读核对这些标识以阻止录制重入，不对持久保存的 PID 发信号。
只有所属 Recorder 确认退出后才清除 lease；检测线程异常未退出也不能跳过 Recorder 收尾。

The worker uses atomic, bounded JSON status files and owns only its subprocesses.
The result manifest records source hashes, host-time ranges, identity binding,
independent-validation reports and the selected immutable extrinsic. `READY` does
not mean activation or publication. Those are separate acknowledged backend states.
Vision runs on a bounded latest-value worker thread. Before exec, the Recorder runner
persists a lease with its PID, start token and system boot ID. Restart recovery compares
these identities read-only; a saved PID is never authority to send a signal. Only confirmed
Recorder exit clears the lease, including cancellation during a blocked native detection.

Hardware-free ROS integration requires explicit opt-in and an isolated domain:

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
VT_GUIDED_ROS_TEST=1 ROS_DOMAIN_ID=186 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 .venv-calibration/bin/python -m pytest -q \
  tools/tracker_camera_calibration/tests/test_guided_ros.py
```

## 独立离线图形界面（兼容入口）


兼容命令 `vt-tracker-camera-calibrate-gui` 启动独立的**离线薄界面**：
配置生成／导入、ChArUco 板图预览与 PNG 导出、已有 bag 求解、报告与日志查看。
它复用本文 CLI 的配置、ChArUco 检测、时间关联、手眼求解及质量门禁，不是另一套算法。
GUI 默认数据目录为 `~/visiontactile-data/calibration/gui`，真实数据和结果必须
留在 Git 仓库外。GUI **不连接硬件、不订阅 ROS、不提供专用录包按钮**；
在线预览和专用录包尚未接入此界面，三次重复性比较继续使用本文 CLI。

先完成第 4 节的软件安装；专用 bag 按第 8 节采集、停止 Recorder 并等待完整关闭。
离线使用 GUI 时不需要启动相机或 Tracker Publisher。推荐从仓库根目录运行：

```bash
./tools/tracker_camera_calibration/run_gui.sh
```

可以指定数据根目录，或加载已有配置；加载配置本身不会自动订阅或录制：

```bash
./tools/tracker_camera_calibration/run_gui.sh \
  --output-root "${HOME}/visiontactile-data/calibration/gui"

./tools/tracker_camera_calibration/run_gui.sh \
  --config "${HOME}/visiontactile-data/calibration/my-campaign/config/d405_1.yaml"
```

脚本使用自身所在仓库的 `.venv-calibration` 和源码；不会跟随当前终端误用旧
worktree 中的工具。系统 ROS 和本仓库的 workspace setup 存在时会被加载。
脚本不会执行 `sudo`、自动安装依赖、构建 workspace 或启动硬件 Publisher。
脚本从本仓库根目录启动，配置/输出参数建议使用绝对路径。也可以在正确激活该
虚拟环境、加载 ROS/workspace 后运行 GUI 命令。

界面左侧为「配置与板图」「已有 bag 求解」两个页签，右侧为板图、质量报告与日志。

1. **生成或导入配置**：选择仓库外的会话数据根目录。可选择已有 YAML 后点击
   「导入配置并保存独立快照」；或手动选择相机与实际同支架 Tracker 角色，填写
   实测方格／Marker 边长、方格数和字典，勾选身份与尺寸确认后生成配置。
   相机和 Tracker 不默认配对。新配置使用第 6 节的默认板参数，导入配置保留原有
   实测参数；默认的 `15 / 11.25 mm` 仍须以实物测量核对。
2. **检查板图**：配置后右侧显示定尺寸像素预览；预览不是打印尺寸证明。
   「导出 300 DPI 板图 PNG」只写新文件、不覆盖已有板图。打印保持原尺寸并重新测量。
3. **准备专用 bag**：按第 8 节先录制、停止并关闭 bag；此 GUI 不负责在线录制。
   仍需至少 40 个通过筛选的静止姿态，动作与刚体约束遵循本文后续章节。
4. **开始离线求解**：在「已有 bag 求解」选择 bag 和全新的仓库外结果目录，
   勾选 bag 已关闭确认，点击「开始离线求解」。每次运行保存独立配置快照；
   不覆盖结果，不写入源 bag，不自动应用外参。
5. **查看结果**：右侧显示 `report.json` 的质量等级、计数与误差指标，以及
   求解日志末尾 128 KiB。「查看已有 report.json」可读取历史报告。
   退出码为零且报告为 `TARGET / ACCEPTABLE` 才显示本次求解完成；`REJECTED`
   或异常退出保留诊断，不能视为可用外参。关闭运行中的窗口会先请求取消本次计算，
   等待该子进程退出，不控制其他设备或 Recorder。

三次独立重复验证与最终归档仍按本文 CLI 流程进行。软件测试、板图显示和示例结果
均不代表真机标定已通过；本 GUI 不修改采集配置、角色映射或 TF。

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

CLI 的配置、板图生成、求解和重复性比较严格离线。独立 Tk GUI 仅封装配置、板图、
已有 bag 求解与报告查看。此处描述这两个兼容入口；主程序引导模式见前文。它们都不会：

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

读 bag 时先按每个 Topic 的记录顺序检查 host realtime 和该 Topic 的 monotonic
时钟，再进行时间排序与插值。两个时钟必须递增；相邻样本的 realtime 增量与
monotonic 增量之差不得超过 `5 ms`。Tracker 的 `host_monotonic_ns` 与相机的
`group_host_monotonic_raw_ns` 分别检查，原始数值随有效位姿和板观测保留，不跨域
相减。完全相同的相机时间映射可以重复，重用图像 stamp 却改变任一 host 时钟会被拒绝。
系统时间跳变、主机时钟重置或记录乱序会中止求解，不能靠排序把不同时钟时期混合。
该门禁不检测所有缓慢漂移，也不能证明跨主机同步；本工具要求输入来自同一采集主机。

静止筛选覆盖图像时刻前后各 `stability_window_ms` 的完整窗口：两端必须有
Tracker 样本支持插值，窗口内及跨越边界的采样间隔不得超过
`max_interpolation_gap_ms`。运动量取窗口内全部实测位姿和插值端点的最大两两
平移／旋转差。因此窗口中途移动后返回、窗口内部断流都会被拒绝。它只约束已采样的
运动，不能排除两个 Tracker 样本之间未被观测的瞬时运动。

候选姿态按时间选取，既满足最小时间间隔，也必须相对**每一个**已选姿态达到
平移或旋转多样性门槛；返回先前姿态不会重复计数。`CalibrationPair` 中的
`local_translation_motion_m` 和 `local_rotation_motion_deg` 是整个窗口的直径，
不再只表示两端差值。使用更严格筛选后有效姿态可能减少，应延长静止时间、增加
独立姿态或重新采集，不能直接把旧版本的入选计数当作本次验收结果。

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

新电脑先完成[部署指南的 ROS、固定驱动与主工作区构建](../../docs/deployment.zh-CN.md#ros)（[English](../../docs/deployment.md#ros)）。它提供 wrapper `4.58.1` / SDK `2.58.1` 的可复现源码路径，以及测试所需环境。不要在这里再次用未锁版本的 apt 包覆盖相机驱动。

已有匹配环境且工作区已构建时，只需加载并检查消息包：

```bash
test -z "${CONDA_PREFIX:-}"
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 pkg xml realsense2_camera | sed -n 's:.*<version>\(.*\)</version>.*:\1:p'
/usr/bin/python3 -c 'from vt_camera_msgs.msg import CameraFrameTiming; from vt_tracker_msgs.msg import TrackerSample; print("messages=OK")'
```

wrapper 版本应为 `4.58.1`。主工作区测试无失败后再继续；重建时保留部署指南的驱动 underlay。

### 4.3 安装 CLI 与 GUI 共用依赖

```bash
python3 -m venv --system-site-packages "${VT_REPO}/.venv-calibration"
source "${VT_REPO}/.venv-calibration/bin/activate"
python -m pip install --upgrade pip
python -m pip install --upgrade "${VT_REPO}/ros2_ws/src/vt_capture_core" \
  "${VT_REPO}/tools/tracker_camera_calibration"
```

GUI 使用系统 `tkinter`（`python3-tk`）和包依赖 Pillow，不需要 PySide/PyQt。
更新源码后重新执行最后一条安装命令，以更新虚拟环境内的 GUI console 入口。

验证安装和 ROS 消息环境：

```bash
vt-tracker-camera-calibrate --version
vt-tracker-camera-calibrate --help
vt-tracker-camera-calibrate-gui --help
python -c 'import cv2,numpy,rosbag2_py,rclpy; print("opencv=" + cv2.__version__ + " numpy=" + numpy.__version__)'
python -c 'import tkinter; from PIL import Image, ImageTk; print("gui_dependencies=OK")'
python -c 'from vt_camera_msgs.msg import CameraFrameTiming; from vt_tracker_msgs.msg import TrackerSample; print("messages=OK")'
```

预期版本为 `0.1.0`，子命令包含 `configure`、`board`、`calibrate` 和 `compare`，
OpenCV 为 `4.9–4.11`、NumPy 为 `1.26.x`，最后一条输出 `messages=OK`。
`--help` 与上述导入检查不打开相机、Tracker 或录制任务；真正显示 GUI 还需要
可用的桌面显示会话。若缺少依赖，请手动完成本节，不要让启动脚本自动提权安装。

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

新会话及新配置默认采用参考商品板 `CC200-15-11.25` 的几何参数：

| 参数 | 默认值 |
| --- | --- |
| 方格数（横 × 纵） | 12 × 9 |
| 方格边长 | 15 mm |
| 标记黑色外边长 | 11.25 mm |
| 字典 | `DICT_5X5_1000` |
| 有效图案尺寸 | 180 × 135 mm |
| 商品标称外板尺寸 | 200 × 150 mm，不参与求解尺度 |

参考图的标记和角点布局与该字典兼容；这不表示单凭图片能够确定字典容量。
求解使用方格和标记的实测边长，商品标称尺寸仍需核对。导入既有配置时保留其
实测参数；新默认值不迁移历史配置，也不改变已有标定结果。

New sessions and newly generated configurations use the `CC200-15-11.25` geometry:
12 × 9 squares, 15 mm square side, and 11.25 mm black marker side. The active
pattern is 180 × 135 mm. The advertised 200 × 150 mm outer board is not used as
solver scale. `DICT_5X5_1000` is compatible with the reference image; the image alone
does not establish dictionary capacity. Verify nominal dimensions on the physical
board. Imported configurations retain their measured parameters; existing
configurations and calibration results are not migrated to the new defaults.

### 6.1 生成 300 DPI 打印文件

```bash
vt-tracker-camera-calibrate board \
  --config "${VT_REPO}/tools/tracker_camera_calibration/config/calibration.example.yaml" \
  --output "${CAL_ROOT}/boards/charuco-12x9-300dpi.png" \
  --dpi 300
```

默认示例配置生成 `12×9` 方格、`DICT_5X5_1000`，PNG 仅包含名义尺寸为
`180×135 mm` 的有效图案，不包含外板留白。使用 A4 纸张，以 100% / Actual Size
打印，关闭 Fit、Shrink 和 Scale to page；不要将图案拉伸至 `200×150 mm`。
将纸张粘贴到平整、刚性的哑光板材上，不得折叠、翘曲或覆盖反光膜。

The example configuration exports only the 180 × 135 mm active pattern, without
the outer board margins. Print on A4 at 100% / Actual Size with page fitting disabled;
do not stretch the pattern to 200 × 150 mm. Measure the finished print before use.

### 6.2 测量成品

使用卡尺完成以下测量：

1. 水平方向测量连续 5 个方格的总长度并除以 5。
2. 垂直方向测量连续 5 个方格的总长度并除以 5。
3. 在左、中、右三个区域测量 Marker 黑色外边长并取平均。
4. 水平与垂直方格边长相差超过 `0.20 mm` 时重新打印。
5. 使用实测平均值，不使用名义打印尺寸。

```bash
export SQUARE_LENGTH_MM="15.00"  # 名义示例，须替换为成品实测值
export MARKER_LENGTH_MM="11.25"  # 名义示例，须替换为成品实测值

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
  config_path:="${VT_DATA_ROOT}/config/cameras.yaml" \
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
| `non-increasing host clock` / `host clock step exceeds 5 ms` | 时钟跳变、重置或记录乱序 | 检查采集主机时钟与输入连续性，修复后录制新 bag；不要排序或拼接以绕过检查。 |
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
