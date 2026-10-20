# 触觉 ROS 2 采集、手形热力图与录包

本包将原 `Glove.py` 的 STM32 触觉协议接入 ROS 2 Jazzy。
采集节点独占指定的触觉串口，界面只订阅 ROS；**不会打开 IMU 串口、运行手势
求解器或改变手势标定**。关闭热力图不影响后台采集。

当前测量是 **ADC 计数**，不是 N、Pa 或 kPa。已确认有效区域为原始行、列
**0–19，即 20×20、400 个触觉值**；其余协议地址不是有效触觉区域。
ROS `state`、二维热图、三维手表面叠加、显示参考及离线导出统一使用这个矩阵。
矩阵是逐块更新的滚动快照，不是同时采样的完整扫描帧。逐点空间位置、真实
扫描速度、接触方向以及压力转换曲线仍须另行验证。

串口仍按原始 80×80 寻址协议解析，`samples` 保留原始数据包，保证旧 bag
兼容和传输追溯。**`samples` 是包批次，不是 80×80 的有效整掌矩阵。**
独立旧入口 `Glove.py` 的显示和 CSV 也已统一为 20×20；使用 ROS 时不要同时
启动它，它会自行占用串口。

## 可选：在手势标定界面的三维手上显示

已完成二维触觉调试后，可使用[三维手表面触觉说明](../../../tools/fsglove12/README.md#在三维手模型上显示-ros-触觉)。
`vt_tactile_display_bridge` 只订阅本包的 `state` 并转发到手势 GUI 的私有本机 socket；
不再开串口、不修改采集节点或 bag 数据。两个视图可以同时运行。
手势联合启动增加 `tactile_overlay:=true` 会启动这个桥接；不要同时重复启动它。
也可独立运行：

```bash
source /opt/ros/jazzy/setup.bash
source "$HOME/VisionTactile-Dataset/ros2_ws/install/setup.bash"
ros2 launch vt_tactile_capture tactile_display_bridge.launch.py
```

GUI 必须带上匹配的 `--tactile-socket`，详见链接。显示的仍是有符号 ADC 差值，
不是压力；三维空间位置是候选映射。二维与三维窗口分别手动采集无载参考，
不会自动同步零点。新三维叠加的真机验收与已通过的二维触觉验收是不同项目。

## 1. 安装与编译

新电脑或主工作台用户先按[部署指南](../../../docs/deployment.zh-CN.md)构建主工作区；后续增加包时继续采用普通安装方式，不在已有构建目录切换 `--symlink-install`。下文的 `VT_REPO` 必须指向实际克隆路径。

以下命令在 Ubuntu 24.04 桌面终端中执行。不要使用手势的 Conda/PyTorch 环境
编译 ROS 包。依赖已安装时不必重复安装：

```bash
sudo apt install python3-serial python3-numpy python3-pyqt5 python3-pyqtgraph \
  ros-jazzy-diagnostic-msgs ros-jazzy-rosbag2-storage-mcap
export VT_REPO="$HOME/VisionTactile-Dataset"
source /opt/ros/jazzy/setup.bash
cd "$VT_REPO/ros2_ws"
colcon build --packages-select vt_tactile_msgs vt_tactile_capture
source install/setup.bash
```

也可用 `rosdep install --from-paths src/vt_tactile_msgs src/vt_tactile_capture
--ignore-src -r -y` 安装包声明的依赖。

若没有 sudo 权限但系统已有 ROS、PyQt5、NumPy、pyserial，GUI 可以使用单独的
虚拟环境，不改动系统 Python：

```bash
cd "$VT_REPO"
/usr/bin/python3 -m venv --system-site-packages .venv-tactile
.venv-tactile/bin/python -m pip install pyqtgraph==0.13.7
```

下文 launch 增加 `gui_python:="$VT_REPO/.venv-tactile/bin/python"` 即可使用它。
不要使用 `sudo pip` 或为了触觉功能修改现有 FSGlove 环境。

## 2. 确认触觉串口

仅枚举设备，不会打开串口：

```bash
/usr/bin/python3 -m serial.tools.list_ports -v
```

确认触觉板对应的 `/dev/serial/by-id/...`；若设备没有唯一序列号或 by-id 不唯一，
使用确认过的 `/dev/serial/by-path/...`。也支持显式 `/dev/ttyACM*`，但重新插拔后
编号可能改变。**不能仅凭 STM32/USB 产品名区分触觉板与 IMU 板。**

启动前只关闭占用该触觉端口的旧 `Glove.py`/串口工具，保留手势程序。
新节点检查端口占用并请求独占；不会杀掉旧进程或抢占端口。
没有权限时将用户加入 `dialout` 并重新登录，不要对设备设置全局可写权限。
若 `id "$USER"` 已包含 `dialout`，但当前终端的 `id` 不包含，说明旧会话尚未
继承新组。可以在用于启动触觉的终端执行 `sg dialout` 进入子 shell，再执行下节
的 source 和启动命令，无需关闭其他传感器。**必须在切换组之后重新 source ROS**，
因为切换组可能清除 `LD_LIBRARY_PATH`，否则会出现 `librcl_action.so` 找不到。

## 3. 一条命令启动采集和热力图

每个新终端先 source ROS 和本仓库的 workspace：

```bash
export VT_REPO="$HOME/VisionTactile-Dataset"
source /opt/ros/jazzy/setup.bash
source "$VT_REPO/ros2_ws/install/setup.bash"
ros2 launch vt_tactile_capture tactile.launch.py \
  port:=/dev/serial/by-id/替换为触觉设备 \
  gui_python:="$VT_REPO/.venv-tactile/bin/python"
```

系统已安装 pyqtgraph 时可以省略 `gui_python`。
`gui:=false` 仅采集。未指定或传空 `port`/`hand` 时沿用 `config:=...` 指定的
YAML 文件设置；默认 YAML 的 `hand` 是 `right`、`port` 为空。命令行与配置文件
都未设置串口时不猜测设备、不连接任何串口。非空命令行值优先于 YAML。
默认命名空间为 `/gloves/right/tactile`，`hand:=left` 时同时指定
`namespace:=/gloves/left/tactile`，防止左右手混入同一 Topic。
缺省手形布局仍适用于本次确认的**右手**；左手默认原始矩阵，无显式左布局时不展示解剖手形。
支持加载手别匹配的独立 `hand_layout_path`；内置 `left_glove_unverified_candidate.json` 是无实测锚点的
候选模板，不能冒充左手物理映射已验证。左手或其他硬件先使用
`initial_view:=matrix` 查看原始矩阵，不套用右手位置关系。

设备与速率设置位于 [config/right_tactile.yaml](config/right_tactile.yaml)。可复制
此文件到数据目录，使用 `config:=/绝对路径/自定义.yaml` 加载，不必改源代码。
`max_cell_age_ms` 默认 100 ms（区域有效期），`stale_timeout_sec` 默认 2 秒
（无完整子包时报告状态），`state_rate_hz` 默认 30（显示快照率）。它们分别控制
不同环节；参数为启动时配置，不通过动态改参中途切换采集语义。

默认 `read_batch_bytes=1024`、`read_timeout_sec=0.002`：每次真实串口 `read()`
以至少 1024 字节为目标（已有积压时最多读取 4096 字节），收到目标数量即返回，
否则约 2 ms 超时后返回已有字节。这样降低许多小 ROS 消息的开销，而不丢弃子包、
不为凑频率限流，也不合并已带时间戳的批次。短读和跨读的半包仍由解析器保留处理。
`read_batch_bytes` 必须是 1..4096 的整数，`read_timeout_sec` 必须为正有限数；
二者也是启动时参数。默认采集队列 512 批、samples QoS depth 128 只提供拥塞缓冲，
不能保证零丢包；吞吐优化主要来自真实 read 批量化。

### 3.1 手形视图与原始矩阵

GUI 默认打开手形页，另一个页签显示 **20×20 有效区域 ADC 矩阵**，可以随时切换。
右手手心朝向观察者，拇指在左；原始 ADC 仍使用 plasma 配色，界面主题不变。
手形页以局部圆角色块显示候选地址，一个色块对应一个原始地址；不跨手指插值，
不把指间空隙填成额外触点。轮廓与色块位置是示意，**不是三维手势模型，也不
跟随 IMU 改变手形**。鼠标悬停在色块上可查看原始行列、ADC，以及存在可用
参考时的基线差值。行列从 0 编号。

原始矩阵的行列裁剪只在矩阵页显示；“高级 / 诊断”默认收起，展开后可以查看
更详细的显示设置、采集身份和状态。收起诊断只是整理界面，不减少记录数据。

内置布局位于
[right_glove_candidate.json](vt_tactile_capture/layouts/right_glove_candidate.json)。
它结合手套照片和 2026-10-05 的交互按压结果，将原始行列映射到右手示意图：

| 部位 | 候选原始行列范围（含端点） | 实测锚点 `(row, col)` |
| --- | --- | --- |
| 拇指 | 行 8–19、列 16–19 | `(19, 19)` |
| 食指 | 行 8–19、列 12–15 | `(18, 15)` |
| 中指 | 行 8–19、列 8–11 | `(19, 11)` |
| 无名指 | 行 8–19、列 4–7 | `(18, 7)` |
| 小指 | 行 8–19、列 0–3 | `(18, 1)` |
| 手掌 | 行 0–7、列 0–19 | 掌心 `(4, 9)`、`(4, 10)`；掌根 `(0, 9)`、`(0, 10)` |

定位圈默认隐藏，需要检查映射时勾选“显示定位圈”。青色描边表示
**7 个区域的 9 个实测锚点**，不是正在接触的热点，也不是额外的触觉信号。
图中共 **400 个有效地址的候选空间位置，不代表逐点空间映射全部验证完毕**；
每个区域的完整边界、物理尺寸、逐点对应关系仍待验证。原始行或列编号 ≥20
的数据仍保留在 `samples` 录包，但不进入状态矩阵、有效率统计和显示参考。
裁剪不重排有效区行列，不修改有效 ADC 数值，也不改变串口协议与原始录包。

### 3.2 采集不受压参考，显示接触变化

1. 保持手套触觉面不受压；不要把掌心压在桌面上。
2. 点击界面的“采集静止参考（3 秒）”按钮，按界面提示保持不受压约 **3 秒**。
3. 参考采集需要至少 **10 个不同的新快照**，不会通过重复旧帧凑数。成功后可
   查看相对参考的变化；区域无有效参考时不把它当成零接触。
4. 再按压指腹或掌心，检查该部位颜色变化，并用悬停读数核对原始 ADC。

参考仅用于界面显示：`delta = 当前 ADC - 不受压参考`，不是压力单位，也不
修改后续 ROS 或离线数据。**不会自动归零、自动消除持续按压**；需要重新取
参考时由用户明确点击。断连重连或采集重启导致 `session_id`/`stream_epoch`
变化后，应重新采集参考，不沿用上一段数据掩盖变化。

也可以通过界面或启动参数加载参考 `.npz`，同时需要**同名 `.json` 元数据**。
当前支持本工具采集流程保存的参考数据，不是任意 NumPy 文件；载入参考必须
与当前流的 `session_id` 和 `stream_epoch` 一致，否则不能作为该流的有效参考。
本轮实验的可选示例是：

```text
$HOME/visiontactile-data/tactile_mapping_20261005_02/baseline.npz
$HOME/visiontactile-data/tactile_mapping_20261005_02/baseline.json
```

同一采集会话的旧 80×80 参考可加载，工具会显式裁剪到 `[0:20, 0:20]`；
新采集的内存参考同样为 20×20。兼容尺寸不意味着可以跨会话复用参考。
这些是本机本轮实验文件，**不是其他电脑运行软件的必备文件**。新采集会话直接
使用按钮采集自己的不受压参考，不要复制旧参考强行复用。

原始 ADC 默认色限为 **20000..24000**。ΔADC 视图提供三种显示范围：

| 选项 | 色限 | 用途 |
| --- | --- | --- |
| 轻触显示（默认） | **−3000..3000 ADC** | 放大小幅变化的颜色差异 |
| 强响应 | **−16000..16000 ADC** | 查看较大幅度的正负变化 |
| 自定义 | 用户指定 | 按实际数据选择固定显示范围 |

“轻触”“强响应”只是色限预设名称，不是已标定的压力等级。ΔADC 的正向变化用
暖色、负向变化用蓝色，参考附近用中性色；缺少数据或可用参考用深灰，不当成
零接触。默认中性带为 **±200 ADC**，可在高级设置调整：这仅改变颜色映射，
**不会把原始 ADC 或差值归零，不会滤除、改写 ROS 与导出数据**。中性色不等于
已经判定“没有接触”，也不是噪声或压力阈值的最终标定结论。

色限只控制颜色，**不截断数值、不判定传感器饱和**；0..65535 的 uint16 原始
ADC 仍完整保留。超出色限的值使用端点颜色，但悬停数值和右侧峰值仍显示真实
幅度。原始 ADC 基线低于原始色限时画面可能颜色相近，应查看数值或使用差值
视图，不能仅凭颜色判定断流。

手形右侧汇总 **五指、掌心、掌根 7 个候选区域**：每区显示有效地址中
`abs(ΔADC)` 最大的一项，并保留其正负号；例如显示 `−1200` 表示该区最大的
绝对变化是下降 1200 ADC，不是平均值。另显示严格超过当前显示阈值的地址数
（默认 `abs(ΔADC) > 200`）及有效地址数。这些统计**不是接触面积、接触概率
或压力**，没有有效参考/数据时显示未就绪，不用零代替。

“超色标”仅表示真实变化超出选定显示色限，**不是传感器截顶或硬件饱和告警**。
右侧也提示未映射地址的最大变化及原始行列；若它比手形区域更显著，请切换
原始矩阵核对，不能将未映射响应隐藏或擅自归入某根手指。

### 3.3 已在采集时，只打开或重开 GUI

不要再次运行采集 launch 去抢同一个触觉串口。可以关闭旧 GUI，保留采集节点，
在新的桌面终端仅启动订阅窗口：

```bash
export VT_REPO="$HOME/VisionTactile-Dataset"
source /opt/ros/jazzy/setup.bash
source "$VT_REPO/ros2_ws/install/setup.bash"
"$VT_REPO/.venv-tactile/bin/python" \
  "$(ros2 pkg prefix vt_tactile_capture)/lib/vt_tactile_capture/vt_tactile_viewer" \
  --ros-args -r __ns:=/gloves/right/tactile
```

可选参数放在 `--ros-args` 之后，例如：

```text
-p initial_view:=matrix
-p baseline_path:=$HOME/visiontactile-data/tactile_mapping_20261005_02/baseline.npz
-p hand_layout_path:=/绝对路径/候选布局.json
```

`initial_view` 仅接受 `hand` 或 `matrix`；空 `hand_layout_path` 使用内置候选
布局，空 `baseline_path` 表示不预加载参考。自定义布局遵循随附 JSON 的
schema，不合法的地址、重复映射、非有限坐标或不支持的左右手配置会被拒绝。
以上三个参数也可以作为 `tactile.launch.py` 和 `replay.launch.py` 的同名参数
（如 `initial_view:=matrix`），只传给 GUI，不改变采集或回放节点。

### 3.4 断流与导出

“Clear display / 清除显示和参考”只清理 GUI 画面和显示参考，不清除采集和录包数据。
窗口是滚动视图；断流时可以保持最后
画面，但显示源数据年龄与断连/过期状态。**保持的画面不是新测量。**
PNG 导出当前页画面；CSV 始终按原始行列顺序导出 **20×20 有效区域 ADC 矩阵**，
不是手形重排，也不是差值。伴随 JSON 记录有效 mask、显示像素的历史来源及
映射/参考等显示元数据。PNG、CSV 和 JSON 都是查看快照，**不替代带时间戳
的 ROS bag**，也不构成逐点布局或压力标定结果。

## 4. Topic 与数据契约

| Topic 后缀 | 类型 | 默认录制 |
| --- | --- | --- |
| `samples` | `vt_tactile_msgs/msg/TactileSampleArray` | 是，原始解码数值及故障事件 |
| `state` | `vt_tactile_msgs/msg/TactileState` | 否，20×20 派生滚动快照 |
| `heatmap` | `sensor_msgs/msg/Image`，可选 20×20 RGB 预览 | 否 |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 否 |

`publish_heatmap:=true` 可启用通用图像预览；其颜色只用于显示，不可反推压力。
使用质量字段时以 `.msg` 定义为准：
[TactileSampleArray](../vt_tactile_msgs/msg/TactileSampleArray.msg)、
[TactileState](../vt_tactile_msgs/msg/TactileState.msg)。

`state.schema_version=2`、`rows=cols=20`、
`layout_id=row20_col_group_stride8_v2`。`adc`、`valid_mask`、`updated_mask` 各有
400 项，行优先索引为 `row*20+col`；`block_*` 各有 160 项。
新 GUI 与三维桥接也接受旧 `schema_version=1, rows=cols=80` 状态并裁剪；
旧版 GUI 不识别新状态，需要关闭并用新版重新启动。只重编译不会替换运行中
进程的内存代码。升级前保存手势标定结果；无需重新进行 IMU 求解器开发或配置。
状态发布仍默认 **30 Hz**；缩小矩阵降低传输和显示开销，不改变固件扫描速率。

### 旧订阅程序适配

仓库内的二维窗口、ROS→三维显示桥接、手套三维叠加、RGB 热图转换和离线
重建均已同步；不需要用户逐个修改。独立脚本应读取消息尺寸，不能继续使用
`reshape(80, 80)`、`row*80+col` 或固定 6400 项计数。已 source 新 workspace
的 Python 订阅回调可以复用同一个兼容入口：

```python
import numpy as np
from vt_tactile_capture.grid import normalize_state

def on_state(message):
    state = normalize_state(message)  # 新版直接使用；旧版按行列裁取 [:20, :20]
    shape = (state.rows, state.cols)
    adc = np.asarray(state.adc, dtype=np.uint16).reshape(shape)
    valid = np.asarray(state.valid_mask, dtype=bool).reshape(shape)
    if not state.connected or not state.time_valid:
        valid = np.zeros(shape, dtype=bool)
    # 使用 adc 和 valid；ADC=0 可能是真实测量，不能用非零值代替有效掩码。
    # 保留 state.header.stamp 与逐块时钟，不以此回调的执行时间替换采集时间。
```

不要简单截取 `message.adc[:400]`：旧矩阵每行的步长为 80，正确的转换是
先还原旧行列，再裁取 `[0:20, 0:20]`。兼容入口不修改输入消息；接收新格式后
无需再对其重复裁剪。`samples` 订阅端仍按原协议解码，不应把每包 10 个 ADC
擅自改成其他数量。

### 协议保持不变

每个 26 字节包：

```text
A5 5A | row_hi row_lo | unknown_byte4 | column_group | 10 × uint16 big-endian ADC
```

- `row` 为 0..79；`column_group` 为 0..7。
- 第 k 个值写到 `(row, column_group + 8*k)`，k=0..9。
- `state` 仅提取 `row<20` 且 `column_group+8*k<20` 的值，每个有效子包
  贡献 2 或 3 个触点；不会把一个 10 值原始子包误当作 10 个有效相邻触点。
- `unknown_byte4` 原样保存在 `status_raw`，不臆测为 CRC 或硬件序号。
- 当前没有已确认的校验和，软件只能做帧头、长度、地址检查和重同步；无法发现
  所有看起来仍合法的传输损坏，也不能从主机计数推断固件端全部丢样。
- 零 ADC 是合法数值，缺失由 mask/时间表示，不用零值代表无接触。

### 时间与连续性

`samples` 的每个消息对应一次读完成时解析出的子包集合。所有子包共享这次主机
接收时间；包顺序保留，但不宣称它们在硬件上同时采样。
批量读取最多引入约 `read_timeout_sec` 的主机等待（另有操作系统调度延迟）；
这是主机接收语义，不是硬件采样时间或同步精度。时间戳始终在该次真实 read
返回后立即取得，不回填或篡改为批次开头时刻。

- `header.stamp == host_realtime_ns`：主机读完成时刻，不是 ROS 发布时间。
- `host_monotonic_ns` 的域为 `CLOCK_MONOTONIC`；`timing_source=host_receive`。
- `session_id` 标识采集进程，`stream_epoch` 隔离断连重连。
- `sequence` 是软件批次序号，`first_packet_index` 是软件解码包序号，均不是固件序号。
- `producer_dropped_total` 统计有界队列丢弃的批次数，包括空状态事件。相关缺口
  可能重复反映同一丢失，不应把各种计数简单相加。
- 无数据/断连事件的 `blocks` 为空，不是全零触觉帧。
- `state` 保留最后一次贡献有效区域数据的输入时刻；外围无效地址包不会刷新
  此时间，`snapshot_monotonic_ns` 只用于计算年龄。
  `valid_mask` 表示当前可用，`updated_mask` 表示本次快照消费的新更新。
- `state.block_*` 按 `row*8+column_group` 索引，共 160 项；零表示尚未观测。

同机跨模态使用 host realtime 对齐。相机 `CLOCK_MONOTONIC_RAW` 与手套/触觉
`CLOCK_MONOTONIC` 不能直接相减。USB 缓冲、固件扫描延迟仍存在，这不是硬件同步。
实时采集节点拒绝 `use_sim_time=true`；回放节点和 GUI 可以使用 `/clock`。

采集线程与发布/显示分离、队列有界。拥塞可能丢批次，但不会无限积压旧数据。
采集、ROS、录包和 GUI 的速度必须区分；30 Hz 画面不等于每触点 30 Hz 更新，
更不会为了凑 120 Hz 重复发送旧测量。

低频 `diagnostics` 提供 `batch_rate_hz`、`packet_rate_hz`（上个诊断窗口内的主机
发布批次/解码子包速率；批次包含空状态事件）、`valid_cell_count`（最大 400）、
`cell_count=400`、`rows=cols=20` 和
`source_age_ns`（最后贡献数据到当前主机单调时钟的年龄，未观测为 -1）。这些
是调试主机吞吐与陈旧阈值的指标，**不是硬件扫描率或每触点更新率**；选择
`max_cell_age_ms` 仍须观察各区域更新时间与实际扫描时长。

## 5. 录包

先进行仅触觉的小包验证：

```bash
ros2 topic info -v /gloves/right/tactile/samples
ros2 topic hz /gloves/right/tactile/samples
ros2 bag record -s mcap -o "$HOME/visiontactile-data/tactile_test_01" \
  /gloves/right/tactile/samples
```

输出目录必须尚不存在，结束录包使用 Ctrl+C，等待 rosbag 正常关闭。
`topic hz` 是批次/事件频率，不是完整扫描率。

手势加触觉最小录制：

```bash
ros2 bag record -s mcap -o "$HOME/visiontactile-data/glove_tactile_01" \
  /gloves/right/state /gloves/right/tactile/samples
```

三相机 + 三 Tracker + 手势 + 触觉使用新增的
[录包配置](../vt_realsense_capture/config/cameras.with_right_glove_tactile.yaml)：

```bash
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  config_path:="$VT_REPO/ros2_ws/src/vt_realsense_capture/config/cameras.with_right_glove_tactile.yaml" \
  output_root:="$HOME/visiontactile-data"
```

该 launch 只启动相机与采集控制，不会代替 Tracker、手势或触觉的启动。
待各数据流运行后按[现有采集流程](../../../docs/capture-guide.md)开始录制。
不要在已经启动的三相机上重复运行该 launch。

联合 Recorder 默认保持 best-effort / volatile；相机和 Tracker 的 depth 为 30，
按配置声明的类型为触觉 `TactileSampleArray` 使用 depth 512、手势 `GloveState`
使用 depth 256。触觉约 500 批/秒时，512 批在均匀流下约对应 1 秒的订阅历史容量，
不是时延目标或零丢包保证。可通过相机采集配置的 `recording.qos_overrides` 按
明确 Topic 覆盖 depth / reliability；最终参数保存在会话的 `qos_overrides.yaml`。
不要使用 `-a` 把额外预览 Topic 随意录入严格
离线配置的 bag。传感器质量问题不会自动停止其他采集节点。

结束录制后可以生成[统一采集质量报告](../../../tools/multisensor_alignment/README.md#116-录制结束后的统一质量报告)，
汇总消息数、频率依据、时间间断及触觉覆盖情况，不改变原 bag，也不拦截下一次采集。

## 6. 回放与离线重建

主录制流是 `samples`，因此回放时需要一个**只订阅 samples、不打开串口**的
重建节点，把它转换为 `state` 后再交给同一热力图。不能把实时采集节点接到
录包上，或依靠重复发布保存的截图模拟回放。

先停止同一命名空间的实时触觉节点，防止 live 与 replay 混流。单独的回放终端：

```bash
ros2 launch vt_tactile_capture replay.launch.py \
  gui_python:="$VT_REPO/.venv-tactile/bin/python"
```

另一终端播放之前录制的 bag：

```bash
ros2 bag play "$HOME/visiontactile-data/tactile_test_01" --clock
```

先启动订阅端，再播放 bag，避免遗漏开头数据。GUI 显示 REPLAY；暂停播放时
不会用当前主机单调时钟将历史记录误判为新数据。正常查看历史时间，不连接设备。

### 与相机、Tracker、手势对齐

按[离线工具手册](../../../tools/multisensor_alignment/README.md)安装
`vt-multisensor-align`，使用新增的
[alignment.right_glove_tactile.yaml](../../../tools/multisensor_alignment/config/alignment.right_glove_tactile.yaml)
执行 `inspect`、`align`、`validate`，得到 `aligned_frames.jsonl`。
该配置保留原手势要求，触觉作为 optional 扩展流，尚未经过真机覆盖率验收时
不阻止其他模态生成索引。缺失触觉不能解释为零接触。

随后导出相机参考时刻的触觉矩阵：

```bash
vt-multisensor-align export-tactile \
  --bag "$HOME/visiontactile-data/unified_01" \
  --aligned-frames "$HOME/visiontactile-data/aligned_01/aligned_frames.jsonl" \
  --output "$HOME/visiontactile-data/tactile_aligned_01" \
  --max-age-ms 100
```

输出是 `manifest.json`、`source_metadata.jsonl` 和每参考帧一个
`frames/000000000.npz`，以及汇总有效覆盖率的 `quality_report.json`。导出自动核对
同目录的对齐 manifest；缺少历史来源信息时明确标为未验证，来源不符则报错。
正式交付可添加 `--require-provenance`。每个 NPZ 包含：

- `timestamp_ns`：相机参考时刻，host realtime；
- `adc[20,20]`、`valid_mask[20,20]`、`age_ns[20,20]`；
- `block_realtime_ns[20,8]`、`block_monotonic_ns[20,8]`；
- `block_message_sequence[20,8]`、`block_packet_index[20,8]`，追溯源消息和包。

每个区域只用目标时刻及以前收到的数据，超过最大年龄则无效，不做触觉线性
插值。缺失区域 `age_ns=-1`；过期区域保留其年龄，导出 ADC 占位值为零，**必须
同时读取 valid_mask**。例：

```python
import numpy as np
with np.load("frames/000000000.npz", allow_pickle=False) as frame:
    values = frame["adc"]
    mask = frame["valid_mask"]
    valid_values = values[mask]  # 零值也可能是有效测量
```

`--max-age-ms 100` 是可配置初始阈值，不是已测同步精度。断连、重连、批次/包
序号缺口、解析丢弃以及时钟异常会切断连续性；不会将断点前旧值带到新段。
离线发现缺口时还会保守标记相邻接收之间的未知区间。这不会修改原 bag，也不
会让采集端停机。导出为流式处理，只保留一个滚动矩阵及一个批次前瞻，不为
每条串口消息复制整张网格；输出目录已存在时拒绝覆盖。

## 7. 验证范围与真机验收

软件测试应覆盖协议逐字节分片/粘包/错位重同步、stride-8 地址映射、零 ADC、
时间戳保留、队列溢出、独占串口、断连重连隔离、区域过期、GUI 无串口订阅、
DDS 发布以及 MCAP 数值回放。软件通过不代表真实触点/扫描率或压力精度通过。

首轮真机建议：

1. 仅触觉运行，逐个按压已知区域，确认位置、ADC 变化方向和真实更新率。
2. 同时运行手势，确认触觉与 IMU 端口没有互换、没有第二个读取者。
3. 连续录制 300 秒，检查批次/包序号缺口、各区域更新时间、队列丢弃和 CPU。
4. 回放同一个 bag，核对数值/位置/时间戳；断流时保持画面但明确标记过期。
5. 再加入相机与 Tracker，依据实测扫描时长设置区域最大年龄和离线匹配容差。

遇到“无有效区域”先检查实际区域更新间隔与 `max_cell_age_ms` 设置；不要通过无限延长
过期时间把历史触觉变成有效测量。稀疏布局允许部分区域未观测，不要求填满
20×20 才发布或录制。

### 不接硬件的开发测试

```bash
source /opt/ros/jazzy/setup.bash
source "$VT_REPO/ros2_ws/install/setup.bash"
cd "$VT_REPO"
ROS_DOMAIN_ID=196 QT_QPA_PLATFORM=offscreen PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  VT_TACTILE_E2E=1 VT_TACTILE_TEST_DOMAIN_ID=156 \
  .venv-tactile/bin/python -m pytest ros2_ws/src/vt_tactile_capture/test -q
PYTHONPATH="$VT_REPO/tools/multisensor_alignment/src:$PYTHONPATH" PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /usr/bin/python3 -m pytest tools/multisensor_alignment/tests -q
```

E2E 测试只创建自己的 `/dev/pts` 虚拟串口并使用隔离 DDS domain，验证串口→
ROS→MCAP→离线 NPZ；不打开 `/dev/ttyACM*`、`/dev/ttyUSB*` 或传感器。
未设置 `VT_TACTILE_E2E=1` 时跳过该端到端测试。GUI 测试采用 offscreen，
不会关闭或重启用户现有窗口。`test_legacy_parity.py` 直接提取旧 `Glove.py` 的
数值解析方法作对照，不执行其 GUI 构造和自动串口连接。
