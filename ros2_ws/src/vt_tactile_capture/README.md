# 触觉 ROS 2 采集、手形热力图与录包

本包将新版 STM32 **44 字节整行触觉协议**接入 ROS 2 Jazzy。
采集节点独占指定的触觉串口，界面只订阅 ROS；不会打开 IMU 串口、运行手势
求解器或改变手势标定。关闭热力图不影响后台采集。

串口原始数据是 **ADC 计数**，不是 N、Pa 或 kPa。每包提供一行的 20 个值，
原生矩阵为 **20×20、400 个地址**。ROS `state`、二维热图、三维手表面叠加、
显示参考及离线导出使用同一行列顺序。矩阵按行更新，每行保留实际主机接收时间，
是滚动快照，不是同时采样的完整扫描帧。逐点空间位置、真实扫描速度、受压响应
方向以及压力转换曲线仍须实机确认。

实时采集使用新版协议：原始 `TactileRow44Array` schema 2、显示 `TactileGridState` schema 3。
旧 26 字节 `TactileSampleArray`/`TactileState` 消息定义保持不变，供档案及离线工具读取；
旧参考不会自动变成新板的参考。历史原始 bag 由离线触觉适配器按实际类型重建，
保留其 160 个 block 时钟，不伪装成 20 个同步行时钟。`samples` 保存真实行包及故障事件，不能将其消息频率
当作完整矩阵扫描率。独立示例或其他串口工具与 ROS 节点不能同时占用同一端口。

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

本轮在 `vt_tactile_msgs` 新增独立消息类型：`TactileRow44` 只有行号和 20 个 ADC，
`TactileRow44Array` 保存原始批次，`TactileGridState` 使用 20 项逐行时钟。
旧三个消息类型的字段不变。升级源码后必须重新编译上述两个包，并在
新终端 source 新的 `install/setup.bash`；发布者、订阅者和回放环境使用同一版本。
已经运行的进程不会热加载新消息，须正常关闭后重启。旧消息生成文件不能与新代码混用。
本包源码迁移不代表 Windows 安装包已重建；已有 `.exe` 仍包含构建时的代码。

综合工作台使用 `tactile.launch.py display_bridge:=true display_socket:=... gui:=false`。
`vt_tactile_capture.display_transport.default_workbench_socket(hand)` 给出每手独立的私有路径，
与原 FSGlove 叠加窗口隔离。ROS 节点是唯一串口读取者；工作台 `ManagedTactileSource`
只接收完整状态、维护共享显示参考和发送管理请求。关闭视图不停止采集。

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

启动前只关闭占用该触觉端口的示例程序或串口工具，保留手势程序。
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
缺省手形布局是既有**右手候选映射**，新版板卡的位置对应关系仍需核对；左手默认原始矩阵，无显式左布局时不展示解剖手形。
支持加载手别匹配的独立 `hand_layout_path`；内置 `left_glove_unverified_candidate.json` 是无实测锚点的
候选模板，不能冒充左手物理映射已验证。左手或其他硬件先使用
`initial_view:=matrix` 查看原始矩阵，不套用右手位置关系。

设备与速率设置位于 [config/right_tactile.yaml](config/right_tactile.yaml)。可复制
此文件到数据目录，使用 `config:=/绝对路径/自定义.yaml` 加载，不必改源代码。
`max_cell_age_ms` 默认 100 ms（区域有效期），`stale_timeout_sec` 默认 2 秒
（无有效行包时报告状态），`state_rate_hz` 默认 30（显示快照率）。它们分别控制
不同环节；参数为启动时配置，不通过动态改参中途切换采集语义。

默认 `read_batch_bytes=1024`、`read_timeout_sec=0.002`：每次真实串口 `read()`
以至少 1024 字节为目标（已有积压时最多读取 4096 字节），收到目标数量即返回，
否则约 2 ms 超时后返回已有字节。这样降低许多小 ROS 消息的开销，而不按显示频率丢弃行包、
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
它来源于手套照片和 2026-10-05 的历史交互按压结果，将行列映射到右手示意图。
这些锚点描述既有布局的依据，不能替代对新版板卡布线的重新核对：

| 部位 | 候选原始行列范围（含端点） | 实测锚点 `(row, col)` |
| --- | --- | --- |
| 拇指 | 行 8–19、列 16–19 | `(19, 19)` |
| 食指 | 行 8–19、列 12–15 | `(18, 15)` |
| 中指 | 行 8–19、列 8–11 | `(19, 11)` |
| 无名指 | 行 8–19、列 4–7 | `(18, 7)` |
| 小指 | 行 8–19、列 0–3 | `(18, 1)` |
| 手掌 | 行 0–7、列 0–19 | 掌心 `(4, 9)`、`(4, 10)`；掌根 `(0, 9)`、`(0, 10)` |

定位圈默认隐藏，需要检查映射时勾选“显示定位圈”。青色描边表示
**7 个区域的 9 个历史实测锚点**，不是正在接触的热点，也不是额外的触觉信号。
图中共 **400 个有效地址的候选空间位置，不代表逐点空间映射全部验证完毕**；
每个区域的完整边界、物理尺寸、逐点对应关系仍待验证。新版协议仅接受 0–19 的
行号，一包的 20 个值依次对应列 0–19；越界行号拒绝解析。矩阵无需从更大地址空间
裁剪，界面的显示范围设置不改变原始 ADC、协议内容或录包。

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

也可以通过界面或启动参数加载按当前契约生成的参考 `.npz`，同时需要
**同名 `.json` 元数据**。界面按钮采集的是内存参考，不会自动保存 NPZ 归档；
文件加载接口面向自行归档的采集数据，不支持任意 NumPy 文件。

归档中的 `adc` 必须为 uint16、`valid_mask` 必须为 bool，尺寸均为
`(frames, 20, 20)`。JSON 必须声明 `protocol="vt_tactile_row44_v1"`、
`layout_id="row20_col20_v3"`、`state_schema_version=3`；
`session_epoch_frame_counts` 只能包含当前 `session_id:stream_epoch` 一个键，
其值等于归档帧数。会话、连接代次及手别必须与当前流一致。旧参考即使同为
20×20 也不接受，不裁剪旧尺寸、不为旧文件重写协议标识。
这些文件不是新电脑运行软件的必备文件，新会话可直接通过按钮采集不受压参考。

原始 ADC 默认色限为 **0..65535**。ΔADC 视图提供三种显示范围：

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
-p baseline_path:=/绝对路径/当前会话参考/baseline.npz
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
| `samples` | `vt_tactile_msgs/msg/TactileRow44Array` | 是，原始解码数值及故障事件 |
| `state` | `vt_tactile_msgs/msg/TactileGridState` | 否，20×20 派生滚动快照 |
| `heatmap` | `sensor_msgs/msg/Image`，可选 20×20 RGB 预览 | 否 |
| `diagnostics` | `diagnostic_msgs/msg/DiagnosticArray` | 否 |

`publish_heatmap:=true` 可启用通用图像预览；其颜色只用于显示，不可反推压力。
使用质量字段时以 `.msg` 定义为准：
[TactileRow44Array](../vt_tactile_msgs/msg/TactileRow44Array.msg)、
[TactileGridState](../vt_tactile_msgs/msg/TactileGridState.msg)。

`samples.schema_version=2`，每个 `blocks` 元素是一行：`row` 为 0–19，
`adc` 为依列号递增排列的 20 个 uint16 值；没有列组或状态字节。
`state.schema_version=3`、`rows=cols=20`、`layout_id=row20_col20_v3`。
`adc`、`valid_mask`、`updated_mask` 各有 400 项，行优先索引为 `row*20+col`；
`row_realtime_ns` 与 `row_monotonic_ns` 各有 20 项，索引就是行号。
状态发布默认 **30 Hz**，这是显示快照配置，不改变固件扫描速率。

### 订阅程序接口

仓库内的二维窗口、ROS→三维显示桥接、手套三维叠加、RGB 热图转换和离线
重建使用新版契约。独立订阅程序应校验 schema、布局及消息尺寸。可复用下述
校验入口；它只接受当前格式，不转换旧数据，也不修改传入消息：

```python
import numpy as np
from vt_tactile_capture.grid import normalize_state

def on_state(message):
    state = normalize_state(message)  # schema 3 / 20x20 / row20_col20_v3
    shape = (state.rows, state.cols)
    adc = np.asarray(state.adc, dtype=np.uint16).reshape(shape)
    valid = np.asarray(state.valid_mask, dtype=bool).reshape(shape)
    if not state.connected or not state.time_valid:
        valid = np.zeros(shape, dtype=bool)
    # ADC=0 可以是真实测量；使用 valid 判断有效性，不以非零值替代掩码。
    # 每行时效需结合 row_monotonic_ns、max_age_ns 和本机时钟检查。
    # 保留 header.stamp 与逐行时钟，不以订阅回调时间替换采集时间。
```

### 44 字节整行协议

本轮定义来自用户提供的 Python 解析示例
[`20by20.py`](https://tmp.link/f/6ae2b81fa2d53)，不是 IMU 厂商文档或 STM32
固件规范。下载原文件为 16,394 字节，SHA-256 为
`380f4dbe8a3addeea2027e8ff38f7a241ce7386b16306d7e2be818aedaa0de5d`。

```text
偏移 0..1    A5 5A
偏移 2..3    行号：uint16，大端，0..19
偏移 4..43   列 0..19：20 × uint16，大端 ADC
```

- 每包固定 44 字节；第 k 个 ADC 写入 `(row, k)`，k=0..19。
- ADC 数值范围为 0..65535。零是合法数据；缺失或过期由 mask 与时间表示。
- 参考程序设置串口为 115200，没有发送启动命令。USB CDC 的该设置不能据此
  推算实际传输速率或硬件扫描频率。
- 帧内没有帧尾、校验和、固件序号或硬件时间戳。软件只能校验结构和时序边界，
  无法发现所有仍具有合法结构的载荷损坏，也不能精确推断固件端全部丢样。

`Parser` 用于已知格式的纯增量解码，处理拆包、粘包及滑动重同步；实际串口
接收使用 `StreamParser`。它需要至少 **8 个连续完整行包、至少 2 个不同行号**
才能确认格式；锁定后，每包还需下一包的 4 字节帧头/行号作为边界前瞻。
一旦帧边界失配，立即解除锁定并重新确认，不把未知输入当作有效触觉。
这不要求凑齐 20 行才发布。最后一个尚无后继边界证据的包保留等待，断流重置时
丢弃，不能为了补齐计数直接判定有效。

确认与前瞻期间保留原始读取批次及时间。同一次读取的完整包中若有包尚待边界
确认，该读取中已确认的包也暂存，以免将一次读取拆成多个同时间戳批次。
包获准输出的时间可能晚于其读取时间，但记录的始终是该包最后字节所在
`read()` 的完成时间，不回填为确认时刻。串口类型探测与持续接收共用新版
帧结构校验；打开串口成功不等于协议已确认。

### 可选的电阻换算

`vt_tactile_capture.conversion.adc_to_resistance(adc)` 提供独立派生值，沿用
参考程序设置：ADC 满码 65535、参考电压 3.3 V、偏置 0.9 V、固定电阻 20 kΩ。

```text
V = ADC × 3.3 / 65535
R = 0.9 × 20000 / (V − 0.9) Ω
```

`V <= 0.9` 时返回 `None`，表示该公式无法计算正有限电阻；原始 ADC 仍有效，
不会因此被清零或作为坏包。上述常数是示例采用的电路假设，仍需与实物电路
核对；换算结果不是力或压力，未建立 N、Pa 或 kPa 标定。
示例的 `0..24000 Ω` 只是显示色阶，不是传感器量程，也不作为解析或有效性门槛。

### 时间与连续性

`samples` 的每个消息包含在同一次实际读取中完成、且通过流边界确认的行包。
同一消息中的包共享原始主机接收时间；包顺序保留，不宣称硬件同时采样。
`read_timeout_sec` 约束单次串口读取等待；协议确认、后继包前瞻和操作系统调度
还会影响数据可用延迟。时间戳在真实 `read()` 返回后立即取得，并随暂存数据保留，
不改写为协议确认、ROS 发布或界面刷新时刻。

- `header.stamp == host_realtime_ns`：主机读完成时刻，不是 ROS 发布时间。
- `host_monotonic_ns` 的域为 `CLOCK_MONOTONIC`；`timing_source=host_receive`。
- `session_id` 标识采集进程，`stream_epoch` 隔离断连重连。
- `sequence` 是软件批次序号，`first_packet_index` 是软件解码包序号，均不是固件序号。
- `producer_dropped_total` 统计有界队列丢弃的批次数，包括空状态事件。相关缺口
  可能重复反映同一丢失，不应把各种计数简单相加。
- 无数据/断连事件的 `blocks` 为空，不是全零触觉帧。
- `state` 保留最后一次有效行包的原始接收时间，`snapshot_monotonic_ns` 只用于
  计算年龄。`valid_mask` 表示当前可用，`updated_mask` 表示本次快照消费的新更新。
- `state.row_*` 各有 20 项，按行号索引；零表示该行尚未观测。一行的 20 个触点
  共享该行实际接收时间；真实包计数只增加 1，不拆成虚构的旧分组包。

同机跨模态使用 host realtime 对齐。相机 `CLOCK_MONOTONIC_RAW` 与手套/触觉
`CLOCK_MONOTONIC` 不能直接相减。USB 缓冲、固件扫描延迟仍存在，这不是硬件同步。
实时采集节点拒绝 `use_sim_time=true`；回放节点和 GUI 可以使用 `/clock`。

采集线程与发布/显示分离、队列有界。拥塞可能丢批次，但不会无限积压旧数据。
采集、ROS、录包和 GUI 的速度必须区分；30 Hz 画面不等于每触点 30 Hz 更新，
更不会为了凑 120 Hz 重复发送旧测量。

低频 `diagnostics` 提供 `batch_rate_hz`、`packet_rate_hz`（上个诊断窗口内的主机
发布批次/解码行包速率；批次包含空状态事件）、`valid_cell_count`（最大 400）、
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
使用 depth 256。队列容量以消息批次计数，能容纳的时间取决于实际批次率，
不是时延目标或零丢包保证。可通过相机采集配置的 `recording.qos_overrides` 按
明确 Topic 覆盖 depth / reliability；最终参数保存在会话的 `qos_overrides.yaml`。
不要使用 `-a` 把额外预览 Topic 随意录入严格
离线配置的 bag。传感器质量问题不会自动停止其他采集节点。

结束录制后可以生成[统一采集质量报告](../../../tools/multisensor_alignment/README.md#116-录制结束后的统一质量报告)，
汇总消息数、频率依据、时间间断及触觉覆盖情况，不改变原 bag，也不拦截下一次采集。

## 6. 回放与离线重建

档案及离线 `export-tactile` 同时识别旧 `TactileSampleArray` schema 1 与新
`TactileRow44Array` schema 2；来源类型、原始行/块时钟和包索引随导出保留。
某些历史原型曾把 44 字节结构写在旧 `TactileSampleArray` 类型名下：这种 schema 2
记录会在解码数组前明确拒绝，不能通过猜测 ADC 个数自动兼容；需有独立来源证据的迁移流程。
原 bag、历史录制计划和历史配置不改写。以下 ROS 重建节点面向新命名的 row44 类型。

主录制流是 `samples`，因此回放时需要一个**只订阅 samples、不打开串口**的
重建节点，把它转换为 `state` 后再交给同一热力图。不能把实时采集节点接到
录包上，或依靠重复发布保存的截图模拟回放。

先停止同一命名空间的实时触觉节点，防止 live 与 replay 混流。单独的回放终端：

```bash
ros2 launch vt_tactile_capture replay.launch.py \
  gui_python:="$VT_REPO/.venv-tactile/bin/python"
```

另一终端播放使用当前 schema 2 消息录制的 bag：

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
- `row_realtime_ns[20]`、`row_monotonic_ns[20]`；
- `row_message_sequence[20]`、`row_packet_index[20]`，追溯各行的源消息和真实行包。

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

软件测试应覆盖逐字节分片/粘包/错位重同步、连续帧确认与失锁恢复、整行地址映射、零 ADC、
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
不会关闭或重启用户现有窗口。`test_reference_parity.py` 按用户提供的新版字段定义
构造数据，核对解析数值与矩阵位置；不导入或执行下载示例的 GUI 和自动串口入口。
历史采样的离线结构检查与持续串口验收分别记录；离线字节文件没有完整接收时钟，
不能仅凭其包数宣称达到某个扫描频率。
