# FSGlove 解算结果接入 ROS 2

保留现有 **Qt 标定界面和三维手模型**，从同一次解算直接发布
`/gloves/right/state` 或 `/gloves/left/state`（`vt_glove_msgs/msg/GloveState`）。默认保持右手。不发布原始 IMU topic，
不重新打开串口，不运行第二套求解器，不改变原来的本地调试记录策略。

本包是 ROS 桥接层，不代表手套已完成精度、漂移或硬件量程验收。
默认只启动窗口，**连接、静止参考、姿态和功能标定仍由用户在原界面操作**。

新右手实物会话省略 `config` 时，由 GUI 统一选择当前近节＋末节布局及 STM32 104 Hz 输入配置。
需要显式文件时，使用主程序“配置设备”生成的完整输入配置；单独的布局模板保留历史量程。
桥接从配置的掌部骨段查找参考节点，
新布局发布 `reference_node_id=11`，辅助腕部 12 不会自动加入骨架。
已知右手部署的新会话启用版本化拇指复合模型；历史回放保留原配置与解算语义。
桥接不再另设掌心默认编号，而是消费生产端按实际配置发布的 `reference_node_id`，并校验
手别、节点范围、掌心源数据和时间戳的一致性。显式 `config` 则同时传给 GUI，并将桥接的
掌心编号固定为该配置的映射。不能从串口自动推断每颗 IMU 的佩戴位置。
ROS 中的 MANO 骨架顺序和 topic 名称不随物理编号改变；录制自带配置摘要、实测与推断掩码。

## 1. 环境准备

目标环境：Linux、ROS 2 Jazzy、已能正常启动的官方 FSGlove 12 路适配界面。
GUI 使用独立虚拟环境；ROS 进程使用 ROS Python。GUI 不依赖 `rclpy`，
桥接节点不依赖 PyTorch、Qt 或 MANO。先完成 [手套安装与标定说明](../../../tools/fsglove12/README.md)。
MANO 模型仍由用户按其许可自行准备，本包不复制或分发模型文件。

在仓库根目录编译：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws
colcon build --packages-up-to vt_glove_bridge
source install/setup.bash
```

在手套虚拟环境中安装当前源码（若之前已 editable 安装则无需重复）：

```bash
/absolute/path/to/glove-venv/bin/python -m pip install -e /absolute/path/to/VisionTactile-Dataset/tools/fsglove12
```

下面所有 `/absolute/path/...` 都应替换为本机真实绝对路径。
先验证隔离环境可导入 CLI，不连接任何设备：

```bash
/absolute/path/to/glove-venv/bin/python -I -m vt_fsglove12 upstream-live --help
```

## 2. 一条 ROS launch 同时打开原界面

在图形桌面的终端中，source 上面的 ROS workspace 后：

```bash
ros2 launch vt_glove_bridge glove.launch.py \
  glove_python:=/absolute/path/to/glove-venv/bin/python \
  fsglove_source:=/absolute/path/to/official-fsglove-checkout \
  mano_assets:=/absolute/path/to/mano-assets \
  output_root:=/absolute/path/to/glove-sessions \
  port:=/dev/serial/by-id/your-imu-device
```

界面外观、标定向导和三维可视化保留。`port` 仅预填，不会自动连接。
新版实物会话连接后，先将手套平放桌面，点击“静止准备”；通过后佩戴并完成三步标定。
准备失败会提示具体节点，不能沿用旧会话的滤波参考。操作顺序和质量边界见
[姿态标定窗口说明](../../../tools/fsglove12/README.md#姿态标定窗口)。
需要测试高频解算时，在 launch 命令末尾增加 `output_rate_hz:=120`；省略则沿用配置（默认 30 Hz）。
这是求解器目标，不是桥接节点重复旧姿态的发布定时器，也不保证当前机器已达标。
独立 GUI 命令对应 `--output-rate-hz 120`。详见[高频解算与离线性能基准](../../../tools/fsglove12/README.md#高频姿态解算120-hz-目标)。
按手势工具手册建立独立 Python 环境后，将 `glove_python` 指向该环境的
`bin/python`。开源仓库不包含预装虚拟环境；原环境和正在运行的窗口不会自动切换。
同一界面也包含[三姿态自动输入尺度检查](../../../tools/fsglove12/README.md#无尺子流程三姿态自动检查输入尺度)，
无需手部尺寸或精确转角。采用候选会正常断开、清除旧标定；需手动连接并重新标定，
不会用旧姿态填补这段中断。程序重启后要继续使用已审阅候选，可增加
`config:=/absolute/path/config.candidate.json`；桥接接口和 topic 名称不变。
在窗口中连接并完成标定后，使用以下命令检查发布结果：

```bash
ros2 topic info /gloves/right/state -v
ros2 topic hz /gloves/right/state
ros2 topic echo /gloves/right/state --once
```

频率是实际解算结果频率，不是 12 路 IMU 的原始接收频率。检查 `pose_valid`、
`time_valid`、`quality`、`reason`，不能仅以 topic 有消息判断姿态有效。
未标定状态可能发布无效状态；输入完全中断不会伪造新的手势或刷新输入时间。
启用功能铰链估计后，还须按下文解析 `quality_json` 中的逐关节质量。
暂估或局部不可用不会令串口断开，但也不会作为 `pose_valid=true` 的完整测量发布。
还应检查 `metadata_json.source_fresh`、`source_sample_indices` 与 `source_filter_epochs`：
新输入结果、同源标定重算、不可用状态不能混为“120 Hz 新鲜有效姿态”。

launch 通过虚拟环境 Python `-I` 隔离 ROS `PYTHONPATH`，保留 `DISPLAY`、
`XAUTHORITY`、`XDG_RUNTIME_DIR`、Wayland/Qt 桌面变量。没有桌面会话时不能显示 GUI；
不要用盲目设置 `DISPLAY=:0` 代替正确的桌面权限。

### 2.1 可选：在当前三维手上叠加触觉

按[三维触觉安装与操作步骤](../../../tools/fsglove12/README.md#在三维手模型上显示-ros-触觉)
给 GUI 环境补装本仓库的纯显示模块，并先启动触觉采集。上面的联合启动命令增加：

```bash
tactile_overlay:=true
```

可选参数 `tactile_topic:=/gloves/right/tactile/state`、
`tactile_socket:=/tmp/vt-tactile-UID/right.sock`（UID 替换为实际用户 ID）。
该功能启动 `vt_tactile_display_bridge`，只读已有状态 Topic，不打开触觉或 IMU 串口，
不运行第二套解算器。GUI 增加独立触觉参考、色限和叠加开关，完成参考后在掌侧着色。
该 socket 与 `socket_path` 指定的手势结果出口**不同**，方向也相反；不要填写相同路径。
`launch_gui:=false tactile_overlay:=true` 可只启动两个桥接，让已有新版 GUI 连接它们。
历史姿态回放不允许同时使用实时触觉叠加。

### 2.2 单手、双手和仅触觉：统一入口

`glove.launch.py` 仍启动一只手，增加 `hand:=left` 即选择左手；不提供 `hand` 时
从显式 `config` 读取，否则默认右手。手别与配置冲突会明确报错。左右手各使用自己的
12 路 IMU、标定、MANO 模型、窗口和结果桥，未合并为一套 24 路求解器。
节点默认位于 `/gloves/left`、`/gloves/right` namespace，旧右手 Topic 不变。

新入口 `hands.launch.py` 独立选择手势与触觉：

| `pose_hands` | `tactile_hands` | 工作模式 |
| --- | --- | --- |
| `right` | `right` | 原右手手势＋触觉 |
| `left` | `left` | 左手手势＋触觉 |
| `both` | `both` | 双手手势＋触觉 |
| `both` | `none` | 双手仅手势 |
| `none` | `both` | 双手仅触觉，无需 IMU、PyTorch、MANO |
| `right` | `left` | 右手手势、左手触觉 |

也支持 `none/left/right/both/left,right` 的其余组合。没有启用的模块不会启动，
不占串口、不等待设备；全部 `none` 时只打印提示，不启动采集。

准备工作：

1. 左手姿态需要可信的 `models/MANO_LEFT.pkl`，右手需要 `models/MANO_RIGHT.pkl`。
   本仓库不下载、镜像生成或分发模型。仅触觉不需要任意 MANO 文件。
2. 用设备固定路径区分左右 IMU 和触觉。每个串口只能有一个采集者；不要使用两手共享
   的 `/dev/ttyACM0` 猜测身份。没有唯一 by-id 时使用经确认的 by-path/udev 绑定。
3. 核对左手 `node_to_slot`、单位和安装方向。内置 `left_current.json` 是沿用现有编号的
   **未实测模板**，不是已经通过左手硬件验收的标定；必要时用 `left_pose_config:=...` 替换。
4. 若需修改更多参数，复制并编辑 [hands.example.yaml](config/hands.example.yaml)，
   使用 `config:=/绝对路径/hands.yaml`。文件内相对路径以 YAML 所在目录为基准；
   命令行路径以当前目录为基准。支持每手独立 GUI、触觉布局、基线及目标解算频率。
   自定义 `tactile_config` 若不是标准 `/**: ros__parameters:` YAML，必须在 hands 配置中
   显式设置该手的 `tactile_port`，以便启动前检查串口是否被重复分配。

双手完整启动示例（先按前文设置环境；模型和官方源码路径替换为本机真实路径）：

```bash
export VT_REPO="$HOME/Vision-Tactile-Dataset"
export FSGLOVE_SOURCE=/absolute/path/to/official-fsglove-checkout
export MANO_ASSETS=/absolute/path/to/mano-assets
ros2 launch vt_glove_bridge hands.launch.py \
  pose_hands:=both tactile_hands:=both \
  glove_python:="$VT_REPO/.venv-fsglove-120hz/bin/python" \
  fsglove_source:="$FSGLOVE_SOURCE" mano_assets:="$MANO_ASSETS" \
  tactile_gui_python:="$VT_REPO/.venv-tactile/bin/python" \
  left_imu_port:=/dev/serial/by-id/LEFT_IMU \
  right_imu_port:=/dev/serial/by-id/RIGHT_IMU \
  left_tactile_port:=/dev/serial/by-id/LEFT_TACTILE \
  right_tactile_port:=/dev/serial/by-id/RIGHT_TACTILE
```

IMU 端口仍只是窗口预填，需在对应窗口点击连接、分别标定。触觉端口明确指定后由其
采集节点打开；空端口保持未连接。不要在旧串口采集程序运行时重复启动。
两只手独立退出/断流不会触发关闭另一只手的动作；各自的无效状态如实报告。
默认 `launch_gui:=true` 时，每只手的标定窗口和姿态桥接作为一组管理：窗口关闭或
任一必要进程启动失败、退出，会同时结束本组的窗口、姿态桥接和可选触觉显示桥接，
避免无窗口的残留发布器阻挡下次打开。另一只手和独立触觉采集不受影响；单纯数据断流
不会自动关闭窗口。需要分别管理窗口和桥接时，使用下文的 `launch_gui:=false` 方式。

仅触觉示例，不需要填写官方源码或模型目录：

```bash
ros2 launch vt_glove_bridge hands.launch.py \
  pose_hands:=none tactile_hands:=both \
  tactile_gui_python:="$VT_REPO/.venv-tactile/bin/python" \
  left_tactile_port:=/dev/serial/by-id/LEFT_TACTILE \
  right_tactile_port:=/dev/serial/by-id/RIGHT_TACTILE
```

只用一只触觉时把 `both` 改为 `left` 或 `right`，未选设备参数可以省略。
仅手势时 `tactile_hands:=none`。关闭某手 GUI 在 YAML 设置 `pose_gui: false` /
`tactile_gui: false`；前者只启动姿态桥，**不会另起无界面的求解器**，需有独立 GUI/生产者连接该桥。

左手没有提供 `left_tactile_layout` 时默认显示 **20×20 原始矩阵**，不启动左手 3D 贴图。
左右手 ADC 地址到指腹/掌心的关系必须分别核对；左手不自动套用右手已观测锚点。
需显式选择左手 JSON 才开启左手手形/3D 贴图，例如传
`left_tactile_layout:=/absolute/path/left_layout.json`。随包的
`vt_tactile_capture/layouts/left_glove_unverified_candidate.json` 仅是零实测锚点的候选模板，
用于开发验证，不能当作已完成物理映射标定。镜像显示不改写原始 ADC。

### 2.3 双手数据与录包

| 数据 | 左手 | 右手 |
| --- | --- | --- |
| 解算结果 | `/gloves/left/state` | `/gloves/right/state` |
| 触觉采集批次 | `/gloves/left/tactile/samples` | `/gloves/right/tactile/samples` |
| 触觉滚动显示 | `/gloves/left/tactile/state` | `/gloves/right/tactile/state` |

每手仍独立 20×20/400 点，触觉显示默认 30 Hz。两手都启用并不保证每手姿态 120 Hz；
频率需分别实测，不重复旧姿态来补齐。默认 Unix socket 分别是
`/tmp/vt-glove-UID/{left,right}.sock` 与 `/tmp/vt-tactile-UID/{left,right}.sock`。
姿态标定/会话、触觉基线、时钟与丢包计数分别维护，不共享重置状态。

启动入口只采集/发布，**不自动录 bag**。纯手套模式不强制要求相机或 Tracker：

```bash
ros2 bag record -s mcap -o /absolute/path/new_hands_bag \
  /gloves/left/state /gloves/right/state \
  /gloves/left/tactile/samples /gloves/right/tactile/samples
```

按启用组合删除不需要的 Topic。触觉 `state` 是派生显示，默认录 `samples`；手势仍只录解算结果。
需要与三相机/三 Tracker 同 bag 时，使用
[双手录包与对齐 profile 生成器](../../../tools/multisensor_alignment/README.md#117-单双手独立选择与配套配置)。
不要用旧右手对齐 YAML 读取包含额外左手 Topic 的 bag。

双手是各自腕原点的局部姿态，不会自动测得两手空间距离；共同世界坐标仍需手腕 Tracker 与外参。
本轮验证包括双 bridge 的真实 DDS 回环、双手 MCAP 离线重建与模块隔离，不等同于左手硬件精度/漂移验收。

## 3. 分别启动桥接和界面

桥接节点可以单独重启，不要求关闭界面或清除标定：

```bash
ros2 launch vt_glove_bridge glove.launch.py launch_gui:=false
```

另一个终端启动带导出功能的 GUI（将 `1000` 换为当前 `id -u` 输出）：

```bash
/absolute/path/to/glove-venv/bin/python -I -m vt_fsglove12 upstream-live \
  --fsglove-source /absolute/path/to/official-fsglove-checkout \
  --mano-assets /absolute/path/to/mano-assets \
  --output-root /absolute/path/to/glove-sessions \
  --ros-socket /tmp/vt-glove-1000/right.sock
```

普通启动而没有 `--ros-socket` 的旧进程不会自动开始 ROS 导出。需要在下一次
用户安排的启动中带上该参数；不要为接入 ROS 擅自重启正在运行的标定/采集进程。

IPC 是本机私有 Unix datagram，目录必须归当前用户所有且权限为 `0700`，
socket 为 `0600`。默认 `/tmp/vt-glove-UID/right.sock`；可用 `socket_path:=...`
和 GUI 的 `--ros-socket ...` 同时指定另一个绝对路径。不要使用共享目录 `/tmp` 本身。
桥接拒绝抢占已有监听者；正常退出清理自己的 socket，崩溃后允许清理无监听的旧 socket。

非阻塞发送失败只增加 producer 丢包计数，不阻塞串口、求解和显示。
无桥接期间不保留无限 backlog；恢复后发送新结果，不补发过期姿态。

## 4. 消息、坐标与时间语义

| 内容 | 字段与约定 |
|---|---|
| 输入参考时间 | `header.stamp == host_realtime_ns`，实际 `reference_node_id` 掌部输入的主机接收时间；默认模板为节点 12 |
| 单调时间 | `host_monotonic_ns`，`clock_domain=CLOCK_MONOTONIC`，不是 `CLOCK_MONOTONIC_RAW` |
| 解算与桥接时刻 | `solve_completed_monotonic_ns`、`bridge_received_*`，不覆盖输入参考时间 |
| 多节点输入 | `source_node_ids/source_*` 并行数组，包含实际输入样本序号、接收时间和滤波 epoch |
| MANO 姿态 | `mano_pose_rotvec`，16×3 展平，弧度，包含 root；`shape_params` 为 10 个 beta |
| 手掌朝向 | `root_quat_xyzw`，已经包含在 MANO root 和关键点中，不要再次重复旋转 |
| 关键点 | `joint_positions_m` 可为空，否则 21×3 展平、米，与 GUI 使用同一模型几何 |
| 坐标系 | `glove_left_reference` / `glove_right_reference`：腕原点、标定参考轴，不是世界坐标，没有绝对平移 |
| 数据质量 | `pose_valid/time_valid`、观测/推算 mask、`source_labels`、`quality_json` |
| 追溯信息 | session、标定版本、配置/模型 hash、算法和 `metadata_json`；不包含 MANO 模型资产 |
| 丢样信息 | `producer_dropped_total`、`bridge_sequence_gaps_total`、`bridge_rejected_total` |

`segment_observed_mask` 只表示有传感器观测支持，不等同于关节角独立真值。
缺测骨段的先验或推算结果会保留，但带推断标识；具体缺测位置由录制时的配置决定。当前 `units_verified` 和
`sample_rate_verified` 按实际配置传递，不能因接入 ROS 自动变为已验证。
不宣称绝对航向漂移已修正或解剖学精度已验收。

### 逐关节估计有效性与部分姿态

统一功能层让四指 PIP 和拇指 IP 共用一条连续估计路径；MCP 保留侧摆/弯曲两个自由度。
这些功能标定要求父子骨段均配置 IMU；近节＋末节布局不支持四指 PIP 功能标定，不能借助该路径独立恢复 PIP/DIP。
旧窗口预校正与小指独立校正不再串联执行。拒绝一次航向更新不会切换成未校正角度。
缺测 DIP 的局部有效性依赖其 PIP，仍是推断。ROS `.msg` 和 `schema_version=1` 保持不变，
当前契约通过 `quality_json.joint_quality_revision=local_joint_geometry_v2` 显式启用，
使用 **MANO16 槽位顺序**，不是 21 个关键点的顺序：

| `joint_quality[slot]` | `joint_valid_mask[slot]` | `joint_display_available_mask[slot]` | 消费方式 |
|---|---|---|---|
| `valid` | `true` | `true` | 当前估计通过内部条件，仍不代表解剖学精度认证或独立观测 |
| `valid_hidden` | `true` | `false` | 局部估计通过自身检查，但父链几何不可用；不可作为完整骨链或空间位置使用，仅 v2 支持 |
| `degraded` | `false` | `true` | 当前暂估仅供明确标记的显示/诊断，不能作为有效角度测量 |
| `unavailable` | `false` | `false` | 无可信当前角度；数组中的有限值可能是网格占位，不可解释为伸直或末次有效测量 |

`joint_estimation` 以槽位字符串（如 `"8"`）索引本帧原因、偏轴残差、航向参考等诊断。
三个 mask/quality 字段必须同时出现且各有 16 项，桥接验证类型、状态一致性及 v2 的父链显示依赖。
`metadata_json.kinematic_rejections` 记录局部模型、缺测或推断依赖的拒绝；
`quality_json.joint_display_rejections` 单独记录父链不可显示原因。父关节模型拒绝不自动否定
由可信骨段输入计算的子关节，实际缺测与模型门槛仍有效。
整手有效仍要求全部局部估计有效、全部几何可用，并通过原有输入与时序检查。
任一槽位不是 `valid`（包括 `valid_hidden`）时，出口立即设 `pose_valid=false`、
`reason=joint_estimation_degraded` 或 `joint_estimation_unavailable`；桥接拒绝把这种包标为有效。
`time_valid` 仍独立表达源时间是否可靠。缺少 `joint_quality_revision` 的历史结果保留 v1 三态
语义，不自动升级；显式未知版本（含空值）被拒绝。历史算法允许省略质量字段的规则不变，
当前算法仍要求完整字段。局部角度有效或可以显示不代表独立实测、整手准确或漂移已解决。

只要完整有限表示和源时间仍可用，**`pose_valid=false` 仍可能保留非空的 MANO、root、shape 和关键点数组**，
以及原观测/推断来源 mask。这样了解新合同的消费者可以局部使用未受影响部分；不检查新字段的旧消费者
必须按 `pose_valid=false` 拒绝整帧，不能只检查数组长度或 `solver_success`（后者不是 ROS 字段）。
`metadata_json.joint_validity_contract` 明确记录该语义。关节空间位置还受父链影响；消费 21 点时须按
MANO 槽位与其父链映射质量，不能把 16 项 mask 直接当 21 点索引。

GUI 的局部失效不清空整手；未过期历史可明确标记为保持，没有可用当前值或历史的区域及其后继隐藏。
`valid_hidden` 的局部角度可在显示父链仍可用时参与展示组合，但不能延长祖先历史寿命，
也不把原测量改为有效。根部失效仍受整手显示时限约束；暂估、历史保持和源时间分别标明。
输入监视采用独立快照，不等待解算；约 200 ms 稳定恢复仅用于显示状态，日志/ROS 有效性没有该延迟。
展示缓存 `display_pose/display_geometry` 不作为测量发布；局部模型使用历史显示值时，
`joint_positions_m=[]`，其他结果仍来自原始解算及其质量 mask，不把历史几何塞入 ROS。
`display_only` 数据误传给结果出口时会被拒绝。稳定画面不能视为新的有效姿态或准确度验收。
若要与官方未处理输出比较，由用户明确关闭功能轴开关；这是主动对照操作，不是坏数据时自动回退。
原始记录、标定连接和界面布局不因关节降级而改变。

### 输入完整性故障

新版右手配置及本机启动配置启用 `input_health`；历史配置缺少该字段时保持旧行为，
已运行窗口不热切换。初始约 2 秒确认接收节奏，持续速率失配或有另一 bank 活动佐证的完整
bank 重复数据会触发保护；详见[接收完整性保护](../../../tools/fsglove12/README.md#高频稳定性接收完整性保护与完整标定窗口)。

受影响输入暂停 VQF 积分，但本地原始记录继续；GUI 提示“正在确认接收节奏”或
“冻结／接收速率异常／恢复中”，保持显示的手形不是新测量。ROS 此时为
`pose_valid=false`、`reason=input_health_invalid`、`metadata_json.source_fresh=false`。
`quality_json.input_health` 提供节点状态、原因和接收率摘要，**不导出原始六轴值、payload 或 status 字节**。
源时间仍保留其原含义，不能用新到达消息数证明传感器数据正在更新。

确认故障使旧标定 epoch 失效；清除标定不能解除故障。先恢复设备输入，待持续健康的新数据通过
恢复检查后重新标定，不要反复重标定冻结流。软件不会发送设备命令、以主机包间隔替换积分周期，
也不能补回丢失运动或保证绝对航向无漂移。120 Hz 目标仍需在真实输入、GUI、ROS 和录包全链路实测。

源时间缺失时为 `time_valid=false`，不回填当前 ROS 时间。过期输入会使
`pose_valid=false, reason=bridge_input_stale`，原时间仍保留。
`max_input_age_s` 默认 `0.15` 秒，可按实测计算延迟调整；调大它不会减少真实时延。

这实现的是**同机主机接收时间对齐**，不是硬件同步。USB/STM32 缓冲和节点间采样
差异仍存在。与相机/Tracker 联合采集时应在同一主机时钟域；跨主机另需时钟同步。
若要世界坐标手模型，还需 Tracker 与手腕/手掌安装外参，不能把局部关键点当作世界位置。

## 5. 录制和离线对齐

最小单手录制示例：

```bash
ros2 bag record -s mcap /gloves/right/state
```

在现有多模态 Recorder 中，把 `/gloves/right/state` 作为 additional stream
加入与相机、Tracker 相同的一次 bag。仅加入这个解算结果 topic，无需 IMU raw topic。
详见 [离线对齐工具](../../../tools/multisensor_alignment/README.md)。

消息 QoS 为 reliable、volatile、depth 30，与 best-effort Recorder 订阅兼容；
不设置 transient-local，不把旧结果当作刚产生的新姿态。
桥接与本机 IPC 不承担无限缓存；录制后检查序号缺口、producer 丢包、有效状态比例和时间年龄。
丢包总数是分环节诊断量，不能简单相加（同一次丢样可能同时形成序号缺口）。

原始数据未进入 bag 时，bag 不能用于重跑 VQF 或重新标定。需要此类排查时仍可使用
现有本地调试记录，与 ROS bag 策略互不影响。

## 6. 可配置项与故障排查

- 工作台无法打开标定窗口时，先查看设备进程组的具体原因。外部发布程序可能在窗口关闭后
  仍运行，需确认并退出原程序；串口不存在时，在“设备 → 配置设备”选择实际手套端口后
  保存并应用。不要把触觉串口指定为姿态手套串口，也不要删除锁文件来绕过正在运行的程序。
- `launch_gui:=false`：只启动 ROS 桥接；GUI 不受桥接重启影响。
- `config:=/absolute/config.json`：选择手套配置，并固定桥接期望的掌心编号。省略时由
  GUI/生产端选择配置，桥接按经过校验的数据包读取掌心编号；`launch_gui:=false` 和直接运行
  桥接节点也使用这一规则。直接运行节点时可用 `reference_node_id` 参数指定期望编号，
  默认 `0` 表示读取数据包中的编号，`0` 本身不是有效 IMU 编号。
- `topic:=/gloves/right/state`：修改发布名，但同步更新录制/对齐配置。
- `allow_replay:=true replay_raw:=/absolute/raw.csv`：仅用于明确的离线测试，消息带 `source_mode=replay`。
  历史单调时钟不能当当前实时源使用，默认拒绝 replay；不要混入正式 live bag。
- `use_sim_time` 不支持：此桥接发布真实主机输入时间，不把 `/clock` 当硬件时间。

没有消息：先看 GUI 是否连接、是否产生解算、是否使用同一 `--ros-socket` 路径。
窗口不显示：确认正在桌面会话中运行，以及 venv 的 Qt/OpenGL/MANO 依赖已安装。
socket 被占用：保留原桥接或改用新的私有路径，不要删除正在使用的 socket。
持续 stale：看源时间、解算耗时、桥接收到时间和系统负载，不要通过重复发送旧姿态隐藏问题。

## 7. 开发验证

```bash
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest ros2_ws/src/vt_glove_bridge/test ros2_ws/src/vt_glove_msgs/test -q
```

测试覆盖时间戳保留、非法/过期输入、序号缺口、无效姿态、私有 socket、桥接重启、
GUI 独立解释器和不自动连接，以及逐关节字段一致性、无效完整数组保留和矛盾有效标志拒绝。
测试不打开真实串口，不代表联合硬件或物理精度验收。

本机现有手套隔离环境还可同时验证 GUI/export/bridge，不启动现场进程（从仓库根目录执行）：

```bash
env QT_QPA_PLATFORM=offscreen PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  OMP_NUM_THREADS=1 OPENBLAS_NUM_THREADS=1 MKL_NUM_THREADS=1 \
  artifacts/glove_hand_pose_live/upstream_baselines_20260930/fsglove12-venv/bin/python -I -m pytest \
  tools/fsglove12/tests/test_live_display.py tools/fsglove12/tests/test_live_ui.py \
  tools/fsglove12/tests/test_pose_export.py ros2_ws/src/vt_glove_bridge/test/test_protocol.py -q
```

其他机器改用安装相同源码和可选依赖的隔离 Python；offscreen 检查不是硬件动作或 GPU 帧率验收。

编译并 source workspace 后，可显式打开真实 ROS/DDS 合成消息测试；默认使用独立
domain 147，不连接串口、不启动手套窗口：

```bash
VT_GLOVE_ROS_INTEGRATION=1 PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 python3 -m pytest \
  ros2_ws/src/vt_glove_bridge/test/test_ros_integration.py -q
```
