# VIVE Hub → ROS 2 → VisionTactile

适用于 Linux VIVE Hub **1.0.3 随包 `vut::Client` SDK**。Hub 负责配对、地图和 Dongle；本桥只订阅本机 socket，不打开 HID、不初始化或复位设备。旧 PyVUT 入口保留，不能与 Hub 同时操作 Dongle。

官方软件：[VIVE Hub for Linux](https://github.com/ViveRoboticsSoftware/vivehub-linux)。其他独立 SDK 仓库的 API/坐标约定可能不同，不可直接混用。

## 安装与运行

1. 启动官方 Hub，完成连接/建图，确认三只为 Ready。
2. 使用随包 SDK 编译读取器（不需要改动或安装 Hub 到系统）：

   ```bash
   bash tools/vivehub_bridge/build.sh /absolute/VIVEHub-Linux/sdk /absolute/runtime/vt-vivehub-reader
   ```

3. 复制 `ros2_ws/src/vt_vive_tracker/config/hub_bindings.example.yaml` 到个人配置目录。根据**实物安装位置**填写 torso（头部）、left_wrist、right_wrist 的序列号，执行 `chmod 600 /absolute/hub-bindings.yaml`。不能使用 Hub 显示编号或临时 slot 代替序列号。
4. 从 `/opt/ros/jazzy/setup.bash` 构建本工作区，并 source 本工作区 `ros2_ws/install/setup.bash`。命令行启动：

   ```bash
   ros2 launch vt_vive_tracker triple_tracker.launch.py \
     backend:=vivehub frame_id:=vive_hub_map \
     hub_bindings_path:=/absolute/hub-bindings.yaml \
     hub_reader_path:=/absolute/runtime/vt-vivehub-reader \
     hub_socket_path:=/tmp/vut.sock
   ```

   工作台方式：设备 → 配置设备 → 追踪器，位姿来源选择 **VIVE Hub · 官方 SDK**，填写上述三条路径（切换来源时自动设置 `frame_id=vive_hub_map`），保存并应用后，在设备页点击追踪器“启动”。不再运行旧初始化向导。空间页的 `scene_config_path` 使用 `ros2_ws/src/vt_capture_manager/config/scene.vivehub.yaml`；这份配置只显示 Tracker，不自动应用旧外参。

5. 预览页查看三只 Tracker；空间页查看共同坐标。退出工作台窗口不关闭 Hub；设备页“停止”只停止 ROS 桥。

### 打开工作台自动连接

在正式桌面模式的 **设备 → VIVE Hub 连接** 中，选择官方 Hub 安装目录
（含 `run-vivehub.sh`、`vivehub`、`tracker_server`），勾选“打开工作台时，自动启动
VIVE Hub 并接入 Tracker 位姿”并保存。设备配置仍须启用 VIVE Hub 来源及正确的角色绑定、
SDK 读取器和 Socket。可点击“立即连接”在当前窗口执行一次。

下次打开工作台时，先复用或启动官方 Hub，等待实际 Socket 服务就绪，再通过原设备管理器
启动位姿桥；已有发布者或运行中的桥会复用，不重复启动。默认未勾选，演示与只读模式不执行。
设置单独保存到 `~/.config/visiontactile/workbench/vivehub-startup.json`，修改这项偏好无需
重启相机或后台。启动结果和日志保存在同目录的 `vivehub-startup-status.json` 与
`vivehub-startup.log`，异常会显示在设备页及连接窗口。

实体 Tracker 仍需开机并完成定位。自动连接不执行配对、建图、标定、相机启动或录制。
关闭窗口保留 Hub 和位姿桥；设备页停止桥后，本次窗口不会自动重新拉起它。失败时不循环
启动，排除原因后点击“立即连接”。旧 PyVUT 占用设备时，先停止旧读取器。

## Topic 与语义

每个角色继续使用 `/vive/{role}/sample`、`/pose`、`/status`，兼容工作台与后续录包。`/pose` 仅发布有效 6DOF；`/sample.pose_valid` 为 false 时不能用于标定。`/vive/source_info`（String，reliable/transient-local）公布来源、时间、身份和不可用字段，录包时建议同时保存。

- **坐标**：官方文档的右手 OpenVR 坐标、米、四元数 **xyzw**，不再走 PyVUT 的 wzyx 反序。不校正朝向、不旋转地图；独立 frame `vive_hub_map`。未经实测不能把旧 `vive_map` / `vive_map_v2` 外参直接套上。
- **时间**：`host_monotonic_ns` 保存 SDK daemon 的 `CLOCK_MONOTONIC` 位姿时间，非设备曝光时间、非 `MONOTONIC_RAW`。ROS header 与 `host_realtime_ns` 使用本次桥进程内三角色共用、固定的 REALTIME−MONOTONIC 偏差换算（`anchored_linux_realtime_monotonic_v2`）。不逐条重估偏差、不排序或加一纳秒，不把旧消息盖成当前时间。读取器同时报告配对采样跨度，超过100微秒的样本不用于建立或检查偏差且不发布。后续可靠配对只监测偏差；偏离锚点超过5毫秒则锁存报错并停止输出位姿，必须明确重启软件桥，不自动重新锚定。标定时钟门槛保持不变。
- **身份**：SHA256(`vivehub_serial_v1:` + 序列号)，独立于旧 RF 地址哈希。序列号未知/不在绑定表时不猜角色。
- **Ready 兼容**：文档枚举是 2；本机 1.0.3 实测三只 Hub Ready 时 SDK 返回原始状态 **18 (`0x12`)**，保留原值，明确接受 2 和 18。其他状态不当作有效跟踪；不是任意状态都按低四位放行。
- **兼容消息中的空字段**：SDK 不提供加速度/按钮，填 0 仅为消息兼容，**不得当成传感器测量**。`packet_index` 是每角色桥接计数器，不是原无线包序号；角速度按 SDK 转发（SDK 的零也可能表示不可用）。源描述明确记录这些区别。
- **断流**：SDK 自动重连，旧缓存不回放；时间超过 500 ms、非单调、非有限坐标、异常四元数不发布。设备状态默认 1 s 无消息转为断开。SDK 回调只入有界队列，不被 ROS/GUI 阻塞。

更新此时间策略时必须同时构建 Python 桥和 C++ 读取器；旧读取器没有
`receive_clock_span_ns`，新桥会明确拒绝，不假定其采样精度。Linux 的两个时钟
均受频率校正影响，而 REALTIME 可发生不连续调整，见
[Linux clock_gettime 文档](https://man7.org/linux/man-pages/man2/clock_gettime.2.html)。
同机固定偏差加持续监测适用于这里的 Hub daemon，不可直接推广到跨主机或设备时钟。

## 更新与时间戳验收

先完成或取消正在进行的标定/录制，再停止并重新启动 **ROS 位姿桥**；不必退出
VIVE Hub、重置设备、重新配对或建图。只修改磁盘文件不会更新已运行的进程。
编译新读取器时使用临时输出文件，停止旧桥后再替换，避免直接覆盖正在执行的文件。

在已经 source 本工作区的 ROS 环境中执行只读验收：

```bash
python3 tools/vivehub_bridge/validate_live.py \
  --bindings /absolute/hub-bindings.yaml \
  --seconds 60 --output /absolute/live-clock-validation.json
```

验收要求三路都有有效位姿、每路仅一个发布者、身份/坐标和 header 一致，
**两个主机时间戳均递增**，换算前后的间隔完全一致，三角色共用固定偏差，
且 `/vive/source_info` 声明新版时间策略。报告只保存统计，不录制传感器原始数据。
这验证的是读取和时钟换算，不代表硬件同步精度或外参几何精度通过。

## 限制与授权

此接入只解决读取、身份和工作台展示，不等于三套设备的相机外参或共图精度已验收。新地图、接口坐标变化均需单独验证。

本仓库只包含自行实现的适配源码，不包含 HTC SDK、私有角色序列号或编译后二进制。SDK 与链接产物的再分发须遵守 HTC 授权。
