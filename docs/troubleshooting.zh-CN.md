# 故障排查

[English](troubleshooting.md) · [新电脑部署指南](deployment.zh-CN.md)

在未激活 Conda 的终端中，先加载 ROS 2 Jazzy，再加载启动系统时使用的同一工作空间安装。
诊断帮助操作者判断系统和数据情况，不会给 Recorder 生命周期增加质量准入条件。

## 按现象排查

| 现象 | 检查内容 | 判断与处理 |
| --- | --- | --- |
| `ros2 topic list` 看不到相机 Topic | 确认三个 RealSense 进程、`timing_normalizer` 和 `capture_controller` 正在运行。检查 `ROS_DOMAIN_ID`、`ros2 node list`、USB 设备枚举和启动日志。 | 先加载 ROS 2，再加载工作空间覆盖环境；各终端使用相同域 ID。必要时重启发现守护进程，确认三台相机可见后重新启动。 |
| RViz 不显示彩色图像 | 为一个确切的 `*/color/image_raw` Topic 添加 RViz **Image** 显示，检查 `ros2 topic info -v`。受管 RealSense 图像使用 **Reliable** 可靠性和 **Volatile** 持久性；外部发布端按其实际发布的 QoS 配置。 | 受管彩色和深度发布端明确使用 reliable QoS。仅提供 best-effort 的外部发布端无法服务 reliable 订阅端。Image 显示不需要 TF 固定坐标系。正式录制时不要同时运行三个高分辨率 RViz Image 显示，额外订阅和渲染负载会影响观察结果。 |
| 出现 `Incomplete timing group ... missing color observation` 或 `Incomplete timing group ... missing depth observation` | 对比彩色/深度元数据频率、每台相机的 `/frame_timing` 频率、端点 QoS、启动日志和 `lsusb -t`。 | 一组彩色/深度元数据在 150,000,000 ns 或四个更新的时间戳以内未能配对完成，该时间组被记录到日志并省略。单次或持续出现的提示都属于 Recorder 诊断信息，不会使录制失败或阻止 `COMPLETE`。排查数据源、QoS、驱动和 USB 负载，再单独判断数据是否适用。 |
| D436 时钟重置或时间戳不连续 | 检查 D436 驱动日志、重连历史、内核 USB 消息及相邻 `CameraFrameTiming` 的设备时间戳。 | D436 时钟重置属于诊断信息，不会使录制流程失败。但它可能导致相关时间段不适合特定分析，因此应保留日志上下文并独立检查该区间。 |
| Recorder 一直处于 `FINALIZING` | 检查 `recorder.log`、进程状态、文件系统错误和剩余空间。 | 控制器尚未确认 Recorder 已退出。不要直接终止启动终端；先等待其停止及强制终止升级流程完成，再排查文件系统或进程问题。 |
| 状态为 `COMPLETE`，但数据似乎不完整 | 运行 `ros2 bag info`，检查 Topic 消息数和时长，用常规 rosbag 工具抽查消息。 | `COMPLETE` 只表示 Recorder 进程生命周期结束，不能证明帧覆盖完整、时间质量合格或满足实验要求。 |
| Tracker 设备绿灯，但界面卡片全红 | 确认 Publisher 在运行、所有终端的 `ROS_DOMAIN_ID` 一致、`/vive/*/status` 存在。检查是否有初始化程序、验证器或第二个 Publisher 占用 Dongle。 | 硬件绿灯表示无线连接或映射就绪，不表示当前 ROS 进程正在接收报告。保持正常硬件通电，先修正进程、域 ID 或私有输入文件的问题。 |
| Host 正常发布，但某个 Client 没有数据 | 查看三路 `/vive/<role>/status`，检查 Tracker ID 和计数器，再运行 `vt-vive-validate-topics`。 | Publisher 按角色分别降级，健康角色应继续发布。缺失角色恢复前，三角色验收仍然不能通过。不要自动重写角色映射。 |
| 启动程序后 Tracker 才开始闪蓝灯 | 停止所有占用 Dongle 的程序，确认经批准的手动初始化进程已经退出，再启动只读 Publisher。 | 多个 HID 使用者争用，或未经批准的初始化流程，都可能干扰当前会话。ROS 可视化器只是只读数据消费者，不占用 Dongle。 |
| Topic 频率正常，但 GUI 动作明显卡顿 | 对比各 GUI 卡片接收频率与 `/vive/<role>/pose` 频率，注意它们不同于顶部显示的渲染 FPS。 | 仅显示异常时只重启 GUI，不要重置或重新通断一个正常工作的三角色 Publisher 会话。 |

## 节点、进程与发现诊断命令

```bash
test -z "${CONDA_PREFIX:-}"
type -a ros2
printenv ROS_DISTRO ROS_DOMAIN_ID AMENT_PREFIX_PATH
pgrep -a -f 'realsense2_camera|timing_normalizer|capture_controller|ros2 bag'
ros2 daemon stop
ros2 daemon start
ros2 node list
ros2 topic list
ros2 service list -t
find -L "${ROS_LOG_DIR:-$HOME/.ros/log}/latest" \
  -maxdepth 2 -type f -print
lsusb -t
rs-enumerate-devices -s
journalctl -k -b --no-pager | tail -n 200
```

参与同一次运行的终端必须使用相同的 `ROS_DOMAIN_ID`，
并先加载 `/opt/ros/jazzy/setup.bash`，再加载所选工作空间安装。

## 图像与时间信息 Topic 诊断命令

受管图像诊断使用 reliable、volatile QoS；
时间信息诊断使用 sensor-data QoS：

```bash
ros2 topic info -v /d405_1/color/image_raw
timeout 10 ros2 topic hz /d405_1/color/image_raw
ros2 topic echo --once /d405_1/color/image_raw sensor_msgs/msg/Image \
  --qos-reliability reliable --qos-durability volatile --no-arr

ros2 topic info -v /d405_1/frame_timing
timeout 10 ros2 topic hz /d405_1/frame_timing
ros2 topic echo --once /d405_1/frame_timing \
  vt_camera_msgs/msg/CameraFrameTiming --qos-profile sensor_data

ros2 topic info -v /d405_1/color/metadata
timeout 10 ros2 topic hz /d405_1/color/metadata
ros2 topic info -v /d405_1/depth/metadata
timeout 10 ros2 topic hz /d405_1/depth/metadata
```

对 `d405_2` 和 `d436` 重复这些检查。受管图像发布端采用 RealSense wrapper 的
`DEFAULT` QoS：reliable、volatile、keep-last 深度 10。
Recorder 的受管图像订阅请求 reliable、volatile、keep-last 深度 30。
时间信息发布端继续采用 best-effort、volatile 的 sensor-data QoS。
时间信息频率较低表示部分彩色/深度元数据未形成完整配对；Recorder 独立继续运行。
`ros2 topic hz` 测量的是它自身订阅收到的频率，不是实际写入 bag 的消息数。
受管传输与录制配置详见[采集指南](capture-guide.zh-CN.md#相机图像传输)。

## USB 身份与图像配置诊断命令

```bash
lsusb -t
rs-enumerate-devices -s
ros2 service list -t
for camera in d405_1 d405_2 d436
do
  ros2 service call "/$camera/device_info" \
    realsense2_camera_msgs/srv/DeviceInfo '{}'
  ros2 param get "/$camera" depth_module.depth_profile
  ros2 param get "/$camera" depth_module.depth_format
done
ros2 param get /d405_1 depth_module.color_profile
ros2 param get /d405_1 depth_module.color_format
ros2 param get /d405_2 depth_module.color_profile
ros2 param get /d405_2 depth_module.color_format
ros2 param get /d436 rgb_camera.color_profile
ros2 param get /d436 rgb_camera.color_format
journalctl -k -b --no-pager | tail -n 200
```

通过 `lsusb -t` 确认链路为 `5000M`，并将 D436 放到独立根总线。
USB 路径只作为当前的诊断依据；启动实例通过配置中的序列号绑定相机。

发生访问权限错误或 RSUSB 缓冲区耗尽时，在仓库根目录执行只读检查：

```bash
python3 tools/capture_deploy/configure_realsense_usb.py check
```

任何修正都按 [USB 检查、应用与回滚指南](../tools/capture_deploy/REALSENSE_USB.md)
操作。应用修改前先停止相机进程；权限修改不能释放其他进程占用的接口。
缓冲区修改仅在本次开机期间有效，RSUSB 缓冲区要求不能用于判断 V4L2 丢帧原因。
如果缺少软件包，或 wrapper 版本不是 4.58.1，应先按[部署指南](deployment.zh-CN.md)
修复环境，再调整设备设置。

## 试次数据检查

```bash
export VT_DATA_ROOT="${HOME}/visiontactile-data"
read -r -p "Session ID to inspect: " SESSION_ID
sed -n '1,240p' "${VT_DATA_ROOT}/${SESSION_ID}/recorder.log"
sed -n '1,240p' "${VT_DATA_ROOT}/${SESSION_ID}/bag/metadata.yaml"
ros2 bag info "${VT_DATA_ROOT}/${SESSION_ID}/bag"
```

`ros2 bag info` 是操作者可以按需执行的检查，其结果不会改变 `COMPLETE` 状态。
生命周期状态与数据质量评估应分别判断。
