# Tracker 与 RealSense 离线外参标定

本工具用于求解刚性固定的 VIVE Ultimate Tracker 与 RealSense 彩色光学坐标系
之间的固定外参。工具位于 `tools/tracker_camera_calibration/`，与
`ros2_ws/src/` 下的在线采集和 Tracker Publisher 完全分离。

当相机和 Tracker ROS Topic 已经稳定发布后，完整安装、三组身份配置、输入门禁、
标定板生成、专用 bag 录制、单次求解、三次重复性比较、故障恢复和最终验收遵循
[一本式产品操作手册](../tools/tracker_camera_calibration/README.md)。其中
`configure` 生成绑定参考相机序列号与物理 Tracker role 的配置，`calibrate`
生成不可覆盖的单次结果，`compare` 对至少三次有效结果执行身份一致性和
`5 mm / 0.5 deg` 重复性门禁。

## 软件边界

```text
在线相机与只读 Tracker Publisher
              |
              | 手动录制专用 calibration bag
              v
tools/tracker_camera_calibration（纯离线）
              |
              +-- extrinsics.yaml
              +-- report.json
              +-- residuals.csv
              +-- diagnostics.svg
```

离线工具不会打开 Dongle、执行 Tracker 初始化、配对或建图，也不会修改正式统一
Recorder。标定数据必须继续单独录制；正式实验的相机与 Tracker 数据进入一个
统一 bag，并由[离线对齐工具](../tools/multisensor_alignment/README.md)消费。
Tracker 私有启动属于在线输入系统，不复制到离线工具手册中。

## 坐标与时间语义

输出为 `^tracker T_camera`，即 ROS 中 Tracker 是 parent、RealSense color
optical frame 是 child。程序使用 `CameraFrameTiming.shared_ros_timestamp_ns`
将图像精确关联到 `group_host_realtime_ns`，再在同一 host realtime 时间域内插值
`TrackerSample.host_realtime_ns`。相机设备时间、D436 可能重置的硬件时间和
`CLOCK_MONOTONIC_RAW` 不会与 wall-clock 时间直接混用。

标定工具通过静止窗口降低 callback 延迟和曝光时刻差异的影响，不同时估计时间
偏移与空间外参。正式数据的离线动态匹配由独立对齐工具完成，仍不代表硬件同时
曝光。

## 真机状态

当前已经完成软件实现与已知真值合成测试，但尚未生成三组真实硬件外参。真实
结果只有在以下条件全部满足后才能标为产品有效：

- Tracker 与相机使用最终刚性支架，标定后不再拆装；
- 三组物理对应关系已确认；
- 打印后的 ChArUco 方格和 Marker 尺寸已经实测；
- 每组至少获得 40 个被程序接受的静止、多轴、多距离姿态；
- 独立留出验证不超过 10 mm 与 1 度；
- 三次独立重复标定的外参差异不超过 5 mm 与 0.5 度。

实际外参、bag、现场图像、角色映射和 Tracker 私有信息只能保存在仓库外。公开
仓库提交配置模板、算法、测试和说明，不提交设备专属标定产物。
