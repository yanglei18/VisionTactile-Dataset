# Tracker 与 RealSense 外参标定

本工具用于求解刚性固定的 VIVE Ultimate Tracker 与 RealSense 彩色光学坐标系
之间的固定外参。求解工具位于 `tools/tracker_camera_calibration/`，保留独立离线
CLI，并由主程序的“标定与离线工具 → 相机–Tracker 标定”复用。
引导模式固定板、移动刚性整体，自动录制专用 bag 并在关闭后使用同一求解器。
GUI、求解 worker、配置应用和 ROS 发布各有独立职责。

新会话默认使用参考板 `CC200-15-11.25` 的 12 × 9 方格、15 mm 方格边长、
11.25 mm 标记黑色外边长和兼容参考图的 `DICT_5X5_1000`。有效图案为
180 × 135 mm，商品标称的 200 × 150 mm 外板不参与求解尺度。商品标称或打印
尺寸须用实物测量核对；导入已有配置保留原有实测参数，不迁移历史结果。

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
              | 引导自动录制，或使用已有专用 calibration bag
              v
tools/tracker_camera_calibration（纯离线）
              |
              +-- extrinsics.yaml
              +-- report.json
              +-- residuals.csv
              +-- diagnostics.svg + independent-validation.json
              +-- 三轮重复性通过 -> 后台活动版本 -> 标定 Topic
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

引导流程的软件验收包括已知真值合成数据、真实 ROS 消息和 MCAP 文件；新增界面的
现场操作、真实安装精度和长时间运行仍需在最终设备上验收。已有离线工具的实际
验证与新引导流程的现场验收应分别记录。设备专属外参保存在仓库外。真实结果只有
在以下条件全部满足后才能标为产品有效：

- Tracker 与相机使用最终刚性支架，标定后不再拆装；
- 三组物理对应关系已确认；
- 打印后的 ChArUco 方格和 Marker 尺寸已经实测；
- 每组至少获得 40 个被程序接受的静止、多轴、多距离姿态；
- 引导模式另外保留至少 10 个不参与方法选择或拟合的验证姿态；
- 独立留出验证不超过 10 mm 与 1 度；
- 三次独立重复标定的外参差异不超过 5 mm 与 0.5 度。

实际外参、bag、现场图像、角色映射和 Tracker 私有信息只能保存在仓库外。公开
仓库提交配置模板、算法、测试和说明，不提交设备专属标定产物。

The workbench embeds guided and offline modes under **Calibration & Offline
Tools**. A fixed board and moving rigid camera–Tracker assembly are required.
The separately installed worker closes each dedicated recording before solving;
independent validation and three-run repeatability precede automatic activation.
The manager stores immutable result versions and publishes
`/calibration/<camera>/tracker_from_camera`. This is distinct from RealSense's
factory `extrinsics/depth_to_color`, and does not add another optical-frame TF
parent. Synthetic/ROS integration tests do not replace hardware acceptance of
the new guided workflow.

New sessions default to the `CC200-15-11.25` reference geometry: 12 × 9 squares,
15 mm square side, 11.25 mm black marker side, and the compatible `DICT_5X5_1000`
dictionary. The active pattern is 180 × 135 mm; the advertised 200 × 150 mm outer
board does not determine solver scale. Verify physical dimensions before use.
Imported configurations retain their measured board parameters, and historical
results remain unchanged.
