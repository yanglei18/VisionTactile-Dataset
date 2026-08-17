# VisionTactile-Dataset

![Ubuntu 24.04](https://img.shields.io/badge/Ubuntu-24.04-E95420.svg)
![ROS 2 Jazzy](https://img.shields.io/badge/ROS%202-Jazzy-22314E.svg)
![License](https://img.shields.io/badge/license-Apache--2.0-blue.svg)

[English](README.md)

VisionTactile-Dataset 通过 ROS 2 采集两台 D405 和一台 D436。公开工作流负责
启动相机、发布每台相机的软件时序消息，并为每个 session 管理一个不压缩的
rosbag Recorder 进程。

## 状态

- 软件目标环境：Ubuntu 24.04、ROS 2 Jazzy、RealSense ROS 4.58.1。
- 运行时录制使用 ROS 2 Jazzy 的 `ros2bag` CLI。
- 参考配置不代表硬件曝光同步，也不代表已录数据的质量。
- 已提供相机采集和独立的三 Tracker 只读 ROS 2 发布节点；Tracker 尚未加入默认
  相机 bag。
- 相机与 Tracker 源码均可通过带 submodule 的公开仓库复现；Tracker 真机启动还
  需要操作者在 Git 之外保存的私有启动抓包、bundle 和角色映射，这些私有输入
  永远不写入仓库。

## 默认 bag 契约

默认 bag = **6 Image + 3 CameraFrameTiming = 9 topics**。每台
`d405_1`、`d405_2` 和 `d436` 分别贡献彩色图像、深度图像和成组时序
三个 topic；allowlist 是明确且不含通配符的。

全部九个录制 topic 均使用 **keep-last depth 30、best-effort、volatile**
QoS overrides。

`COMPLETE = Recorder process lifecycle complete; it is not a data-quality claim.`
也就是说，`COMPLETE` 只表示 Recorder 进程生命周期完整，不是数据质量声明。

Recorder 以 MCAP 写入，no real-time rosbag or MCAP compression.

标定信息、TF、RealSense 原始 metadata 和 session 描述不在 bag 中；如需这些
信息，应从录制 allowlist 之外的来源另行获取。

## 参考硬件

已测设备身份和 USB 拓扑建议见[硬件参考](docs/hardware-reference.md)。

| 相机 | 型号 | 序列号 | 固件 | ASIC 序列号 |
| --- | --- | --- | --- | --- |
| `d405_1` | D405 | `260322278433` | `5.15.1.55` | `255323071625` |
| `d405_2` | D405 | `260322276463` | `5.15.1.55` | `255323071742` |
| `d436` | D436 | `408322071716` | `5.17.0.213` | `343123151280` |

## 数据速率规划

`1280×720@30 raw estimate = 414.72 MB/s, about 124.4 GB per 300 s.`

这是六路图像流的算术规划值，不是运行时容量门槛。请针对实际目标文件系统进行
测量并预留合适的运行余量。

## 快速开始

请使用仓库外已经存在的绝对数据目录。按[采集指南](docs/capture-guide.md)安装
依赖并构建，然后启动：

```bash
export VT_REPO="$(git rev-parse --show-toplevel)"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
mkdir -p "${VT_DATA_ROOT}"
git -C "${VT_REPO}" submodule update --init --recursive
source /opt/ros/jazzy/setup.bash
cd "${VT_REPO}/ros2_ws"
colcon build --event-handlers console_direct+
source install/setup.bash
ros2 launch vt_realsense_capture triple_realsense.launch.py \
  output_root:="${VT_DATA_ROOT}"
```

## 文档

- [最终用户手册（开源发布版）](docs/user-manual.md)
- [启动参数、Topic、QoS 与命令行接口参考](docs/interface-reference.md)
- [架构与九 topic 契约](docs/architecture.md)
- [采集指南](docs/capture-guide.md)
- [硬件参考](docs/hardware-reference.md)
- [故障排查](docs/troubleshooting.md)
- [VIVE Tracker ROS 2 发布与验收](docs/tracker-ros2-publisher.md)
- [维护者发布检查表](docs/release-checklist.md)

## 范围与限制

本版本不宣称跨相机硬件曝光同步，也未实现离线跨相机对齐、空间标定、点云生成、
触觉采集、相机/Tracker 空间标定或相机与 Tracker 联合录制。Tracker 节点只发布
原生 `vive_map` 坐标且不发布 TF。`CameraFrameTiming` 只描述单台相机内部的软件
分组；实验适用性需要对已录数据另行检查。

## 贡献

软件测试和文档要求见 [CONTRIBUTING.md](CONTRIBUTING.md)。敏感缺陷请按
[SECURITY.md](SECURITY.md) 中的私密流程报告。

## 许可证

本项目自有文件采用 Apache-2.0；固定版本的 PyVUT submodule 具有独立的
Apache-2.0 许可证和署名文件。详见 [LICENSE](LICENSE)、
[第三方说明](THIRD_PARTY_NOTICES.md)及 submodule 内的许可证文件。
