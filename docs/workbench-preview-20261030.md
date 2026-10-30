# 工作台源码快照：VIVE Hub 与多相机点云

此分支保留当前工作台及其相关 ROS、标定、点云和界面代码，是开发预览，
不是已经完成长期真机验收的产品发行版。它基于已公开的源码预览继续开发，
没有合并此后上游分支的其他更新。

## 本次包含

- 官方 Linux VIVE Hub SDK 的只读位姿桥、显式序列号角色绑定、固定主机时钟
  偏差换算；工作台可按本机偏好启动或复用 Hub 和 ROS 桥。不执行重新配对。
- 标定逐轮保存、退出后续做、指定轮次重拍及独立重试报告；不覆盖已保留的数据。
- RealSense 原生 PointCloud2 输入，正确的 depth → color → Tracker → world
  变换；原 RGB-D 重建输入仍可选择。
- 各相机独立匹配自己的源时刻与 Tracker 位姿、独立 Tracker 接收子进程、
  有界异步计算和低复制传输。暂留画面明确标为非实时，不用旧云套新位姿。
- 全量点显示、基于实际点云范围取景、线框视锥、可关闭的腕部 7–50 cm
  显示裁剪，以及触控板缩放和平移。显示筛选不修改 ROS 原始数据或录包。
- 三相机错开启动，降低同时初始化的 USB 争用。

## 在另一台电脑使用

从本次源码分支开始（若已取得该分支，无需重复克隆）：

```bash
git clone --branch feature/workbench-vivehub-pointcloud-20261030 \
  https://github.com/yanglei18/VisionTactile-Dataset.git
cd VisionTactile-Dataset
```

1. 按[新电脑部署指南](deployment.zh-CN.md)准备依赖和 ROS 工作区。
2. 按[VIVE Hub 桥说明](../tools/vivehub_bridge/README.md)使用单独取得的官方 SDK
   编译读取器，填写本机私有角色绑定表；不要同时运行旧 PyVUT Dongle 读取器。
3. 按[工作台手册](../tools/capture_console/README.md)配置设备及空间页面；启动入口：

   ```bash
   bash tools/capture_deploy/start_workbench.sh
   ```

4. 原生点云需在相机启动配置中显式启用，并在场景配置设置相应来源和匹配模式。
   参见手册“直接显示 RealSense 驱动点云”与“实时逐路匹配与时间分组”。
   正式采集默认配置不会因为更新显示代码而自动改变。
5. 使用自己设备的身份、地图与外参。旧 PyVUT 外参不能仅改坐标系名称后用于
   Hub；请按[标定工具手册](../tools/tracker_camera_calibration/README.md)核验或重标定。

## 发布边界与仍待验证项

- 仅上传源码、测试、文档及通用配置示例；不包含传感器数据、私人设备绑定、
  标定结果、SDK 库或编译产物。本机已运行不等于干净电脑无需配置即可运行。
- 逐路预览不是三相机同时曝光，也不是录制后的严格时间对齐。
- 三路全量点云的持续刷新率、源龄和完整 GUI 负载仍需真机验收；不宣称
  30 Hz 显示、无延迟或全部卡顿已经解决。
- 实时空间页显示单帧点云及有界短暂留存，不把它称为累计建图或地图导出。
- 显示端不裁剪不等于相机提供所有距离的有效深度；驱动的 `clip_distance`
  也影响深度图，不能只作为无副作用的显示选项理解。
- 依赖授权范围见[第三方说明](../THIRD_PARTY_NOTICES.md)。
