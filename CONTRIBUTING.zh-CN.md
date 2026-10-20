# 开发与贡献指南

[English](CONTRIBUTING.md) · [新电脑部署指南](docs/deployment.zh-CN.md)

感谢你帮助保持公开采集接口精简、可测试，并让文档准确描述实际能力。

## 开发环境

- 使用 Ubuntu 24.04 和 ROS 2 Jazzy 开发、测试。
- ROS 节点和构建使用系统 Python。PySide6 工作台、离线工具、可选 PyQt5/FSGlove/MANO
  运行环境分别使用独立虚拟环境；采集、构建或测试终端中不要激活 Conda。
- 涉及生产 wrapper 的改动使用 RealSense ROS 4.58.1。
- 运行时录制使用 Jazzy 的 `ros2bag` 命令。
- 在独立的功能分支中开发，避免直接修改发布分支。

递归克隆 `source-preview-20261019`，或在该分支已有的克隆中运行
`git submodule update --init --recursive`，保留固定的子模块版本。
系统依赖、[ROS 初始化](docs/deployment.zh-CN.md#ros)、
[RealSense wrapper 版本选择](docs/deployment.zh-CN.md#realsense)、
USB 访问和工作空间配置见[部署指南](docs/deployment.zh-CN.md)，其中包含完整的克隆命令。
环境已经准备好的电脑可以参考[快速开始](README.zh-CN.md)。
公开软件测试不能依赖本地 Tracker 凭据、实体传感器、外部 FSGlove 源码或授权 MANO 资产。
需要这些前提的可选测试必须明确标识，并分别报告跳过和通过的数量。

## 修改流程

每次行为改动前，先编写或更新一个会失败的测试，确认其因预期原因失败，
然后实现能够通过测试的最小改动。仅修改文档时也必须满足公开源码树约束。

提交 Pull Request 前：

1. 构建并运行软件测试：

   以下命令采用部署指南的源码驱动工作空间。
   如果采用指南中另行核验的 APT 路线，省略驱动工作空间的 `source` 命令。

   ```bash
   export VT_REPO="$(git rev-parse --show-toplevel)"
   test -z "${CONDA_PREFIX:-}"
   source /opt/ros/jazzy/setup.bash
   source "${HOME}/visiontactile-drivers_ws/install/setup.bash"
   cd "${VT_REPO}/ros2_ws"
   colcon build --event-handlers console_direct+
   source install/setup.bash
   PYTHONPATH="${VT_REPO}/tools/multisensor_alignment/src:${VT_REPO}/ros2_ws/src/vt_realsense_capture${PYTHONPATH:+:$PYTHONPATH}" \
     colcon test --event-handlers console_direct+
   colcon test-result --all --verbose
   cd "${VT_REPO}"
   ```

   此流程不要使用符号链接安装。还应运行下方对应的独立工具测试集：
   `unittest discover` 不会收集全部 pytest GUI 和工作流测试。
   显式测试路径用于提供离线回放解析器和纯 Python 采集控制器，
   以执行跨组件回归测试，不会启动 ROS 硬件。

2. 检查当前发布候选，只暂存计划公开的文件，再检查暂存区。
   发布检查不会自动暂存文件。公开源码树检查器特意读取 Git 暂存区中的内容，
   因此只修改工作区不能让旧的暂存内容被误判为已更新。

   ```bash
   python3 tools/release/prepare_release.py check
   git add -- path/to/changed-file
   python3 tools/test_check_public_tree.py
   python3 tools/check_public_tree.py
   git diff --cached --check
   ```

3. 修改公开行为、命令、兼容性或限制时，同步更新中英文用户文档。

4. 生命周期状态必须与软件实现一致。特别是 `COMPLETE` 表示已经确认 Recorder
   进程退出，不能用它证明消息覆盖完整、时间质量合格、没有丢帧或符合实验要求。

保持修改范围集中，在 Pull Request 中解释可观察的效果及软件测试结果。
硬件观察可作为参考，但不是 Recorder 生命周期本身的必要验收条件。

## 工作台组件边界

命令结果规则放在独立于 Qt 的操作协调器中，领域规则放在 `vt_capture_core`；
主窗口负责应用展示更新。共享组件通过快照和生命周期方法提供服务，
使用方不能读取其私有缓存或调用私有布局、渲染辅助方法。
新需求需要扩展时，先扩展现有公共 API，再加入行为测试。
依赖方向和针对性的源码约束见
[架构指南](docs/architecture.md#desktop-operation-coordination)
及 `tools/capture_console/tests/test_shared_component_boundaries.py`。

记录本地开发基线前，明确检查已跟踪和未跟踪文件。
将无关工作保留在可以区分的独立提交中，并针对实际暂存区执行公开源码树检查。
被忽略的本地参考材料应始终留在暂存区之外；取消 Git 跟踪时不能删除操作者的本地副本。
当前工作区干净不代表历史提交适合公开，发布前还要按独立的发布准备指南处理。

## 无需硬件的工具测试

以下命令从仓库根目录执行。离线工具在 Ubuntu 24.04 上使用 Python 3.12；
测试 ROS 消息和 bag 序列化时需要加载已构建的 ROS 工作空间。
Tk 测试需要显示环境，在无显示器电脑上用 Xvfb 提供：

```bash
sudo apt install -y python3-venv python3-tk xvfb xauth
python3 -m venv --system-site-packages "${HOME}/.venvs/vt-offline-tests"
"${HOME}/.venvs/vt-offline-tests/bin/python" -m pip install \
  -e 'tools/tracker_camera_calibration[test]' \
  -e 'tools/multisensor_alignment[test]' \
  -e ros2_ws/src/vt_capture_core -e ros2_ws/src/vt_capture_manager
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 xvfb-run -a \
  "${HOME}/.venvs/vt-offline-tests/bin/python" -m pytest \
  tools/tracker_camera_calibration/tests -q
PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  "${HOME}/.venvs/vt-offline-tests/bin/python" -m pytest \
  tools/multisensor_alignment/tests -q
```

两个测试集包含同名测试模块，须分别运行。
工作台有单独验证过的 Qt 依赖组合，不要把离线工具的 `[test]`
附加依赖装进工作台环境，因为它们对 pytest 版本的约束不同：

```bash
bash tools/capture_deploy/install.sh
.venv-capture-console/bin/python -m pip install \
  -e ros2_ws/src/vt_capture_manager -e tools/multisensor_alignment \
  'opencv-python-headless>=4.9,<4.12'
env PYTHONPATH=ros2_ws/src/vt_realsense_capture:tools/multisensor_alignment/src \
  QT_QPA_PLATFORM=offscreen PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-capture-console/bin/python -m pytest -q \
  ros2_ws/src/vt_capture_core/test ros2_ws/src/vt_capture_manager/test \
  tools/capture_console/tests tools/capture_deploy/tests tools/tracker_deploy/tests
QT_QPA_PLATFORM=offscreen bash tools/capture_deploy/start_demo.sh \
  --smoke-seconds 2
```

`install.sh` 同时为 Linux 工作台安装可选的 `tools/tracker_deploy` 包。
其测试使用合成私有输入，不导入或初始化真实 Tracker 设备。
Windows 独立触觉小工具排除此包，须继续通过原生运行测试验证导入边界。

这些命令安装当前用户的桌面入口并运行合成数据测试，不会启动传感器或录制 bag。
可选 ROS 集成测试使用合成发布端和专用测试域，
按[工作台测试指南](tools/capture_console/README.md#测试与截图)执行，
并与实时采集域隔离。纯软件测试和离屏渲染不能替代硬件、OpenGL 驱动、
姿态精度或长时间运行验收。
上方显式源码路径提供 CMake 包中的纯 Python 采集控制器和离线回放解析器，
用于控制器、管理后台故障恢复及回放测试，不会启动 ROS 节点或 Recorder。
修改受管录制接口任一侧时，应保留这些跨组件测试。
修改具体适配器时，还应运行对应的[手套](tools/fsglove12/README.md)、
[桥接](ros2_ws/src/vt_glove_bridge/README.md)
或[触觉](ros2_ws/src/vt_tactile_capture/README.md)指南中的测试。

## 分发包检查

修改包依赖、入口、资源或发布工具时，除了对应源码测试，
还要验证安装产物。使用 Python 3.12，每次验证使用新的输出目录：

```bash
python3 -m unittest discover -s tools/release/tests -v
python3 tools/release/verify_distributions.py \
  --profile all --output .runtime/release/distribution-check-1
```

验证器从源码分发包构建 wheel，检查包许可证和资源，再将 wheel
安装到全新环境，在仓库之外执行不依赖硬件的冒烟检查。
PySide6 和 PyQt5 环境分别验证。`core`、`offline` 和 `console`
配置可用于局部验证；`--build-only` 不能证明安装成功或运行正常。
完整配置仍不能替代 ROS 构建或硬件验收。
前提条件、依赖约束、证据保存和限制见
[发布准备指南](tools/release/README.md#安装包验证)。

## 数据与验证材料

不要提交录制数据或本地验证材料，包括 bag、MCAP 文件、图像、视频、录音录像、
凭据、私有场景数据、设备转储、抓包、初始化 bundle、角色映射、工作站配置、
以及大型生成日志。这些产物应保存在仓库之外。
不要将 MANO 模型、派生网格缓存或另行获取的 FSGlove/Tracker 源码加入源码归档；
本仓库许可证不授予这些输入的使用或分发权利。

如果改动包含针对特定硬件的结论，应明确限定测试设备和连接方式，
并区分实际观察与软件保证。通过合适的审查渠道只分享已去除敏感内容的文本摘要，
不要把原始验证材料加入 Git。

首次公开上传前，请按[发布准备指南](tools/release/README.md)操作。
当前源码树检查和 `.gitignore` 无法删除历史 Git 提交中的敏感或未授权材料。
应单独审查拟发布的历史，或使用文档描述的干净源码导出流程；
不能因为当前检查通过就认为已有 Git 历史适合公开。
源码导出不会打包 PyVUT 子模块内容。
