# 新电脑部署指南

[English](deployment.md) · [文档首页](../README.zh-CN.md) · [日常操作](../tools/capture_console/README.md)

普通用户使用配套完整安装包时，请先看[用户快速上手](quickstart.zh-CN.md)：下载一次，打开图形安装器，等待环境自动准备完成。以下分组件命令供维护者从源码部署、排错及增加可选模块使用，不是普通用户的安装清单。

本维护指南对应 `source-preview-20261019` 分支。它将环境安装集中在一处；各模块手册负责安装后的配置和操作。命令用于目标新电脑，按章节执行；遇到错误先停止，不要继续粘贴后续命令。

**新款右手 IMU 手套**：完成手势环境安装后，在工作台“设备 → 配置设备 → 右手”选择
LSM6DSR 硬件版本与近节＋末节布局，保存时自动生成私有配置并匹配 104 Hz 上限，
无需从旧电脑复制配置路径。已有自定义配置不自动替换。重新标定后再验收手势效果；
固件 v2 需要主控和从机配套刷写，软件安装不会完成刷写。
详见[手套操作说明](../tools/fsglove12/README.md#图形化配置与可靠性固件-v2)与
[固件构建及升级说明](../data_imu_stm32/README.md)。

| 你的用途 | 安装路径 | 完成标准 |
| --- | --- | --- |
| 只用 Windows 触觉三维小工具 | 直接看[第 1 节](#windows) | 连接 COM 口、加载通用或该设备已保存参考，验证轻触、持续按压和松开响应 |
| Windows 独立手势＋触觉工具 | 见[新工具安装与使用](../tools/hand_tactile_viewer/README.md) | 两个串口、三步手势标定、同一动态右手上的触觉响应；与纯触觉工具并存 |
| Ubuntu 仅体验界面 | 第 2、3 节，然后第 7 节的演示 | 标有 DEMO 的合成画面正常显示 |
| Ubuntu 真实采集、回放与质检 | 第 2–8 节 | 真实流更新，完成短包录制、检查与档案回放 |
| 增加 Tracker、手势或标定 | 基础部署后按[第 9 节](#modules)增加 | 各模块分别通过输入和操作检查 |

公开仓库提供源码。Tracker 私有启动输入、扩展 PyVUT、FSGlove 外部源码和许可模型需要另行准备；相机采集、触觉原始数据及二维显示不要求 Tracker 或 MANO 模型。

已有 Tracker 设备组可由部署人员交付包含 **Install-VisionTactile** 的完整安装包，自动安装本指南中的基础运行环境、固定版本相机驱动、工作台和配套 Tracker 输入。仅含 `install.sh` 的旧源码包仍需维护者准备系统及 ROS 环境。完整交付的构建方式见[自动安装器维护说明](../tools/capture_deploy/README.md)，单独导入已有 `.vttracker` 见[Tracker 部署指南](tracker-deployment.zh-CN.md)。

<a id="windows"></a>
## 1. Windows：只安装独立触觉工具

使用 Windows 10 1809 及以上或 Windows 11 的 x64 桌面电脑，需要可用的桌面 OpenGL。系统范围和构建依据见 [Windows 工程说明](../tools/tactile_viewer_windows/README.md)。不需要安装 ROS、Python、PyTorch 或主采集程序。

1. 从交付人员取得 `VisionTactile3D-<版本>-windows-x64-setup.exe` 和对应 `.sha256`。**当前 GitHub 预览分支只有源码，没有公开安装包下载**；缺少安装包时，应由维护人员按[构建说明](../tools/tactile_viewer_windows/README.md)提供包含获准分发模型的安装包。源码 ZIP 不能直接安装。
2. 用 PowerShell 核对文件摘要与交付的 `.sha256` 一致：

   ```powershell
   Get-FileHash -Algorithm SHA256 .\VisionTactile3D-0.6.2-windows-x64-setup.exe
   ```

3. 关闭旧版程序，运行安装包，选择中文或 English，完成当前用户安装。
4. 插入触觉板，启动“触觉三维预览”，刷新并选择实际 COM 口，确认波特率（默认 `115200`），点击“连接”。
5. 显示“通用参考”“默认参考”或“已保存参考”时可直接使用；同协议同布局右手 400 点设备换电脑或换板不强制归零。无载偏差明显时，可保持触觉区域不受压，点击“无载归零”并保持约 3 秒；有 USB 唯一身份时会自动保存该设备的新参考。
6. 对不同区域轻触、持续按住、松开，确认无载区域保持底色、持续按压保持响应、信号回到无载范围后恢复底色。拖动旋转、滚轮缩放；语言切换应保留连接。
7. 点击“断开”，退出。升级时运行新安装包；卸载通过 Windows“设置 → 应用”，用户偏好会保留。

找不到 COM 口时检查数据线及设备管理器，必要时安装**该设备厂商提供的驱动**；安装包不包含 USB 驱动。三维画面异常时检查显卡驱动。不要让两个程序同时占用同一串口。

安装目录的 `使用说明.txt` 是完整中英用户指南，对应仓库 [USER_GUIDE.txt](../tools/tactile_viewer_windows/USER_GUIDE.txt)。`0.6.2` 增加不绑定 USB 序列号的通用参考，优先使用本机已有的设备专属参考；专属参考仍不跨设备混用。通用参考是显示起点，不代表已经对新板完成实测归零。源码构建未提供默认参考时仍需手动归零。程序不自动识别硬件型号，更换板卡后需刷新并选择新 COM 口再连接。新电脑无需配置 Python 或模型路径。颜色表示噪声门槛处理后的相对 ADC 响应，不是压力单位；“配置设备”提供显示范围（默认 `0～1000 ΔADC`）和响应方向，通用模式设置单独保存，设备专属设置仍优先。“历史保持”不是实时测量。摘要命令中的版本不表示已有公开下载，旧安装包需重新构建并升级。Windows 真机串口、显卡、无载/轻触响应和长期运行仍须在目标电脑上验收。

<a id="ubuntu"></a>
## 2. Ubuntu：准备基础环境

本流程以原生 Ubuntu 24.04 LTS x86_64 桌面为参考；不把 WSL、虚拟机 USB 转发或其他发行版当作已验收配置。首次安装需要网络、普通用户账户和用于系统包安装的 sudo 权限。先不要启动传感器。

在 Bash 终端确认系统、Python 和空间：

```bash
cat /etc/os-release
uname -m
/usr/bin/python3 --version
df -h "$HOME"
test -z "${CONDA_PREFIX:-}"
test -z "${VIRTUAL_ENV:-}"
```

应为 Ubuntu `24.04`、`x86_64`、系统 Python `3.12`。退出已激活的 Conda/venv 后再部署 ROS。编译和环境需要额外空间；三相机默认 300 秒录制的容量门槛约为 **129.73 GiB**，与软件安装空间分开计算。参考 USB 和存储条件见[硬件说明](hardware-reference.md)。

```bash
sudo apt update
sudo apt install -y software-properties-common
sudo add-apt-repository -y universe
sudo apt update
sudo apt install -y git curl ca-certificates locales acl usbutils pkexec \
  python3-venv python3-tk libegl1 libgl1 libxkbcommon0 libxcb-cursor0 fonts-noto-cjk
locale charmap
```

若最后不是 `UTF-8`，执行 `sudo locale-gen en_US.UTF-8`，并在当前终端 `export LANG=en_US.UTF-8`；已有中文 UTF-8 环境可保留。

<a id="source"></a>
## 3. 获取正确版本

**使用程序与 Tracker 一体交付包时**，跳过本节的 Git 克隆命令，在解压后的 `VisionTactile-Delivery/Vision-Tactile-Dataset` 目录执行以下命令，再继续第 4 节。源码包没有 `.git`，版本和实际文件摘要见外层的 `DELIVERY_MANIFEST.json` 与源码中的 `PUBLIC_SOURCE_MANIFEST.json`。

```bash
export VT_REPO="$(pwd -P)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export VT_DRIVER_WS="${HOME}/visiontactile-drivers_ws"
mkdir -p "${VT_DATA_ROOT}/config" "${VT_DATA_ROOT}/deployment"
cp "${VT_REPO}/../DELIVERY_MANIFEST.json" "${VT_DATA_ROOT}/deployment/source-delivery.json"
```

到第 7 节安装工作台时，使用外层 `bash "${VT_REPO}/../install.sh"`，替代普通安装命令，以同时导入随包 Tracker。安装后保留整个解压目录。

**直接从 GitHub 获取源码时**，使用下面的克隆流程。

在选择的源码父目录执行。这里明确选择本次发布分支，避免默认克隆到保留的旧 `main`：

```bash
git clone --branch source-preview-20261019 --recurse-submodules \
  https://github.com/yanglei18/VisionTactile-Dataset.git
cd VisionTactile-Dataset
export VT_REPO="$(pwd -P)"
export VT_WS="${VT_REPO}/ros2_ws"
export VT_DATA_ROOT="${HOME}/visiontactile-data"
export VT_DRIVER_WS="${HOME}/visiontactile-drivers_ws"
mkdir -p "${VT_DATA_ROOT}/config" "${VT_DATA_ROOT}/deployment"
git branch --show-current
git rev-parse HEAD
git submodule status
git rev-parse HEAD > "${VT_DATA_ROOT}/deployment/source-commit.txt"
```

预期分支为 `source-preview-20261019`；保存实际提交号用于复现。公开 PyVUT 固定引用不等于 Tracker 生产所需扩展版本。需要审查子模块许可时见[第三方说明](../THIRD_PARTY_NOTICES.md)。

已有修改的旧目录请保留，采用新目录克隆，不用 `reset --hard` 覆盖它。更换终端后需重新设置上述路径变量；将 `VT_REPO` 指向实际源码目录，其余变量从它重建。源码、数据、私有输入分别保存。

仅体验界面的用户可以直接去[第 7 节](#desktop)，无需安装 ROS 和相机驱动。

<a id="ros"></a>
## 4. 安装 ROS 2 Jazzy

本节依据 [ROS 官方 Ubuntu Deb 安装指南](https://docs.ros.org/en/jazzy/Installation/Ubuntu-Install-Debs.html)及其[软件源配置](https://github.com/ros2/ros2_documentation/blob/jazzy/source/Installation/_Apt-Repositories.rst)。第 2 节已启用 Universe，本节用官方 `ros2-apt-source` 包配置签名和源；不要使用来历不明的安装脚本。

```bash
mkdir -p "${VT_DATA_ROOT}/deployment/downloads"
curl --fail --location --show-error \
  https://api.github.com/repos/ros-infrastructure/ros-apt-source/releases/latest \
  -o "${VT_DATA_ROOT}/deployment/downloads/ros-apt-source-release.json"
VT_ROS_APT_VERSION="$(/usr/bin/python3 -c \
  'import json,sys; print(json.load(open(sys.argv[1]))["tag_name"])' \
  "${VT_DATA_ROOT}/deployment/downloads/ros-apt-source-release.json")"
curl --fail --location --show-error \
  "https://github.com/ros-infrastructure/ros-apt-source/releases/download/${VT_ROS_APT_VERSION}/ros2-apt-source_${VT_ROS_APT_VERSION}.noble_all.deb" \
  -o "${VT_DATA_ROOT}/deployment/downloads/ros2-apt-source.deb"
sudo dpkg -i "${VT_DATA_ROOT}/deployment/downloads/ros2-apt-source.deb"
sudo apt update
sudo apt install -y ros-jazzy-desktop ros-dev-tools \
  python3-colcon-common-extensions python3-rosdep \
  ros-jazzy-ros2bag ros-jazzy-rosbag2-py ros-jazzy-rosbag2-storage-mcap \
  python3-yaml python3-numpy python3-opencv python3-serial fio
source /opt/ros/jazzy/setup.bash
test "$ROS_DISTRO" = jazzy
ros2 pkg prefix rosbag2_storage_mcap
```

软件源包按官方最新发行获取并保留下载记录；这不是固定整个 apt 仓库快照。若 apt 报依赖冲突，先核对 `/etc/apt/sources.list.d/ubuntu.sources` 的 Ubuntu 源是否包含 `noble-updates`、`noble-backports`，按官方指南处理，不混入其他 Ubuntu 版本的包。

首次初始化 rosdep：

```bash
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
  sudo rosdep init
fi
rosdep update
```

在两个终端分别 source ROS 后，运行 `ros2 run demo_nodes_cpp talker` 与 `ros2 run demo_nodes_py listener`。收到演示消息后，分别用 Ctrl+C 结束。这一步检查 ROS 通信，不连接传感器。

<a id="realsense"></a>
## 5. 固定 RealSense 驱动版本

项目参考 wrapper 为 **4.58.1**。上游该版 [CMake 配置](https://github.com/realsenseai/realsense-ros/blob/4.58.1/realsense2_camera/CMakeLists.txt)要求 SDK 至少 `2.58.0`；这里固定使用 SDK **2.58.1**。上游将 wrapper 4.58.1 标为 beta，因此保留软件与目标硬件分别验收的要求。

本指南选择**源码 SDK + 源码 wrapper**，不依赖 apt 仓库是否仍保留旧版。不要另外安装未锁版本的 `ros-jazzy-realsense2-camera`、`ros-jazzy-librealsense2*` 或 `librealsense2*` 来替代这一步。已装过其他 SDK/driver 的电脑应先检查并选择一种安装来源，不要直接覆盖其现有环境。

### 5.1 构建 SDK 2.58.1

以下采用 RSUSB 兼容后端，与项目的 RSUSB 权限工具配套。它不需要本指南修改内核模块；上游对这条[兼容安装路线](https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/libuvc_installation.md)未作正式验证，不能据此承诺三相机 30 Hz。其他 V4L2/DKMS 路径请按[上游安装说明](https://github.com/realsenseai/librealsense/blob/v2.58.1/doc/installation.md)单独部署、验收，不混用两种后端。

```bash
sudo apt install -y build-essential cmake pkg-config \
  libusb-1.0-0-dev libudev-dev libssl-dev
mkdir -p "${HOME}/visiontactile-deps"
git clone --branch v2.58.1 --depth 1 \
  https://github.com/realsenseai/librealsense.git \
  "${HOME}/visiontactile-deps/librealsense-2.58.1"
cmake -S "${HOME}/visiontactile-deps/librealsense-2.58.1" \
  -B "${HOME}/visiontactile-deps/librealsense-2.58.1/build" \
  -DCMAKE_BUILD_TYPE=Release -DCMAKE_INSTALL_PREFIX=/usr/local \
  -DFORCE_RSUSB_BACKEND=ON -DBUILD_EXAMPLES=OFF -DBUILD_TOOLS=ON \
  -DBUILD_GRAPHICAL_EXAMPLES=OFF -DBUILD_GLSL_EXTENSIONS=OFF \
  -DBUILD_PYTHON_BINDINGS=OFF -DBUILD_WITH_DDS=OFF -DBUILD_ROSBAG2=OFF \
  -DBUILD_UNIT_TESTS=OFF -DCHECK_FOR_UPDATES=OFF
cmake --build "${HOME}/visiontactile-deps/librealsense-2.58.1/build" --parallel 2
sudo cmake --install "${HOME}/visiontactile-deps/librealsense-2.58.1/build"
sudo ldconfig
pkg-config --modversion realsense2
```

预期输出 `2.58.1`。本流程构建命令行工具，不构建 RealSense Viewer；权限和连接检查在第 8 节通过本项目工具完成，不叠加上游的另一套 udev 规则。`--parallel 2` 限制编译并发，内存不足可降为 `1`。关闭的是 SDK 内部 DDS/ROS 2 录包功能，本项目的 ROS Recorder/MCAP 不受影响；参数定义见[SDK 构建选项](https://github.com/realsenseai/librealsense/blob/v2.58.1/CMake/lrs_options.cmake)。

### 5.2 构建 wrapper 4.58.1

驱动源放在项目外的独立工作区。使用固定 tag；tag clone 显示 detached HEAD 是正常现象。

```bash
mkdir -p "${VT_DRIVER_WS}/src"
git clone --branch 4.58.1 --depth 1 \
  https://github.com/realsenseai/realsense-ros.git \
  "${VT_DRIVER_WS}/src/realsense-ros"
source /opt/ros/jazzy/setup.bash
cd "${VT_DRIVER_WS}"
rosdep install --from-paths src/realsense-ros/realsense2_camera \
  src/realsense-ros/realsense2_camera_msgs src/realsense-ros/realsense2_description \
  --ignore-src -r -y --rosdistro jazzy \
  --skip-keys librealsense2
colcon build --packages-select realsense2_camera_msgs realsense2_description \
  realsense2_camera --event-handlers console_direct+ \
  --cmake-args -DCMAKE_BUILD_TYPE=Release -DBUILD_TESTING=OFF \
  -DUSE_LIFECYCLE_NODE=OFF -DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF
source install/setup.bash
ros2 pkg xml realsense2_camera | sed -n 's:.*<version>\(.*\)</version>.*:\1:p'
ros2 pkg prefix realsense2_camera
```

预期为 `4.58.1`，包路径应在 `visiontactile-drivers_ws/install` 下。出现其他版本或 `/opt/ros/jazzy` 下的 wrapper 路径时，先修正加载顺序，再构建主系统。官方 wrapper 源见[固定 tag](https://github.com/realsenseai/realsense-ros/tree/4.58.1)。

### 5.3 已有 apt 环境的版本分支

如果电脑已经由维护人员部署了匹配版本，可先用 `apt-cache policy ros-jazzy-realsense2-camera ros-jazzy-realsense2-camera-msgs librealsense2-dev` 检查包来源和可用版本，再用上面的 `ros2 pkg xml`、`ros2 pkg prefix`、`pkg-config` 检查**当前实际加载的版本**。

仅当来源提供配套的精确 Debian 版本时才使用 `apt install 包名=完整版本号`，并记录完整版本与来源。这里不提供猜测的 Debian 版本串。仓库没有 `4.58.1` 时采用第 5.1–5.2 节的独立源码方案；不要把 `4.58.x` 的其他版本当作 `4.58.1`。已有工作环境的切换和软件包卸载应由维护人员规划，不在运行中的采集机上混装。

<a id="build"></a>
## 6. 构建主系统

仍使用系统 Python。先加载 ROS，再加载刚构建的驱动，最后构建本项目：

```bash
test -z "${CONDA_PREFIX:-}"
test -z "${VIRTUAL_ENV:-}"
source /opt/ros/jazzy/setup.bash
source "${VT_DRIVER_WS}/install/setup.bash"
cd "${VT_WS}"
rosdep install --from-paths src --ignore-src -r -y --rosdistro jazzy \
  --skip-keys "librealsense2 realsense2_camera realsense2_camera_msgs"
colcon build --event-handlers console_direct+
source install/setup.bash
PYTHONPATH="${VT_REPO}/tools/multisensor_alignment/src:${VT_REPO}/ros2_ws/src/vt_realsense_capture${PYTHONPATH:+:$PYTHONPATH}" \
  colcon test --event-handlers console_direct+
colcon test-result --all --verbose
```

这里跳过的 rosdep 键由第 5 节提供，并非可以缺少依赖。若采用已验证的 apt 路线，则省略 `VT_DRIVER_WS` 的 source，保留版本核对。不要在同一构建目录来回切换普通安装和 `--symlink-install`。

主工作区的 `install/setup.bash` 会记录构建时的驱动 underlay；桌面后台随后加载主工作区即可获得这条依赖链，参见 [colcon 工作区链说明](https://colcon.readthedocs.io/en/released/user/using-multiple-workspaces.html)。因此不要移动或删除驱动目录。打开新终端、重新设置第 3 节的路径变量后验收：

```bash
source /opt/ros/jazzy/setup.bash
source "${VT_WS}/install/setup.bash"
ros2 pkg prefix vt_capture_manager
ros2 pkg xml realsense2_camera | sed -n 's:.*<version>\(.*\)</version>.*:\1:p'
/usr/bin/python3 -c 'import rclpy,rosbag2_py,cv2,numpy,yaml; from vt_camera_msgs.msg import CameraFrameTiming; print("runtime imports OK")'
```

测试应无失败，wrapper 仍为 `4.58.1`。测试和模块导入不代表实际硬件已经验收。开发者的独立测试环境见[中文开发指南](../CONTRIBUTING.zh-CN.md)。

<a id="desktop"></a>
## 7. 安装工作台并先运行演示

```bash
cd "${VT_REPO}"
bash tools/capture_deploy/install.sh
bash tools/capture_deploy/start_demo.sh
```

安装器建立独立 Qt 环境与桌面入口；不会安装 ROS/相机驱动、下载模型或启动硬件。DEMO 画面应能显示、切换中文/English；演示录制不产生真实 bag。

已取得 Tracker 私有包时，可将上面的安装命令改为 `bash tools/capture_deploy/install.sh --tracker-package "/绝对路径/设备组.vttracker"`，同时安装独立 Tracker 环境及对应输入，前提与步骤见[私有包指南](tracker-deployment.zh-CN.md)。Tracker wheel 离线安装；工作台依赖安装仍可能联网。

安装器会把实际使用的工作台 Python 环境记录到桌面入口；即使安装时使用了自定义 `VT_CONSOLE_PYTHON`，之后从桌面启动也无需重复设置该变量。其他环境变量不会因此自动保存到桌面入口。

不要复制旧电脑的 `.venv-*`。ROS 使用系统 Python，Qt 使用 `.venv-capture-console`，标定和手势分别使用自己的环境。演示失败时先修复字体、Qt 系统库或显卡问题，再连接硬件。

<a id="permissions"></a>
## 8. 权限、设备配置和第一次真实录制

### 8.1 串口与 RealSense USB

只读取串口列表：

```bash
/usr/bin/python3 -m serial.tools.list_ports -v
id -nG
```

需要串口且当前账户未加入 `dialout` 时执行 `sudo usermod -aG dialout "$(id -un)"`，然后完整注销桌面并重新登录。优先填写确认过的 `/dev/serial/by-id/...`；没有唯一 ID 时按实物确认 `/dev/serial/by-path/...`。IMU 与触觉板可能同名，不能凭名称猜端口，不使用全局 `chmod 777`。

接入相机后，在没有相机进程运行时检查：

```bash
cd "${VT_REPO}"
lsusb -t
python3 tools/capture_deploy/configure_realsense_usb.py check
```

本项目权限工具只覆盖参考相机 USB 产品 ID。若报告对应权限或 RSUSB 缓冲不足，按 [USB 配置说明](../tools/capture_deploy/REALSENSE_USB.md)执行：

```bash
pkexec /usr/bin/python3 "${VT_REPO}/tools/capture_deploy/configure_realsense_usb.py" apply
python3 tools/capture_deploy/configure_realsense_usb.py check
rs-enumerate-devices -S
```

确认实际序列号和 USB `5000M` 链路。该工具的 64 MiB usbfs 调整仅对本次开机生效，重启后重新检查；需要撤销时按同一说明使用 `rollback`。它不停止进程，也不打开相机。不支持的产品 ID 需采用对应厂商规则。

### 8.2 填入本机设备身份

将参考配置复制到源码之外：

```bash
cp -n "${VT_REPO}/ros2_ws/src/vt_realsense_capture/config/cameras.yaml" \
  "${VT_DATA_ROOT}/config/cameras.yaml"
```

编辑副本，按枚举结果填写每台设备的 `serial`、`firmware`、`asic_serial`，保持型号和角色对应；**仓库中的序列号属于参考设备，不能直接用于另一套相机**。手套／触觉录制应选择[对应的录制配置](../ros2_ws/src/vt_glove_bridge/README.md)，明确将相应 Topic 加入清单。

<a id="local-paths"></a>
### 8.3 默认路径与本机环境检测

工作台每次启动新的本机后台前，都会查找当前源码目录和本机常用安装位置，补齐能够确定的路径并自动保存；无需先打开“配置设备”、点击检测或手动保存基础路径。新电脑和迁移后的配置使用同一准备流程。程序只连接已有后台时，保持该后台配置不变。

数据输出目录默认为 `~/visiontactile-data`；需要其他位置时，可在启动工作台的环境中设置绝对路径 `VT_DATA_ROOT`。默认目录不存在时会在新后台启动准备阶段创建，已有输出路径保持不变。当前有效的软件路径保留；失效的软件路径仅在找到唯一有效候选时修复。

“设备 → 配置设备”用于调整模块、设备身份或自定义位置，**检测本机环境** 按钮保留为手动复检入口。手动检测只修改当前表单，不创建目录或重启后台；核对后点击 **保存并应用** 才提交，取消丢弃本次表单修改。保存时也会创建尚不存在的默认目录。底部显示补全数量和已启用模块仍需配置的必需项数量，悬停字段可查看缺失或多个候选的原因。

| 配置项 | 自动查找的标准位置 | 自定义位置的环境变量 |
| --- | --- | --- |
| 数据输出目录 | `~/visiontactile-data` | `VT_DATA_ROOT` |
| 相机配置 | 数据目录中的 `config/cameras.yaml`；不存在时提供仓库参考配置 | 无；在表单中选择 |
| 手套 Python | 仓库中的 `.venv-fsglove-120hz/bin/python`、`.venv-fsglove/bin/python`、`.venv-glove/bin/python`、`tools/fsglove12/.venv/bin/python`，或 `~/.venvs/vt-fsglove/bin/python` | `VT_FSGLOVE_PYTHON` |
| FSGlove 源码 | 仓库中的 `external/FSGlove`、`~/visiontactile-deps/FSGlove`、`~/.local/share/visiontactile/fsglove` | `VT_FSGLOVE_SOURCE`；兼容 `FSGLOVE12_UPSTREAM_CHECKOUT`、`FSGLOVE_SOURCE` |
| MANO 资源 | 仓库中的 `external/mano-assets`、数据目录中的 `models/mano-assets` 或 `models/mano`、`~/visiontactile-data/models/mano-assets`、`~/visiontactile-deps/mano-assets` | `VT_MANO_ASSETS`；兼容 `FSGLOVE12_MANO_ASSETS`、`MANO_ASSETS` |
| 扩展 PyVUT 源码 | `~/.local/share/visiontactile/pyvut-src`、仓库中的 `external/pyvut` | `VT_PYVUT_ROOT` |
| PyVUT 依赖目录 | `~/.local/share/visiontactile/pyvut-venv/lib/python*/site-packages` | `VT_PYVUT_SITE` 直接指定目录，或用 `VT_PYVUT_VENV` 指定虚拟环境根目录 |
| Tracker 启动包 | 已安装 `.vttracker` 的匹配版本；否则填写明确路径，且文件须位于仓库外 | `VT_BUNDLE` |
| Tracker 角色映射 | 已安装 `.vttracker` 的匹配版本；否则为数据目录中的 `tracker-validation/roles.yaml` | `VT_ROLE_MAP` |

环境变量用于填入空项或修复允许修复的软件路径，不覆盖当前有效的配置。同一行列出的兼容变量按从左到右的顺序取第一个已设置项；显式位置无效时会提示缺失，不会另选其他目录。变量应传入本次工作台进程，例如在已设置变量的同一终端运行 `bash tools/capture_deploy/start_workbench.sh`；保存后的路径不依赖下次启动时继续设置变量。路径可使用 `~`，程序不执行 `$变量` 或 shell 命令展开。

检测不会替换已经填写但失效的相机配置、Tracker 启动包、角色映射或标定配置，以免换成另一套设备的身份；这些项需手动重新选择。串口只枚举，不自动分配左右手或 IMU／触觉用途。MANO 按对应手检查 `models/MANO_RIGHT.pkl` 或 `models/MANO_LEFT.pkl`；缺少已安装的依赖、获准使用的模型或实际设备时，仍需按模块手册准备。**检测检查的是文件位置和结构，不代表依赖已完整安装或硬件已通过验证。**

已安装的 Tracker 包还提供对应的 PyVUT 源码和依赖路径。要替换旧配置中明确指定的设备组，使用 **追踪器 → 导入 Tracker 部署包…**，核对填入的草稿后 **保存并应用**；导入本身不修改活动后台，也不启动硬件。

<a id="first-run"></a>
### 8.4 从工作台完成短包验收

关闭演示，然后：

```bash
cd "${VT_REPO}"
bash tools/capture_deploy/start_workbench.sh
```

1. 等待工作台自动准备路径并显示后台已就绪。已保存设备配置且设备身份未变化时，可直接选择需要的模块启动，无需重复检测或保存路径。
2. 首次绑定设备或需要调整时，再进入“设备 → 配置设备”，只启用实际需要的模块，核对相机身份和实际串口后“保存并应用”。该操作重载**空闲后台**，不是重启电脑；需要重载时先结束录制、停止受管设备。默认位置与复检规则见[上一节](#local-paths)。
3. 启动需要的设备，确认预览持续更新，未显示历史保持。没有 Tracker 不妨碍录制已有配置流。
4. 查看录制计划，核对真正需要的 Topic。只启用预览不会自动把新的 Topic 加入 Recorder 清单。
5. 先录制一个约 10 秒的测试包，停止后等待收尾。在“档案”查看话题、质检报告和内嵌回放；已有 Topic 应可播放。
6. 相机目标为 30 Hz 时，检查每一路真实图像的观测频率和时间缺口；`COMPLETE` 只表示录制进程正常收尾。触觉的显示快照频率不等于整掌扫描频率。
7. 结束时先停止录制和受管设备，再关闭工作台；关闭窗口本身不会停止后台设备。

操作细节见[工作台手册](../tools/capture_console/README.md)，CLI 流程见[中文采集指南](capture-guide.zh-CN.md)。短包通过后再做目标时长、目标设备数量下的验收。

<a id="modules"></a>
## 9. 按需增加模块

| 模块 | 需要额外准备 | 操作文档 |
| --- | --- | --- |
| Tracker | Windows 配对建图及匹配的运行环境和私有输入，可通过 `.vttracker` 包交付 | [私有包部署](tracker-deployment.zh-CN.md)、[发布与验收](tracker-ros2-publisher.md) |
| 手势解算 | 对应串口、外部 FSGlove、获准使用的 MANO、独立模型环境 | [手势手册](../tools/fsglove12/README.md) |
| 触觉二维采集 | 串口权限、实际设备和手别；不要求模型 | [触觉采集](../ros2_ws/src/vt_tactile_capture/README.md) |
| Linux 固定手形触觉工具 | Qt 环境、触觉串口、获准使用的静态模型或独立模型环境 | [独立工具说明](../tools/capture_console/README.md#独立触觉三维小窗口) |
| 相机–Tracker 标定 | 稳定图像与位姿、固定标定板、刚性相机–Tracker 整体、独立标定环境 | [标定安装与操作](../tools/tracker_camera_calibration/README.md) |
| 离线对齐 | 已关闭的 bag、对应身份外参、独立对齐环境 | [离线对齐](../tools/multisensor_alignment/README.md) |

主工作台的录后质检由系统 Python 加载仓库中的对齐模块，依赖已由第 4、6 节提供。独立标定/对齐 CLI 则按各自手册建立 `.venv-calibration`、`.venv-alignment`；不要把它们的依赖装入 Qt 环境。

迁移外参时核对物理设备、刚性安装关系与地图。不同电脑不一定需要重新标定，但更换设备、安装关系或地图后不能直接沿用旧结果。

<a id="upgrade"></a>
## 10. 留存环境、升级和迁移

首次验收后在数据目录保存记录：

```bash
if [[ -f "${VT_REPO}/../DELIVERY_MANIFEST.json" ]]; then
  cp "${VT_REPO}/../DELIVERY_MANIFEST.json" "${VT_DATA_ROOT}/deployment/source-delivery.json"
else
  git -C "${VT_REPO}" rev-parse HEAD > "${VT_DATA_ROOT}/deployment/source-commit.txt"
fi
git -C "${VT_DRIVER_WS}/src/realsense-ros" rev-parse HEAD \
  > "${VT_DATA_ROOT}/deployment/realsense-ros-commit.txt"
git -C "${HOME}/visiontactile-deps/librealsense-2.58.1" rev-parse HEAD \
  > "${VT_DATA_ROOT}/deployment/librealsense-commit.txt"
dpkg-query -W > "${VT_DATA_ROOT}/deployment/ubuntu-packages.txt"
"${VT_REPO}/.venv-capture-console/bin/python" -m pip freeze \
  > "${VT_DATA_ROOT}/deployment/console-packages.txt"
```

使用 apt 驱动路线时，用已安装包的完整版本记录代替不存在的驱动源码提交记录。保留短包质检结果、设备固件、USB 拓扑与实际配置；这些属于本机资料，不上传公共仓库。

升级前正常停止录制、设备和后台，再备份桌面模式的 `~/.config/visiontactile/workbench/`、独立后台实际使用的状态目录及数据配置。数据库须在后台停止后备份，或采用 SQLite 一致性备份，不能漏掉活动 WAL。路径说明见[工作台手册](../tools/capture_console/README.md)。

在工作区干净时使用 `git pull --ff-only` 获取该分支更新；有本地修改时先保留并处理，不强制覆盖。按变更重建 ROS、重新运行桌面安装器及更新各工具环境，再做短包回归。移动目录或更换电脑时重新建立 venv 和桌面入口，不要复制虚拟环境。下一次启动新的本机后台前，程序会自动补齐缺项并修复能够确定的软件路径；有效配置及设备身份不变。仅当仍有缺项或设备身份、串口、输出位置需要调整时打开“配置设备”处理；“检测本机环境”可用于手动复检。具体规则见[默认路径与本机环境检测](#local-paths)。

<a id="troubleshooting"></a>
## 11. 首次部署常见问题

| 现象 | 下一步 |
| --- | --- |
| 克隆后找不到新功能 | `git branch --show-current`；应为本指南分支，并核对提交 |
| 找不到 `ros-jazzy-*` 包 | 检查 Noble/Universe 和官方 ROS apt 源，第 4 节 |
| wrapper 版本不是 4.58.1 | 核对驱动安装来源、`ros2 pkg prefix` 与 source 顺序，第 5–6 节 |
| 编译找不到 librealsense2 | `pkg-config --modversion realsense2`；确认 SDK 安装完成并运行过 `ldconfig` |
| `xcb`、EGL 或中文字体异常 | 补齐第 2 节 Qt 系统库，检查桌面显卡驱动 |
| 自动准备后仍提示配置缺项 | 打开“配置设备”查看字段提示；安装缺失依赖或确认设备身份、串口，需要时手动复检并保存，第 8.3 节 |
| 串口权限不足 | 确认 `dialout` 和当前登录会话，不能只改用户组后继续用旧桌面 |
| 相机枚举但不能读取 | 先排除设备占用，再检查 RSUSB 权限/缓冲、USB 链路及实际身份 |
| 触觉或图像显示“历史保持” | 表示当前数据不再新鲜；检查源是否持续输出，不把缓存当作实时 |
| 录制完成但有缺流/丢帧告警 | 检查冻结计划、逐 Topic 频率和日志；已有数据仍可在档案内回放 |

完整诊断见[中文故障排查](troubleshooting.zh-CN.md)及[工作台故障处理](../tools/capture_console/README.md#故障处理)。

维护此指南时同步更新[英文版](deployment.md)。以上安装命令已经与源码入口及官方依赖说明核对；新电脑的实际安装、图形显示和硬件性能需要按本指南完成现场验收。
