# 手势与触觉可视化

[English](README.en.md) · [安装后的中英使用说明](windows/USER_GUIDE.txt) · [仓库首页](../../README.zh-CN.md)

独立应用 **0.1.0**：从姿态手套和触觉设备的两个串口读取数据，在**同一个动态右手模型**上展示手势与触觉。界面沿用主程序主题，支持中文、English。原[固定手形触觉工具 0.6.2](../tactile_viewer_windows/README.md)保留，两款应用有独立安装身份、入口和用户设置。

## 普通用户：安装与使用

使用交付的 `VisionHandMotionTouch-0.1.0-windows-x64-setup.exe`，安装后打开“手势与触觉可视化”。完整安装包包含运行环境与配套资源，不要求安装 Python、ROS、主采集程序或填写模型路径。目标系统为 Windows 10 1809 及以上、Windows 11 x64，需可用的桌面 OpenGL。公开仓库提供源码与构建流程；模型与配套参考由维护人员按交付范围准备。

1. 插入姿态手套和触觉板，串口列表每秒在后台自动更新，也可点击**刷新**立即扫描。更新保留已选端口和手动输入；操作下拉列表时，更新会等待选择完成。
2. 点击**识别类型**，列表会依据收到的数据标注**动捕／触觉／未识别**。分别选择对应端口，点击各自的**连接**。可单独连接任一路。启动和刷新不会自动打开串口；两路必须使用不同端口，也不能被其他串口程序占用。
3. 使用手势时展开**手势标定**，完成下述三步采集；仅查看触觉时可直接使用固定参考手形。
4. 触觉默认使用配套通用参考，优先恢复本机为该设备保存的参考。相同协议、相同右手 20×20 地址布局的板卡换电脑或换板不强制归零。
5. 拖动旋转，滚轮缩放；**掌面／手背／复位**切换视角。**界面语言**即时切换并保存，不断开连接。
6. 结束时关闭窗口，等待串口和计算进程释放。安装目录的 `使用说明.txt` 包含完整中英操作与排障说明。

触觉波特率默认为 `115200`，可在**显示设置**修改。识别只在点击后短时读取空闲串口，不发送固件命令；正在使用的端口跳过。类型需要连续帧边界和数据协议一致，不能仅凭已连接或零散解析成功就确定。已连接的未知端口需先断开再识别。数据不足、占用或读取失败会分别显示，不凭“STM32”等名称猜测。识别期间仍可断开已有连接，新的连接需等待识别完成。

类型依据实际协议，没有硬编码板卡序列号；USB 身份用于在换板时清除旧结果。没有稳定身份的设备显示“本次识别”，更换设备后应重新识别，程序不会据此阻止手动连接。结果只在本次运行使用；识别类型不代表标定或数据质量验收。Windows 打开串口仍经过驱动初始化。无法识别时仍可分别拔插确认端口；USB 驱动由设备厂商提供。

上述自动刷新和类型标注属于当前源码更新，之前已导出的 `0.1.0` 安装包未包含这些更新，需要重新构建后交付。

**当前兼容范围：触觉采集仍使用现有的 26 字节协议，尚未适配实测的 44 字节整行协议。** 串口能打开、能收到字节，不等于数据已正确解析；未识别设备即使出现零散颜色，也不能据此确认触点数据有效。44 字节设备需要完成协议解析和触点排列核对后才能用于正常触觉显示。当前版本按源码开发预览交付。

## 手势标定与显示含义

戴好并固定右手手套，按顺序选择姿势，停稳后点击**采集当前姿势**：

| 步骤 | 动作 |
| --- | --- |
| 1. 平掌 | 掌心朝下，五指伸直，保持静止 |
| 2. 指尖上倾 | 保持五指伸直，转动整掌，使指尖向上倾约 50° |
| 3. 指尖下倾 | 保持五指伸直，沿同一转轴，使指尖向下倾约 50° |

三步完成后自动计算。等待“基础标定已生效”，再观察动作。可选的**全手弯曲校准**沿用现有功能标定：按提示先伸直，再缓慢弯曲五指。界面显示实际执行结果；“请求已提交”与“部分手指已更新”不等同于全手成功。**取消采集**结束当前采集，**重新标定**清除当前手势标定；更换佩戴位置后应重新采集。

| 画面状态 | 含义 |
| --- | --- |
| 固定参考手形 · 非实时手势 | 姿态源未连接；可在固定手形上查看触觉 |
| 等待手势标定 | 姿态源已连接，需要完成三步采集 |
| 实时手势与触觉 | 当前有效手势与时间相近的触觉在同一模型上显示 |
| 等待触觉／等待时间对应的触觉 | 保留有效手势，暂不叠加旧触觉颜色 |
| 手势数据已过期／手势模型暂不可用 | 等待有效手势，不把历史模型冒充实时数据 |

颜色表示扣除参考并经过噪声门槛处理的**相对 ADC 响应**，从浅绿平滑过渡到深红，不是压力单位。无有效数据或参考的触点显示灰色，未映射表面保留底色。**显示设置**提供从 0 开始的固定范围和响应方向，不按当前按压力度自动缩放。

如需改善新板或重新佩戴后的无载偏差，点击**触觉归零（可选）**，保持触觉区域不受压约 3 秒。具有稳定 USB 身份时，新参考自动保存到本应用；无唯一身份时只在当前连接使用。触觉归零与手势标定彼此独立，原触觉工具中单独保存的参考不会自动复制到本应用。

两路按电脑接收时间关联显示，属于软件显示关联，**不是硬件同步**。本版本不包含 Tracker 世界定位，也不根据真实指尖接触自动约束虚拟指尖位置；局部不可用关节的保形显示不代表准确测量。GUI 的约 60 Hz 刷新目标不等于设备采样或全掌扫描频率。

## 当前硬件配置

运行资源准备时明确传入现有 [`right_new_imu.json`](../fsglove12/src/vt_fsglove12/configs/right_new_imu.json)，继续使用已有 FSGlove 解算与功能标定，不新增一套算法。

| 位置 | 原始 IMU 编号 |
| --- | --- |
| 拇指 | 1、2 |
| 食指／中指／无名指／小拇指 | 3、4 ／ 5、6 ／ 7、8 ／ 9、10 |
| 掌心 | 11 |
| 手腕外部 | 12，保留原始输入，不映射进手指模型 |

3、5、7、9 号按用户确认安装在相应手指的**中间节**；本工具保留当前配置的模型映射。该配置当前包含 `104 Hz` 输入／输出目标、`0.035 °/s/LSB` 陀螺换算和 `0.000244 g/LSB` 加速度换算，验证状态仍以配置内的记录为准，不将配置值等同于另一副手套的实测结果。量程、布局或佩戴变化需要匹配配置并重新标定。

## 维护者：环境与资源准备

普通用户不执行本节。维护者需要 Python 3.12，以及**同一个环境**中的 Qt/OpenGL、串口、NumPy/SciPy/VQF、PyTorch CPU、manotorch、chumpy、FSGlove 核心依赖和 QMT 功能标定依赖。仅安装本目录的基础 Python 包不足以准备动态模型运行环境。

Windows 完整依赖固定在 [`windows/requirements-build.lock`](windows/requirements-build.lock)，包含传递依赖与 SHA-256；本地构建的上游 wheel 也必须匹配锁文件。Windows 环境由下节构建脚本整体准备。Linux 源码运行应在完整 MANO／FSGlove 数值环境中加入 PySide6 与共享界面包，原数值环境说明见 [FSGlove 手册](../fsglove12/README.md)及其 `requirements-*-tested.txt`；这些直接依赖清单不能作为 Windows 锁文件的替代品。

在已准备好上述数值环境的仓库根目录，安装本地五个包：

```bash
python -m pip install -e ros2_ws/src/vt_capture_core \
  -e ros2_ws/src/vt_tactile_capture -e tools/fsglove12 \
  -e tools/capture_console -e tools/hand_tactile_viewer
python -m pip check
```

需要维护人员明确提供：获准使用的 `models/MANO_RIGHT.pkl` 所在资源目录、兼容的 FSGlove checkout、当前右手配置、通用触觉参考以及模型许可说明。动态模型资源保持私有，不提交公共 Git，也不从开发机其他目录隐式搜集。模型许可说明用于记录来源，不自动授予再分发权。

```bash
python -m vt_hand_tactile_viewer.prepare \
  --output /path/to/private/hand-motion-runtime \
  --mano-assets /path/to/authorized/mano-assets \
  --fsglove-source /path/to/FSGlove \
  --config tools/fsglove12/src/vt_fsglove12/configs/right_new_imu.json \
  --response-profile /path/to/private/default_response.json \
  --license-file /path/to/private/MODEL_LICENSE.txt
```

输出目录必须不存在。准备过程从真实模型生成中性几何、模型轴和静态参考，携带动态 MANO 模型、必要上游文件、配置与触觉参考，并生成校验清单。运行和构建均校验资源，不依赖原电脑的绝对路径。

```bash
python -m vt_hand_tactile_viewer --resources /path/to/private/hand-motion-runtime
python -m vt_hand_tactile_viewer --resources /path/to/private/hand-motion-runtime \
  --self-check /path/to/check-report.json
```

可加 `--language zh_CN` 或 `--language en` 设置并保存应用语言。自检加载真实模型、触觉映射与计算子进程，确认能初始化和正常关闭；它不连接串口，也不验证桌面 OpenGL 上下文或硬件效果。

## Windows 构建与交付

构建机需要 Windows x64 Python 3.12 和 Inno Setup 6.7.3。构建使用完整 wheelhouse 与上面显式准备的资源；不要求最终用户具备这些工具。

在仓库根目录的 PowerShell 执行：

```powershell
py -3.12 tools\hand_tactile_viewer\windows\prepare_dependencies.py `
  --output C:\build\wheels --sources C:\build\sources

powershell -NoProfile -ExecutionPolicy Bypass `
  -File .\tools\hand_tactile_viewer\windows\build.ps1 `
  -Resources C:\private\hand-motion-runtime -Wheelhouse C:\build\wheels
```

依赖准备首次需联网获取固定的公开源和 Windows wheel，亦可在 Linux Python 3.12 中执行；来源、校验与重建方式见[完整依赖准备说明](windows/DEPENDENCIES.md)。该步骤不获取模型，也不要求 C++ 编译器。`--wheelhouse` 使正式构建仅从本地安装锁定依赖。构建脚本建立隔离环境、执行 `pip check`、源程序与冻结程序自检、打包审计，然后生成安装包。`-Output` 可指定输出目录，`-Iscc` 可指定非默认位置的编译器。

默认发布目录为 `dist/hand-tactile-viewer-windows/release/`：

- `VisionHandMotionTouch-0.1.0-windows-x64-setup.exe`：独立离线安装包。
- 同名 `.exe.sha256` 与 `.manifest.json`：校验值和构建记录。

主程序名为 `VisionHandMotionTouch.exe`，AppId 为 `VisionTactile.HandMotionTouch.Windows.x64`，与原 `VisionTactile3D` 不同。安装到当前用户目录，不覆盖原触觉工具。设置与触觉参考独立保存，卸载保留用户数据。默认关闭连续原始／位姿流录制，只保留标定必要证据、结果与日志。

## 开发与验证

`window.py` 管理操作和简洁双语界面；`composition.py` 只关联公开的手势／触觉快照，负责会话、新鲜度与时间有效性；`resources.py` 校验显式资源；`app.py` 组合现有进程控制器、触觉数据源和渲染器。算法、串口读取和触觉参考逻辑继续维护在共享模块中。

```bash
QT_QPA_PLATFORM=offscreen python -m pytest -q tools/hand_tactile_viewer/tests
```

测试覆盖双串口互斥、实际操作回执、断流／重连边界、显示关联、语言切换、关闭释放与构建输入。离线测试、自检和 Wine 运行不能代替 Windows 真机验收；交付前仍需在目标电脑验证双串口、三步及弯曲标定、触觉响应、显卡显示、断线重连和持续运行。
