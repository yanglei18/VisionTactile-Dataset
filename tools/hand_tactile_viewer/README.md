# 单手三维展示（左手版与右手版）

[English](README.en.md) · [安装后的中英使用说明](windows/USER_GUIDE.txt) · [仓库首页](../../README.zh-CN.md)

左手版和右手版分别从一副手套的两个串口读取姿态与触觉，在对应的动态 MANO 模型上显示。左右手的 **0.2.4** 共用三维标定、姿态解算、短时缺帧保形和触觉显示。两款程序可同时安装，各自保存端口、标定和触觉归零参考。

| 手别 | Windows x64 离线安装包 | 开始菜单 | Python 命令入口 |
| --- | --- | --- | --- |
| 左手 | `VisionLeftHandDisplay-0.2.4-windows-x64-setup.exe` | 左手三维展示 | `vt-left-hand-display` |
| 右手 | `VisionRightHandDisplay-0.2.4-windows-x64-setup.exe` | 右手三维展示 | `vt-right-hand-display` |

## 安装与使用

选择对应手套的安装包。支持 Windows 10 1809 及以上和 Windows 11 x64，需要桌面 OpenGL。完整安装包已携带运行环境、模型及布局文件。安装后按如下步骤操作。
1. 插入姿态手套和触觉板，串口列表每秒在后台自动更新，也可点击**刷新**立即扫描。更新保留已选端口和手动输入；操作下拉列表时，更新会等待选择完成。
2. 点击**识别类型**，列表会依据收到的数据标注**动捕／触觉／未识别**。分别选择对应端口，点击各自的**连接**。可单独连接任一路。启动和刷新不会自动打开串口；两路必须使用不同端口，也不能被其他串口程序占用。
3. 使用手势时展开**手势标定**，完成下述三步采集；仅查看触觉时可直接使用固定参考手形。
4. 触觉仅使用新版协议的设备参考或经验证的通用参考。未配套参考时，展开**运行详情**可直接查看原始 20×20 ADC；需要相对响应颜色时采集一次无载参考。已有匹配参考时无需重复归零。
5. 拖动旋转，滚轮缩放；**掌面／手背／复位**切换视角。**界面语言**即时切换并保存，不断开连接。
6. 结束时关闭窗口，等待串口和计算进程释放。安装目录的 `使用说明.txt` 包含完整中英操作与排障说明。

触觉波特率默认为 `115200`，可在**显示设置**修改。识别只在点击后短时读取空闲串口，不发送固件命令；正在使用的端口跳过。类型需要连续帧边界和数据协议一致，不能仅凭已连接或零散解析成功就确定。已连接的未知端口需先断开再识别。数据不足、占用或读取失败会分别显示，不凭“STM32”等名称猜测。识别期间仍可断开已有连接，新的连接需等待识别完成。

类型依据实际协议，没有硬编码板卡序列号；USB 身份用于在换板时清除旧结果。没有稳定身份的设备显示“本次识别”，更换设备后应重新识别，程序不会据此阻止手动连接。结果只在本次运行使用；识别类型不代表标定或数据质量验收。Windows 打开串口仍经过驱动初始化。无法识别时仍可分别拔插确认端口；USB 驱动由设备厂商提供。

上述自动刷新和类型标注属于当前源码更新，之前已导出的 `0.1.0` 安装包未包含这些更新，需要重新构建后交付。

**本分支仅支持新版 44 字节整行协议，不读取旧 26 字节数据或旧触觉参考配置。** 格式为 `A5 5A | uint16 大端行号 0..19 | 20 个 uint16 大端 ADC`，连续 20 行构成 400 点矩阵。连接时先确认连续帧边界，之后才输出数据；每行保存实际主机接收时间。矩阵正确不等于手部物理位置已验证，首次使用新布线需核对掌心和五指触点。没有校验和、硬件时间戳或序号，程序不宣称能检测全部传输损坏。

运行详情同时显示原始 ADC 和参考电路公式的电阻估计。电阻不是压力；换算不可用不影响合法 ADC。旧版默认参考不会自动迁移。

## 手势标定与显示含义

戴好手套并停稳后，依次点击对应的姿势按钮开始采集。左侧显示动作示范，右侧标出当前手形的来源。查看三个按钮下方的“手势标定进度”，准备和采集分别计时。看到勾选标记后即可进行下一步。

| 步骤 | 动作 |
| --- | --- |
| 1. 平掌 | 掌心朝下，五指伸直，保持静止 |
| 2. 指尖上倾 | 保持五指伸直，转动整掌，使指尖向上倾约 50° |
| 3. 指尖下倾 | 保持五指伸直，沿同一转轴，使指尖向下倾约 50° |

右侧“我的手势”会标明数据来源。第一步采集前显示固定参考手形。采到平掌后可显示偏离平掌的角度；再采到有效上倾参考后，掌部倾转可驱动整只参考手，手指仍保持参考姿势。三步标定生效后切换到实时解算的手形。

三个勾表示三段姿势已经采集，随后程序会计算基础标定。如果部分传感器需要补采，界面会列出仍可标定的手指。只要已有可用的手指关节，就可以先点 **4. 开始手指弯曲标定**；要补齐其余关节时，再单独重采提示的姿势。先伸直五指并保持稳定，再随左侧示范一起缓慢弯伸食指至小指约 8 秒，再按提示单独弯伸拇指末节约 8 秒。程序只安排本轮有可用关节的动作组。右侧显示当前有效的手形。结束时界面会区分全部完成和仍需补采的手指。**取消采集**结束当前采集，**重新标定**清除当前手势标定。

若倾转不足，按界面点名的姿势补采；若某指 IMU 与掌心运动不一致，先检查该传感器是否戴牢。运行中如提示局部数据缺失或参考失效，该处关节暂不可测。收到重新标定提示后再采集。手处于静止且输入健康时，点“重新标定”可保留同一连接内已学好的陀螺零偏。手势的三分钟稳定性需用实际佩戴数据验收。

| 画面状态 | 含义 |
| --- | --- |
| 姿态串口未连接 | 显示固定参考手形。单独连接触觉时，有效触点仍显示在参考手形上 |
| 等待 IMU 数据 | 姿态串口已打开，参考手形保持静止。如果提示持续存在，核对所选的动捕串口 |
| 等待手势标定／正在标定 | 参考手形保持可见。收到 IMU 数据后按提示完成三步采集，标定生效并收到有效姿态后自动切换为实时手形 |
| 实时手势与触觉 | 动态手形使用各行最新的有效触觉读数。按压和松开都随触觉更新显示 |
| 等待触觉 | 保留有效手势，暂不叠加旧触觉颜色 |
| 保留上一帧手形 · 等待姿态更新 | 掌根数据短时延迟时，保持上一帧实测手形和掌背配色。触觉继续按自己的采样时间更新。自上一帧姿态的源时间起最多保留 0.5 秒 |
| 手势数据已过期／手势模型暂不可用 | 切回固定参考手形。有效姿态恢复后自动切回实时手形 |

等待 IMU 数据、标定或姿态恢复时，触觉可独立显示在固定参考手形上。某个触点的数据过期后，该点的颜色会清除。触觉参考失效时，所有触觉颜色都会清除。

“有效读数 400/400 · 有响应 5 点”表示全部 400 个触点都有可用读数，其中 5 个超过了响应门槛。手指姿态缺测时，界面单独提示关节状态，该指的有效触觉仍会显示。

标定生效后，实测手形与动作示范使用相同的底色，掌心为浅青绿，手背为较深的青绿。固定参考手形的底色是灰色。按压只改变接触区域的颜色，表示扣除参考并经过噪声门槛处理的**相对 ADC 响应**：轻压渐变为浅黄，更强时变橙，满量程为红色。松开到零时恢复手形底色。一帧中局部缺少有效读数时，实测手的对应区域为暗蓝色，参考手则为浅灰；未映射表面保留底色。**显示设置**中的默认显示范围为 0–3000 ΔADC，可调整上限和响应方向。连接设备时，已有匹配参考的显示范围优先。

如需改善新板或重新佩戴后的无载偏差，点击**触觉归零（可选）**，保持触觉区域不受压约 3 秒。具有稳定 USB 身份时，新参考自动保存到本应用；无唯一身份时只在当前连接使用。触觉归零与手势标定彼此独立，原触觉工具中单独保存的参考不会自动复制到本应用。

手势与触觉各自更新；界面的“两路时差”是这两次采样到达电脑的时间差绝对值。本版本不包含 Tracker 世界定位，也不根据真实指尖接触自动约束虚拟指尖位置；局部不可用关节的保形显示不代表准确测量。GUI 的约 60 Hz 刷新目标不等于设备采样或全掌扫描频率。

## 左手 IMU 与触觉布局

左手版渲染原生 MANO_LEFT。下表给出输入文件中的原始编号与模型对应关系。[左手配置](../fsglove12/src/vt_fsglove12/configs/left_from_right_initial.json)

| 部位 | 靠指尖的 IMU | 靠手掌的 IMU |
| --- | --- | --- |
| 拇指 | 9 末节 | 10 近节 |
| 食指 | 7 中节 | 8 近节 |
| 中指 | 5 中节 | 6 近节 |
| 无名指 | 3 中节 | 4 近节 |
| 小指 | 1 中节 | 2 近节 |

掌心是 11 号，腕部 12 号作为辅助原始输入。长指的节段分配、104 Hz 名义采样频率及输入量程仍需按实际左手硬件核实。初次佩戴或改变传感器位置时，用左手做本页的三姿势标定。

左手触觉使用 44 字节整行协议，20×20 原始地址按照下表解释。此布局来自双手工具的左手实机方向反馈。[左手布局](../../ros2_ws/src/vt_tactile_capture/vt_tactile_capture/layouts/left_glove_from_right_initial.json)

| 原始地址方向 | 左手上的方向 |
| --- | --- |
| 行号减小 | 朝指尖 |
| 列号增大 | 朝拇指 |

解剖坐标为 `(19 - 行号, 列号)`。连接后依次轻按掌心与五个指腹，核对同名区域是否亮起。400 点均有模型绑定，区域边界与逐点皮肤位置仍为候选。实际佩戴后的三分钟姿态稳定性需要在左手上确认。

## 右手硬件配置

运行资源准备时明确传入现有 [`right_new_imu.json`](../fsglove12/src/vt_fsglove12/configs/right_new_imu.json)，继续使用已有 FSGlove 解算与功能标定，不新增一套算法。

| 位置 | 原始 IMU 编号 |
| --- | --- |
| 拇指 | 1、2 |
| 食指／中指／无名指／小拇指 | 3、4 ／ 5、6 ／ 7、8 ／ 9、10 |
| 掌心 | 11 |
| 手腕外部 | 12，保留原始输入，不映射进手指模型 |

3、5、7、9 号按用户确认安装在相应手指的**中间节**；本工具保留当前配置的模型映射。该配置当前包含 `104 Hz` 输入／输出目标、`0.035 °/s/LSB` 陀螺换算和 `0.000244 g/LSB` 加速度换算，验证状态仍以配置内的记录为准，不将配置值等同于另一副手套的实测结果。量程、布局或佩戴变化需要匹配配置并重新标定。

## 维护者：环境与资源准备

左手源码使用本工作树内的 `.venv-left-hand`。按本节右手示例用左手解释器安装相同的五个包。准备左手资源时，模型目录须含 `models/MANO_LEFT.pkl`，输出目录须尚不存在。

```bash
.venv-left-hand/bin/python -m vt_hand_tactile_viewer.prepare \
  --output .runtime/left-hand --mano-assets /path/to/authorized/mano-assets \
  --fsglove-source /path/to/FSGlove \
  --config tools/fsglove12/src/vt_fsglove12/configs/left_from_right_initial.json \
  --tactile-layout ros2_ws/src/vt_tactile_capture/vt_tactile_capture/layouts/left_glove_from_right_initial.json \
  --license-file /path/to/private/MODEL_LICENSE.txt
tools/hand_tactile_viewer/start_left_hand.sh --self-check /tmp/left-hand-check.json
tools/hand_tactile_viewer/start_left_hand.sh
```

左手偏好位于用户配置目录的 `visiontactile/left-hand-display-row44.ini`，标定和归零参考在相邻的 `left-hand-display-row44/` 中。左右手窗口同时运行时，各自连接自己的两个串口。

普通用户不执行本节。维护者需要 Python 3.12，以及**同一个环境**中的 Qt/OpenGL、串口、NumPy/SciPy/VQF、PyTorch CPU、manotorch、chumpy、FSGlove 核心依赖和 QMT 功能标定依赖。仅安装本目录的基础 Python 包不足以准备动态模型运行环境。

Windows 完整依赖固定在 [`windows/requirements-build.lock`](windows/requirements-build.lock)，包含传递依赖与 SHA-256；本地构建的上游 wheel 也必须匹配锁文件。Windows 环境由下节构建脚本整体准备。Linux 源码运行应在完整 MANO／FSGlove 数值环境中加入 PySide6 与共享界面包，原数值环境说明见 [FSGlove 手册](../fsglove12/README.md)及其 `requirements-*-tested.txt`；这些直接依赖清单不能作为 Windows 锁文件的替代品。

左右手从同一份源码开发和构建。在此工作树创建 `.venv-right-hand` 并安装下列五个项目包。右手启动器读取 `.runtime/oct29-right`，左手启动器读取 `.runtime/left-hand`。右手可沿用 10 月 29 日的完整资源包。

```bash
tools/hand_tactile_viewer/start_right_hand.sh
tools/hand_tactile_viewer/start_right_hand.sh --self-check /tmp/right-hand-check.json
```

启动器检查项目包的实际导入路径。右手版在用户配置目录的 `visiontactile/right-hand-display-row44.ini` 保存偏好，在同级 `right-hand-display-row44/` 保存标定与触觉参考。双手版沿用 `hand-motion-touch`。两个窗口可以同时打开；使用同一手套时先在占用方断开串口，再在另一窗口连接。修改共用功能后，用两个启动器分别执行自检。硬件变更只更新对应手的 IMU 配置、MANO 模型或触觉布局。串口识别能区分动捕板和触觉板，佩戴的左右手由操作者选择。

在已准备好上述数值环境的仓库根目录，安装本地五个包：

```bash
.venv-right-hand/bin/python -m pip install -e ros2_ws/src/vt_capture_core \
  -e ros2_ws/src/vt_tactile_capture -e tools/fsglove12 \
  -e tools/capture_console -e tools/hand_tactile_viewer
.venv-right-hand/bin/python -m pip check
```

需要维护人员明确提供：获准使用的 `models/MANO_RIGHT.pkl` 所在资源目录、兼容的 FSGlove checkout、当前右手配置以及模型许可说明；新版通用触觉参考可选。动态模型资源保持私有，不提交公共 Git，也不从开发机其他目录隐式搜集。模型许可说明用于记录来源，不自动授予再分发权。

```bash
python -m vt_hand_tactile_viewer.prepare \
  --output /path/to/private/hand-motion-runtime \
  --mano-assets /path/to/authorized/mano-assets \
  --fsglove-source /path/to/FSGlove \
  --config tools/fsglove12/src/vt_fsglove12/configs/right_new_imu.json \
  --license-file /path/to/private/MODEL_LICENSE.txt
```

如已有经过验证的新版参考，可另加 `--response-profile /path/to/private/default_response.json`；没有参考也能准备资源和读取 ADC。

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

左手构建时在同一个 PowerShell 命令中传入 `-Hand left` 和左手的 `-Resources` 目录。输出位于 `dist/left-hand-display-windows/release/`，程序名为 `VisionLeftHandDisplay.exe`。

右手 0.2.4 的默认交付目录为 `dist/right-hand-display-windows/release/`：

- `VisionRightHandDisplay-0.2.4-windows-x64-setup.exe`：独立离线安装包。
- 同名 `.exe.sha256` 与 `.manifest.json`：校验值和构建记录。

主程序名为 `VisionRightHandDisplay.exe`，AppId 为 `VisionTactile.RightHandDisplay.Windows.x64`，与原 `VisionTactile3D` 不同。安装到当前用户目录，不覆盖原触觉工具。设置与触觉参考独立保存，卸载保留用户数据。默认关闭连续原始／位姿流录制，只保留标定必要证据、结果与日志。

## 开发与验证

`window.py` 管理操作和简洁双语界面；`composition.py` 只关联公开的手势／触觉快照，负责会话、新鲜度与时间有效性；`resources.py` 校验显式资源；`app.py` 组合现有进程控制器、触觉数据源和渲染器。算法、串口读取和触觉参考逻辑继续维护在共享模块中。

```bash
QT_QPA_PLATFORM=offscreen python -m pytest -q tools/hand_tactile_viewer/tests
```

测试覆盖双串口互斥、实际操作回执、断流／重连边界、显示关联、语言切换、关闭释放与构建输入。离线测试、自检和 Wine 运行不能代替 Windows 真机验收；交付前仍需在目标电脑验证双串口、三步及弯曲标定、触觉响应、显卡显示、断线重连和持续运行。
