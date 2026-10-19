# Vision Tactile 集成采集工作台

本工具提供独立的桌面工作台，用于管理相机、Tracker、左右手姿态和触觉采集。当前为 **v0.5 开发预览版**：增加桌面启动入口、图形化设备配置、Tracker 手动初始化向导、外参标定窗口入口和离线对齐窗口。界面参考 [SpaceX Human Spaceflight](https://www.spacex.com/humanspaceflight/overview) 的连续大画面、尺度对比和动态叙事，重构为 Immersive Studio 工作区：顶部采用设备、预览、空间、录制、档案五个主题标签；录制表单、进度与操作集中在独立录制页，预览和空间页面使用完整画布。主题仅使用纯黑、白色与中性灰，数据画面保留相机原色、坐标轴与实体分类色及触觉 ADC 色标。原生绘制的灰度光影、页面过渡与三维巡航均可通过顶部“动效”开关减少；采集、解算和标定算法继续复用原有工具。主题、图标和程序化合成演示随安装包提供，无需联网加载。

## 独立触觉三维小窗口

右手触觉工具直接读取触觉串口，**不需要启动主程序、ROS、采集后台或 IMU**。
它直接复用主采集程序的 `PreviewWidget` 触觉三维预览、协议解析和主界面主题；运行时只拥有
用户明确选择的一个触觉串口，不发布 ROS Topic、不录制数据。

在本机仓库根目录运行，或使用桌面的“触觉三维预览”入口：

```bash
.venv-capture-console/bin/python -I -m vt_capture_console.tactile_app
```

重新安装本包后也可运行 `vt-tactile-3d`。Python 环境需要本包及其 Python
依赖（包含 pyserial），不需要构建或 source ROS 工作空间。默认开发方式通过
单独的模型 Python 环境加载 `vt-fsglove12`、torch、manotorch 和可信 MANO 资源。

**Windows 安装版**的构建入口、固定依赖和验收方法见
[Windows 安装包工程](../tactile_viewer_windows/README.md)。安装版自带运行依赖和
明确提供的静态模型资源，用户无需配置 Python 或模型路径；支持 COM 串口选择。
模型分发许可由发布者按实际使用范围准备，受限模型不随仓库提交。
构建由 Windows x64 Python 执行，Linux 维护者可在隔离 Wine 中运行同一工具链。
Linux 或 Wine 检查通过不代表 Windows 真机串口验收完成；最终用户只需取得 `.exe` 安装包。

窗口右上角提供 **中文 / English** 即时切换，保留串口连接、三维视角和当前触觉画面。
语言偏好仅保存在本工具的设置中；首次启动跟随系统语言（中文系统使用简体中文，其他系统使用英文）。
维护时可用 `--language zh_CN` 或 `--language en` 临时指定语言，不改写已保存的选择。

1. 点击 **刷新**，选择触觉板的串口，也可直接输入完整设备路径。
2. 确认波特率（默认 **115200**），点击 **连接**。启动窗口、刷新列表及
   `--port` 参数都不会自动打开串口；不能只凭 STM32 名称区分触觉板与 IMU 板。
3. 接收到触觉数据后，直接显示原始 ADC，与主采集程序的触觉三维预览一致。
4. 点击 **断开**释放端口。端口被其他程序占用时会提示原因，不抢占、不结束
   其他程序；拔插或读取错误后需手动重新连接，不自动重连。

**掌面 / 手背 / 复位**切换视角；拖动旋转、滚轮缩放。颜色映射、固定表面绑定、
原始 ADC 插值、有效掩码和历史状态均使用主程序原有实现。颜色表示原始 ADC，
不是压力；灰色为无效采样，3D 未映射部位保留模型底色。

色标按首次有效数据的最小值和最大值确定（最小跨度 100 ADC），此后保持固定，
范围显示在预览下方。重新连接会清除前一连接的画面和范围，再从新数据确定色标。
小工具不提供额外归零、差值显示或灵敏度参数；早期实验参数
`--change-max`、`--adc-min`、`--adc-max` 已移除。
固定模型不跟随手势，表面映射沿用右手候选
布局：9 个锚点有观测依据，其余位置仍未逐点标定；当前不自动镜像为左手。

状态行显示有效地址数、送入显示的不同源快照频率和主机源数据年龄，频率不是
硬件扫描频率。触觉矩阵逐块更新，不代表 400 点同步采样。每块显示有效期为
100 ms；断流、无效样本或断开时，按主程序规则保留最后有效画面，并明确标记
“历史保持”，不把历史画面视为实时测量。重新连接时清除历史画面。
关闭窗口会停止本工具的读取线程并释放其串口。

开发方式的模型设置保存在本工具独立的 `visiontactile/tactile-3d.ini`，不读取或修改主程序
的设备配置。**配置设备** 中的 **手套 Python 环境**、**MANO 资源目录**
与主程序采用相同参数含义。资源目录须含 `models/MANO_RIGHT.pkl`，点击
**保存并加载** 应用本工具的模型配置。也可以临时通过参数指定：

```bash
vt-tactile-3d --port /dev/serial/by-id/你的触觉板 --baudrate 115200 \
  --glove-python /绝对路径/模型环境/bin/python --mano-assets /绝对路径/MANO资源
```

以上命令只预填端口，仍需点击“连接”。模型缺失时可在窗口内配置并重新加载。

也可用 `--model-bundle /绝对路径/right_hand.json` 直接加载预先导出的静态模型，
此时不启动模型子进程，也不要求模型 Python 环境。安装版自动定位内置 JSON；
两种加载方式共用相同的几何验证、触觉映射和渲染器。JSON 不能替代来源模型的授权。
维护人员可执行 `vt-tactile-3d --self-check /报告目录/install-check.json` 检查静态安装资源，
该命令不打开窗口或串口，不修改设置；实际显卡渲染和 USB 通讯仍需单独测试。
首次连接真实触觉板前，可使用伪终端验证协议与显示链路；合成验证不代表真实
传感器、逐点映射或压力精度已通过验收。

## 日常操作：从桌面开始

工作台支持 **中文 / English**。点击右上角 **中 / EN** 即时切换语言，当前语言以白字和细下划线标记，选择会在下次启动时保留。五个工作主题、配置与帮助弹窗、预览和空间提示、录制与质量检查界面均使用同一套本地翻译资源，无需联网。切换语言保留当前页面、输入草稿和已打开窗口，不发送设备启停或录制命令。

中文模式统一显示中文操作名称、内置设备类别、追踪器角色和常见后台状态说明，文件选择框使用相同语言。翻译仅作用于显示层；设备型号与身份、用户名称、文件路径、实际数据主题、保存的元数据和原始日志保留原值。尚未登记的第三方错误保留原文，便于核对诊断。

The workstation supports **Chinese and English**. Click **中** or **EN** at the top right; white text and a fine underline mark the current language. The interface updates immediately and remembers your choice. Your current page, open dialogs, recording state, and input drafts are preserved. Device identities, user-provided names, file paths, saved metadata, and raw device/tool logs keep their original values. Known backend diagnostics are translated for display; unrecognized third-party messages are shown as received.

Chinese mode uses Chinese action labels, built-in device categories, Tracker roles, and known backend status messages. File selectors follow the same language. Localization changes presentation only; device models and actual topic names remain unchanged as well.

也可以通过 `vt-capture-console --desktop --language en` 或 `vt-capture-console --demo --language en` 指定本次新窗口的语言；中文使用 `--language zh_CN`。工作台外部独立启动的原标定工具保留其自身的语言设置。

一次性部署完成后，在应用菜单或桌面双击 **Vision Tactile 采集工作台**。不用先打开终端、启动后台或手改 YAML。桌面入口会连接本工作台已有后台，或启动一个新的本机后台；**不会自动启动传感器、初始化 Tracker 或录制**。

第一次使用按下面的顺序操作：

1. 进入 **设备 → 配置设备**。只勾选需要的模块；未使用的左手、触觉或 Tracker 可以保持禁用。
2. 在“相机”页选择已有相机配置和数据输出目录；在“Tracker”页选择已有启动 bundle、角色映射、扩展 PyVUT 源码与其 `site-packages` 目录。在“右手／左手”页选择各自的 IMU、触觉串口及手套模型、Python 和配置路径。**刷新串口列表只枚举端口，不会连接；不会猜测某个串口属于哪只手。**
3. 点击 **保存并应用**，等待提示配置已应用。它只更新并重启空闲管理后台，不会启动设备。已有工作台设备运行或录制未结束时，先停录、停止相关模块，再应用配置。
4. 回到设备页，选择模块，点击 **启动**。相机组启动三相机和录制控制器；触觉组启动指定手的采集节点；手套组同时打开原 FSGlove 标定窗口并启动 ROS 桥接。在原窗口中完成连接、归零和标定。
5. Tracker 尚未初始化时，先用 **标定与离线工具 → Tracker 初始化向导**；完成后点击 **启动读取并查看 Tracker**。已有连续有效位姿时，不需要为了使用工作台而重做初始化或配对。
6. 在“预览”查看所需数据源。然后进入 **录制**，创建或选择批次，填写试次名称、录制时长，点击 **开始录制**。等待结束或在录制页手动停止。
7. 进入 **档案**，选中试次并打开 **试次详情 / 质检**，查看实际 bag 路径、元信息和质量报告；通过“打开数据目录”访问文件。需要时间对齐时，使用 **标定与离线工具 → 离线检查与时间对齐**。
8. 结束工作时先停止录制，再逐组停止设备。只关闭工作台窗口不会自动关闭后台或传感器；重新打开会复用原本机后台。

“实时预览在线”与“被录入 bag”不是一回事。**录制 Topic 清单由相机页选中的采集配置决定**，界面筛选、勾选启动某个手套不会自动把对应 Topic 加入 bag。需要手势或触觉时，选择 `ros2_ws/src/vt_realsense_capture/config/cameras.with_*.yaml` 中与本次组合相符的现有配置。

### 采集计划与录前核对

连接支持 IPC v5 的管理后台后，录制页显示配置范围，悬停 Topic 提示可查看完整订阅清单。默认按可用数据录制：未连接 Tracker、未启动的相机或手部设备不阻止开始；至少有一路已监测的数据正在接收即可。配置范围不等于本次实际录入数量。Recorder 保留配置清单订阅，稍后上线的数据也会录入，不会自动订阅清单外的 ROS Topic。

点击开始时，后台核对计划、时长、控制器实例及清单指纹，将正在接收的数据流冻结为本次必需流，其余候选流记为可选，保存为 v2 采集计划。实际发送前再次复核；已经接收的数据在排队期间中断时，明确提示且不发送 START。正在接收的无效姿态、异常时间戳或畸形消息仍可保存，具体问题由预览和录后质检报告。

CameraInfo、机内外参和 frame timing 尚无实时观测，但有图像接收的相机会要求这些配套流，缺失时由录后检查提示。触觉可用性直接观察原始 `samples` 的到达，不以断连后仍可能发送的 `state` 心跳代替；运行时消息类型未验证。存储空间检查仍由实际写入目录的 Recorder 负责。

新默认 Recipe v2 每台相机订阅六路：彩色、原始深度、彩色内参、深度内参、深度→彩色机内外参和 frame timing。三相机加三 Tracker 共 21 个候选 Topic；保存的历史 Recipe v1 / `unified-dataset-v1` 计划仍是原四路相机合同，指纹不变。机内外参使用 reliable/transient_local/depth 1，晚启动的 Recorder 也能收到驱动保留的标定。

质检对未接入且没有数据的可选流只显示缺失数量和清单，不产生质量告警；已经接收的必需流缺失、实际数据丢帧或无效等问题仍告警。历史 v1 计划与无计划档案保留原来的检查依据，不回写历史报告。回放继续按包内实际内容播放。

### 帮助与演示设置

顶部采用 **设备、预览、空间、录制、档案** 五个主题标签。设备页管理连接与配置；预览和空间页分别检查实时数据与三维关系；录制页填写批次、试次、任务、操作员和时长，并执行开始或停止；档案页查看已有试次、数据路径、元信息与质量检查。切换页面不会中断录制，录制标签显示简短状态与计时；需要停止时点击标签返回录制页。右上角 **帮助** 打开“帮助／关于”窗口，集中展示操作说明、运行模式、LIVE 后台地址和关闭窗口对采集的影响；打开帮助保留当前页面，录制继续运行。原“工作站”页面已移除。演示模式的场景和单双手选择移至右上角 **DEMO** 菜单；实时模式只显示 LIVE 状态，不提供演示选项。设备配置、连接重试、日志及标定工具仍在 **设备**。

### GPU 渲染与预览性能

相机大画面在检测到硬件 OpenGL 时使用 GPU 缩放绘制，复用已经解码的 JPEG 图像；小缩略图保留轻量绘制。无硬件加速、无显示设备或图形上下文失败时自动回退，不影响当前帧与断流后的历史标注。GPU 绘制不提高预览本身的分辨率，也不改变采集或录制帧率。排查显卡兼容性时，可通过 `VT_CAMERA_RENDERER=raster bash tools/capture_deploy/start_workbench.sh` 对新窗口强制使用软件绘制。

空间视图角标显示实际渲染状态：**三维 · GPU**、**三维 · CPU**、**三维 · 待确认** 或 **投影 · CPU**。悬停可查看实际渲染器和版本；成功创建 OpenGL 上下文不等于启用 GPU，`llvmpipe` 等软件渲染器会明确显示 CPU。

背景动效将柔光缓存，轨道与星点按局部区域更新，避免每帧重算全屏渐变。动效开关仍独立于数据刷新：主窗口转入后台时暂停背景动效；正在前台查看的相机、Tracker、手势、触觉或空间独立窗口继续使用正常数据刷新周期。所有数据窗口转入后台后才降低共享轮询频率。

后台相机预览先降采样再计算深度对比度，避免为小预览处理完整深度图。深度采用输出像素中心的原始样本，无效样本保持黑色，细于预览像素的结构可能消失；精细检查以录制的原始图像为准。隐藏的三维手势视图暂停模型提交与网格上传，重新显示时补上最新帧；相同帧不重复上传几何。上述优化不修改原始图像、姿态消息或 bag 数据。

相机实时预览与在线状态使用相同的源时间容差：时间戳最多领先主机 500 ms 时，
仍可显示新收到的预览图；数据陈旧或断流时继续保留最后画面。这仅用于显示，
不修改录制时间戳或质量检查规则。“历史画面 · 已保持”表示当前未获得可用新画面，
不表示已切换到档案回放。

空间点云按每个窗口的实际需求运行：隐藏、最小化或冻结的窗口撤销自身需求，其他仍在查看的窗口继续获取数据。相机数据源退出或切换后释放对应点云对象，持续存在的相机复用对象；关闭窗口释放其显示资源。这只影响可视化，不启停相机或录制。

预览的 **接收 Hz** 是后台最近一秒收到的消息数，不能作为相机硬件输出或 bag 写入帧率。录制结束后在 **档案 → 试次详情 → 质量报告** 查看实际检查结果；未执行或执行失败均不表示质量合格。当前实时落盘吞吐、逐 Topic 写入丢帧统计仍未提供，详见[采集状态与指标说明](../../docs/capture-guide.md#8-observe-status)。

### 预览小图放大

“预览”默认收起横向数据流条，将空间留给当前画面。点击“浏览数据流”或选择模块筛选即可展开，选择相机、Tracker、手势和触觉；所选数据流显示在上方大画面，手势／触觉切换控件随之出现。收起缩略条保留当前选择和数据更新。点击“数据详情”可展开或收起右侧信息面板。每个缩略卡片右上角都有 **放大** 按钮。点击可打开独立的实时大图窗口，拖动窗口边缘调整尺寸，或使用标题栏的最大化按钮。重复点击同一卡片只切换到已有窗口；不同卡片可同时打开，便于对照观察。

相机小图也支持双击放大；RGB 和深度图保持原始宽高比。大图显示的是工作台已有的降采样预览，不会因此切换到相机原始分辨率。Tracker 大图支持拖动旋转、滚轮缩放、双击复位；手势和触觉大图保留各自的视图切换与交互。

小图和大图共用同一份数据快照。筛选隐藏小图后，已打开的大图仍持续更新；断流时保留最后画面并标为历史，不当作实时数据。关闭大图不停止传感器、不影响录包；设备移除或关闭工作台时，相应大图一起关闭。

独立大图获得焦点后仍保持默认 100 ms 更新周期，不随主窗口失焦降到 500 ms。相机预览本身仍为降采样 JPEG、每路最多 10 Hz；这与原始图像的 30 Hz 采集目标分别计量。

### 三维联合可视化

顶部 **空间** 页面提供大幅三维画布，可切换“全部实体 / Tracker / 相机”。右侧“场景面板”默认收起，展开后包含实体、图层、点云和数据四个标签页；点击画布实体可直接查看数据。相机缩略图通过“相机预览”展开。三个 Tracker 保留同一 `vive_map` 下的位置和姿态，不分别归零；已核验并载入刚性外参的相机显示机身、光学坐标轴与视锥。左腕为青色、右腕为紫色、躯干为橙色。相机绑定由配置决定。

- 首次接入场景时自动取景；后续数据更新保留观察视角。拖动旋转、Shift / 中键平移、滚轮缩放；使用俯视、前视、侧视和“自动取景”检查空间关系。
- “巡航视角”缓慢环绕场景，只改变观察视角；拖拽、缩放或选择标准视角会停止巡航。合成演示默认开启，LIVE 默认关闭；冻结、隐藏、最小化或关闭全局动效时暂停。机身外形为示意，位置和姿态来自相应数据源。
- Tracker 使用圆角设备外形、传感器孔和双面安装细节，细色带区分角色；方向标记指示局部 +X，示意外形不代表实际硬件标定。悬浮标牌显示角色与数据状态，点击标牌或设备可选中；运动轨迹由旧到新渐亮，位姿无效或断流时角色强调色灰化。
- 勾选设备与图层控制显示；点选实体查看坐标、四元数、源时间和状态。单位为米，网格是参考平面，未标定为真实地面。
- “冻结画面”只冻结本视图。“放大”打开独立窗口，共用后台数据，拥有独立视角与冻结状态；关闭窗口不会停止设备或录制。
- 默认是实时监看：各设备使用各自最近有效测量，不表示所有设备处于严格相同采样时刻。断流或无效位姿保留历史提示；未知地图身份不会被当作已核验的跨运行公共地图。
- 下方 RGB 小图复用已有相机预览，与三维刚体配合查看。

新安装默认启用 Tracker 空间显示。已有工作台使用 **空间 → 空间配置** 选择场景配置，通过现有“保存并应用”流程更新空闲后台。模板为 [`scene.example.yaml`](../../ros2_ws/src/vt_capture_manager/config/scene.example.yaml)。要显示真实相机，在本机副本中填写 `alignment_config_path`、`extrinsics_directory` 和每台相机的显式 `camera_device_info_services` 路径。相机角色、型号、序列号、三个消息 frame 和标定变换必须匹配；只读 DeviceInfo 服务需要与图像发布节点对应。无法核验身份时只保留二维预览，不猜测相机在公共空间中的位置。

演示模式含三个合成 Tracker 和三个合成相机，页面明确标注 DEMO，可在不连接硬件的情况下检查交互。真实模式不会使用演示位置替代缺失数据。


点云需要额外填写场景配置的 `depth_units` 并设置 `cloud.enabled: true`。推荐从[三 frame 示例](../multisensor_alignment/config/alignment.spatial.example.yaml)填写实机身份：color optical、depth optical 与 frame group 分开核对。RealSense ROS 的 Z16 `16UC1` 输入使用明确的 `realsense_ros_z16_mm` 合同，每单位 `0.001 m`；不得再乘设备原始 `depth_scale`。管理后台只读订阅驱动的 depth-to-color 外参，原始深度不会被当作已对齐到彩色图像。

- 勾选“点云”，可切换来源分色 / 真实 RGB，并调节点大小、每路显示点数和显示深度上限；这些显示筛选不改变采集或 bag 内容。
- 计算在独立系统 Python 进程进行，每路最多 50,000 点；关闭图层或隐藏所有空间窗口后释放显示需求，不停止传感器。
- 开启点云时使用图像帧组主机时间匹配 Tracker 位姿，默认相机容差 20 ms、Tracker 插值间隙最多 50 ms、最多等待 100 ms，不外推。缺某一路时显示部分结果和原因；这是软件时间匹配，不是硬件曝光同步。
- 目前支持常用无裁剪针孔 / Brown 模型。深度 rectified 图像使用 P，非单位 R、非零立体投影平移、特殊 binning / ROI 等不支持的几何会明确拒绝。RGB 投影越界区域使用灰色。

录制由工作台管理且开启空间后台时，会在原始 bag **旁边**生成 `<bag目录名>.spatial/`，包含独立 manifest、内外参内容和 SHA-256、几何版本、身份有效区间及显示坐标变换。空间侧文件不修改原始 bag 或录制清单。当前默认 21 个候选 Topic 已独立保存官方相机内参和机内外参，即使未开启空间后台也会录入；相机与 Tracker、其他相机或世界之间的安装外参仍需单独配置。侧文件 `complete` 表示已观察到录制终态，`geometry_status` 分别表示几何材料是否齐全，不能代替质量验收。

使用页面 **回放 → 打开目录** 选择已关闭的 bag，或选择已有离线对齐结果目录。提供播放、暂停、上一帧 / 下一帧、时间轴、速率和“返回实时”。原始 bag 的索引、图像解码与点云重建全部在系统 Python 子进程中执行；Qt 进程不导入 ROS。快照 JSON / 对齐 JSONL 只回放其中保存的位姿，不把图像引用当作像素。

历史 bag 没有有效几何侧文件时可查看记录的 Tracker 位姿；无法证明相机绑定、深度单位、内参或身份有效区间时不重建相机点云，也不借用当前机器标定。空间回放当前不显示三张原始 RGB 缩略图，真实 RGB 已用于点云着色；查看原始 RGB／深度请使用下述档案兼容回放。精确跨实体测距尚未提供。软件测试包含隔离 ROS 与真实 MCAP 序列化，但已完成本机 OpenGL 画面冒烟检查；真机空间精度、不同显卡兼容性和长时间采集性能仍需单独验收。

### 档案兼容回放

在 **档案** 选择已结束的试次，点击“回放所选试次”进入档案内的回放视图，
不另开回放窗口。“返回试次列表”暂停播放并回到档案列表。
话题列表显示包内实际 Topic、消息类型和数量；空话题和未录入的数据类别会明确提示。
质检中的“存在告警”不禁止播放：只有相机、只有手套、只有触觉，或缺少某个 Tracker
的包均可浏览已录入的数据。还可在档案页打开本机已关闭的 bag 目录或其会话目录。

- 回放默认进入“空间”，不设独立“预览”页。RGB／深度、Tracker、手部骨架和触觉矩阵
  可在“话题详情”中选择对应 Topic 查看；不会连接实时设备。
- “空间”复用采集时的三维组件及视角、图层、跟随和轨迹操作。只绘制记录中具有共同
  `frame_id` 的有效 Tracker 原始位姿；未知或相互不同的坐标系不强行合并。轨迹只覆盖
  本次连续播放经过的历史样本，切包或跳转后重置。当前兼容空间页尚未重建相机位姿、
  视锥与点云。顶部仅保留简短状态，详细说明可悬停查看，底部不再重复；
  坐标系缺失或不一致仍明确提示。原有严格空间回放的几何校验不变。
- “话题详情”保留单 Topic 预览及有界消息字段；没有本机消息类型支持时显示序列化摘要
  并说明无法解码，不阻断其他话题。“缺失与限制”汇总各数据流提示。
- 话题详情与空间共用播放、暂停、时间轴拖动和倍速，均使用 bag 记录时间；切换视图或
  Topic 保留同一播放位置，选择空 Topic 不影响已有空间数据。
  所选时刻之前尚无样本时显示无数据，不借用未来帧。停更话题显示最近历史样本及其时间，
  不宣称各话题硬件同步。触觉原始批次展示消息字段，不伪造完整手掌覆盖。
- 缺少标定仅限制三维空间重建，不限制原始图像／消息浏览。兼容回放不修改质量报告、
  补齐缺失数据或把历史测量作为实时消息发布。
- 已知正在录制或状态未确认的试次不能回放。原文件已移动、缺少 `metadata.yaml` 或
  存储文件损坏时显示实际原因，档案记录本身不能代替原始 bag。
- 解码在独立系统 Python 子进程中进行，使用已安装的 ROS 2 Jazzy 和工作区消息包。
  Qt 不导入 ROS；切换录制包、清空选择及退出工作台时释放只读回放资源。

两种回放共用请求关联和资源释放机制：定位超时后到达的旧响应会被丢弃，
首帧读取或校验失败也会释放解码进程。包目录、`metadata.yaml` 和存储文件的
路径链不能含符号链接，存储文件须为普通文件；元数据最多 4 MiB、存储清单
最多 4096 项，正常嵌套目录仍受支持。需要使用链接指向的包时，请将完整包
复制到普通目录后打开。读取期间请保持包已关闭且不被修改；独立解码进程
不等于操作系统沙箱，也不构成恶意输入的硬内存上限。

### 手形触觉预览

在“预览”选择“触觉”筛选，右手触觉卡片默认按手形展示 400 个有效区域地址。它直接复用[原触觉工具的手形布局](../../ros2_ws/src/vt_tactile_capture/README.md#31-手形视图与原始矩阵)，不需要启动手势解算或重新连接串口。

- 使用卡片上的“手形 / 矩阵 / 3D 手形”切换视图；矩阵仍是原始行列顺序的 20×20 网格。
- 点击“放大”打开实时大图。二维视图可悬停格点查看部位、原始行列和 ADC 数值；三维视图可拖动旋转、滚轮缩放，用“掌面 / 手背 / 复位”恢复观察方向，用“关节”显示或隐藏模型关节中心。切换视图或关闭大图不会启停采集。
- 颜色表示原始 ADC，不是压力；无效格点不当作零接触。断流时保留已显示画面并标识历史，不能把保留图像当作新数据。
- 左手及未指定手别的数据没有确认布局时，自动使用矩阵，并提示缺少映射；不套用或镜像右手布局。

“3D 手形”复用[手势标定工具](../fsglove12/README.md#在三维手模型上显示-ros-触觉)的 **MANO 模型、渲染器、掌背材质和固定触觉表面绑定**。它展示静态参考手上的实时触觉，**不随 IMU 弯曲，不表示当前手势或个体化手形**。无需启动手套姿态节点。三种视图使用同一份 ADC、有效掩码和固定色标；三维表面只做局部颜色插值，不改变原始数据。灰色表示对应表面没有有效采样；手背等未映射区域保留模型底色。当前布局包含 7 个区域、9 个实测锚点，其余空间位置仍为候选，并非逐点完成皮肤位置标定。

首次使用 3D 前，在 **设备 → 配置设备 → 右手** 设置原手势工具的 Python 环境与可信 MANO 资源目录（其中含 `models/MANO_RIGHT.pkl`），保存配置。该环境需已安装 `vt-fsglove12`、torch 和 manotorch；从旧版本升级时重新运行 `bash tools/capture_deploy/install.sh` 更新工作台显示依赖。许可模型不随仓库或安装脚本分发。

首次切到 3D 会在独立子进程生成一次中立网格，卡片与放大窗口共用内存缓存；不会连接串口、启动解算或重做标定。加载失败会显示具体原因，可在排除问题后点击“重新加载模型”，也可继续使用二维视图。需要可用的桌面 OpenGL 环境；无显示设备的 `offscreen` 测试模式不渲染三维网格。

本工作台暂不提供触觉差值参考采集、随手势运动的三维触觉或姿态–触觉时间配对。需要这些功能时使用[原触觉工具的参考采集](../../ros2_ws/src/vt_tactile_capture/README.md#32-采集不受压参考显示接触变化)或[手势窗口三维触觉](../fsglove12/README.md#在三维手模型上显示-ros-触觉)。

卡片和大图共享工作台快照更新，默认界面刷新上限为 10 Hz；这不改变触觉 `state` Topic 的发布频率，也不改变原始 ADC、时间戳和录包内容。

### 手势三维预览与标定

在“预览”选择“手势”筛选。真实模式下，左右手姿态卡片默认使用 **3D 手形**，也可切换 **骨架**，或点击 **放大**。三维显示复用原 FSGlove 窗口的 MANO 模型和渲染器，展示 `/gloves/right/state` 或 `/gloves/left/state` 已发布的解算结果，不运行第二套姿态估计算法。

1. 在 **设备 → 配置设备 → 对应手** 启用姿态模块，设置 IMU 串口、原手势工具 Python、MANO 资源和手套配置，保留打开原标定窗口的选项。右手使用 `MANO_RIGHT.pkl`，左手使用匹配的 `MANO_LEFT.pkl`；模型环境要求同前文三维触觉预览。
2. 点击姿态卡片或放大窗口中的 **标定**，也可使用菜单 **标定与离线工具 → 右手／左手姿态标定工具**。模块未启动时，按钮通过既有设备管理流程启动该手的 ROS 桥接和原标定窗口；模块已启动时，只尝试切换到已有窗口，不再启动第二个串口读取程序。X11 无法切换或使用其他桌面协议时，按提示用 Alt+Tab 切换。
3. 在原 FSGlove 窗口点击 **连接**，按[手势工具手册](../fsglove12/README.md)完成所需的参考采集和标定。启动 ROS 节点只启动桥接和窗口，**不等于已经连接 IMU，也不会自动归零、标定或清除旧标定**。原窗口显示 0/12 新鲜通道、0 累计样本时，应先确认连接和输入，工作台此时没有可显示的姿态。
4. 当对应 Topic 提供有效、新鲜的姿态时，工作台自动显示动态手形。拖动旋转、滚轮缩放，使用 **掌面 / 手背 / 复位 / 关节** 调整观察方式。卡片与放大窗口共享显示结果；切换视图或关闭放大窗口不停止采集。

工作台区分“未启动”“已启动 · 待结果”“姿态待就绪”和“姿态断流”；只有 Topic 状态不足以确认时，不猜测串口连接或具体标定步骤。出现等待状态时先点击“标定”查看原窗口，而不是反复启动节点。

三维网格由消息中的 48 维 `mano_pose_rotvec` 和 10 维 `shape_params` 重建，根部朝向只应用一次；坐标为各手腕部相对坐标，不是 Tracker 世界定位。模型计算在独立、持久化的 CPU 子进程完成，左右手分别处理，卡片和大图共享缓存；只保留正在计算的一帧与最新待处理帧，不累积显示队列。工作台默认刷新上限为 10 Hz，后台轮询约 6 Hz，**不是 120 Hz 手势显示**，也不会改变解算速率、ROS 发布、源时间戳或录包数据。

缺少模型、模型重建失败或模型与消息中已发布关节点不一致时，会提示具体原因，可重新加载模型或切换骨架。关节点存在时的 2 mm 一致性检查只用于发现模型/参数不匹配，不是手势精度承诺。旧后台没有提供 MANO 参数时仍可使用骨架；部署更新后，先结束录制并停止受管设备，再通过配置窗口“保存并应用”重载空闲后台，随后手动恢复模块，不要直接重启正在管理设备的后台。

同一会话断流或出现无效结果时保留最后有效画面并标为历史，不生成中立手冒充实时结果；会话、标定版本或模型身份改变后等待新结果，不沿用上一标定的网格。演示模式只提供合成骨架，不伪造 MANO 解算。左右手显示链路已实现，左手真实硬件仍需独立验收。触觉卡片中的静态三维触觉与这里的动态手势仍是独立视图，工作台尚未实现两者的同步叠加。

### Tracker 初始化向导

这个入口封装现有、经过批准的 `live_windows_bootstrap.py`，不是新的配对算法，也不替代 Windows 建图。部署好已有启动文件后，日常初始化、启动 ROS 读取和查看位姿都可以在界面完成，无需逐次使用终端或让大模型代为执行；开关 Tracker 等硬件动作仍由操作员完成。

首次配置：

1. 在 **设备 → 配置设备 → Tracker** 启用模块，填写已有 bundle、角色映射和扩展 PyVUT 环境，保存并应用，但先不要启动只读发布器。私有 bundle、角色映射放在仓库外，文件仅供当前用户读写，不随代码发布。
2. 打开 **标定与离线工具 → Tracker 初始化向导**，核对 Python、脚本、已批准 Capture 和 bundle。向导与设备配置必须使用同一个 bundle。可以用“选择”定位文件，用 **计算指纹** 计算本机 Capture 的 SHA-256。计算指纹不会访问 Tracker，也不表示批准陌生抓包。
3. 点击 **保存本机初始化配置**。下次打开会自动填入这五项；明确点击“开始初始化”时也会先保存。保存不会启动脚本、访问设备或记住执行批准。

日常操作：

1. 打开向导，核对已填入的配置，勾选使用所选已批准输入，再点击 **开始初始化**。重新打开窗口需要重新勾选；打开窗口本身不会执行初始化。
2. 按界面依次确认三只关机、Host 准备好、两个 Client 准备好。**只有出现开机指令后才执行对应开机动作**：先开启 Host，再按提示同时开启两个 Client，不必在每一步询问大模型。
3. 等待“初始化完成”。只有脚本报告完整完成、正常退出并释放初始化锁后，才显示完成及 **启动读取并查看 Tracker** 按钮。
4. 点击该按钮。工作台通过既有设备管理流程请求启动 ROS 读取，收到后台接受回执后进入 Tracker 预览；拒绝、失联或超时会保留向导并说明原因，不会自动重试。若已存在读取进程，则只进入预览，不重复启动，也不接管外部进程。
5. 在预览中确认三路位姿持续更新、频率和有效状态正常。**启动请求被接受、初始化完成或灯色正常，都不等于三路有效 6DOF 已验收通过。**

向导配置保存在 `~/.config/visiontactile/workbench/tracker-bootstrap.json`，仅含 Python、脚本、Capture、bundle 路径及 Capture 指纹，不保存凭据正文、批准勾选或“已初始化”状态。设备模块配置仍在同目录的 `workstation.json`；两者分别负责初始化输入与 ROS 读取参数。迁移电脑后，失效路径会保留在表单中供重新选择，不会自动更换抓包或重做配对。部署人员显式设置的 `VT_*` 初始化环境变量优先于保存值；日常桌面启动无需设置这些变量。

配置目录权限为 `0700`，向导配置文件权限为 `0600`。配置损坏或权限不符时会显示错误并保留原文件，需要修复本机配置后重开向导；不会静默覆盖。

移动数据目录后，需要同步核对初始化抓包、启动包和设备配置中的角色表路径。后台无录制或发布器冲突，不代表这些输入文件存在；缺失输入会在启动脚本前被拒绝。再次尝试时，界面保留本次提交的错误，不会被上一轮初始化的失败说明覆盖。

After moving a data directory, check the capture and bundle paths in the initialization settings and the role-map path in device configuration. An idle backend does not establish that these files exist; missing inputs are rejected before launching the script. A failed retry keeps its current submission error visible instead of replacing it with an earlier initialization failure.

初始化期间不能同时启动设备、开始录制或应用配置；窗口关闭会请求取消本次初始化并等待子进程退出。取消不能回滚已经发生的设备写入，也不会自动重试。已有连续有效位姿时直接查看和采集，不要为了打开工作台而重新初始化。

### 标定与离线窗口

- **手套标定**：通过姿态卡片“标定”或菜单的左右手入口打开／切换现有 FSGlove 窗口，保留原标定与三维模型功能。它仍是独立窗口，不是把另一套 Qt 嵌入工作台。关闭该原窗口可能停止其持有的解算进程。
- **相机–Tracker 标定**：保留 **标定与离线工具 → 相机–Tracker 标定** 入口，在主程序设备页内打开工作区，提供“引导标定”和“离线标定”。界面沿用主程序主题；实时画面、单条动作提示和轮次进度默认可见，详细诊断与结果路径默认折叠。原独立离线 CLI 继续保留。
- **离线检查与时间对齐**：选择 bag、已有对齐配置、身份绑定外参及新的输出目录，执行检查／对齐／校验。原始 bag 仅读取，输出不得覆盖原 bag 或既有结果。取消会停止本窗口的计算进程，可能留下不完整结果，重新执行请使用新目录。算法与质量口径见[离线工具手册](../multisensor_alignment/README.md)。

相机引导标定先选择相机与 Tracker 角色，核对实物标定板尺寸，确认 **固定标定板，移动相机–Tracker 刚性整体**。新会话默认采用参考板 `CC200-15-11.25` 的参数：12 × 9 方格、方格边长 15 mm、标记黑色外边长 11.25 mm，以及与参考图兼容的 `DICT_5X5_1000`。有效图案为 180 × 135 mm；商品标称的 200 × 150 mm 是外板尺寸，不参与求解尺度。商品标称和打印尺寸都须用实物测量核对。在“标定板与配置”导入已有配置时保留其中的实测参数，不替换为新默认值。先在设备页启动输入设备，标定工作区不自动启动传感器。

点击“开始标定”后，后台拥有专用录包与求解任务，逐轮采集 40 个求解姿态及 10 个独立验证姿态；姿态数量满足后仍须通过运动充分性与实际落盘数据检查。三轮独立结果通过质量和重复性检查后才允许自动应用。“候选已保存”“等待应用”“已应用，发布尚未完成”“已应用并发布”分别展示，失败候选不覆盖旧有效标定。离线模式选择一个已关闭的 bag 可检查候选，选择三个独立录制包可完成相同的验证与应用流程。

切换页面或返回设备页不会取消任务；重新进入仍显示后台的当前会话。取消按钮请求后台有序结束录制；界面退出不拥有、也不强杀采集或标定子进程。结果详情显示活动版本、Topic 和报告路径；有上一版本且后台空闲时，可使用折叠详情中的“恢复上一版本”，后台重新核验身份及版本并留下新的恢复记录。设备离线或身份未核验时，历史完成结果不显示为当前身份有效。未连接支持标定协议的真实后台时，界面显示不可用原因且禁止提交。误差门槛、结果语义和环境要求见[标定工具手册](../tracker_camera_calibration/README.md)。界面及软件测试不代表实际装配的精度或性能已经通过验收。

**English — Camera–Tracker calibration:** Keep the **Calibration & offline tools → Camera–Tracker calibration** entry. It opens an embedded workspace on the Devices page, using the main theme, one action instruction and progress indicator, with diagnostics collapsed. Guided mode uses a fixed board and a rigidly attached camera–Tracker assembly. Select the devices, check measured board dimensions, and confirm the assembly. New sessions default to the reference `CC200-15-11.25` board: 12 × 9 squares, 15 mm square side, 11.25 mm black marker side, and the compatible `DICT_5X5_1000` dictionary. The active pattern is 180 × 135 mm; the advertised 200 × 150 mm outer board does not determine solver scale. Verify advertised or printed dimensions on the physical board. Importing an existing configuration preserves its measured parameters. Start the input devices from Devices first; the workspace does not start sensors.

The manager owns recording and solving. Each of three independent rounds collects 40 training poses and 10 independent validation poses. Pose count alone does not establish sufficient motion or complete recorded data. Only passing quality and repeatability checks permits automatic activation. Candidate, waiting to apply, applied with publication pending, and applied/published are distinct states. An invalid candidate does not replace the prior valid calibration. Offline mode accepts one closed bag for candidate inspection, or three independent bags for validation and activation. The standalone offline CLI remains available.

Page changes preserve the session; reopening the workspace restores the manager's current state. Cancel requests an orderly stop. Closing the GUI does not kill manager-owned jobs. Collapsed result details show the active revision, Topic and reports. When a previous revision exists and the backend is idle, **Restore previous revision** requests identity and version checks and creates an audited restoration. Historical completion does not imply that disconnected or unverified devices are currently valid. Submission requires a live backend that advertises calibration support. Software and GUI checks do not establish accuracy or performance for a physical assembly. See the [calibration manual](../tracker_camera_calibration/README.md) for quality gates, result semantics and environment requirements.

### 本机配置与部署边界

桌面配置位于 `~/.config/visiontactile/workbench/workstation.json`；后台状态位于同目录的 `manager-state/`，日志为 `manager.log`，IPC 为 `private-desktop.sock`。这些是本机文件，不上传硬件凭据或模型。图形化配置保留未由表单编辑的其他配置字段。

首次安装 ROS、设备驱动、消息包、FSGlove／MANO 资源及 Tracker 扩展环境仍属于部署工作；桌面入口不会自动下载许可模型或修复系统依赖。相机目前按三相机组控制，单相机启停、脱离相机组的独立录制控制器入口、任意 Topic 配置编辑、归档和异常录制强制接管仍未集成到 GUI。**本版本覆盖常用采集流程，不宣称所有维护操作或商业交付验收都已完成。**

下文的命令行方式保留给部署人员、旧流程兼容和故障定位；正常桌面使用无需逐次执行。

### 手套或触觉提示串口权限不足

账户加入 `dialout` 后，已经打开的窗口及后台不会自动获得新权限。即使系统账户列表显示已经授权，旧后台仍可能报 `Permission denied`。

桌面入口会检查当前进程的实际权限；对于**账户已经授权、但当前会话未继承**的情况，自动刷新工作台及其后续子进程的串口组权限。重新打开时，工作台也会检查原后台：只有确认属于本工作台且没有运行设备、录制或待处理命令时，才替换权限过期的空闲后台。不会自动启动任何设备。

遇到此类错误时，在界面停止相关模块，关闭工作台，再从桌面快捷方式打开，然后手动启动模块即可；通常无需重启电脑。若界面提示旧后台仍有活动任务，先完成录制并停止设备，再点击“重试连接”。启动前会检查串口是否存在及是否可读写，分别提示未接入、账户未授权或旧会话权限，不再把这类错误当作正常运行。

如果账户本身尚未获得串口授权，仍需管理员完成用户组或 udev 部署；桌面入口不会新增授权、修改设备权限，也不会以 root 运行。未授权不妨碍打开工作台或使用非串口功能。

## 当前功能

- 中文 Immersive Studio 界面，包含设备、预览、空间、录制和档案五个主题页面；适配 1080×720 起的窗口尺寸。录制设置与控制集中在录制页；切换页面后录制继续，录制标签显示状态与计时，点击即可返回操作。
- 相机 RGB 和深度预览、Tracker 坐标轴与相对轨迹、手势的动态 MANO 三维手形／21 关节点骨架，以及触觉的二维手形、20×20 矩阵、静态 MANO 三维手形切换与放大查看。
- 手势卡片和大图可调用对应手的原标定窗口；已有受管窗口只切换，不重复启动。骨架和 Tracker 视图可拖动旋转、滚轮缩放、双击复位；MANO 视图另有掌面、手背、复位与关节按钮。
- 合成演示支持单手、双手、不展示手部，以及数据陈旧和相机离线场景。
- 真实模式展示 Topic 接收频率、有效样本频率、接收龄期与后台状态；断线后保留可用历史画面并标明陈旧。
- 创建批次和命名试次，设置 1–300 秒时长，手动开始、提前停止或等待控制器定时结束。
- 持久化试次状态、实际会话 ID、bag 路径和操作意图；重连后查询控制器，不自动重发命令。
- 冻结开始／结束元信息，记录可选任务与操作员、配置引用和可取得的设备身份；记录缺失不伪造为已确认。
- 录制结束后异步调用已有质量报告工具；在试次详情中查看结果、告警与检查范围，执行失败可手动重试。
- 在“设备”按模块启动、停止或重启已有 ROS 启动流程，查看进程归属、数据就绪状态及运行日志；后台须单独启用设备控制。
- 桌面模式自动启动带控制能力的本机后台，通过表单配置六个模块；手动初始化、标定和离线工具均有图形化入口。
- 界面和 ROS 分进程、分 Python 环境运行。关闭本工作台不停止已有采集节点。

命令行独立后台**默认只读**；桌面模式自动启用设备／录制管理能力，但仍要求新版控制器可达、状态可确认以及用户明确点击。演示批次与试次只存在内存中，不生成 bag，退出即清空。

设备启停已实现配置分组管理，相机–Tracker 标定提供主程序内的工作区；自动配对、旧手套标定窗口内嵌、完整归档及商业交付验收尚未完成。自动质检不等于传感器精度认证，真实硬件启停、录制与质检闭环仍需现场验收；不应把开发预览版当作完整商业交付。

现有手套标定窗口仍按原方式运行；关闭那个窗口可能停止其持有的解算进程。本版本尚未实现手套解算服务与原标定窗口的生命周期分离。

## 安装

在仓库根目录执行：

```bash
bash tools/capture_deploy/install.sh
```

安装器创建独立 `.venv-capture-console`，安装 PySide6、共享协议、触觉包和工作台，并添加当前用户的应用菜单／桌面入口。GUI 仅导入触觉包的纯 Python 布局模块和 JSON 资源，不加载其 ROS 节点、串口驱动或 PyQt5 窗口。不修改手套虚拟环境，不安装开机服务，不连接传感器。首次安装需要联网下载依赖。ROS 与硬件专用环境仍需按对应手册完成部署。

迁移仓库后重新运行安装器可更新桌面入口路径。也可从文件管理器运行 `tools/capture_deploy/start_workbench.sh`；相同入口的维护命令为 `bash tools/capture_deploy/start_workbench.sh`。

Ubuntu 桌面需要 Qt 所需系统库和中文字体。缺失时安装：

```bash
sudo apt install python3-venv libegl1 libgl1 libxkbcommon0 libxcb-cursor0 fonts-noto-cjk
```

不要在同一 Python 进程中混用 PyQt5、PySide6 和 ROS。下文启动脚本会隔离会影响 Qt 的 ROS/Python 环境变量，但保留桌面显示环境。

## 无硬件演示

```bash
bash tools/capture_deploy/start_demo.sh
```

窗口始终显示 DEMO，画面和频率均为合成数据，不代表真机性能。

1. 在“预览”点击“浏览数据流”展开缩略条并选择设备卡片，使用模块分段按钮筛选相机、Tracker、手势或触觉；按钮上的数字表示该类数据流数量。
2. 在“录制”使用默认演示批次，或输入名称并“创建批次”；填写试次名称。
3. 点击“模拟开始”，观察倒计时；可提前“模拟停止”。结束后进入“档案”选择试次，点击“试次详情 / 质检”，查看合成元信息与演示报告。DEMO 不执行真实质检，也不写出报告文件。
4. 点击右上角 **DEMO** 菜单，切换单手/双手、数据陈旧、相机离线场景；选项互斥，当前选项带勾选标记。切换立即更新演示画面。
5. 在“设备”选择演示模块，体验启动、停止、重启与归属状态。演示操作仅修改内存，不启动系统进程，也不连接硬件。

## 真实只读预览

本模式仅订阅，不启动任何硬件。请先按原流程启动需要查看的采集节点；未启动的模块显示等待数据或离线，不会阻止其他模块查看。

### 构建 ROS 包

在已安装 ROS 2 Jazzy 的终端中执行：

```bash
source /opt/ros/jazzy/setup.bash
cd ros2_ws  # 从仓库根目录进入
colcon build --packages-up-to vt_capture_manager vt_realsense_capture
```

### 启动后台

沿用已有工作区的构建方式，不在已有普通构建目录中切换 `--symlink-install`；若此前已混用两种方式，请使用独立 build/install 目录重新构建，勿删除正在使用的安装目录。

另开终端，在仓库根目录运行：

```bash
bash tools/capture_deploy/start_manager.sh
```

后台使用系统 Python 和 ROS 工作区。默认监视三台相机的 RGB/深度、三个 Tracker，以及左右手姿态和触觉，共 13 路预览流。后台须与现有节点使用相同的 `ROS_DOMAIN_ID` 和 DDS 配置。完整配置示例位于 `ros2_ws/src/vt_capture_manager/config/workstation.example.yaml`。

### 启动界面

再开终端，在仓库根目录运行：

```bash
bash tools/capture_deploy/start_console.sh --live
```

默认 socket 为 `$XDG_RUNTIME_DIR/visiontactile/manager-control.sock`；未设置 `XDG_RUNTIME_DIR` 时使用 `~/.local/state/visiontactile/manager-control.sock`。双方也可以用 `--socket /绝对路径/manager.sock` 指定同一个地址。后台要求 socket 所在目录为当前用户所有的私有目录，权限为 `0700`。

没有后台时，真实模式显示未连接，**不会自动切换到演示数据**。关闭界面仅退出客户端；后台和既有采集节点继续运行。后台可在所属终端用 Ctrl+C 结束。

### 仅查看部分模块

界面筛选只影响显示；要减少 ROS 订阅数量，可给后台指定 JSON/YAML 配置。以下为仅订阅右手姿态与触觉的配置内容：

```yaml
devices:
  - id: right_glove
    label: 右手姿态
    kind: glove
    side: right
    topic: /gloves/right/state
  - id: right_tactile
    label: 右手触觉
    kind: tactile
    side: right
    topic: /gloves/right/tactile/state
stale_after_s: 1.0
offline_after_s: 5.0
disk_path: /absolute/path/to/visiontactile-data  # 替换为本机数据目录
```

保存为本机配置文件，然后运行：

```bash
bash tools/capture_deploy/start_manager.sh --config /绝对路径/workstation.yaml
```

手势和触觉可以独立配置；左手使用 `side: left` 和 `/gloves/left/...`。这只改变监视范围，不改变现有录制合同，也不代表左手硬件已经验收。

## 真实批次采集

### 准备录制控制器

先完成上述 ROS 构建，并按[采集指南](../../docs/capture-guide.md)启动需要采集的节点。当前相机启动文件 `triple_realsense.launch.py` 已包含 `capture_controller`，**不要再启动第二个控制器**。已运行的旧版控制器不会随源码更新自动升级；须在没有录制时，按原流程重启为新构建版本。

如果传感器节点是分别启动的，且当前没有录制控制器，可以单独启动它：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
ros2 run vt_realsense_capture capture_controller --ros-args \
  -p output_root:="${HOME}/visiontactile-data"
```

此命令启动控制器，不会自行开始录包。控制器的 `config_path` 参数决定录制 Topic 集合；默认配置是三相机与三个 Tracker。加入手势或触觉须使用 `ros2_ws/src/vt_realsense_capture/config/` 中对应的 `cameras.with_*.yaml` 配置。后台的预览配置和界面筛选**不会**更改录制 Topic。工作台不修改现有 Recorder 的 QoS、缓存或 MCAP 格式，也不因预览质量告警自动停止录制。

在同一 ROS 域确认新版服务已就绪：

```bash
ros2 service type /capture/get_snapshot
ros2 service type /capture/managed_command
```

应分别返回 `vt_camera_msgs/srv/GetCaptureSnapshot` 和 `vt_camera_msgs/srv/ManagedCaptureCommand`。服务不存在时，先检查控制器版本和 ROS 域；不要仅为解锁界面重复启动控制器。

### 启用管理后台

若已启动只读后台，先在其终端按 Ctrl+C 结束，再启动管理模式。不要同时运行两个后台占用同一 socket 或状态目录：

```bash
bash tools/capture_deploy/start_manager.sh --enable-recording
```

随后打开界面：

```bash
bash tools/capture_deploy/start_console.sh --live
```

界面显示 `LIVE · 采集管理`。启用管理后台只允许用户主动操作，不自动启动传感器、不修改配对或标定。它是本机操作开关，不是企业用户认证或权限系统。

### 创建并录制试次

1. 进入“录制”，输入批次名称并点击“创建批次”；选中新批次。
2. 填写试次名称，按需填写任务名称、操作员代号，设置 1–300 秒时长；检查预览与所需数据源是否符合本次任务。
3. 点击“开始录制”。命令已受理时先显示启动中；控制器确认后才显示录制中。
4. 等待定时结束，或在“录制”页点击“停止录制”。切换页面后录制继续，可通过录制标签的状态与计时返回操作。停止请求绑定当前试次及实际会话，不能用于结束别的外部录制。
5. 进入“档案”，查看试次表中的状态和 bag 路径。双击试次，或选中后点击“试次详情 / 质检”，查看概览、元信息与质量报告。

默认启用录后质检；录制结束后先排队，等待控制器状态可确认且没有录制任务时执行。`COMPLETE` 只表示录制流程已结束，不证明所有 Topic 完整、无丢帧或数据合格。关闭质检或查看旧版试次时仍显示“未执行”。桌面菜单可以打开离线检查／对齐／校验窗口；触觉 NPZ 导出等高级参数继续见[离线对齐工具](../multisensor_alignment/README.md)。

| 试次状态 | 含义 |
| --- | --- |
| 启动中 STARTING | 已保存启动意图，等待控制器确认 |
| 录制中 RECORDING | 已确认对应会话正在录制 |
| 正在收尾 STOPPING | 已保存停止意图，等待录制结束 |
| 录制结束 COMPLETE | 该会话已结束，质量另行检查 |
| 失败 FAILED | 已得到明确失败结果，查看详细说明 |
| 状态待核实 UNKNOWN | 超时、失联或会话身份不一致，不能据此断言已停录 |

### 磁盘空间保护

控制器在创建会话前检查目标盘空间，并在启动 Recorder 前核对会话目录所在分区。预算按三相机 RGB8 + Z16 原始体积、计划时长、10% 消息／容器余量与停止预留计算，不假设压缩或丢帧能节约容量。默认配置约需：30 秒 **15.02 GiB**、60 秒 **27.77 GiB**、300 秒 **129.73 GiB**。预算不足或空间查询失败时拒绝启动，并在录制页显示具体原因。

录制中默认每 0.5 秒检查空间；触及停止预留或查询失败时走已有停止与收尾流程。未确认 Recorder 退出时保持收尾状态；确认退出后保留提前结束原因。该保护不替代录后质检，也不能保证其他进程瞬间占满磁盘或 Recorder 无法退出时的数据完整性。

需要调整时，在选用相机 YAML 的 `recording` 下添加 `disk_guard`，保留原有字段：`min_free_bytes` 默认 `2147483648`，`check_interval_sec` 默认 `0.5`。实际停止预留还包括缓存和正常停止期间的数据余量，可能高于最低值。`additional_stream_bytes_per_second` 默认为 `0`，可填写追加 Topic 合计的预计字节率；未填写时明确提示预算仅覆盖已知图像流，运行中的空间保护仍有效。参数在控制器下次启动时加载。

### 断线与重启

- **关闭工作台**：仅关闭界面，Recorder 和后台继续运行。重新打开后加载后台持有的批次与试次。
- **后台重启**：使用同一个状态目录启动。后台先恢复 SQLite 记录，再向仍在运行的控制器查询；实例、原 START 请求和会话匹配后恢复录制状态，不重发 START 或 STOP。
- **命令响应超时**：显示状态待核实，后台继续查询。不自动重发，也不要通过别的终端重复启动录制。恢复连接后以控制器和试次状态为准。
- **并发操作回执**：录制、设备及 Tracker 操作按各自请求编号获取结果，彼此不覆盖。
  本地最多保留 128 份回执，排队与发送中的请求受保护；旧回执过期会明确提示结果
  待核实，不会永久等待或自动重发。新试次不能重用当前控制器会话的 START 编号，
  首次归属还须由晚于录前防护版本的会话证明，避免把旧录制归到新档案。
- **质检数据库暂时不可用**：后台报告错误并退避重试；已完成结果仅补交原尝试，
  不重跑其子进程。若结果持久化前后台退出，重启后按原恢复规则创建新的只读尝试，
  保留已有报告。永久数据库损坏仍需维护，不会自动改写或修复历史档案。
- **控制器自身重启**：不等于后台重启。若磁盘旁车仍记载未结束会话，新控制器进入 `recovery_required`，不能证明旧 Recorder 已退出，也不擅自接管。当前版本没有界面内强制接管或清除 UNKNOWN 功能，需要先核实原录制进程及文件，再进行维护；不要删除状态记录来强行开始新录制。
- **由其他程序启动的会话**：可以监视，但工作台不自动获得停止权限；请使用原启动流程结束。

关闭本工作台或管理后台不发送 STOP，但**关闭录制控制器本身仍遵循原控制器的收尾行为**。这与关闭工作台不是同一操作。

### 数据保存位置

默认管理目录为 `~/.local/state/visiontactile/capture-manager`，也可以明确指定：

```bash
bash tools/capture_deploy/start_manager.sh --enable-recording \
  --state-dir "${HOME}/.local/state/visiontactile/capture-manager"
```

该目录须为当前用户所有且权限 `0700`。其中 `capture.sqlite3` 保存批次、试次和操作意图，`trials/<试次ID>.json` 是单试次清单。录制控制器在会话目录中另写 `controller_session.json`；bag 仍写入控制器配置的 `output_root` 下，最终以界面返回的实际路径为准。工作台不会把 bag 移入数据库目录。

备份管理目录时先正常关闭后台，或使用 SQLite 一致性备份；不要只复制活动数据库而漏掉 WAL。界面最多显示最近 200 个批次和 200 个试次，较早记录仍保留在数据库中，分页与归档检索尚未实现。

## 设备进程管理

设备控制与录制控制分别启用。默认启动后台仍只读；`--enable-recording` 不会自动开放设备启停，`--enable-device-control` 也不会自动录包。启用后仍须在界面明确点击，不在后台启动、重连或恢复配置时自动启动设备。

### 配置可控模块

将 [设备配置模板](../../ros2_ws/src/vt_capture_manager/config/workstation.devices.example.yaml) 复制到本机配置目录，填写实际路径，再将需要的组设为 `enabled: true`。模板所有组默认禁用，未填写串口时不猜测设备。

| 预设 | 实际启动内容 | 使用边界 |
| --- | --- | --- |
| `cameras` | 既有三相机、时间归一化节点及录制控制器 | 当前作为一个组管理；不能与另一个相机启动流程或控制器重复运行 |
| `trackers` | 三 Tracker 的只读 ROS 发布节点 | 复用已有 bundle 与角色表；仍须先按原流程完成硬件初始化，不执行配对或重建地图 |
| `glove` | 指定左／右手的 ROS 桥接和原标定窗口 | 串口、连接和标定仍在原窗口内操作；`launch_gui: 'false'` 可仅启动桥接，不等于启动解算器 |
| `tactile` | 指定左／右手的触觉采集节点 | 显式指定触觉串口；工作台负责预览，不另开独立触觉窗口 |

左、右手以及姿态、触觉均可独立启用。例如仅控制右手触觉，可使用以下 YAML；省略 `devices` 时保留默认预览范围，不影响启动范围：

```yaml
process_groups:
  - id: right_tactile
    label: 右手触觉采集
    preset: tactile
    enabled: true
    parameters:
      hand: right
      port: /dev/serial/by-id/实际触觉设备
```

所有路径参数须为绝对路径或 `~/...`；串口优先使用稳定的 `/dev/serial/by-id/...`。IMU 与触觉使用不同串口，不应让两个读取程序占用同一设备。模板参数只映射到仓库已有启动文件，客户端不能传入任意命令、可执行文件或 shell 脚本。修改配置后需重启后台才能加载新配置；重启不会自动接管旧后台留下的进程。

串口别名在每次明确启动前重新解析，预检与本次启动使用同一个实际节点；
已启动组保留原资源身份。日志准备失败且尚未尝试创建进程时，不会留下“不明启动”
记录。桌面后台自行维护 `manager.log`：每 250 ms 检查，超过 5 MiB 后保留末尾
1 MiB，关闭窗口后仍继续维护；这不是磁盘配额，写入可短暂超限，截断时可能丢弃
并发日志。手动启动后台不传 `--log-path` 时维持原日志方式。已运行的旧后台需在
空闲时明确重启才能使用这些修复，工作台不会自动停止现场进程。

### 启动和操作

完成前文 ROS 构建；新增手套／触觉／Tracker 模块还须按各自文档构建对应包，安装其现有运行依赖。然后在仓库根目录运行：

```bash
bash tools/capture_deploy/start_manager.sh \
  --enable-device-control --enable-recording \
  --config /绝对路径/workstation.yaml
```

另一个终端打开工作台：

```bash
bash tools/capture_deploy/start_console.sh --live
```

不需要录制时省略 `--enable-recording`。也可用 ROS launch 的 `enable_device_control:=true` 启用相同功能。

1. 进入“设备”，选择已配置模块并点击启动。
2. “已受理”仅表示操作排队；查看模块状态和日志确认实际结果。
3. “启动中”表示进程已启动、数据尚未就绪。超过等待时间后仍无预期数据会显示降级，不反复重启硬件。
4. “运行中”表示本后台持有的进程存活，且配置的预期 Topics 有新鲜有效数据。它不是硬件精度验收。
5. 完成采集并确认录制已结束后，可选择模块停止或重启。停止相机组也会结束该组的录制控制器；停止手套组会关闭该组原标定窗口及桥接，请先保存需要保留的标定结果。

录制启动／运行／收尾或存在未核实试次时，设备启停暂不可用；设备启动／停止命令实际执行期间也不能同时开始录制。等待传感器数据就绪不作为录制质量门禁，不因某一路缺数据自动停止录包。

后台从 ROS 图检测预期 Topic 的现有发布者，并对已配置组的串口、角色和共享资源检查冲突。此检查不等于扫描或接管所有外部串口程序：直接占用硬件却没有发布 ROS Topic 的外部程序，仍应先按原流程关闭。

### 进程归属与退出

- **本后台启动**：可在空闲状态下停止或重启。停止使用该启动进程的独立进程组，先请求正常退出，超时才升级终止；不按进程名称批量结束其他程序。
- **外部运行**：已存在发布者或由以前后台启动的进程。当前后台不接管，不提供停止／重启操作，也不重复启动。
- **状态未知**：无法确认 ROS 观测或残留进程身份。查看日志和原启动终端，确认状态后再操作，不通过删除进程记录强行解锁。
- **关闭界面或后台**：不自动停止传感器或 Recorder。若希望全部关闭，先停录，再逐组停止本后台持有的模块，最后退出后台。不要先关闭后台再期待新后台可以一键接管旧进程。

进程记录位于管理目录的 `processes/device-processes.json`，日志位于 `processes/device-logs/`。这些记录不会保存硬件凭据文件的内容。后台运行期间会限制日志大小和保留数量；后台退出后继续运行的子进程仍可写日志，需要人工关注磁盘空间。不同状态目录不共享进程锁，正式采集应只运行一个设备管理后台。

## 元信息与录后质检

### 配置后台

质检复用仓库的 `tools/multisensor_alignment`，在独立的系统 Python 子进程中运行。推荐使用本文的 `start_manager.sh`，它会加入该工具的源码路径，并保留 ROS 消息环境；不会向 GUI 导入 ROS。系统依赖不足时安装：

```bash
sudo apt install python3-numpy python3-yaml
```

读取真实 MCAP 还需要已有 ROS 环境中的 `rosbag2_py`、MCAP 存储插件和对应消息包。缺少依赖会显示“检查执行失败”，不改变已录 bag 或录制状态。

在后台配置 YAML 中可加入以下字段；也可以单独保存为配置文件，未填写的设备列表沿用默认值：

```yaml
quality_enabled: true
quality_timeout_s: 900
quality_alignment_config: ''
metadata_refs: []
```

```bash
bash tools/capture_deploy/start_manager.sh --enable-recording \
  --config /绝对路径/workstation.yaml
```

- `quality_enabled: false` 同时禁用自动质检和手动重试，不影响录制与元信息保存。
- `quality_timeout_s` 是单次检查的时间上限，不是录制时长。
- `quality_alignment_config` 可填写已有**离线对齐配置**的绝对路径，不是相机启动配置；为空时按本试次冻结的采集计划检查缺流；v2 可选缺失不告警，v1 与旧版无计划试次保留原检查要求。
- 质检逐条检查 Image 消息头，不解码像素。有冻结采集计划的试次对配置中的实际图像流按 30 Hz 检查，平均频率容差 1%，超过 50 ms 的缺口单独告警；有效源区间不足一秒报告样本不足。旧版无计划试次仍显示观测频率，不擅自假定 30 Hz。此检查不认证像素内容或 RGB-D 配对完整性。
- `metadata_refs` 可记录显式指定的配置或标定文件，例如：

```yaml
metadata_refs:
  - name: right_hand_calibration
    kind: calibration
    path: /absolute/path/to/calibration/right_hand.json
  - name: capture_config
    kind: config
    path: /absolute/path/to/repository/ros2_ws/src/vt_realsense_capture/config/cameras.yaml
```

示例路径必须改成现场实际文件。只接受有界的 JSON／YAML 文本文件引用，不读取模型或私密凭据。普通引用保存路径、大小、SHA-256 和观测时间，不将文件内容打包上传；读取不到时标记未知，不阻止 START。文件哈希仅证明观测到的文件内容，不证明求解器已经加载该文件。

每次 START 重新观测引用，冻结开始快照；软件版本在后台启动时读取。开始快照中的控制器上下文是 **START 前**的观测，不应把其中上一会话的路径当作本次 bag；本次实际会话以试次记录和结束快照为准。结束时另存结束快照，不覆盖开始记录。

若提供质检用对齐配置，其内容会随试次冻结，后续检查使用该副本，避免检查排队期间配置发生变化。未能读取的对齐配置会被明确告警，检查仍可覆盖控制器实际声明的 Topics。

### 查看结果

在“档案”选择一行，点击“试次详情 / 质检”：

- **概览**：试次、会话、时长、bag 路径、快照和报告路径。
- **元信息快照**：冻结的开始与结束记录，包含配置、引用和观测来源。
- **质量报告**：Topic 消息数、频率统计依据、告警及检查范围。图像的 metadata 平均 Hz 按整 bag 时长计算，不是逐帧源频率。

详情只在打开后按需获取，不随实时预览反复传输完整报告。报告较大时显示有标记的摘要，使用“打开报告”查看完整本机文件；“打开数据目录”仅打开当前试次路径。回放在档案页内进行，质检告警不阻断已有话题浏览。断线时缓存详情会标为陈旧，重试按钮不可用。

| 质量状态 | 操作含义 |
| --- | --- |
| 未执行 | 录制未结束、质检被关闭，或旧试次没有新版本的完整结束证据 |
| 等待检查 | 任务已持久保存，等待空闲执行 |
| 检查中 | 后台只读子进程正在检查 |
| 已检查项无告警 | 报告覆盖的检查项没有告警，不代表物理精度或全部图像内容合格 |
| 存在告警 | 展开报告查看缺流、时间间断等信息；不自动删除数据或要求重录 |
| 检查执行失败 | 程序、依赖、路径或输入错误；处理原因后可点击“重试检查” |
| 检查中断 | 新录制或后台关闭优先，任务将在空闲时恢复 |

质检不对 Recorder 添加质量门禁。新录制到来时，只中断本后台持有的只读质检子进程并重新排队，不终止采集节点；后台重启恢复未完成的质检任务，不重发 START／STOP。已经完成的报告不会自动重算，旧版 COMPLETE 试次也不会被批量补跑或补写虚假的开始快照。

### 保存与兼容

在管理目录的 `trials/` 下，每个新试次可产生：

- `<试次ID>.start.json`、`<试次ID>.finish.json`：不可变开始／结束快照。
- `<试次ID>.alignment.yaml`：可选的冻结质检配置。
- `<试次ID>.quality.<次数>.json`：按次保存的完整报告，不覆盖先前报告。
- `<试次ID>.json`：更新后的试次清单，关联快照和质量状态。

文件均位于 bag 外；元信息缺失、质检失败或重试不会修改原始采集数据。当前输入指纹结合小型 metadata 内容与数据文件属性，不是对整份大型 bag 做全量内容哈希，也不是长期归档完整性认证。

v0.3 将 SQLite 从 schema 1 事务迁移到 schema 2，保留旧批次和试次。升级前先正常关闭后台并备份完整状态目录；旧版后台不能直接打开升级后的数据库。旧记录没有采集快照时显示未提供，不从当前配置伪造历史。

新客户端优先协商 IPC v5，新增采集计划与录前核对；v4 保留设备控制，v3 保留详情和质检功能，v2 仍可使用原录制功能，v1 保持只读。新增字段和设备命令不会发送给不支持的旧后台；带计划的请求不会降级为无计划启动。连接陈旧时保留历史状态但禁用控制，不自动重发操作。冻结计划作为现有开始快照的新增字段保存，不新增数据库迁移，也不为历史试次伪造计划。

## 如何理解显示结果

- “在线”表示收到新鲜的 Topic 数据，不是对 USB/串口物理连接的完整诊断。
- 接收 Hz 统计后台最近一秒收到的消息；有效 Hz 根据有效标志、新鲜度和样本标识统计。刚启动时需要一个窗口稳定。
- 界面状态刷新上限为 10 Hz，后台连接轮询约 6 Hz；图像预览缩小后传输。它们都不等于传感器或 rosbag 的采集频率。
- 深度预览支持 16UC1、mono16、32FC1，是相对深度灰度图，不推断未经配置的米制比例，不用于测量。
- 触觉显示原始 ADC 值；20×20 是滚动状态，不应理解成全部像素同时测量。色标不是压力单位，当前未接入工作台内的基线标定。
- 手部关节点和 MANO 网格位于各手本地参考系，已应用根部朝向；不与 Tracker 直接拼成同一世界坐标系。没有关节点但有有效 MANO 参数时仍能重建三维手形；缺少对应视图所需数据时显示等待/历史画面，不用零坐标伪造新姿态。
- 外部 CaptureStatus 可用于查看录制状态，但其 COMPLETE 不等同于质量合格；缺少实际开始时间时显示“时长未知”。

## 测试与截图

GUI 和核心测试不需要硬件：

```bash
env -u PYTHONPATH QT_QPA_PLATFORM=offscreen PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-capture-console/bin/python -m pytest -q \
  ros2_ws/src/vt_capture_core/test tools/capture_console/tests tools/capture_deploy/tests

QT_QPA_PLATFORM=offscreen bash tools/capture_deploy/start_demo.sh \
  --size 1366x768 --smoke-seconds 2 --screenshot /tmp/capture-console.png
```

后台的模型、持久化管理、质检、图像编解码和 socket 测试还需要安装该包、NumPy、OpenCV、PyYAML：

```bash
.venv-capture-console/bin/python -m pip install \
  -e ros2_ws/src/vt_capture_manager numpy opencv-python-headless PyYAML
env -u PYTHONPATH PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  .venv-capture-console/bin/python -m pytest -q ros2_ws/src/vt_capture_manager/test
```

ROS 端到端测试默认跳过，需要显式启用并在系统 Python 下运行：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
env ROS_DOMAIN_ID=179 ROS_LOCALHOST_ONLY=1 VT_CAPTURE_MANAGER_ROS_TEST=1 \
  PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 /usr/bin/python3 -m pytest -q \
  ros2_ws/src/vt_capture_manager/test/test_ros_integration.py \
  ros2_ws/src/vt_capture_manager/test/test_managed_ros.py
```

测试使用独立 ROS 上下文、唯一测试 Topic、合成发布者和模拟录制服务，不启动设备或真正的 Recorder。请勿把测试域用于正式采集。没有编译新的消息服务时相关测试会跳过；跳过不等于验证通过。

完整控制链路测试位于 `ros2_ws/src/vt_realsense_capture/test/test_managed_capture_ros.py`，使用真实 socket、管理后台、DDS 和控制器，只替换 Recorder；固定使用独立测试域 191／192：

```bash
source /opt/ros/jazzy/setup.bash
source ros2_ws/install/setup.bash
env ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST PYTEST_DISABLE_PLUGIN_AUTOLOAD=1 \
  /usr/bin/python3 -m pytest -q \
  ros2_ws/src/vt_realsense_capture/test/test_managed_capture_ros.py
```

## 故障处理

- **桌面后台未就绪**：在“设备”点击“后台日志”查看原因，修复后点击“重试连接”。不会循环重启后台或自动重启传感器。
- **系统重启后残留私有 socket**：若持续 `Connection refused` 且提示无法安全清理 socket，等待不会恢复。维护人员仅可在独占 `desktop-operation.lock` 后，核验 `desktop-manager.json` 属于旧 boot、登记 PID 已不存在、固定 socket 路径及 inode 与记录一致、目录和文件归属当前用户且权限私有、连接明确被拒绝，再把该 socket 和登记文件一并移至同目录的私有备份目录；随后重新打开工作台。任一条件不符或锁被占用时停止，不删除未知 socket、不结束进程、不自动启动设备。
- **配置无法应用**：先停录，并停止由本后台启动的模块；等设备操作完成再应用。后台外部归属或状态未知时不能通过重启强行接管。
- **桌面第一次没有可启动模块**：打开“配置设备”，填写实际参数并勾选模块，再保存并应用；默认全部禁用不是连接失败。
- **找不到模块**：先运行安装脚本；不要用另一套 Python 直接启动工作台。
- **Qt 无法打开显示**：桌面启动须有有效 DISPLAY/Wayland 环境；无桌面测试使用 `QT_QPA_PLATFORM=offscreen`。
- **后台未连接**：确认后台仍运行、两端 socket 路径一致；界面会自动重试。
- **socket 已存在**：不要覆盖运行中的后台。先确认旧后台状态并正常结束；意外退出留下的 socket 经人工确认无监听者后才能清理。
- **有节点但没有数据**：检查 Topic 名称、ROS_DOMAIN_ID、QoS 和消息类型；当前后台按主机实时钟判断新鲜度，不支持直接把历史 bag 时间当作实时新鲜数据。
- **图像不支持编码**：查看设备详情及事件。预览失败不改变源图像或录制行为。
- **开始按钮不可用**：检查是否启用 `--enable-recording`、新版控制器服务是否可达、是否有未决或外部录制。先处理状态原因，不要反复点击或发送旧 Topic 命令绕过状态绑定。
- **状态目录已被使用**：同一目录只允许一个管理后台；先正常关闭旧后台。不要手动删锁文件绕过正在运行的进程。
- **命令已受理但尚未录制**：受理只表示意图已保存。以试次状态和控制器返回的实际会话为准；超时参见“断线与重启”。
- **质检一直等待**：检查控制器是否连接、是否仍在录制或存在未决试次。录制优先，不需要为催促质检关闭传感器。
- **质检执行失败**：打开试次详情查看原因，检查依赖、bag 路径、metadata 和配置；修复后手动重试。失败不代表该试次被自动判废。
- **设备按钮不可用**：检查 `--enable-device-control`、组的 `enabled`、录制状态及进程归属。外部运行和旧后台遗留进程不能由本后台停止；仅有预览数据不代表拥有该进程。
- **设备启动后没有数据**：查看组日志和各 Topic 的有效 Hz。手套需在原窗口连接并标定，Tracker 需先完成既有硬件初始化；后台不将缺数据解释为需要重新配对。
- **三个 Tracker 已在线，空间页提示服务未启动**：旧版 `workstation.json` 可能缺少 `scene_config_path`，因此只有设备预览，尚未启动空间服务。在 **空间 → 空间配置** 选择 [`scene.example.yaml`](../../ros2_ws/src/vt_capture_manager/config/scene.example.yaml)；结束录制、停止受管设备后“保存并应用”，再恢复需要的模块。该模板可直接显示三个 Tracker 的公共坐标，无需相机外参或重新初始化 Tracker。

## 实现边界与后续阶段

当前源码主要分为：

- `ros2_ws/src/vt_capture_core/`：共享模型、Recipe 与不可变采集计划，以及 v1 只读、v2 管理、v3 按需详情、v4 设备控制和 v5 计划校验协议。
- `ros2_ws/src/vt_capture_manager/`：ROS 订阅、录制适配、SQLite 意图记录、元信息、只读质检、设备进程管理与本地 socket 服务。
- `ros2_ws/src/vt_camera_msgs/`：录制上下文与管理服务接口。
- `ros2_ws/src/vt_realsense_capture/`：既有 Recorder 控制器，新增会话绑定、请求去重和旁车恢复信息。
- `tools/capture_console/`：独立 PySide6 界面、演示、后台客户端和录制视图模型。

二次开发时，录制命令的等待、确认和未知结果判定集中在 `viewmodels/operations.py`；主窗口只提交显式操作并应用显示结果。协调器不依赖 Qt，也不会自动重发命令。共享组件通过公开快照、清帧、布局刷新等接口协作，禁止外部读取私有缓存或调用内部布局函数；相关边界和行为由测试约束。扩展前可参考[架构说明](../../docs/architecture.md#desktop-operation-coordination)。

后续阶段为真实设备启停、采集与质检闭环验收，手套服务与标定客户端、历史检索及归档。多工位调度、角色权限、完整运行环境归档、旧后台设备进程接管、断电后 Recorder 接管也不属于本版本已交付功能。商业化使用还需完成真实硬件验收、部署恢复测试和适用的第三方许可确认。
