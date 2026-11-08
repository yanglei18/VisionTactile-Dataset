# 左右手独立实现接入工作台

本次集成以已经调试的独立程序提交
`eaafec6bb0cce77b7fb7019c9f828d5afb97950a` 为行为基线，工作台基线为
`7f7fccae0c2e077d8291723039248a264e9f1c3f`。开发分支为
`codex/integrate-hand-runtime-workbench`。

## 入口与职责

工作台不增加“手套”工作区。选中原有左手或右手姿态/触觉设备，主预览区域显示该手的共享三维面板；原有标定按钮展开同一个面板内的动作示范、实时手势和采集反馈。相机预览、VIVE Hub、联合空间及点云仍使用原来的工作区和控制入口。

| 层 | 当前实现和唯一职责 |
| --- | --- |
| 数值核心 | `tools/fsglove12`：复用独立程序的读取、融合、安装标定、功能标定、形状/触指及 MANO 解算；工作台不重新计算角度。 |
| 手部服务 | `vt_fsglove12.hand_service`：每手一个无 GUI 服务，使用原 `ProcessLiveController`。数值运行于该手明确配置的 Python 环境。 |
| 设备管理 | `vt_capture_manager.hand_control` 与 `ProcessSupervisor`：统一启停、资源归属、命令回执与录制准入；界面不另开 IMU 串口。 |
| 姿态录制 | 原 `vt_glove_bridge` 消费同一服务发布的姿态。示范动画、显示保持和触觉颜色不进入姿态测量。 |
| 触觉采集 | `vt_tactile_capture`：每手一个 44 字节行协议读取器，发布原始样本、完整状态和只读显示数据。 |
| 触觉展示 | `ManagedTactileSource` 消费完整状态，复用独立程序的参考、响应和配色。不打开串口，不复用抽稀的设备卡片缩略图作输入。 |
| 共享界面 | `vt_capture_console.ui.hand`：工作台与 `vt_hand_tactile_viewer` 使用同一个面板、显示合成和标定动作定义。独立入口只负责自己的启动与资源装配。 |

切换预览只切换展示，缓存的手部订阅仍可继续更新；关闭工作台只分离显示订阅，不断开正在运行的服务或录制。
显式设备停止操作由管理器执行。独立程序仍拥有自己启动的设备，不能与工作台同时读取同一串口。

## 配置与部署

每个 `glove` profile 使用 `hand`、`glove_python`、`port` 和 `resources`。
`resources` 是对应独立程序的完整资源包，包含 `manifest.json`、配置、静态手形、MANO 模型、上游固定源码，以及左手明确指定的触觉布局。清单哈希和手别在加载前校验。

为兼容显式维护的配置，也支持 `fsglove_source`、`mano_assets`、`config`、`rest` 四项代替资源包；缺少任何一项会在配置或启动预检中明确拒绝，不再等子进程启动才失败。`rest` 是该安装对应的静态手形参考，不能从其他安装猜测。资源包与显式字段同时填写时，显式字段必须精确对应包内成员；冲突会拒绝。共享三维面板需要完整资源包。旧 `launch_gui` 字段可被配置解析器读取，但工作台统一使用 `hand_service.launch.py`，不再启动外部旧标定窗口。

纯触觉模式使用触觉 profile 的 `resources`。对应动捕 profile 不存在，或明确禁用而触觉启用时，共享面板只展示静态手形与实时触觉，不创建 IMU 控制器、动作示范或掌倾推算，也不报告有效动捕姿态。纯触觉无需数值 Python；旧的无资源矩阵配置仍有效，三维页面会提示配置显示资源。启停仅作用于触觉进程组。动捕与触觉均启用时继续使用同一手的动捕资源，候选预检拒绝两者资源冲突。

触觉设备身份在后台读取 USB 枚举信息，并绑定到当前有效连接的会话/epoch。先打开页面再连接设备也能保存参考；同一路径换设备会隔离旧参考，迟到的旧身份结果不会应用到新连接。此过程不扫描串口数据，不重建手部面板或 IMU 标定。

默认发现分别查找本 checkout 的 `.runtime/left-hand` 与 `.runtime/oct29-right`，以及 `.venv-left-hand`、`.venv-right-hand`。可通过 `VT_LEFT_HAND_RESOURCES`、`VT_RIGHT_HAND_RESOURCES` 和对应的 `VT_LEFT_HAND_PYTHON`、`VT_RIGHT_HAND_PYTHON` 指定其他部署目录。发现只是建议；不会自动连接硬件。

工作台 Qt 环境与数值环境分离。`teaching_geometry` 在对应的数值 Python 中生成有限的示范帧，界面在后台读取结果；该子进程不接触设备，不能提交标定或覆盖实时姿态。

部署步骤仍为构建 `ros2_ws`，并执行 `tools/capture_deploy/install.sh` 安装工作台。两手数值环境需安装本分支的 `vt-fsglove12>=0.1.3`，候选工作台需 `vt-capture-console>=0.9.0`、`vt-capture-core>=0.8.0`、`vt-capture-manager>=0.7.0`、`vt-tactile-capture>=0.3.0`。使用 `start_workbench.sh` 保持 ROS 与 Qt 解释器隔离。

## 候选配置迁移与启动

候选入口与常用工作台分离。迁移只读取旧配置，按显式选定的已验证左右手资源包替换手部实现，保留设备端口、启用状态、相机、VIVE、点云和数据输出设置。旧配置可缺少新增的 `rest` 字段，转换成完整资源包配置后才进行新版校验。迁移不会继承 VIVE Hub 自动启动偏好，不启动硬件，也不改历史标定文件。

在当前 checkout 根目录执行一次（资源位置以本地实际部署为准）：

```bash
.venv-capture-console/bin/python -I -m vt_capture_console.backend.candidate_migration \
  --source "$HOME/.config/visiontactile/workbench/workstation.json" \
  --repo-root "$PWD" \
  --hand left "$PWD/.runtime/left-hand" "$PWD/.venv-left-hand/bin/python" \
  --hand right "$PWD/.runtime/oct29-right" "$PWD/.venv-right-hand/bin/python"
```

默认目标是 `~/.config/visiontactile/candidates/<checkout路径摘要>/`，也可明确指定短的私有 `--settings-dir`。目录通过暂存后原子重命名建立，包含原配置备份、迁移报告与新配置。相同来源再次迁移保留候选里的用户编辑；不同来源或资源选择要求另选目录，不能静默覆盖。每个候选的管理器、手部 socket、显示设置、触觉参考与默认手部采样目录均隔离。已明确指定的历史采样输出路径保持原样。

随后使用：

```bash
bash tools/capture_deploy/start_candidate_workbench.sh --check-only
bash tools/capture_deploy/start_candidate_workbench.sh
```

启动入口不会再次导入全局配置。它验证实际启用手的资源与 Python 来源；动捕数值预检载入 MANO 并检查依赖，但不创建控制器或接触串口。纯触觉只验证显示资源。新版界面和管理器只使用独立候选目录，窗口标题显示后台是否已核验。未准备的目录、其他工作树安装的界面或数值环境、旧协议后台均明确失败。

## 控制、状态和并发

工作台 IPC v7 增加 `hand_sessions` 和 `hand.command`。v1–v6 保持原协议，旧客户端不获得手部修改能力。

IPC v8 增加 `software` 身份：源码根目录、Git 提交、运行源码内容摘要与配置路径。管理器启动时冻结身份，修改代码不会让已运行进程冒充新版本。候选客户端要求精确匹配 v8，禁止降级连接旧后台；每个修改命令携带期望身份，管理器在执行前核验。桌面运行时还检查 Unix 对端 PID、实际工作目录及自己的启动记录。v1–v7 的报文保持不变，普通非候选客户端仍可显式协商旧协议。源码更新后必须重新启动候选后台；不能通过伪造窗口版本文字宣称已经更新。

`hand_sessions` 分开报告服务就绪、传感器新鲜度、标定版本和测量姿态有效性。未标定的服务可以是 `ready`，不能因此声称姿态有效。命令必须绑定管理器实例、服务实例、会话和标定版本；执行前再检查绑定，旧页面或排队期间失效的命令不得作用于新会话。

所有修改通过管理器准入，只有当前管理器持有的运行组可以修改。命令队列与回执数量有界；同编号同请求去重，不同请求复用编号会拒绝；超时只表示结果待核实，不自动重复标定。高频显示直接读取只读服务快照，避免将网格数据塞入管理器的设备状态协议。

标定忙碌状态由手部服务发布，与命令完成计数共同绑定；输入快照先到、网格快照后到，也不能产生“命令已完成但仍显示旧空闲状态”的录制准入窗口。采集、拟合、触指优化和未确认命令期间禁止开始新录制。服务超时不会直接解除这个保护；必须收到执行结果或确认所属进程已经结束。

## 触觉格式与历史记录

旧消息 `TactileSampleArray` / `TactileState` 保持定义不变。新版分别使用
`TactileRow44Array`（schema 2）与 `TactileGridState`（schema 3）。逻辑话题名称不变。

新状态携带 20 行源时间，并由完整 20×20 网格产生触觉显示。新 capture plan v3 固定新消息类型；历史 v1/v2 计划继续按旧类型校验其指纹，相机 recipe 和相机 schema 不随之改变。离线读取根据录包的实际类型选择旧 26 字节或新 44 字节解释器，不根据当前默认格式猜测。历史上如有使用旧同名 IDL 写入新 44 字节结构的录包，会明确拒绝，不能伪装为兼容成功。

旧标定文件的来源兼容只针对一个已定位的问题：资源包内的上游源码并非独立 Git checkout，旧版本却把宿主应用 HEAD 记录为上游 HEAD。兼容检查仍要求固定源文件哈希和其他 provenance 一致，不能借此接受被修改的算法、模型、传感器安装或运行依赖。历史文件原件保留。

## 验证范围

已有真手记录对比确认：左右手各前 1,000 个原始融合输出与独立基线相同；保存的三姿态采样分别重新拟合后，右手 353 帧（349 帧可用）、左手 349 帧（347 帧可用）的完整解算 JSON 哈希与基线一致，覆盖关节、MANO 姿态/根部、质量、可用性和观测掩码。原有不可用帧也保持不可用，没有为通过测试补成有效数据。

另有新旧两格式、左右手的实际 ROS CDR/MCAP 写入与离线重建测试，以及合成串口→实际读取器→ROS→录包的验证。实际左右 MANO 资源的无端口 ROS 启动测试确认可在未标定时启动，并能正常清理进程和 socket。完整 ROS 工作区的 11 个包已构建成功。

对照脚本与去除本机私有路径的证据已入库：

- `tools/fsglove12/scripts/compare_hand_runtime.py`
- `tools/fsglove12/evidence/hand_runtime_equivalence_20261108.json`
- `tools/fsglove12/evidence/candidate_runtime_equivalence_20261108.json`：本轮配置、后台身份和纯触觉修复后的再次回放，2,000 融合帧及 702 姿态帧逐字节一致。
- `tools/fsglove12/HAND_SERVICE.md` 中的复跑命令和原 profile 兼容说明。

这些验证证明迁移的数值一致性和软件链路行为，不证明所有佩戴条件下的解剖精度、长期漂移或实物触指精度。本次未重新连接实体串口，也没有要求重做整套标定。今后的功能改动应先复跑保存的数据和上述控制/录包合同，再对新行为进行有针对性的真手测试。

### 2026-11-08 候选部署修复回归

| 范围 | 结果 |
| --- | --- |
| Core 与 Manager | 1,480 passed，14 skipped |
| 工作台完整界面与后台适配测试 | 1,882 passed |
| 完整手部数值测试，提供真实左右 MANO 与固定上游 | 2,239 passed，16 skipped |
| 独立左右手展示程序 | 339 passed |
| 部署脚本和依赖合同 | 148 passed |
| 左右手 × 资源包/显式配置实际无串口 ROS 启动 | 4 passed |
| 新候选相机/Tracker 标定环境的离线合成测试 | 23 passed |

数值套件剩余跳过项为旧 PyQt5 UI（14）、显式 X11 OpenGL 测试（1）与未指定额外记录的可选 scalar/batch 对照（1）；不包含因缺少 MANO 或上游源码而跳过的测试。Core/Manager 的可选 ROS 集成项目未在该纯 Python 命令中运行，实际服务启动另行验证。候选 GUI 和管理器已做无硬件启动检查，核验源码、配置、Unix 对端进程与所属工作树，并确认没有受管设备被启动；原全局配置摘要与原管理器保持不变。
