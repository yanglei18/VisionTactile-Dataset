# 第三方依赖说明

本目录通过 Python 依赖调用第三方库，没有在本目录内复制或打包这些库的源代码，也没有引入来源不明、未声明许可的手套项目代码。以下是主要直接依赖的说明，不是安装环境全部传递依赖的许可证清单；实际分发前应以所锁定版本的发行包、许可证与版权声明为准。

| 依赖 | 用途 | 上游许可及来源 |
| --- | --- | --- |
| VQF | 陀螺仪 / 加速度计六轴姿态融合 | MIT；[官方文档的许可说明](https://vqf.readthedocs.io/en/latest/) |
| NumPy | 数值数组、四元数与功能轴计算 | BSD 3-Clause；[官方许可证](https://numpy.org/doc/stable/license.html) |
| pySerial | 串口枚举与读取 | BSD 3-Clause；[官方许可证](https://pyserial.readthedocs.io/en/latest/appendix.html#license) |
| PyQt5 | Qt 桌面界面及 QPainter 绘制 | GNU GPL v3 或 Riverbank 商业许可；[官方许可说明](https://www.riverbankcomputing.com/software/pyqt/) |

VQF 的滤波器由上游包提供；本工具自行实现原始协议接入、标定流程、受限手部运动学与显示。算法介绍、实现和论文入口参见 [VQF 官方文档](https://vqf.readthedocs.io/en/latest/)，输入单位、四元数顺序及坐标系问题参见 [官方 FAQ](https://vqf.readthedocs.io/en/latest/faq.html)。这些引用不表示上游作者认可或验证了本工具的手套模型、标定流程或姿态精度。

离线自然运动关节轴模块 `hinge_axes.py` 使用公开论文中的陀螺铰链约束，参考 [Olsson 等，Robust Plug-and-Play Joint Axis Estimation Using Inertial Sensors，Sensors 2020](https://doi.org/10.3390/s20123534)。数值优化、功能坐标构造和本地回放适配为本项目独立实现；没有复制论文作者或 qmt 项目中关节轴求解器的源代码，也不声称完整复现其联合陀螺／加速度算法。论文方法引用不是第三方源代码许可授权，论文中的实验精度不代表本手套精度。

`qmt_reference.py` 是另外提供的可选离线适配器，实际调用独立环境安装的 qmt 0.2.4，而不是上段所述的独立求解器。执行过的函数对应 [上游固定版本源文件](https://github.com/dlaidig/qmt/blob/0fa8d32eb461e14d78e9ddbd569664ea59bcea19/qmt/functions/joint_axis_est_hinge_olsson.py)，其文件头标记 `LicenseRef-Unspecified`，不能用项目整体 MIT 标记代替该文件级例外。本仓库未复制或捆绑其实现，未列为默认依赖；报告记录实际执行文件的版本、哈希和许可标记。该参考对照不构成对其产品分发许可或本手套精度的保证。

实验性 `hinge_observer.py` 的相对角／偏置状态估计、横向角速度约束及配套解析压力测试同样为本项目独立实现，没有复制 VQF 或其他项目的观察器代码。陀螺和加速度联合估计关节角的背景可参考 [Seel 等，IMU-Based Joint Angle Measurement for Gait Analysis，2014](https://pmc.ncbi.nlm.nih.gov/articles/PMC4029684/)；这不是该论文算法、实验或精度的复现。

PyQt5 不是 LGPL 许可的 PyQt。上游提供 GPL v3 与商业许可选择；其二进制轮子还可能带有 Qt 及相关组件，需要分别核对这些组件的许可。仓库或本包对自有代码的许可声明不能代替第三方依赖的许可义务，也不能据此认为所有组合、打包或闭源分发方式均已获许可。[Riverbank 官方说明](https://www.riverbankcomputing.com/software/pyqt/)

重新分发源代码、二进制、打包后的 GUI 或包含依赖的环境时，应按实际使用的版本保留所需版权和许可文本，核实 GPL / 商业许可适用条件，并检查 Qt、SIP、数值运行库及测试/构建工具等随包组件。本文仅为依赖披露，不提供许可兼容性或法律合规保证；具体分发方案应由相应负责人评估。
