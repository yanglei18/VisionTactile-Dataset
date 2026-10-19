"""Non-modal, read-only help for the five workflow pages."""
from __future__ import annotations

from ...i18n import tr

from PySide6.QtCore import Qt
from ...i18n.widgets import (
    QDialog, QFrame, QHBoxLayout, QLabel, QPushButton, QScrollArea,
    QSizePolicy, QVBoxLayout, QWidget,
)


def _label(text, name="muted"):
    label = QLabel(text)
    label.setObjectName(name)
    label.setTextFormat(Qt.TextFormat.PlainText)
    label.setWordWrap(True)
    label.setSizePolicy(QSizePolicy.Policy.Ignored, QSizePolicy.Policy.Preferred)
    label.setTextInteractionFlags(Qt.TextInteractionFlag.TextSelectableByMouse)
    return label


class HelpDialog(QDialog):
    """Informational window; opening or closing it never sends backend commands."""

    def __init__(self, backend, parent=None):
        super().__init__(parent)
        self.backend = backend
        self.setObjectName("helpDialog")
        self.setWindowTitle(tr('帮助／关于'))
        self.setModal(False)
        self.setWindowModality(Qt.WindowModality.NonModal)
        self.resize(640, 580)
        self.setMinimumSize(440, 360)

        layout = QVBoxLayout(self)
        layout.setContentsMargins(24, 22, 24, 18)
        layout.setSpacing(14)
        layout.addWidget(_label("VisionTactile", "operationTitle"))
        layout.addWidget(_label(tr('视觉、空间与触觉的联合采集工作台。')))

        self.scroll = QScrollArea()
        self.scroll.setWidgetResizable(True)
        self.scroll.setFrameShape(QFrame.Shape.NoFrame)
        self.scroll.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.scroll.setAccessibleName(tr('帮助内容'))
        content = QWidget()
        content_layout = QVBoxLayout(content)
        content_layout.setContentsMargins(0, 2, 14, 8)
        content_layout.setSpacing(9)

        self.mode_label = _label("", "operationStep")
        self.mode_description = _label("")
        self.backend_address = _label("")
        self.backend_address.setAccessibleName(tr('后台地址'))
        content_layout.addWidget(self.mode_label)
        content_layout.addWidget(self.mode_description)
        content_layout.addWidget(self.backend_address)

        def section(title, paragraphs):
            content_layout.addSpacing(12)
            content_layout.addWidget(_label(title, "sectionTitle"))
            for paragraph in paragraphs:
                content_layout.addWidget(_label(paragraph))

        section(tr('五个工作主题'), (
            tr('设备：配置设备、按组启停、查看订阅数据流，以及重试后台连接、查看日志和进入标定工具。'),
            tr('预览：查看相机、追踪器、手部姿态与触觉数据；展开数据流可切换主画面或放大查看。'),
            tr('空间：在统一坐标系中查看追踪器与已核验外参的相机的位置、朝向及空间关系。'),
            tr('录制：创建或选择批次，填写试次、任务与操作员，设置时长，开始或停止录制并查看进度。'),
            tr('档案：查看已有试次、实际数据路径、元信息与录后质量检查。'),
        ))
        section(tr('开始采集'), (
            tr('先在“设备”完成配置并手动启动所需模块，再到“预览”和“空间”检查数据。打开工作台不会自动启动硬件或录制。'),
            tr('在“录制”填写批次与试次信息，设置时长并开始录制。可用操作以当前连接和设备状态为准。'),
            tr('切换页面后录制继续运行，“录制”标签会显示简短状态与计时；需要停止时，点击标签返回录制页操作。'),
            tr('录制结束后，在“档案”查看试次详情与质量检查；录制完成本身不表示数据质量合格。'),
        ))
        section(tr('关闭窗口与设备'), (
            tr('关闭主窗口不会停止后台、受管传感器或录制控制器。结束采集时，请先停止录制，再按需停止设备；重新打开工作台会连接已有后台。'),
            tr('设备控制只管理本后台启动的进程，不接管外部启动的进程。正在进行的追踪器初始化或离线计算，需要先取消并等待任务退出。'),
        ))
        section(tr('标定与离线工具'), (
            tr('“设备”中的标定与离线工具入口，可打开追踪器初始化、相机–追踪器标定以及离线检查与时间对齐。'),
            tr('相机–追踪器标定嵌入主程序，支持引导采集与离线求解；关闭工作台不停止后台标定任务。手套原标定窗口仍持有手势解算进程，关闭该窗口可能停止对应手的姿态采集。'),
        ))
        content_layout.addStretch()
        self.scroll.setWidget(content)
        layout.addWidget(self.scroll, 1)

        actions = QHBoxLayout()
        actions.addStretch()
        self.close_button = QPushButton(tr('关闭'))
        self.close_button.setAccessibleName(tr('关闭帮助窗口'))
        self.close_button.clicked.connect(self.close)
        actions.addWidget(self.close_button)
        layout.addLayout(actions)
        self.refresh_status()

    def refresh_status(self):
        demo = self.backend.mode == "demo"
        self.mode_label.setText(tr('演示模式') if demo else tr('实时模式'))
        self.mode_description.setText(
            tr('所有画面与状态均为合成数据，不连接硬件、不生成数据包文件。演示批次与试次只保存在内存中，关闭后清空。')
            if demo else tr('当前使用实时后台。连接状态和操作权限可在主窗口与“设备”页中查看。')
        )
        address = str(getattr(self.backend, "path", "") or tr('尚未设置'))
        self.backend_address.setText("" if demo else tr('后台地址：{v0}', v0=address))
        self.backend_address.setVisible(not demo)

    def showEvent(self, event):
        self.refresh_status()
        super().showEvent(event)
