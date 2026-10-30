"""CLI entry point; default live mode never silently falls back to demo."""

from __future__ import annotations

import argparse
import sys


def main(argv=None):
    parser = argparse.ArgumentParser(description="Vision Tactile 集成采集工作台")
    mode = parser.add_mutually_exclusive_group()
    mode.add_argument("--demo", action="store_true", help="合成演示，不连接硬件、不保存 bag")
    mode.add_argument("--live", action="store_true", help="连接真实后台；录制控制须后台显式启用（默认模式）")
    mode.add_argument("--desktop", action="store_true", help="桌面工作台：连接本机后台，并按已保存设置自动连接 VIVE Hub")
    parser.add_argument("--socket", help="管理后台 Unix socket 路径")
    parser.add_argument("--scene-socket", help="只读联合空间 socket；默认与控制 socket 同目录")
    parser.add_argument("--smoke-seconds", type=float, default=0, help="指定秒数后退出，用于 GUI 冒烟测试")
    parser.add_argument("--screenshot", help="在退出前保存当前工作台截图，需要 --smoke-seconds")
    parser.add_argument("--size", default="1440x900", help="初始窗口尺寸，如 1366x768")
    parser.add_argument("--language", choices=("zh_CN", "en"),
                        help="界面语言 / Interface language: zh_CN or en; defaults to your saved choice")
    args = parser.parse_args(argv)
    if (args.demo or args.desktop) and (args.socket or args.scene_socket):
        parser.error("演示和桌面模式不能指定外部后台 socket")
    if args.smoke_seconds < 0 or (args.screenshot and args.smoke_seconds <= 0):
        parser.error("截图需要正数 --smoke-seconds")
    try:
        width, height = map(int, args.size.lower().split("x"))
        if not (1080 <= width <= 7680 and 720 <= height <= 4320):
            raise ValueError
    except ValueError:
        parser.error("窗口尺寸须为 WIDTHxHEIGHT，最小 1080x720")
    if "PyQt5" in sys.modules or "PyQt6" in sys.modules or "rclpy" in sys.modules:
        parser.error("界面必须运行在独立 PySide6 环境，不能混载 PyQt 或 ROS")
    instance = None
    try:
        if args.desktop and not args.smoke_seconds:
            from .single_instance import DesktopSingleInstance
            instance = DesktopSingleInstance()
            if not instance.acquire_or_activate():
                return 0
        return _run_application(args, width, height, instance)
    except (OSError, ValueError, RuntimeError) as exc:
        print(f"工作台无法启动：{exc}", file=sys.stderr)
        return 1
    finally:
        if instance is not None:
            instance.close()


def _run_application(args, width, height, instance=None):
    # Duplicate desktop launches return above before Qt, GL or ROS adapters load.
    from PySide6.QtCore import QTimer
    from PySide6.QtGui import QFont
    from PySide6.QtWidgets import QApplication
    from .i18n import initialize, tr
    from .i18n.widgets import QMessageBox
    from .backend.demo_backend import DemoBackend
    from .backend.ipc_client import IpcBackend
    from .backend.scene_backend import SceneBackend
    from .backend.scene_demo import DemoSceneBackend
    from .ui.main_window import MainWindow
    from .theme import FONT_FAMILIES, FONT_POINT_SIZE, stylesheet

    application = QApplication([sys.argv[0]])
    application.setStyle("Fusion")
    application.setApplicationName("Vision Tactile Capture Console")
    application.setOrganizationName("VisionTactile")
    initialize(override=getattr(args, "language", None))
    font = QFont()
    font.setFamilies(list(FONT_FAMILIES))
    font.setPointSize(FONT_POINT_SIZE)
    font.setStyleStrategy(QFont.StyleStrategy.PreferAntialias | QFont.StyleStrategy.NoSubpixelAntialias)
    application.setFont(font)
    application.setStyleSheet(stylesheet())
    runtime = None
    if args.desktop:
        from .backend.local_runtime import DesktopRuntime
        try:
            runtime = DesktopRuntime()
            runtime.start()
        except (OSError, ValueError, RuntimeError) as exc:
            QMessageBox.critical(None, tr("工作台无法启动"), tr("本地工作站设置无法加载：{v0}\n请检查部署目录与本机配置。未启动任何传感器。", v0=exc))
            return 1
    backend = DemoBackend() if args.demo else IpcBackend(runtime.socket_path if runtime else args.socket)
    if runtime is not None:
        backend.runtime = runtime
    scene_backend = DemoSceneBackend() if args.demo else SceneBackend(args.scene_socket, control_path=backend.path)
    window = MainWindow(backend, scene_backend=scene_backend, language_override=getattr(args, "language", None))
    if runtime is not None:
        from .ui.desktop_tools import DesktopTools
        window.desktop_tools = DesktopTools(window, backend, runtime)
        if not any(group.get("enabled") for group in runtime.configuration.get("process_groups", ())):
            window.navigation.button(2).click()
    window.resize(width, height)
    window.show()
    if instance is not None:
        def activate_existing_window():
            if instance.consume_activation():
                if window.isMinimized():
                    window.showNormal()
                else:
                    window.show()
                window.raise_()
                window.activateWindow()

        activation_timer = QTimer(window)
        activation_timer.setInterval(100)
        activation_timer.timeout.connect(activate_existing_window)
        activation_timer.start()
    exit_code = 0

    def finish():
        nonlocal exit_code
        if args.screenshot and not window.grab().save(args.screenshot):
            print(f"截图保存失败：{args.screenshot}", file=sys.stderr)
            exit_code = 1
        window.close()

    if args.smoke_seconds:
        QTimer.singleShot(max(1, int(args.smoke_seconds * 1000)), finish)
    result = application.exec()
    scene_backend.close()
    backend.close()
    if runtime is not None:
        runtime.close()  # Detach; closing the window must not stop data collection.
    return exit_code or result
