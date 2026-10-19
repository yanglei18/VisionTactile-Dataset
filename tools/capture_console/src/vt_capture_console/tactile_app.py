"""Standalone serial tactile display; no ROS or workstation runtime required."""
from __future__ import annotations

import argparse
from pathlib import Path
import sys


def parser():
    result = argparse.ArgumentParser(description="独立右手触觉串口工具：复用主采集程序的原始 ADC 三维预览。")
    result.add_argument("--port", default="", help="仅预填触觉串口，不自动连接")
    result.add_argument("--baudrate", type=int, default=115200, help="串口波特率，默认 115200")
    result.add_argument("--language", choices=("zh_CN", "en"), help="界面语言 / UI language (one launch only)")
    result.add_argument("--glove-python", help="加载 MANO 的 Python；默认读取本工具独立设置")
    result.add_argument("--mano-assets", help="可信 MANO 资源目录；默认读取本工具独立设置")
    result.add_argument("--model-bundle", type=Path, help="静态手模型 JSON；安装版自动使用内置资源")
    result.add_argument("--self-check", type=Path, help="检查安装资源并写入 JSON 报告，不打开串口")
    result.add_argument("--smoke-seconds", type=float, default=0, help=argparse.SUPPRESS)
    result.add_argument("--screenshot", type=Path, help=argparse.SUPPRESS)
    return result


def initialize_language(settings, override=None):
    """Use only the viewer's preferences; a CLI override never overwrites them."""
    from PySide6.QtCore import QLocale
    from .i18n import initialize

    if override is None and not settings.contains("ui/language"):
        override = "zh_CN" if QLocale.system().language() == QLocale.Language.Chinese else "en"
    initialize(settings, override=override)


def main(argv=None):
    import math

    arguments = parser()
    args = arguments.parse_args(argv)
    if not 1 <= args.baudrate <= 4_000_000:
        arguments.error("波特率必须在 1 到 4000000 之间")
    if not math.isfinite(args.smoke_seconds) or args.smoke_seconds < 0:
        arguments.error("冒烟检查时长必须为有限非负数")
    if args.screenshot and not args.smoke_seconds:
        arguments.error("截图参数须与正数 --smoke-seconds 一起使用")
    if args.model_bundle and (args.glove_python or args.mano_assets):
        arguments.error("静态模型与外部模型运行环境不能同时指定")
    if args.self_check:
        if args.smoke_seconds or args.screenshot or args.glove_python or args.mano_assets:
            arguments.error("安装自检不能与窗口预览或外部模型运行环境同时使用")
        from .backend.tactile_installation import check_installation
        return check_installation(args.self_check, model_bundle=args.model_bundle)
    from PySide6 import QtCore, QtGui, QtWidgets
    from .backend.tactile_serial import TactileSerialSource
    from .backend.tactile_settings import TactileSettings
    from .i18n import tr
    from .i18n.messages import message
    from .theme import stylesheet
    from .ui.tactile_window import TactileWindow
    from .ui.widgets.tactile_model_loader import model_loader

    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName("Vision Tactile 3D Viewer")
    app.setOrganizationName("VisionTactile")
    app.setDesktopFileName("visiontactile-tactile-3d")
    app.setStyle("Fusion")
    font = QtGui.QFont()
    font.setFamilies(["Inter", "Noto Sans", "Microsoft YaHei UI", "Noto Sans CJK SC"])
    font.setPointSize(10)
    app.setFont(font)
    app.setStyleSheet(stylesheet())
    source = window = loader = None
    result = 0
    try:
        settings = TactileSettings(glove_python=args.glove_python, mano_assets=args.mano_assets,
                                   model_bundle=args.model_bundle)
        initialize_language(settings.store, args.language)
        loader = model_loader(lambda: settings.configuration)
        source = TactileSerialSource()
        window = TactileWindow(source, port=args.port, baudrate=args.baudrate, model_settings=settings)
        app.aboutToQuit.connect(window.shutdown)
        window.show()
        window.start()

        def finish():
            nonlocal result
            if args.screenshot and not window.grab().save(str(args.screenshot)):
                print(f"截图保存失败：{args.screenshot}", file=sys.stderr)
                result = 1
            window.close()

        if args.smoke_seconds:
            QtCore.QTimer.singleShot(max(1, int(args.smoke_seconds * 1000)), finish)
        result = app.exec() or result
    except (OSError, RuntimeError, ValueError) as exc:
        QtWidgets.QMessageBox.critical(None, str(tr("触觉预览无法启动")), str(message(str(exc))))
        result = 1
    finally:
        if window is not None:
            window.shutdown()
        elif source is not None:
            source.close()
        if loader is not None:
            loader.shutdown()
    return result


if __name__ == "__main__":
    raise SystemExit(main())
