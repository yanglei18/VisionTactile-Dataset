"""Independent desktop entry point. No source is connected on startup."""
from __future__ import annotations

import argparse
import json
import multiprocessing
from pathlib import Path
import sys
import time


def parser():
    result = argparse.ArgumentParser(description="手势与触觉 / Hand Motion & Touch")
    result.add_argument("--resources", type=Path, help="Maintainer runtime bundle; packaged app uses its bundled resources")
    result.add_argument("--language", choices=("zh_CN", "en"))
    result.add_argument("--self-check", type=Path, help="Verify resources and the compute process without opening hardware")
    result.add_argument("--smoke-seconds", type=float, default=0, help=argparse.SUPPRESS)
    result.add_argument("--screenshot", type=Path, help=argparse.SUPPRESS)
    return result


def controller_for(resources, data_root):
    from vt_fsglove12.live_process import ProcessLiveController
    return ProcessLiveController(
        resources["config"], resources["rest"], resources["model_axes"],
        output_root=Path(data_root) / "calibration", mano_assets=resources["mano_assets"],
        upstream_checkout=resources["upstream_checkout"], hand="right", record_streams=False)


def _check_functional_numerics():
    """Run the actual QMT paths and profile provenance required by calibration."""
    import hashlib
    import inspect

    import numpy as np
    import qmt
    from qmt.functions.heading_correction import estimateDelta1d
    from scipy.spatial.transform import Rotation
    from vt_fsglove12.upstream_functional import _qmt_provenance

    # Known rotations exercise both batched projection and the current heading
    # window's real QMT constraint. These are dependency checks, not hand data.
    angles = np.array([-.4, 0., .7])
    rotations = Rotation.from_rotvec(angles[:, None] * np.array([0., 1., 0.]))
    projection = qmt.quatProject(rotations.as_quat()[:, [3, 0, 1, 2]],
                                 [0., 1., 0.], debug=False, plot=False)
    if (not np.allclose(projection["projAngle"], angles, atol=1e-10, rtol=0.)
            or not np.allclose(projection["resAngle"], 0., atol=1e-10, rtol=0.)):
        raise RuntimeError("QMT quaternion projection self-check failed")
    parent = Rotation.from_euler("x", np.linspace(-.4, .6, 12)[:, None])
    child = (Rotation.from_euler("z", -.2) * parent
             * Rotation.from_euler("y", np.linspace(0., .9, 12)[:, None]))
    delta, rating, cost = estimateDelta1d(parent.as_quat()[:, [3, 0, 1, 2]],
        child.as_quat()[:, [3, 0, 1, 2]], np.array([0., 1., 0.]), 0., "1d_corr", 5)
    if (not np.isfinite([delta, rating, cost]).all()
            or not np.isclose(delta, .2, atol=1e-10, rtol=0.) or rating <= 0.):
        raise RuntimeError("QMT relative heading self-check failed")
    # Functional fitting records the executed source digest. Frozen builds must
    # retain these original .py files, not merely importable bytecode.
    provenance = _qmt_provenance()
    if not provenance.get("source_file") or not provenance.get("source_sha256"):
        raise RuntimeError("QMT calibration source provenance is unavailable")
    source = inspect.getsourcefile(estimateDelta1d)
    if not source:
        raise RuntimeError("QMT relative heading source provenance is unavailable")
    return dict(status="ok", projection=provenance,
                relative_heading=dict(function="qmt.functions.heading_correction.estimateDelta1d",
                    source_file=source, source_sha256=hashlib.sha256(Path(source).read_bytes()).hexdigest(),
                    constraint="1d_corr", recovered_offset_rad=float(delta)))


def check_installation(path, resource_root=None):
    """Exercise the actual spawned solver initialization, without serial I/O."""
    import tempfile
    from .resources import load_resources
    from . import __version__

    report = dict(schema_version=1, status="ok", issues=[], details=dict(
        version=__version__, frozen=bool(getattr(sys, "frozen", False)), serial_device="not_opened"))
    controller = None
    try:
        # Establish one Qt binding even in checks; importing creates no desktop.
        import PySide6
        from vt_fsglove12.hand_view import HandView
        from vt_fsglove12.tactile_surface import TactileSurfaceBinding
        resources = load_resources(resource_root)
        binding = TactileSurfaceBinding(resources["neutral_geometry"], resources["model_axes"], hand="right")
        report["details"].update(qt_binding="PySide6", reference_scope="layout_or_device",
                                 tactile_mapping=binding.metadata(), recording="calibration_only")
        report["details"]["functional_numerics"] = _check_functional_numerics()
        with tempfile.TemporaryDirectory(prefix="hand-motion-check-") as directory:
            controller = controller_for(resources, directory)
            controller.start()
            deadline = time.perf_counter() + 90
            while time.perf_counter() < deadline:
                snapshot = controller.latest_snapshot()
                runtime = snapshot["runtime"]
                if runtime.get("error"):
                    raise RuntimeError(runtime["error"])
                if runtime.get("ipc_sequence", 0) > 0:
                    if runtime.get("connected"):
                        raise RuntimeError("Installation check unexpectedly connected a source")
                    report["details"]["solver_process"] = "initialized_without_device"
                    break
                if not controller.is_alive():
                    raise RuntimeError("Compute process exited before initialization")
                time.sleep(.02)
            else:
                raise RuntimeError("Compute initialization timed out")
            controller.stop()
            if not controller.join(timeout=15):
                raise RuntimeError("Compute process did not close")
            controller = None
    except Exception as exc:
        report["status"] = "error"
        report["issues"].append(dict(type=type(exc).__name__, message=str(exc)[:4096]))
    finally:
        if controller is not None:
            controller.stop()
            controller.join(timeout=15)
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(report, ensure_ascii=False, indent=2) + "\n", encoding="utf-8")
    return 0 if report["status"] == "ok" else 1


def main(argv=None):
    multiprocessing.freeze_support()
    args = parser().parse_args(argv)
    import math
    if not math.isfinite(args.smoke_seconds) or args.smoke_seconds < 0:
        raise SystemExit("--smoke-seconds must be finite and nonnegative")
    if args.screenshot and not args.smoke_seconds:
        raise SystemExit("--screenshot requires --smoke-seconds")
    if args.self_check:
        return check_installation(args.self_check, args.resources)
    from PySide6 import QtCore, QtGui, QtWidgets
    from vt_capture_console.backend.tactile_serial import TactileSerialSource
    from vt_capture_console.theme import stylesheet
    from .resources import load_resources
    from .settings import Settings
    from .window import HandTactileWindow

    app = QtWidgets.QApplication([sys.argv[0]])
    app.setApplicationName("Hand Motion & Touch")
    app.setOrganizationName("VisionTactile")
    app.setStyle("Fusion")
    font = QtGui.QFont()
    font.setFamilies(["Inter", "Noto Sans", "Microsoft YaHei UI", "Noto Sans CJK SC"])
    font.setPointSize(10)
    app.setFont(font)
    app.setStyleSheet(stylesheet())
    settings = Settings()
    if args.language:
        settings.save_language(args.language)
    try:
        resources = load_resources(args.resources)
    except (OSError, ValueError, ImportError) as exc:
        title = "模型资源无法加载" if settings.language == "zh_CN" else "Cannot load model resources"
        QtWidgets.QMessageBox.critical(None, title, str(exc))
        return 1
    controller = controller_for(resources, settings.data_root)
    tactile = TactileSerialSource(reference_store=settings.reference_store(resources["response_profile"]))
    window = HandTactileWindow(settings, resources, controller, tactile)
    window.show()
    window.start()
    result = [0]
    if args.smoke_seconds:
        def finish():
            if args.screenshot and not window.grab().save(str(args.screenshot)):
                result[0] = 1
            window.close()
        QtCore.QTimer.singleShot(max(1, int(args.smoke_seconds * 1000)), finish)
    try:
        return app.exec() or result[0]
    finally:
        controller.stop()
        controller.join(timeout=10)
        tactile.close()


if __name__ == "__main__":
    multiprocessing.freeze_support()
    raise SystemExit(main())
