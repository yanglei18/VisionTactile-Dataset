#!/usr/bin/env python3
"""Explicit synthetic MANO/Qt rendering check; no ROS or hardware connection.

Run with the GUI environment and trusted, separately provided MANO assets.
The default captures three labeled views and closes only its own window.
"""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys
import time

import numpy as np

sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "src"))


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mano-assets", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--show", action="store_true", help="Keep this synthetic preview open after saving.")
    args = parser.parse_args()
    outputs = ("neutral_palmar_synthetic.png", "bent_palmar_synthetic.png", "neutral_dorsal_synthetic.png", "validation.json")
    if any((args.output_dir / name).exists() for name in outputs):
        parser.error("output files already exist; choose another output directory")
    args.output_dir.mkdir(parents=True, exist_ok=True)

    from PyQt5 import QtCore, QtGui, QtWidgets
    from OpenGL.GL import GL_VERSION, glGetString
    from vt_fsglove12.guided_calibration import model_motion_axes
    from vt_fsglove12.live_ui import HandView
    from vt_fsglove12.mano import ManoBackend
    from vt_fsglove12.tactile_surface import TactileSurfaceBinding

    backend = ManoBackend(args.mano_assets)
    neutral = backend.render(np.zeros((16, 3)))
    angles = np.zeros((16, 3))
    for slot in (1, 4, 7, 10):
        angles[slot:slot + 3, 2] = [.40, .65, .32]
    bent = backend.render(angles)
    axes = model_motion_axes(neutral["joints_m"])
    binding = TactileSurfaceBinding(neutral, axes)
    delta = np.zeros((20, 20), dtype=np.float64)
    delta[13:20, 12:16] = 2800
    delta[3:6, 7:13] = 2200
    delta[10:20, 0:4] = -2300
    mask = np.ones((20, 20), dtype=bool)
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([sys.argv[0]])
    window = QtWidgets.QWidget()
    window.setWindowTitle("SYNTHETIC tactile surface validation — no ROS / no hardware")
    layout = QtWidgets.QVBoxLayout(window)
    title = QtWidgets.QLabel()
    title.setStyleSheet("font-size: 18px; font-weight: bold; padding: 8px; color: #9a4100;")
    layout.addWidget(title)
    subtitle = QtWidgets.QLabel("SYNTHETIC ΔADC: index +2800 / palm +2200 / little −2300. Candidate locations; not pressure or live sensor data.")
    subtitle.setWordWrap(True)
    layout.addWidget(subtitle)
    view = HandView(neutral, axes)
    view.tactile_binding = binding
    view.show_joints = False
    view.joints.setVisible(False)
    layout.addWidget(view, 1)
    footer = QtWidgets.QLabel("Fixed face + barycentric binding. No current-world nearest-neighbour matching. No serial or ROS subscribers started.")
    footer.setWordWrap(True)
    layout.addWidget(footer)
    window.resize(1040, 860)
    window.show()
    report = {"synthetic": True, "hardware_opened": False, "ros_started": False,
              "binding": binding.metadata(), "model_axes": axes, "images": [], "errors": []}
    stages = (("neutral_palmar", neutral, True), ("bent_palmar", bent, True), ("neutral_dorsal", neutral, False))
    started = time.perf_counter()
    dorsal_pixels = None

    def pixels(image):
        converted = image.convertToFormat(QtGui.QImage.Format_RGBA8888)
        data = converted.bits(); data.setsize(converted.byteCount())
        return np.frombuffer(data, np.uint8).reshape(converted.height(), converted.width(), 4).copy()

    def failed(error):
        report["errors"].append(f"{type(error).__name__}: {error}")
        finish()

    def finish():
        report["elapsed_s"] = time.perf_counter() - started
        (args.output_dir / "validation.json").write_text(json.dumps(report, indent=2) + "\n", encoding="utf-8")
        print(json.dumps(report, ensure_ascii=False), flush=True)
        if not args.show or report["errors"]:
            window.close()
            app.quit()

    def compare_dorsal():
        try:
            plain = pixels(view.grabFramebuffer())
            difference = np.max(np.abs(plain.astype(int) - dorsal_pixels.astype(int)), axis=2)
            report["dorsal_color_difference_pixels_over_5"] = int(np.count_nonzero(difference > 5))
            report["dorsal_comparison_note"] = "Same geometry/camera, overlay enabled versus disabled; oblique view may expose palmar sidewalls."
            view.set_tactile(delta, mask)
            finish()
        except Exception as error:
            failed(error)

    def capture(index):
        nonlocal dorsal_pixels
        try:
            if not view.isValid():
                raise RuntimeError("Qt did not create a valid OpenGL context")
            view.makeCurrent()
            report["opengl_version"] = (glGetString(GL_VERSION) or b"unknown").decode(errors="replace")
            view.doneCurrent()
            framebuffer = view.grabFramebuffer()
            if framebuffer.isNull() or np.std(pixels(framebuffer)[:, :, :3]) < 2:
                raise RuntimeError("OpenGL framebuffer is empty or uniform")
            name = stages[index][0] + "_synthetic.png"
            if not window.grab().save(str(args.output_dir / name)):
                raise RuntimeError("Failed to save labeled Qt screenshot")
            report["images"].append(name)
            if index < len(stages) - 1:
                render(index + 1)
            else:
                dorsal_pixels = pixels(framebuffer)
                view.set_tactile()
                QtCore.QTimer.singleShot(400, compare_dorsal)
        except Exception as error:
            failed(error)

    def render(index):
        try:
            label, geometry, palmar = stages[index]
            title.setText("SYNTHETIC — " + label.replace("_", " ") + " — NOT live data")
            view.set_geometry(geometry)
            view.reset_camera(palmar=palmar)
            view.set_tactile(delta, mask)
            QtCore.QTimer.singleShot(500, lambda: capture(index))
        except Exception as error:
            failed(error)

    QtCore.QTimer.singleShot(100, lambda: render(0))
    app.exec_()
    return 1 if report["errors"] else 0


if __name__ == "__main__":
    raise SystemExit(main())
