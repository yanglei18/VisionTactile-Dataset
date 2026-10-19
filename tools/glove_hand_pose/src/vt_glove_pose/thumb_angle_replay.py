"""Play the recorded conditional thumb bend without opening a serial port."""
from __future__ import annotations

import argparse
from bisect import bisect_right
import json
import math
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from .thumb_angle_stream import validate_candidate
from .thumb_angle_widget import ThumbAngleWidget


def validate_replay(data):
    if (not isinstance(data, dict) or data.get("kind") != "vt_glove_pose.thumb_angle_replay"
            or type(data.get("schema_version")) is not int or data["schema_version"] != 1
            or data.get("recorded_only") is not True):
        raise ValueError("expected a recorded schema 1 thumb-angle replay")
    profile = validate_candidate(data.get("profile"))
    rows = data.get("series")
    if not isinstance(rows, list) or not 1 <= len(rows) <= 30000:
        raise ValueError("replay must contain 1..30000 samples")
    previous = -math.inf
    for row in rows:
        if not isinstance(row, dict):
            raise ValueError("replay samples must be objects")
        stamp = row.get("time_s")
        if type(stamp) not in (int, float) or not math.isfinite(stamp) or stamp < previous:
            raise ValueError("replay times must be finite and nondecreasing")
        previous = stamp
        angle = row.get("angle_deg")
        if angle is not None and (type(angle) not in (int, float) or not math.isfinite(angle)):
            raise ValueError("replay angle must be finite or null")
        if not isinstance(row.get("quality"), str):
            raise ValueError("replay sample must have quality")
        if row.get("node_ids") != profile["node_ids"]:
            raise ValueError("replay node IDs must match its candidate")
    return data


class ThumbAngleReplayWindow(QtWidgets.QMainWindow):
    def __init__(self, data):
        super().__init__()
        self.data = validate_replay(data)
        self.rows = data["series"]
        self.times = [row["time_s"] for row in self.rows]
        self.position_s = self.times[0]
        self.playing = False
        self.elapsed = QtCore.QElapsedTimer()
        self.setWindowTitle("拇指弯曲回放 — 已保存的真实数据 · 非实时")
        self.resize(1060, 720)
        central = QtWidgets.QWidget()
        self.setCentralWidget(central)
        layout = QtWidgets.QVBoxLayout(central)
        intro = QtWidgets.QLabel(
            "已保存的真实 IMU 数据 · 拇指相对弯曲量（实验）\n"
            "冻结运动轴后连续解算；不是两块 IMU 的独立旋转量，也不是已验证的解剖角。\n"
            "本窗口不连接硬件；拖动时间轴可查看弯伸、预测区间和回位，不会重设零位。")
        intro.setWordWrap(True)
        layout.addWidget(intro)
        self.angle = ThumbAngleWidget()
        layout.addWidget(self.angle, 1)
        controls = QtWidgets.QHBoxLayout()
        self.play_button = QtWidgets.QPushButton("播放")
        self.play_button.clicked.connect(self.toggle_play)
        controls.addWidget(self.play_button)
        self.slider = QtWidgets.QSlider(QtCore.Qt.Horizontal)
        self.slider.setRange(0, max(1, round((self.times[-1] - self.times[0]) * 1000)))
        self.slider.valueChanged.connect(self.seek)
        controls.addWidget(self.slider, 1)
        self.time_label = QtWidgets.QLabel()
        controls.addWidget(self.time_label)
        self.speed = QtWidgets.QComboBox()
        for value in (.5, 1., 2.):
            self.speed.addItem(f"{value:g}×", value)
        self.speed.setCurrentIndex(1)
        controls.addWidget(self.speed)
        layout.addLayout(controls)
        protocol = data["profile"].get("protocol", {})
        self.protocol_label = QtWidgets.QLabel(
            f"训练区间：{protocol.get('train_s', '未记录')} 秒　"
            f"独立留出：{protocol.get('holdout_s', '未记录')} 秒　"
            f"原始轴拟合判定：{data['profile']['axis_fit'].get('candidate_status', '未记录')}\n"
            "绿色仅表示当前有模型观测支持；黄色为预测。两者都不代表已验证精度。")
        self.protocol_label.setWordWrap(True)
        layout.addWidget(self.protocol_label)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.advance)
        self.timer.start(33)
        self.render()

    def render(self):
        index = max(0, bisect_right(self.times, self.position_s) - 1)
        current = dict(self.rows[index])
        age = self.position_s - self.times[index]
        if age > self.data["profile"]["config"]["stale_after_s"]:
            current.update(angle_deg=None, paired_fresh=False, current_angle_observed=False,
                           quality="unavailable", reasons=["recorded_data_gap_at_playback_cursor"])
        self.angle.set_state(current, self.rows[max(0, index - 1800):index + 1])
        self.time_label.setText(f"{self.position_s:.2f} / {self.times[-1]:.2f} 秒")
        self.slider.blockSignals(True)
        self.slider.setValue(round((self.position_s - self.times[0]) * 1000))
        self.slider.blockSignals(False)

    def seek(self, value):
        self.position_s = min(self.times[-1], self.times[0] + value / 1000.)
        self.elapsed.restart()
        self.render()

    def toggle_play(self):
        self.playing = not self.playing
        if self.playing and self.position_s >= self.times[-1]:
            self.position_s = self.times[0]
        self.elapsed.restart()
        self.play_button.setText("暂停" if self.playing else "播放")

    def advance(self):
        if not self.playing:
            return
        self.position_s = min(self.times[-1], self.position_s + self.elapsed.restart() / 1000. * self.speed.currentData())
        if self.position_s >= self.times[-1]:
            self.playing = False
            self.play_button.setText("重新播放")
        self.render()

    def closeEvent(self, event):
        self.timer.stop()
        event.accept()


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("replay", type=Path)
    args = parser.parse_args(argv)
    try:
        if args.replay.stat().st_size > 64 * 1024 * 1024:
            raise ValueError("replay file exceeds 64 MiB")
        data = validate_replay(json.loads(args.replay.read_text(encoding="utf-8")))
    except (OSError, ValueError, KeyError) as exc:
        parser.error(str(exc))
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    window = ThumbAngleReplayWindow(data)
    window.show()
    return app.exec_()


if __name__ == "__main__":
    raise SystemExit(main())
