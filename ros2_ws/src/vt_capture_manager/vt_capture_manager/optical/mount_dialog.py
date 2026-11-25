"""Chinese three-click calibration of a cube attached to the back of one hand."""
from pathlib import Path
import time

import cv2
import numpy as np
from PyQt5 import QtCore, QtWidgets

from vt_capture_core.spatial.transforms import RigidTransform
from .mount_calibration import fit_mount, save_mount
from .mount_depth import MountDepthCapture
from .mount_model import load_mount_model
from .preview_image import Preview, draw_hand


LANDMARKS = ('手腕中央', '食指指根', '小指指根')
SIDE_NAMES = {'left': '左手', 'right': '右手'}


def camera_from_cube(observation):
    return RigidTransform(observation.translation_m, observation.quaternion_xyzw)


def hand_to_freeze(row, hand):
    _, _, _, _, observations, results, _ = row
    assigned = results[hand].observation
    if assigned is not None:
        return assigned
    if len(observations) == 1 and not any(value.observation for value in results.values()):
        return observations[0]
    raise ValueError('请只把所选手放入画面，或先在跟踪窗口绑定这只手。')


class MountDialog(QtWidgets.QDialog):
    def __init__(self, parent, node, recent_frames, resources, path):
        super().__init__(parent)
        self.node, self.recent_frames, self.resources = node, recent_frames, resources
        self.path = Path(path)
        self.model = self.frozen = self.depth_frame = self.reference = self.estimate = None
        self.clicks = []
        self.positions = []
        self.deadline = None
        self.last_live = None
        self.setWindowTitle('手背方块标定')
        self.resize(850, 650)
        layout = QtWidgets.QVBoxLayout(self)
        controls = QtWidgets.QHBoxLayout()
        layout.addLayout(controls)
        controls.addWidget(QtWidgets.QLabel('标定哪只手'))
        self.side = QtWidgets.QComboBox()
        for hand, label in SIDE_NAMES.items():
            self.side.addItem(label, hand)
        controls.addWidget(self.side)
        controls.addStretch()
        self.freeze_button = QtWidgets.QPushButton('定格画面')
        self.freeze_button.clicked.connect(self.request_frame)
        controls.addWidget(self.freeze_button)
        self.preview = Preview()
        self.preview.clicked.connect(self.pick)
        self.preview.setText('等待头部相机画面')
        layout.addWidget(self.preview, 1)
        self.note = QtWidgets.QLabel('把方块贴在所选手的手背上，手背朝向相机，自然伸平，然后定格。')
        self.note.setTextFormat(QtCore.Qt.PlainText)
        self.note.setWordWrap(True)
        layout.addWidget(self.note)
        buttons = QtWidgets.QHBoxLayout()
        layout.addLayout(buttons)
        self.undo = QtWidgets.QPushButton('撤销上一点')
        self.undo.clicked.connect(self.undo_point)
        self.undo.setEnabled(False)
        buttons.addWidget(self.undo)
        buttons.addStretch()
        cancel = QtWidgets.QPushButton('取消')
        cancel.clicked.connect(self.reject)
        buttons.addWidget(cancel)
        self.save_button = QtWidgets.QPushButton('保存并使用')
        self.save_button.setEnabled(False)
        self.save_button.clicked.connect(self.save)
        buttons.addWidget(self.save_button)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.depth = MountDepthCapture(node, node.camera_name)
        self.timer.start(80)

    @property
    def hand(self):
        return self.side.currentData()

    def finish_capture(self):
        self.timer.stop()
        self.depth.close()

    def request_frame(self):
        # Discard points only when their image is explicitly replaced.
        if self.frozen is not None:
            self.frozen = self.estimate = self.depth_frame = self.reference = None
            self.clicks.clear()
            self.positions.clear()
            self.undo.setEnabled(False)
            self.save_button.setEnabled(False)
            self.side.setEnabled(True)
            self.freeze_button.setText('定格画面')
            self.last_live = None
            self.note.setText('手自然伸平，准备好后点击定格画面。')
            self.refresh()
            return
        try:
            self.model = load_mount_model(self.resources.get(self.hand), self.hand)
        except (ValueError, OSError) as exc:
            self.note.setText('无法读取这只手的模型。' + str(exc))
            return
        self.deadline = time.monotonic() + 3
        self.side.setEnabled(False)
        self.freeze_button.setEnabled(False)
        self.refresh()

    def refresh(self):
        if self.frozen is not None and self.estimate is None:
            return
        rows = [r for r in self.recent_frames() if time.monotonic() - r[0] < .25]
        if self.deadline is not None:
            reason = '等待头部相机的彩色画面，请确认相机已启动。'
            for row in reversed(rows):
                try:
                    observation = hand_to_freeze(row, self.hand)
                    _, image, camera, distortion, _, results, stamp = row
                    depth_frame = self.depth.freeze(stamp, camera, distortion, (image.shape[1], image.shape[0]))
                    if results[self.hand].observation is None:
                        self.node.bind_hand_at(self.hand, observation.center_px, stamp)
                except (ValueError, OSError) as exc:
                    reason = str(exc)
                    continue
                self.frozen, self.depth_frame = row, depth_frame
                self.reference = camera_from_cube(observation)
                self.deadline = None
                self.freeze_button.setEnabled(True)
                self.freeze_button.setText('重新取景')
                self.paint_frozen()
                self.prompt()
                return
            self.note.setText(reason)
            if time.monotonic() >= self.deadline:
                self.deadline = None
                self.side.setEnabled(True)
                self.freeze_button.setEnabled(True)
            return
        if self.estimate is not None:
            self.show_review(rows[-1] if rows else None)
        elif rows:
            row = rows[-1]
            if self.last_live != row[-1]:
                self.preview.show_frame(row[1], row[-1])
                self.last_live = row[-1]
        else:
            self.preview.source_geometry = None
            self.preview.setText('等待头部相机画面')
            self.last_live = None

    def paint_frozen(self):
        _, image, camera, distortion, _, _, stamp = self.frozen
        frame = image.copy()
        for i, xy in enumerate(self.clicks, 1):
            x, y = (int(round(p)) for p in xy)
            cv2.circle(frame, (x, y), 6, (45, 205, 255), 2, cv2.LINE_AA)
            cv2.putText(frame, str(i), (x+8, y-7), cv2.FONT_HERSHEY_SIMPLEX, .6,
                        (45, 205, 255), 2, cv2.LINE_AA)
        if self.estimate is not None:
            self.overlay(frame, camera, distortion, self.reference)
        self.preview.show_frame(frame, stamp)

    def overlay(self, frame, camera, distortion, cube_pose):
        wrist = cube_pose.compose(self.estimate)
        joints = np.array([wrist.apply(p) for p in self.model.joints_m])
        draw_hand(frame, camera, distortion, joints, (80, 220, 120))

    def show_review(self, row):
        obs = row[5][self.hand].observation if row is not None else None
        if obs is None or row[-1] == self.frozen[-1]:
            if self.last_live is not None:
                self.paint_frozen()
                self.last_live = None
            self.note.setText('正在显示标点时的定格核对图。让所选手的方块保持可见，即可查看实时对齐。')
            return
        if self.last_live != row[-1]:
            frame = row[1].copy()
            self.overlay(frame, row[2], row[3], camera_from_cube(obs))
            self.preview.show_frame(frame, row[-1])
            self.last_live = row[-1]
        self.note.setText('保持手伸平，从侧面看一下绿色骨架。腕部和指根对齐后，点击“保存并使用”。')

    def prompt(self):
        index = len(self.clicks)
        self.note.setText(f'{index+1}/3  请点击{SIDE_NAMES[self.hand]}的{LANDMARKS[index]}。'
                          + ('手腕与手背交界处的中点。' if index == 0 else '手指与手掌相接的关节。'))

    def pick(self, xy, stamp):
        if self.frozen is None or self.estimate is not None or stamp != self.frozen[-1]:
            return
        try:
            point = self.depth_frame.point_at(xy)
            if len(self.positions) == 2:
                self.estimate = fit_mount(self.hand, self.model.joints_m, [*self.positions, point], self.reference)
        except (ValueError, OSError) as exc:
            self.note.setText(str(exc) + f' 请重新点击{LANDMARKS[len(self.positions)]}，也可以撤销上一点。')
            return
        self.clicks.append(xy)
        self.positions.append(point)
        self.undo.setEnabled(True)
        self.paint_frozen()
        if self.estimate is None:
            self.prompt()
        else:
            self.last_live = None
            self.save_button.setEnabled(True)
            self.note.setText('绿色骨架为伸平手模，请核对位置和朝向后保存。')

    def undo_point(self):
        if self.clicks:
            self.clicks.pop()
            self.positions.pop()
        self.estimate = self.last_live = None
        self.save_button.setEnabled(False)
        self.undo.setEnabled(bool(self.clicks))
        self.paint_frozen()
        self.prompt()

    def save(self):
        if self.estimate is None:
            return
        try:
            save_mount(self.path, self.hand, self.node.layout_sha256, self.estimate,
                       model_rest_sha256=self.model.rest_sha256)
        except (ValueError, OSError) as exc:
            self.note.setText('保存失败。' + str(exc))
            return
        self.accept()
