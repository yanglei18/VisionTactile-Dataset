"""Operator window for the optional camera-relative hand cube tracker."""
import argparse
import os
from collections import deque
from concurrent.futures import ThreadPoolExecutor
from pathlib import Path
import signal
import sys
import threading
import time

import cv2
import numpy as np
from PyQt5 import QtCore, QtWidgets
import rclpy
from rclpy.executors import SingleThreadedExecutor
from rclpy.parameter import Parameter
from rclpy.signals import SignalHandlerOptions

from .node import HandCubeNode
from .preview_image import Preview, draw_hand


# Display-only limits. Image capture and the fusion publisher run independently.
FUSION_PREVIEW_WAIT_S = .12
PREVIEW_EXPIRY_S = .5


def select_preview_frame(frames, fused, now, *, wait_for_fusion):
    """Prefer the newest exact pair, allowing unmatched frames to age into view."""
    for frame in reversed(frames):
        age = now - frame[0]
        if age > PREVIEW_EXPIRY_S:
            break
        if not wait_for_fusion or len(fused.get(frame[-1], {})) == 2 or age >= FUSION_PREVIEW_WAIT_S:
            return frame
    # At high frame rates the bounded queue may fill before its oldest frame times out.
    if frames and len(frames) == frames.maxlen and now - frames[0][0] <= PREVIEW_EXPIRY_S:
        return frames[0]
    return None


def annotated_preview(image, camera, distortion, observations, results):
    """Draw only on the display copy. The recorded source image stays pristine."""
    frame = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR) if image.ndim == 2 else image.copy()
    bound = {id(value.observation): side for side, value in results.items() if value.observation is not None}
    for obs in observations:
        x, y, z, w = obs.quaternion_xyzw
        vector = np.array([x, y, z])
        norm = float(np.linalg.norm(vector))
        rvec = vector * (2 * np.arctan2(norm, w) / norm) if norm else vector
        cv2.drawFrameAxes(frame, camera, distortion, rvec, np.asarray(obs.translation_m), .022, 2)
        px, py = (int(round(n)) for n in obs.center_px)
        side = bound.get(id(obs), '?')
        color = {'left': (255, 208, 58), 'right': (70, 166, 255)}.get(side, (215, 215, 215))
        cv2.circle(frame, (px, py), 10, color, 2, cv2.LINE_AA)
        text = '{}  {}'.format(side.upper(), ','.join(map(str, obs.tag_ids)))
        cv2.putText(frame, text, (px+12, py-10), cv2.FONT_HERSHEY_SIMPLEX, .65, (30, 30, 30), 4, cv2.LINE_AA)
        cv2.putText(frame, text, (px+12, py-10), cv2.FONT_HERSHEY_SIMPLEX, .65, color, 1, cv2.LINE_AA)
    return frame


def local_config(name):
    folder = Path(os.environ.get('XDG_CONFIG_HOME') or Path.home() / '.config')
    return folder / 'visiontactile' / 'hand-cubes' / name


class Viewer(QtWidgets.QWidget):
    def __init__(self, *, camera_name, layout_path, language, mount_path="", resources=None):
        super().__init__()
        self.english = language == 'en'
        self.resources = resources or {}
        self.default_mount = local_config('wrist-mounts.json')
        self.mount_dialog = None
        self.node = self.fusion_node = self.executor = self.spin_thread = None
        self.learner = self.learning_job = None
        self.learning_pool = ThreadPoolExecutor(max_workers=1)
        self.inbox_lock = threading.Lock()
        self.frames = deque(maxlen=7)
        self.fused = {}
        self.last_drawn = None
        self.selected = None
        self.setWindowTitle(self.t('双手方块跟踪', 'Bilateral cube tracking'))
        self.resize(920, 690)
        layout = QtWidgets.QVBoxLayout(self)
        top = QtWidgets.QHBoxLayout()
        layout.addLayout(top)
        top.addWidget(QtWidgets.QLabel(self.t('头部相机', 'Head camera')))
        self.camera = QtWidgets.QComboBox()
        self.camera.setEditable(True)
        self.camera.addItems(['d436', 'd405_1', 'd405_2'])
        self.camera.setCurrentText(camera_name)
        top.addWidget(self.camera)
        self.path = QtWidgets.QLineEdit(layout_path)
        self.path.setPlaceholderText(self.t('内置布局 · 2、5 号面', 'Built-in layout · faces 2, 5'))
        top.addWidget(self.path, 1)
        self.browse = QtWidgets.QPushButton(self.t('选择布局', 'Choose layout'))
        self.browse.clicked.connect(self.choose_layout)
        top.addWidget(self.browse)
        self.run = QtWidgets.QPushButton(self.t('开始跟踪', 'Start tracking'))
        self.run.clicked.connect(self.toggle)
        top.addWidget(self.run)
        mount_row = QtWidgets.QHBoxLayout()
        layout.addLayout(mount_row)
        self.mount = QtWidgets.QLineEdit(mount_path or (str(self.default_mount) if self.default_mount.is_file() else ''))
        self.mount.setPlaceholderText(self.t('可选 · 方块到模型腕部的安装参数', 'Optional cube-to-model wrist mount'))
        self.mount_browse = QtWidgets.QPushButton(self.t('选择腕部安装参数', 'Choose wrist mount'))
        self.mount_browse.clicked.connect(self.choose_mount)
        mount_row.addWidget(self.mount, 1)
        mount_row.addWidget(self.mount_browse)
        self.calibrate = QtWidgets.QPushButton('手背标定')
        self.calibrate.clicked.connect(self.open_mount_dialog)
        mount_row.addWidget(self.calibrate)
        self.preview = Preview()
        self.preview.clicked.connect(self.pick)
        layout.addWidget(self.preview, 1)
        row = QtWidgets.QHBoxLayout()
        layout.addLayout(row)
        self.bind_buttons = []
        for hand, cn in (('left', '左手'), ('right', '右手')):
            button = QtWidgets.QPushButton(self.t('绑定'+cn, 'Bind '+hand))
            button.clicked.connect(lambda checked, h=hand: self.arm(h))
            row.addWidget(button)
            self.bind_buttons.append(button)
        self.clear = QtWidgets.QPushButton(self.t('清除绑定', 'Clear bindings'))
        self.clear.clicked.connect(self.clear_bindings)
        row.addWidget(self.clear)
        row.addStretch()
        self.learn = QtWidgets.QPushButton(self.t('识别六面布局', 'Learn cube layout'))
        self.learn.clicked.connect(self.toggle_learning)
        row.addWidget(self.learn)
        self.save = QtWidgets.QPushButton(self.t('保存布局', 'Save layout'))
        self.save.clicked.connect(self.save_layout)
        row.addWidget(self.save)
        self.readout = QtWidgets.QLabel()
        layout.addWidget(self.readout)
        self.notice = QtWidgets.QLabel()
        self.notice.setWordWrap(True)
        self.notice.setTextFormat(QtCore.Qt.PlainText)
        layout.addWidget(self.notice)
        self.timer = QtCore.QTimer(self)
        self.timer.timeout.connect(self.refresh)
        self.timer.start(65)
        self.sync_controls()

    def t(self, chinese, english):
        return english if self.english else chinese

    def sync_controls(self):
        live = self.node is not None
        for widget in (self.camera, self.path, self.browse, self.mount, self.mount_browse):
            widget.setEnabled(not live)
        for widget in (*self.bind_buttons, self.clear, self.learn):
            widget.setEnabled(live)
        self.calibrate.setEnabled(self.learner is None)
        self.save.setEnabled(self.learning_job is None and self.learner is not None and self.learner.complete)
        self.run.setText(self.t('停止跟踪', 'Stop tracking') if live else self.t('开始跟踪', 'Start tracking'))
        self.learn.setText(self.t('结束布局识别', 'End layout learning') if self.learner else self.t('识别六面布局','Learn cube layout'))
        if not live:
            self.invalidate_preview(self.t('请选择头部相机并开始跟踪', 'Select the head camera and start tracking'))

    def invalidate_preview(self, note):
        self.preview.source_geometry = None
        self.preview.setText(note)
        self.readout.clear()

    def choose_layout(self):
        name, _ = QtWidgets.QFileDialog.getOpenFileName(self, self.t('选择方块布局','Choose cube layout'), '', 'JSON (*.json)')
        if name:
            self.path.setText(name)

    def choose_mount(self):
        name, _ = QtWidgets.QFileDialog.getOpenFileName(self, self.t('选择腕部安装参数','Choose wrist mount'), '', 'JSON (*.json)')
        if name:
            self.mount.setText(name)

    def recent_frames(self):
        with self.inbox_lock:
            return tuple(self.frames)

    def apply_mounts(self, path):
        from .fusion_node import OpticalHandFusionNode
        if self.fusion_node is None:
            self.fusion_node = OpticalHandFusionNode(result_listener=self.receive_fusion, parameter_overrides=[
                Parameter('camera_name', value=self.node.camera_name), Parameter('mount_path', value=str(path))])
            self.executor.add_node(self.fusion_node)
        else:
            from .hand_fusion import read_mounts
            self.executor.create_task(self.fusion_node.join.replace_mounts, read_mounts(path))

    def open_mount_dialog(self):
        if self.node is None:
            self.toggle()
        if self.node is None:
            return
        target = Path(self.mount.text().strip()) if self.mount.text().strip() else self.default_mount
        try:
            from .mount_dialog import MountDialog
            dialog = MountDialog(self, self.node, self.recent_frames, self.resources, target)
        except (ImportError, OSError, ValueError, RuntimeError) as exc:
            self.notice.setText('无法打开手背标定。' + str(exc))
            return
        self.mount_dialog = dialog
        try:
            saved = dialog.exec_() == QtWidgets.QDialog.Accepted
        finally:
            if self.mount_dialog is not None:
                dialog.finish_capture()
                self.mount_dialog = None
            dialog.deleteLater()
        if saved:
            self.mount.setText(str(target))
            try:
                self.apply_mounts(target)
                self.notice.setText('手背标定已保存并加载。保持跟踪运行，切回工作台的“空间”即可查看三维位置。')
            except (ValueError, OSError, RuntimeError) as exc:
                self.notice.setText('标定已保存。加载失败，可重新开始跟踪。' + str(exc))

    def toggle(self):
        if self.node is not None:
            self.stop()
            return
        try:
            overrides = [Parameter('camera_name', value=self.camera.currentText().strip()),
                         Parameter('layout_path', value=self.path.text().strip())]
            self.node = HandCubeNode(parameter_overrides=overrides, frame_listener=self.receive)
            self.executor = SingleThreadedExecutor()
            self.executor.add_node(self.node)
            if self.mount.text().strip():
                self.apply_mounts(self.mount.text().strip())
            self.spin_thread = threading.Thread(target=self.executor.spin, name='hand-cube-ros', daemon=True)
            self.spin_thread.start()
        except Exception as exc:
            self.stop()
            self.notice.setText(str(exc))
            return
        self.notice.setText(self.t('30 mm 方块 · 24 mm 码。可依次举手绑定，也可选手后点击方块中心。',
                                   '30 mm cubes · 24 mm tags. Bind one visible hand at a time, or select a hand and click its cube.'))
        self.sync_controls()

    def receive(self, image, camera, distortion, observations, results, stamp):
        with self.inbox_lock:
            if self.frames and stamp <= self.frames[-1][-1]:
                self.frames.clear()
                self.fused.clear()
            self.frames.append((time.monotonic(), image, camera, distortion, observations, results, stamp))
            while len(self.fused) > 12:
                self.fused.pop(next(iter(self.fused)))

    def receive_fusion(self, message):
        if self.node is None or message.cube_instance_id != self.node.instance_id:
            return
        stamp = message.header.stamp.sec * 1_000_000_000 + message.header.stamp.nanosec
        with self.inbox_lock:
            self.fused.setdefault(stamp, {})[message.hand] = message
            while len(self.fused) > 12:
                self.fused.pop(next(iter(self.fused)))

    def refresh(self):
        if self.node is None:
            return
        if self.learning_job is not None and self.learning_job.done():
            try:
                info = self.learning_job.result()
                hints = {
                    'show_adjacent_tags': ('同时展示两个相邻面', 'Show two adjacent faces'),
                    'multiple_cubes': ('只展示一个方块', 'Show one cube only'),
                    'show_unknown_with_known': ('将已识别的面与新面一起转入画面', 'Show a known face alongside a new face'),
                    'rotate_for_more_views': ('稍微转动，从不同角度观察', 'Rotate the cube to show a different angle'),
                    'verify_from_another_angle': ('六面已识别，再从其他方向复核各面', 'All six identified. Turn the cube to check from other angles'),
                    'layout_conflict': ('朝向与当前布局冲突，请结束识别并核对是否换了方块', 'Layout conflict. End learning and check which cube is visible'),
                    'complete': ('复核完成，可以保存', 'Validation complete. Ready to save')}
                hint = self.t(*hints[info['status']])
                self.notice.setText(self.t('已识别', 'Identified') + f" {info['provisional_faces']}/6 · " +
                                    self.t('已复核', 'Verified') + f" {info['verified_faces']}/6 · " + hint)
            except Exception as exc:
                self.notice.setText(str(exc))
            self.learning_job = None
            self.sync_controls()
        now = time.monotonic()
        with self.inbox_lock:
            row = select_preview_frame(self.frames, self.fused, now, wait_for_fusion=self.fusion_node is not None)
            joints = dict(self.fused.get(row[-1], {})) if row else {}
        if row is None:
            if self.last_drawn is None or now - self.last_drawn[0] > PREVIEW_EXPIRY_S:
                self.invalidate_preview(self.t('等待新鲜的彩色图像和匹配的相机内参', 'Waiting for a fresh color image and matching calibration'))
            return
        # An exact result arriving after fallback can still enrich the displayed frame.
        # Never rewind the preview for a delayed result from an older image.
        draw_key = (row[0], len(joints))
        if self.last_drawn is not None and draw_key <= self.last_drawn:
            return
        new_image = self.last_drawn is None or row[0] != self.last_drawn[0]
        self.last_drawn = draw_key
        _, image, camera, distortion, observations, results, stamp = row
        if new_image and self.learner is not None and self.learning_job is None and not self.learner.complete:
            self.learning_job = self.learning_pool.submit(self.learner.update, image, camera, distortion)
        frame = annotated_preview(image, camera, distortion, observations, results)
        for hand, message in joints.items():
            if message.joints_valid and results[hand].observation is not None:
                draw_hand(frame, camera, distortion, message.joint_positions_m,
                          (255, 208, 58) if hand == 'left' else (70, 166, 255))
        self.preview.show_frame(frame, stamp)
        labels = []
        states = {'unbound':'未绑定', 'lost':'失去观测', 'ambiguous':'身份不确定', 'tracked':'跟踪中'}
        for hand, label in (('left','左手'),('right','右手')):
            value = results[hand]
            status = self.t(states.get(value.status, value.status), value.status)
            if value.observation is not None:
                xyz = ', '.join(f'{v*1000:.0f}' for v in value.observation.translation_m)
                status += '  (' + xyz + ') mm'
            if self.fusion_node:
                msg = joints.get(hand)
                if msg is not None and msg.joints_valid:
                    status += self.t(' · 21点', ' · 21 joints')
                elif msg is not None and msg.wrist_valid:
                    status += self.t(' · 仅腕部', ' · wrist only')
                elif msg is not None:
                    explanations = {'mount_missing': '这只手尚未标定', 'layout_mismatch': '请核对方块布局',
                                    'mount_model_mismatch': '手模已更换，请重新标定这只手',
                                    'mount_changed': '正在加载新标定', 'cube_lost': '方块暂时不可见',
                                    'camera_timing_unavailable': '等待相机帧时序'}
                    status += ' · ' + self.t(explanations.get(msg.status, '等待有效手部数据'), msg.status)
            labels.append(self.t(label, hand.title()) + '  ' + status)
        self.readout.setText('     |     '.join(labels))

    def arm(self, hand):
        if self.node is None:
            return
        self.selected = hand
        try:
            self.node.bind_hand(hand)
            self.binding_done(hand)
        except ValueError as exc:
            self.notice.setText(self.t('请点击对应方块的中心圆圈。', 'Click the centre circle of the chosen cube. ') + ' ' + str(exc))

    def binding_done(self, hand):
        self.selected = None
        self.notice.setText(self.t('已绑定左手' if hand == 'left' else '已绑定右手', 'Bound '+hand))

    def pick(self, center_px, stamp):
        if not self.selected or self.node is None:
            return
        try:
            self.node.bind_hand_at(self.selected, center_px, stamp)
            self.binding_done(self.selected)
        except ValueError as exc:
            self.notice.setText(str(exc))

    def clear_bindings(self):
        self.selected = None
        if self.node:
            self.node.reset_bindings()
            self.notice.setText(self.t('已清除左右手绑定。', 'Both bindings cleared.'))

    def toggle_learning(self):
        if self.learner is not None:
            self.learner = self.learning_job = None
            self.sync_controls()
            return
        if self.learner is None:
            from .layout_learning import CubeLayoutLearner
            from .cube import CubeLayout
            try:
                layout = CubeLayout.from_file(self.path.text()) if self.path.text().strip() else None
                self.learner = CubeLayoutLearner(initial=layout)
            except (ValueError, OSError) as exc:
                self.notice.setText(str(exc))
                return
            self.notice.setText(self.t('请只展示一块方块，缓慢翻转。', 'Show just one cube and rotate it slowly.'))
        self.sync_controls()

    def save_layout(self):
        if self.learner is None or not self.learner.complete:
            return
        name, _ = QtWidgets.QFileDialog.getSaveFileName(self, self.t('保存六面布局', 'Save six-face layout'),
                                                     str(Path.home()/'hand-cube-30mm.json'), 'JSON (*.json)')
        if name:
            try:
                self.learner.save(name)
                self.path.setText(name)
                self.notice.setText(self.t('布局已保存，停止后重新开始跟踪即可加载。', 'Layout saved. Stop and restart tracking to apply.'))
            except (ValueError, OSError) as exc:
                self.notice.setText(str(exc))

    def stop(self):
        if self.mount_dialog is not None:
            self.mount_dialog.reject()
            self.mount_dialog.finish_capture()
            self.mount_dialog = None
        if self.executor is not None:
            self.executor.shutdown()
        if self.spin_thread is not None:
            self.spin_thread.join()
        if self.fusion_node is not None:
            self.fusion_node.destroy_node()
            self.fusion_node = None
        if self.node is not None:
            self.node.destroy_node()
        self.executor = self.spin_thread = self.node = None
        self.selected = self.learner = self.learning_job = None
        with self.inbox_lock:
            self.frames.clear()
            self.fused.clear()
        self.last_drawn = None
        self.sync_controls()

    def closeEvent(self, event):
        self.stop()
        self.learning_pool.shutdown(wait=True, cancel_futures=True)
        super().closeEvent(event)


def main():
    args = argparse.ArgumentParser()
    args.add_argument('--camera', default='d436')
    local = local_config('default-30mm.json')
    args.add_argument('--layout', default=str(local) if local.is_file() else '')
    args.add_argument('--mount', default='')
    args.add_argument('--left-resources', default='')
    args.add_argument('--right-resources', default='')
    args.add_argument('--language', default='zh_CN', choices=('zh_CN','en'))
    opts = args.parse_args()
    # The binding and reset services must not be reachable from other hosts.
    from vt_capture_core.ros_transport import capture_discovery_environment
    os.environ.update(capture_discovery_environment(os.environ))
    rclpy.init(args=[], signal_handler_options=SignalHandlerOptions.NO)
    try:
        app = QtWidgets.QApplication(sys.argv[:1])
        window = Viewer(camera_name=opts.camera, layout_path=opts.layout, language=opts.language,
                        mount_path=opts.mount, resources={'left': opts.left_resources, 'right': opts.right_resources})
        for sig in (signal.SIGTERM, signal.SIGINT):
            signal.signal(sig, lambda *_: window.close())
        window.show()
        return app.exec_()
    finally:
        rclpy.try_shutdown()


if __name__ == '__main__':
    sys.exit(main())
