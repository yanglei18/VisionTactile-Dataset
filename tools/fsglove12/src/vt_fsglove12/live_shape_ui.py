"""Explicit, snapshot-driven touch/shape wizard; never reads or fits hardware."""
from __future__ import annotations

from copy import deepcopy
import json
import math
from pathlib import Path

from PyQt5 import QtCore, QtWidgets

from .health_contract import calibration_input_ready
from .calibration_presentation import calibration_status, shape_busy, storage_status
from .ui_sections import CollapsibleSection
from .ui_theme import status_style, text_style


CONTACT_NAMES = {2: "拇指—食指", 3: "拇指—中指", 4: "拇指—无名指", 5: "拇指—小指"}
class ShapeCalibrationPanel(QtWidgets.QGroupBox):
    """All commands require a user action and a fresh eligibility check."""

    def __init__(self, window):
        super().__init__("触指标定")
        self.window = window
        self._context = None
        self._pending = None
        self._pending_evidence = None
        self._local_error = ""
        layout = QtWidgets.QVBoxLayout(self)
        self.guidance = window._label(
            "完成基础标定后，逐对让拇指与指定指尖轻触。确认接触后采集，"
            "每对 3 秒准备、2 秒保持；建议先完成全手弯伸。")
        self.guidance.setStyleSheet(text_style("body"))
        layout.addWidget(self.guidance)
        grid = QtWidgets.QGridLayout()
        self.pairs = QtWidgets.QButtonGroup(self)
        self.pairs.setExclusive(True)
        self.pair_buttons, self.contact_status = {}, {}
        for row, (category, name) in enumerate(CONTACT_NAMES.items()):
            button = QtWidgets.QPushButton(name)
            button.setCheckable(True)
            button.setToolTip("选择这一对；不会自动采集，也不会确认真实接触。")
            self.pairs.addButton(button, category)
            self.pair_buttons[category] = button
            self.contact_status[category] = window._label("尚未采集")
            grid.addWidget(button, row, 0)
            grid.addWidget(self.contact_status[category], row, 1)
        self.pair_buttons[2].setChecked(True)
        self.pairs.buttonClicked.connect(self._pair_changed)
        layout.addLayout(grid)
        self.confirm = QtWidgets.QCheckBox()
        self.confirm.setToolTip("仅由操作者观察确认；IMU、模型外观或优化损失都不能证明真实接触。每次采集需重新勾选。")
        self.confirm.toggled.connect(lambda _: self.refresh_from_controller())
        layout.addWidget(self.confirm)
        self.contact_note = window._label("以实际指尖接触为准，模型画面不能代替确认。")
        self.contact_note.setStyleSheet(text_style("muted"))
        layout.addWidget(self.contact_note)
        self.capture_button = QtWidgets.QPushButton()
        self.capture_button.clicked.connect(self.capture)
        layout.addWidget(self.capture_button)
        self.progress = QtWidgets.QProgressBar()
        self.progress.setObjectName("calibrationProgress")
        layout.addWidget(self.progress)
        self.status = window._label()
        self.status.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.status.setStyleSheet(status_style())
        layout.addWidget(self.status)
        self.retry_save_button = QtWidgets.QPushButton("重试保存标定")
        self.retry_save_button.setToolTip("处理存储故障后，显式重试尚未保存的标定 / 触指任务；不重新采集。")
        self.retry_save_button.clicked.connect(self.window.retry_calibration_save)
        self.retry_save_button.hide()
        self.retry_save_to_button = QtWidgets.QPushButton("另选保存目录")
        self.retry_save_to_button.setToolTip("将待保存标定 / 触指任务写入另一个可写目录；不重新连接或采集。")
        self.retry_save_to_button.clicked.connect(self.retry_save_to)
        self.retry_save_to_button.hide()
        retry_actions = QtWidgets.QHBoxLayout()
        retry_actions.addWidget(self.retry_save_button)
        retry_actions.addWidget(self.retry_save_to_button)
        layout.addLayout(retry_actions)
        actions = QtWidgets.QHBoxLayout()
        self.optimize_button = QtWidgets.QPushButton("优化手形候选")
        self.optimize_button.clicked.connect(lambda: self._command("optimize_shape", "optimize"))
        self.cancel_button = QtWidgets.QPushButton("取消触指采集 / 优化")
        self.cancel_button.clicked.connect(lambda: self._command("cancel_shape", "cancel"))
        actions.addWidget(self.optimize_button)
        actions.addWidget(self.cancel_button)
        layout.addLayout(actions)
        self.preview = QtWidgets.QComboBox()
        for label, mode in (("显示当前已采用手形", "current"),
                            ("预览候选（尚未采用）", "candidate"),
                            ("对比基准手形（仅预览）", "baseline")):
            self.preview.addItem(label, mode)
        self.preview.activated.connect(self._preview_selected)
        layout.addWidget(self.preview)
        self.apply_button = QtWidgets.QPushButton("采用候选手形")
        self.apply_button.setToolTip("显式提交到当前官方解算和后续姿态；仅预览或优化不会采用。")
        self.apply_button.clicked.connect(lambda: self._command("apply_shape_candidate", "apply"))
        layout.addWidget(self.apply_button)

        self.file_actions_section = CollapsibleSection("任务与文件", self)
        file_layout = self.file_actions_section.content_layout
        layout.addWidget(self.file_actions_section)
        task_actions = QtWidgets.QHBoxLayout()
        self.save_task_button = QtWidgets.QPushButton("保存触指任务")
        self.save_task_button.setToolTip("将已采集的触指姿态保存到新 JSON；可另选可写位置，不覆盖历史任务。")
        self.save_task_button.clicked.connect(self.save_task)
        self.load_task_button = QtWidgets.QPushButton("恢复触指任务")
        self.load_task_button.setToolTip("仅在断开时恢复已记录姿态用于离线优化；不连接、不恢复安装或滤波，也不继续采集。")
        self.load_task_button.clicked.connect(self.load_task)
        task_actions.addWidget(self.save_task_button)
        task_actions.addWidget(self.load_task_button)
        file_layout.addLayout(task_actions)
        file_actions = QtWidgets.QHBoxLayout()
        self.load_button = QtWidgets.QPushButton("加载手形文件")
        self.load_button.clicked.connect(self.load_profile)
        self.save_button = QtWidgets.QPushButton("导出手形文件")
        self.save_button.clicked.connect(self.save_profile)
        self.load_button.setToolTip("可在未连接时选择文件作为下次会话的首选手形；不会连接或沿用安装标定。")
        self.save_button.setToolTip("导出候选（优先）或已采用结果到新文件；拒绝覆盖已有文件。")
        file_actions.addWidget(self.load_button)
        file_actions.addWidget(self.save_button)
        file_layout.addLayout(file_actions)
        self.revert_button = QtWidgets.QPushButton("回退到上一次已采用手形")
        self.revert_button.clicked.connect(lambda: self._command("revert_shape", "revert"))
        file_layout.addWidget(self.revert_button)
        self.result_details_section = CollapsibleSection("结果详情", self)
        result_layout = self.result_details_section.content_layout
        layout.addWidget(self.result_details_section)
        self.paths = window._label()
        self.paths.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.paths.setStyleSheet(text_style("muted"))
        result_layout.addWidget(self.paths)
        self.metrics = window._label()
        self.metrics.setTextInteractionFlags(QtCore.Qt.TextSelectableByMouse)
        self.metrics.setStyleSheet(text_style("body"))
        result_layout.addWidget(self.metrics)
        self.capability_note = window._label(
            "边界：官方优化只更新 10 维 MANO 手形 beta，不联合优化姿态或安装，不修正航向漂移。"
            "缺测 DIP / CMC 先验仍会影响触指；损失降低不等于实际接触误差或人体精度已通过。")
        self.capability_note.setStyleSheet(text_style("muted"))
        result_layout.addWidget(self.capability_note)
        self._update_pair_text()
        if not window.official:
            self.hide()

    def reset(self):
        self._pending = self._pending_evidence = None
        self._local_error = ""
        blocked = self.confirm.blockSignals(True)
        self.confirm.setChecked(False)
        self.confirm.blockSignals(blocked)
        self.pair_buttons[2].setChecked(True)
        self._update_pair_text()

    def _update_pair_text(self):
        name = CONTACT_NAMES[self.pairs.checkedId()]
        self.confirm.setText(f"我已确认{name}指尖真实轻触")
        self.capture_button.setText(f"采集{name}（3 秒准备 + 2 秒保持）")

    def _pair_changed(self, _button):
        self.confirm.setChecked(False)
        self._update_pair_text()
        self.refresh_from_controller()

    def _snapshot(self):
        return self.window._calibration_display_snapshot(self.window.controller.latest_snapshot())

    @staticmethod
    def _evidence(snapshot):
        runtime = snapshot.get("runtime") or {}
        return (deepcopy(snapshot.get("calibration", {}).get("shape_calibration") or {}),
                runtime.get("shape_command_generation"), runtime.get("shape_rejected_count"),
                runtime.get("shape_command_error"), runtime.get("error"))

    def _permissions(self, snapshot, clearing, storage=None, presentation=None):
        calibration = snapshot.get("calibration") or {}
        runtime = snapshot.get("runtime") or {}
        state = calibration.get("shape_calibration") or {}
        storage = storage if storage is not None else storage_status(snapshot)
        presentation = presentation if presentation is not None else calibration_status(snapshot)
        busy = shape_busy(calibration)
        available = bool(self.window.official and not clearing
            and calibration.get("active") is None
            and not str(calibration.get("profile_status", "")).startswith("fitting")
            and calibration.get("profile_status") != "clearing")
        idle = available and not busy and self._pending is None and not self.window._save_actions.pending
        calibrated = bool(runtime.get("connected") and presentation.has_profile
                          and not presentation.requires_recalibration)
        contacts = state.get("contacts") or {}
        complete = all((contacts.get(str(category)) or {}).get("frames", 0) > 0
                       and (contacts.get(str(category)) or {}).get("confirmed") is True
                       for category in CONTACT_NAMES)
        candidate = state.get("candidate_available") is True
        imported = state.get("task_loaded") is True
        offline_task = imported and not runtime.get("connected") and state.get("can_optimize") is True
        return dict(select=idle and calibrated and not imported,
            capture=idle and calibrated and not imported and state.get("can_capture", True) is True
                and calibration_input_ready(snapshot.get("orientation") or {}, self.window.config),
            optimize=idle and (calibrated and not imported or offline_task) and complete
                and state.get("can_optimize", True) is True,
            cancel=available and self._pending != "cancel" and (busy or self._pending in {"capture", "optimize"}),
            preview=idle and calibrated,
            apply=idle and calibrated and candidate and state.get("can_apply", True) is True,
            load=idle, save=idle and bool(state.get("can_export", candidate or bool(state.get("applied_path")))),
            save_task=idle and state.get("can_save_task") is True,
            load_task=idle and not runtime.get("connected") and state.get("can_load_task", True) is True,
            retry_save=self.window._save_actions.allowed(snapshot, storage, clearing=clearing,
                official=self.window.official, other_pending=self._pending is not None),
            retry_save_to=self.window._save_actions.allowed(snapshot, storage, clearing=clearing,
                official=self.window.official, other_pending=self._pending is not None),
            revert=idle and bool(state.get("can_revert")))

    def refresh_from_controller(self):
        self.window.refresh()

    def refresh(self, snapshot, *, clearing=False, storage=None, presentation=None):
        calibration = snapshot.get("calibration") or {}
        runtime = snapshot.get("runtime") or {}
        state = calibration.get("shape_calibration") or {}
        storage = storage if storage is not None else storage_status(snapshot)
        context = (runtime.get("session_id"), bool(runtime.get("connected")), calibration.get("clear_generation"))
        if self._context != context or clearing:
            self.reset()
            self._context = context
        if self._pending and self._pending_evidence != self._evidence(snapshot):
            self._pending = self._pending_evidence = None
        permissions = self._permissions(snapshot, clearing, storage, presentation)
        if not permissions["capture"]:
            blocked = self.confirm.blockSignals(True)
            self.confirm.setChecked(False)
            self.confirm.blockSignals(blocked)
        for button in self.pair_buttons.values():
            button.setEnabled(permissions["select"])
        self.confirm.setEnabled(permissions["capture"])
        self.capture_button.setEnabled(permissions["capture"] and self.confirm.isChecked())
        for action in ("optimize", "cancel", "apply", "load", "save", "revert", "save_task", "load_task",
                       "retry_save", "retry_save_to"):
            getattr(self, action + "_button").setEnabled(permissions[action])
        self.retry_save_button.setVisible(storage.has_errors)
        self.retry_save_to_button.setVisible(storage.has_errors)
        self.preview.setEnabled(permissions["preview"])
        self.preview.model().item(1).setEnabled(state.get("candidate_available") is True)
        self.preview.model().item(2).setEnabled(state.get("candidate_available") is True)
        mode = state.get("preview_mode", "current")
        self.preview.setCurrentIndex(max(0, self.preview.findData(mode)))
        contacts = state.get("contacts") or {}
        complete = 0
        for category, label in self.contact_status.items():
            contact = contacts.get(str(category)) or {}
            frames = contact.get("frames", 0)
            confirmed = contact.get("confirmed") is True and frames > 0
            complete += int(confirmed)
            saved = "已保存" if contact.get("saved") is True else "待保存"
            label.setText(f"✓ {frames} 帧 · 已确认 · {saved}" if confirmed else "尚未采集有效确认帧")
        status = state.get("status", "waiting")
        labels = {"waiting": "等待四组真实触指", "capturing": "触指采集中", "ready": "四组已就绪，可优化",
                  "optimizing": "正在优化候选；不改变当前已采用手形", "candidate": "候选就绪，尚未采用",
                  "applied": "手形已采用", "failed": "本次手形操作失败", "cancelled": "本次手形操作已取消"}
        lines = ["正在清除本次标定" if clearing else labels.get(status, "手形状态待确认")]
        if self._pending or self.window._save_actions.pending:
            lines.append("操作请求已发送，等待后台确认；不会自动重试。")
        if state.get("message"):
            lines.append(str(state["message"]))
        if state.get("error"):
            lines.append("错误：" + str(state["error"]))
        if storage.has_notices:
            lines.append(storage.notice_text)
        if storage.message:
            lines.append(storage.message)
        if state.get("task_loaded"):
            lines.append("已恢复触指任务：仅离线优化 / 导出，不恢复安装或滤波，不续采。")
        if runtime.get("shape_command_error"):
            lines.append(str(runtime["shape_command_error"]))
        if self._local_error:
            lines.append(self._local_error)
        if mode != "current":
            lines.append("仅显示对比：" + ("候选手形" if mode == "candidate" else "基准手形") + "；解算 / ROS 未切换。")
        self.status.setText("\n".join(lines))
        if state.get("error") or storage.has_errors or runtime.get("shape_command_error") or self._local_error:
            tone = "error"
        elif status == "failed":
            tone = "warning"
        elif status == "applied":
            tone = "success"
        elif self._pending or status in {"capturing", "optimizing", "candidate"}:
            tone = "info"
        else:
            tone = "neutral"
        style = status_style(tone)
        if self.status.styleSheet() != style:
            self.status.setStyleSheet(style)
        self.progress.setRange(0, 100)
        if status == "capturing":
            remaining = state.get("remaining_s", 0.)
            remaining = max(0., float(remaining)) if isinstance(remaining, (int, float)) and math.isfinite(remaining) else 0.
            preparing = state.get("phase") == "preparing"
            duration = 3. if preparing else 2.
            self.progress.setValue(round(100 * max(0., min(1., 1. - remaining / duration))))
            name = CONTACT_NAMES.get(state.get("active_category"), "所选指对")
            self.progress.setFormat(name + (" · 准备，暂未采集" if preparing else " · 保持真实轻触，正在采集") + f" {remaining:.1f} 秒")
        elif status == "optimizing":
            self.progress.setRange(0, 0)
            self.progress.setFormat("正在优化；可取消")
        else:
            self.progress.setValue(complete * 25)
            self.progress.setFormat(f"已采集 {complete}/4 对 · 每对 3 秒准备 → 2 秒保持")
        paths = []
        for key, label in (("task_path", "触指任务"), ("candidate_path", "候选文件"),
                           ("applied_path", "已采用文件"), ("shape_source", "当前来源")):
            if state.get(key):
                value = state[key]
                if key == "shape_source":
                    value = {"personalized": "个性化触指结果", "configured": "显式配置的手形", "default": "默认手形（未个性化）"}.get(value, value)
                paths.append(f"{label}：{value}")
        self.paths.setText("\n".join(paths) or "尚无个性化手形文件；默认手形不表示个体已标定。")
        metrics = state.get("metrics") or {}
        metric_lines = []
        for finger, label in (("index", "拇指—食指"), ("middle", "拇指—中指"),
                              ("ring", "拇指—无名指"), ("little", "拇指—小指")):
            pair = (metrics.get("per_pair") or {}).get(finger) or {}
            before, after = (pair.get("before") or {}).get("mean_gap_mm"), (pair.get("after") or {}).get("mean_gap_mm")
            if all(isinstance(value, (int, float)) and not isinstance(value, bool) and math.isfinite(value)
                   for value in (before, after)):
                metric_lines.append(f"{label}：{before:.2f} → {after:.2f} mm")
        if metrics:
            metric_lines.insert(0, "模型触指平均间隙：优化前 → 候选")
            metric_lines.append("用于拟合的同批接触，不是独立精度验收。")
            if len(metric_lines) == 2:
                metric_lines.insert(1, json.dumps(metrics, ensure_ascii=False, indent=2))
        self.metrics.setText("\n".join(metric_lines))

    def _command(self, name, permission, **args):
        snapshot, clearing = self._snapshot()
        self.refresh(snapshot, clearing=clearing)
        if not self._permissions(snapshot, clearing)[permission]:
            return False
        if permission == "capture" and not self.confirm.isChecked():
            return False
        evidence = self._evidence(snapshot)
        if not self.window.send(name, **args):
            return False
        self._local_error = ""
        if permission != "preview":
            self._pending, self._pending_evidence = permission, evidence
        if permission in {"capture", "cancel"}:
            self.confirm.setChecked(False)
        self.refresh_from_controller()
        return True

    def capture(self):
        self._command("capture_shape_contact", "capture", category=self.pairs.checkedId(), contact_confirmed=True)

    def _preview_selected(self, index):
        mode = self.preview.itemData(index)
        snapshot, _ = self._snapshot()
        if mode in {"candidate", "baseline"} and not snapshot.get("calibration", {}).get("shape_calibration", {}).get("candidate_available"):
            self.refresh_from_controller()
            return
        self._command("preview_shape", "preview", mode=mode)

    def load_profile(self):
        snapshot, clearing = self._snapshot()
        if not self._permissions(snapshot, clearing)["load"]:
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "加载个性化手形（不会自动连接）", "", "JSON 手形文件 (*.json)")
        if path:
            self._command("load_shape_profile", "load", path=path)

    def load_task(self):
        snapshot, clearing = self._snapshot()
        if not self._permissions(snapshot, clearing)["load_task"]:
            return
        path, _ = QtWidgets.QFileDialog.getOpenFileName(self, "恢复触指任务（仅离线，不会自动连接）", "",
            "JSON 触指任务 (*.json)")
        if path:
            self._command("load_shape_task", "load_task", path=path)

    def retry_save_to(self):
        return self.window.choose_calibration_save_directory()

    def save_task(self):
        snapshot, clearing = self._snapshot()
        if not self._permissions(snapshot, clearing)["save_task"]:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "保存触指任务到新文件", "touch_task.json",
            "JSON 触指任务 (*.json)", options=QtWidgets.QFileDialog.DontConfirmOverwrite)
        if not path:
            return
        if Path(path).exists():
            self._local_error = "目标文件已存在，未保存任务；请选择新文件名，不覆盖历史任务。"
            self.refresh_from_controller()
            return
        self._command("save_shape_task", "save_task", path=path)

    def save_profile(self):
        snapshot, clearing = self._snapshot()
        if not self._permissions(snapshot, clearing)["save"]:
            return
        path, _ = QtWidgets.QFileDialog.getSaveFileName(self, "导出到新文件（拒绝覆盖）", "personalized_shape.json",
            "JSON 手形文件 (*.json)", options=QtWidgets.QFileDialog.DontConfirmOverwrite)
        if not path:
            return
        if Path(path).exists():
            self._local_error = "目标文件已存在，未导出；请选择新文件名，不覆盖历史结果。"
            self.refresh_from_controller()
            return
        self._command("save_shape_profile", "save", path=path)
