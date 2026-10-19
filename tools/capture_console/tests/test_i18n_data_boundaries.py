"""Language changes affect presentation, never machine values or user data."""
from copy import deepcopy
import json
import sys
from types import SimpleNamespace

import pytest
from PySide6.QtCore import QRectF
from PySide6.QtGui import QImage, QPainter

from vt_capture_console import i18n
from vt_capture_console.backend.workstation_settings import default_configuration
from vt_capture_console.ui.widgets.preview import PreviewWidget
from vt_capture_console.ui.widgets.tactile_model_loader import TactileModelLoader, validate_bundle
from vt_capture_console.ui.widgets.trial_details import TrialDetailsDialog
from vt_capture_console.ui.widgets.workstation_settings import WorkstationSettingsDialog


@pytest.fixture(autouse=True)
def restore_language():
    previous = i18n.language()
    i18n.set_language("zh_CN", persist=False)
    yield
    i18n.set_language(previous, persist=False)


def test_configuration_values_survive_language_switch_without_translation(qapp, tmp_path):
    configuration = default_configuration(tmp_path)
    runtime = SimpleNamespace(configuration=configuration, status={"state": "ready"},
                              serial_ports=lambda: [], log_path=None)
    dialog = WorkstationSettingsDialog(runtime, lambda: {})
    try:
        editor = dialog.fields[("right_glove", "output_rate_hz")]
        editor.setValue(87.5)
        dialog.fields[("right_glove", "config")].setText("/tmp/用户标定.json")
        dialog.fields[("right_glove", "port")].setEditText("/dev/serial/by-id/用户设备")
        before = deepcopy(dialog.configuration())
        i18n.set_language("en", persist=False)
        after = deepcopy(dialog.configuration())
        assert after == before
        parameters = next(item for item in after["process_groups"] if item["id"] == "right_glove")["parameters"]
        assert type(parameters["output_rate_hz"]) is str
        assert parameters["output_rate_hz"] == "87.5"
        assert parameters["config"] == "/tmp/用户标定.json"
        assert parameters["port"] == "/dev/serial/by-id/用户设备"
        assert json.loads(json.dumps(after)) == after
    finally:
        dialog.close()


def test_optional_field_is_detected_by_key_after_language_switch(qapp, tmp_path):
    # The module's FIELDS labels were created in Chinese before this switch.
    i18n.set_language("en", persist=False)
    runtime = SimpleNamespace(configuration=default_configuration(tmp_path),
                              status={"state": "ready"}, serial_ports=lambda: [], log_path=None)
    dialog = WorkstationSettingsDialog(runtime, lambda: {})
    try:
        assert dialog.fields[("right_glove", "config")].placeholderText() == str(i18n.tr("留空使用现有默认配置"))
        assert dialog.fields[("right_glove", "glove_python")].placeholderText() == str(i18n.tr("选择本机路径"))
    finally:
        dialog.close()


def test_model_file_and_source_identity_are_not_translatable(qapp, tmp_path, monkeypatch):
    entries = dict(i18n.catalog())
    entries.update({"MANO_{v0}.pkl": "translated_{v0}.pkl",
                    "{v0}_neutral_mano_21_joints": "translated_{v0}"})
    monkeypatch.setattr(i18n, "_catalog", entries)
    i18n.set_language("en", persist=False)
    models = tmp_path / "models"
    models.mkdir()
    (models / "MANO_RIGHT.pkl").touch()
    configuration = {"process_groups": [{"id": "right_glove", "parameters": {
        "hand": "right", "glove_python": sys.executable, "mano_assets": str(tmp_path)}}]}
    loader = TactileModelLoader(lambda: configuration)
    try:
        assert loader.available("right")
        assert not loader._jobs
    finally:
        loader.shutdown()
    vertices = [[0., 0., 0.] for _ in range(778)]
    vertices[1], vertices[2] = [.01, 0., 0.], [0., .01, 0.]
    raw = {"schema_version": 1, "neutral_geometry": {
        "vertices_m": vertices, "faces": [[0, 1, 2]], "joints_m": [[0., 0., 0.] for _ in range(21)],
        "provenance": {"hand": "right"}}, "model_axes": {
        "pitch_axis": [-1., 0., 0.], "roll_axis": [0., 1., 0.],
        "palm_up_axis": [0., 0., 1.], "hand": "right"}}
    source = validate_bundle(raw, "right")["model_axes"]["source"]
    assert type(source) is str
    assert deepcopy(source) == "right_neutral_mano_21_joints"


def test_cached_tactile_note_uses_current_language_on_paint(qapp, monkeypatch):
    preview = PreviewWidget("tactile")
    preview._device = {"side": "left"}
    preview._sync_tactile_view()
    original_note = preview._tactile_layout_note
    captured = []
    monkeypatch.setattr(preview, "_text", lambda _painter, _rect, text, **_kwargs: captured.append(str(text)))
    surface = QImage(300, 300, QImage.Format.Format_ARGB32_Premultiplied)
    painter = QPainter(surface)
    try:
        i18n.set_language("en", persist=False)
        preview._paint_tactile(painter, QRectF(0, 0, 300, 300))
    finally:
        painter.end()
        preview.close()
    assert preview._tactile_layout_note is original_note
    assert captured and "left hand" in captured[0].lower()
    assert "unmapped" in captured[0]
    assert "contact locations" not in captured[0]
    assert not any("\u4e00" <= char <= "\u9fff" for char in captured[0])


def test_open_trial_details_retranslate_without_refreshing_user_metadata(qapp):
    trial = {"id": "trial-1", "batch_id": "batch-1", "label": "用户试次", "state": "COMPLETE",
             "elapsed_s": 12., "duration_s": 15., "output_path": "/data/用户试次", "detail": "用户说明"}
    metadata = {"task_name": "用户任务", "operator_name": "用户姓名"}
    details = {"trial_id": "trial-1", "metadata": metadata, "artifacts": {},
               "quality": {"state": "COMPLETED", "verdict": "ok", "report": {"summary": {}}}}
    dialog = TrialDetailsDialog()
    try:
        dialog.display(trial, details, {"state": "ready"}, supported=True)
        original_metadata = dialog.metadata.toPlainText()
        assert "录制状态" in dialog.overview.toPlainText()
        i18n.set_language("en", persist=False)
        assert "recording status" in dialog.overview.toPlainText().lower()
        assert "试次：" not in dialog.overview.toPlainText()
        assert "/data/用户试次" in dialog.overview.toPlainText()
        assert dialog.metadata.toPlainText() == original_metadata
        assert json.loads(original_metadata) == metadata
    finally:
        dialog.close()
