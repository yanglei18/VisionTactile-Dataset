"""Chinese display terms are localized without rewriting machine or user data."""
from copy import deepcopy
from types import SimpleNamespace

import pytest
from PySide6.QtWidgets import QLabel

from vt_capture_console.i18n import language, set_language


@pytest.fixture(autouse=True)
def restore_language():
    before = language()
    set_language("zh_CN", persist=False)
    yield
    set_language(before, persist=False)


@pytest.mark.parametrize("kind,chinese", [("camera", "相机"), ("tracker", "追踪器"),
                                         ("glove", "手部姿态"), ("tactile", "触觉")])
def test_device_card_kind_is_chinese_and_retranslates(qapp, kind, chinese):
    from vt_capture_console.backend.demo_backend import DemoBackend
    from vt_capture_console.ui.widgets.device_card import DeviceCard
    device = next(d for d in DemoBackend().snapshot()["devices"] if d["kind"] == kind)
    card = DeviceCard(device, compact=True)
    try:
        label = card.findChild(QLabel, "cardKind")
        assert label.text() == chinese
        set_language("en", persist=False)
        assert label.text() != chinese
        set_language("zh_CN", persist=False)
        assert label.text() == chinese
    finally:
        card.close()
        card.deleteLater()


def test_topic_table_header_uses_current_language(qapp):
    from vt_capture_console.ui.widgets.topic_replay import TopicReplayPanel
    panel = TopicReplayPanel()
    try:
        assert panel.topics.horizontalHeaderItem(0).text() == "数据主题"
        set_language("en", persist=False)
        assert panel.topics.horizontalHeaderItem(0).text() == "Topic"
    finally:
        panel.close()
        panel.deleteLater()


def test_spatial_role_names_translate_without_mutating_role_or_external_id():
    from vt_capture_console.ui.widgets.spatial_labels import _subtitle
    model = SimpleNamespace(frozen=False, source_available=True)
    entity = dict(role="left_wrist", id="external-ID", state="valid", valid=True)
    before = deepcopy(entity)
    assert "左腕" in str(_subtitle(model, entity))
    assert entity == before
    external = dict(id="Custom_Frame-7", state="valid")
    assert "Custom_Frame-7" in str(_subtitle(model, external))


def test_builtin_device_labels_translate_but_user_names_and_payloads_do_not():
    from vt_capture_console.i18n.labels import device_label
    values = [dict(id="torso", kind="tracker", side="", label="躯干 Tracker"),
              dict(id="d405_1_color", kind="camera", side="", label="D405_1 · RGB")]
    before = deepcopy(values)
    displayed = [device_label(value) for value in values]
    assert list(map(str, displayed)) == ["躯干追踪器", "D405_1 · 彩色"]
    set_language("en", persist=False)
    assert all("追踪器" not in str(value) and "彩色" not in str(value) for value in displayed)
    assert values == before
    custom = dict(values[0], label="My Tracker / 自定义")
    assert device_label(custom) == custom["label"]
    assert device_label(dict(values[0], id="custom_torso")) == values[0]["label"]


def test_builtin_process_label_uses_chinese_but_custom_label_is_opaque():
    from vt_capture_console.ui.widgets.device_controls import _group_label
    group = dict(id="trackers", kind="tracker", label="三 Tracker ROS 读取")
    before = deepcopy(group)
    assert str(_group_label(group, mode="live")) == "三追踪器数据读取"
    assert group == before
    custom = dict(group, label="Custom Tracker")
    assert _group_label(custom, mode="live") == "Custom Tracker"


def test_live_spatial_labels_are_display_values_and_custom_names_stay_intact():
    from vt_capture_console.i18n.labels import scene_entity_label
    from vt_capture_console.viewmodels.spatial import SpatialViewModel
    raw = dict(mode="live", manager_instance_id="real-manager", trackers=[dict(
        id="torso", role="torso", label="躯干 Tracker", state="valid", tracker_id="device-7")], cameras=[])
    before = deepcopy(raw)
    model = SpatialViewModel()
    model.update(raw)
    entity = model.frame["trackers"][0]
    assert str(entity["label"]) == "躯干追踪器"
    assert str(scene_entity_label(entity, short=True)) == "躯干"
    set_language("en", persist=False)
    assert str(entity["label"]) == "Torso Tracker"
    assert raw == before
    custom = dict(raw["trackers"][0], label="My Tracker Rig")
    assert scene_entity_label(custom, short=True) == "My Tracker Rig"


def test_demo_camera_caption_uses_only_selected_language(qapp, monkeypatch):
    from vt_capture_console.ui.widgets.preview import PreviewWidget
    from test_preview_widgets import device, render
    widget = PreviewWidget("camera")
    widget.resize(600, 400)
    widget.set_presentation("stage")
    widget.set_device(device("camera", {"demo": True, "phase": .2}))
    painted = []
    monkeypatch.setattr(widget, "_text", lambda painter, rect, text, **kwargs: painted.append(str(text)))
    try:
        render(widget)
        assert "合成传感器示意" in painted
        assert not any("SYNTHETIC" in text for text in painted)
        set_language("en", persist=False)
        painted.clear()
        render(widget)
        assert "Synthetic optical module" in painted
        assert not any("合成" in text for text in painted)
    finally:
        widget.close()
        widget.deleteLater()


def test_recording_rejection_translates_code_and_detail_without_changing_receipt():
    from vt_capture_console.viewmodels.operations import OperationCoordinator
    coordinator = OperationCoordinator()
    snapshot = {"instance_id": "manager-a"}
    management = {"enabled": True, "controller": {"controller_instance_id": "controller-a"},
                  "batches": [], "trials": []}
    receipt = dict(request_id="r1", state="rejected", code="controller_unavailable",
                   detail="A fresh managed controller snapshot is required; no command was sent.")
    before = deepcopy(receipt)
    commands = []

    def sender(command, params):
        commands.append((command, params))
        return "r1"

    coordinator.submit("capture.start", {"label": "User Trial"}, sender=sender,
                       snapshot=snapshot, management=management)
    update = coordinator.observe(snapshot, management, receipt)
    assert "controller_unavailable" not in str(update.notice)
    assert "录制控制器当前不可用" in str(update.notice)
    assert "需要录制控制器的最新状态" in str(update.notice)
    set_language("en", persist=False)
    assert "A fresh managed controller snapshot" in str(update.notice)
    assert receipt == before
    assert commands == [("capture.start", {"label": "User Trial"})]
    assert coordinator.pending is None
