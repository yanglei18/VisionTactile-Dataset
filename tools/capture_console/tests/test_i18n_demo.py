from copy import deepcopy
import json
from pathlib import Path

import pytest

from vt_capture_console import i18n
from vt_capture_console.backend.demo_backend import DemoBackend
from vt_capture_console.backend.scene_demo import demo_scene
from vt_capture_console.i18n.demo import demo_device_label, demo_group_label, demo_scene_frame


@pytest.fixture(autouse=True)
def english_demo():
    previous = i18n.language()
    i18n.set_language("en", persist=False)
    yield
    i18n.set_language(previous, persist=False)


def has_chinese(text):
    return any("\u3400" <= char <= "\u9fff" for char in text)


def test_all_default_demo_device_labels_translate_without_changing_payload():
    snapshot = DemoBackend(clock=lambda: 0).snapshot()
    before = deepcopy(snapshot)
    labels = [demo_device_label(device) for device in snapshot["devices"]]
    assert len(labels) == 10
    assert all(not has_chinese(str(label)) for label in labels)
    assert snapshot == before
    i18n.set_language("zh_CN", persist=False)
    assert [str(label) for label in labels] == [row["label"].replace("Tracker", "追踪器") for row in snapshot["devices"]]
    assert snapshot == before


@pytest.mark.parametrize("change", [
    {"id": "custom_tracker"},
    {"kind": "camera"},
    {"label": "用户自定义 · 左腕"},
    {"side": "right"},
    {"preview": {"demo": False}},
])
def test_device_labels_require_demo_marker_and_exact_authored_identity(change):
    device = next(row for row in DemoBackend(clock=lambda: 0).snapshot()["devices"]
                  if row["id"] == "left_wrist")
    device.update(change)
    before = deepcopy(device)
    assert demo_device_label(device) == device["label"]
    assert device == before


def test_demo_groups_translate_only_with_explicit_demo_ui_context():
    from vt_capture_console.ui.widgets.device_controls import _group_label
    groups = DemoBackend().process_control["groups"]
    before = deepcopy(groups)
    for group in groups:
        assert not has_chinese(str(demo_group_label(group)))
        assert not has_chinese(str(_group_label(group, mode="demo")))
        assert _group_label(group, mode="live") == group["label"]
    assert groups == before


@pytest.mark.parametrize("change", [
    {"id": "custom-group"},
    {"kind": "tracker"},
    {"label": "我的三相机 · 演示"},
])
def test_user_group_labels_are_never_translated(change):
    group = DemoBackend().process_control["groups"][0]
    group.update(change)
    before = deepcopy(group)
    assert demo_group_label(group) == group["label"]
    assert group == before


def test_demo_scene_uses_a_display_copy_and_retains_sensor_data():
    original = demo_scene(0, scenario="static")
    before = deepcopy(original)
    display = demo_scene_frame(original)
    assert display is not original
    assert display["trackers"] is not original["trackers"]
    for shown, raw in zip(display["trackers"], original["trackers"]):
        assert shown is not raw
        assert not has_chinese(str(shown["label"]))
        assert {k: v for k, v in shown.items() if k != "label"} == {
            k: v for k, v in raw.items() if k != "label"}
    assert display["cameras"] == original["cameras"]
    assert original == before
    i18n.set_language("zh_CN", persist=False)
    assert [str(row["label"]) for row in display["trackers"]] == ["左腕追踪器", "右腕追踪器", "躯干追踪器"]
    assert original == before


@pytest.mark.parametrize("change", [
    {"mode": "live"},
    {"mode": "replay"},
    {"manager_instance_id": "external-demo-manager"},
])
def test_other_scene_sources_are_never_relabelled(change):
    frame = demo_scene(0)
    frame.update(change)
    assert demo_scene_frame(frame) is frame


@pytest.mark.parametrize("change", [
    {"label": "用户躯干"},
    {"tracker_id": "REAL-TRACKER-ID"},
    {"role": "left_wrist"},
])
def test_demo_scene_preserves_renamed_or_reidentified_trackers(change):
    frame = demo_scene(0)
    frame["trackers"][2].update(change)
    before = deepcopy(frame)
    shown = demo_scene_frame(frame)
    assert shown["trackers"][2]["label"] == frame["trackers"][2]["label"]
    assert frame == before


def test_spatial_page_uses_demo_labels_and_frozen_table_retranslates(qapp):
    from vt_capture_console.backend.scene_demo import DemoSceneBackend
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    frame = demo_scene(0, scenario="static")
    before = deepcopy(frame)
    page.apply_frame(frame, backend.status())
    page.model.set_frozen(True)
    page._render()
    assert not has_chinese(page.positions.item(0, 0).text())
    i18n.set_language("zh_CN", persist=False)
    assert page.positions.item(0, 0).text() == "左腕"
    assert frame == before
    page.close()
    backend.close()


def test_demo_catalog_does_not_duplicate_other_catalog_keys():
    directory = Path(i18n.__file__).parent
    demo_entries = json.loads((directory / "catalog_demo.json").read_text(encoding="utf-8"))
    for path in directory.glob("catalog_*.json"):
        if path.name != "catalog_demo.json":
            assert not demo_entries.keys() & json.loads(path.read_text(encoding="utf-8")).keys()
