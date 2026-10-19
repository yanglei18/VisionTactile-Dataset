"""Archive spatial views share historical time and never infer registration."""
import copy

from vt_capture_core.spatial.scene_protocol import validate_scene
from vt_capture_console.backend.archive_scene import ArchiveSceneBackend
from vt_capture_console.viewmodels.spatial import drawable


def tracker(topic="/tracker/left_wrist", *, time_ns=1_000_000_000, frame_id="recorded_map", position=None):
    return dict(topic=topic, time_ns=time_ns, sample_ns=time_ns,
                device=dict(kind="tracker", role="left_wrist", tracker_id="recorded-id", frame_id=frame_id,
                            source_realtime_ns=time_ns-10,
                            preview=dict(valid=True, position=position or [1., 2., 3.], orientation=[0., 0., 0., 1.])),
                details={}, warnings=[])


def snapshot(frames, *, time_ns=1_000_000_000, generation=1, seek_revision=0):
    return dict(time_ns=time_ns, frames={frame["topic"]: frame for frame in frames},
                generation=generation, seek_revision=seek_revision, path="recorded-bag")


def test_common_recorded_frame_uses_bag_playhead_and_preserves_source_time():
    backend = ArchiveSceneBackend()
    data = snapshot([tracker(), tracker("/tracker/other")], time_ns=1_100_000_000)
    original = copy.deepcopy(data)
    result = backend.update(data)
    validate_scene(result)
    assert result["mode"] == "replay" and result["time_basis"] == "bag_playhead"
    assert result["scene_time_ns"] == result["generated_realtime_ns"] == data["time_ns"]
    assert result["world_frame"] == "recorded_map" and result["map_identity"] is None
    assert all(drawable(item) for item in result["trackers"])
    entity = result["trackers"][0]
    assert entity["source_realtime_ns"] == 999_999_990
    assert entity["bag_timestamp_ns"] == 1_000_000_000 and entity["age_s"] == .1
    assert backend.status()["state"] == "ready"
    assert result["cameras"] == [] and backend.cloud_backend is None
    assert data == original


def test_mixed_or_missing_frames_are_not_combined_or_given_fallback_coordinates():
    backend = ArchiveSceneBackend()
    result = backend.update(snapshot([tracker(), tracker("/other", frame_id="other_map")]))
    assert result["world_frame"] == "unknown"
    assert not any(drawable(item) for item in result["trackers"])
    assert all(item["position_m"] is None for item in result["trackers"])
    assert any("坐标系" in text for text in result["diagnostics"])
    result = backend.update(snapshot([tracker(frame_id="")]))
    assert not drawable(result["trackers"][0])
    # An entity without a known frame does not hide others whose frame is known.
    result = backend.update(snapshot([tracker(), tracker("/unknown", frame_id="")]))
    assert [drawable(item) for item in result["trackers"]] == [True, False]


def test_camera_only_and_empty_tracker_leave_explicit_empty_spatial_view():
    backend = ArchiveSceneBackend()
    metadata = dict(topics=[dict(name="/empty", kind="tracker", count=0), dict(name="/rgb", kind="camera", count=3)])
    result = backend.update(snapshot([]), metadata)
    validate_scene(result)
    assert len(result["trackers"]) == 1 and not drawable(result["trackers"][0])
    assert "没有录制消息" in result["trackers"][0]["reason"]
    assert "话题详情" in backend.status()["message"]
    assert "预览" not in backend.status()["message"]
    assert backend.status()["state"] == "waiting" and result["cameras"] == []
    backend.clear()
    assert backend.latest()["trackers"] == []


def test_future_and_invalid_poses_do_not_leak_into_spatial_scene():
    backend = ArchiveSceneBackend()
    future = tracker(time_ns=1_100_000_000)
    result = backend.update(snapshot([future]))
    assert not drawable(result["trackers"][0])
    invalid = tracker()
    invalid["device"]["preview"]["position"][0] = float("nan")
    result = backend.update(snapshot([invalid]))
    validate_scene(result)
    assert not drawable(result["trackers"][0])


def test_continuous_trails_reset_on_seek_and_recording_change_not_topic_selection():
    backend = ArchiveSceneBackend()
    backend.update(snapshot([tracker()]))
    second = snapshot([tracker(time_ns=1_100_000_000, position=[2., 2., 3.])], time_ns=1_100_000_000)
    second["revision"] = 10
    result = backend.update(second)
    assert result["trackers"][0]["trail_segments"] == [[[1., 2., 3.], [2., 2., 3.]]]
    second["seek_revision"] += 1
    result = backend.update(second)
    assert result["trackers"][0]["trail_segments"] == [[[2., 2., 3.]]]
    third = snapshot([tracker(time_ns=1_200_000_000)], time_ns=1_200_000_000, generation=2, seek_revision=1)
    result = backend.update(third)
    assert result["trackers"][0]["trail_segments"] == [[[1., 2., 3.]]]


def test_inline_spatial_page_uses_shared_snapshot_without_own_clock_or_window(qapp):
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    backend = ArchiveSceneBackend()
    backend.update(snapshot([tracker()]))
    page = SpatialPage(backend, archive_embedded=True)
    page.resize(1000, 650)
    page.show()
    try:
        page.refresh()
        qapp.processEvents()
        assert len(page.model.entities()) == 1
        assert page.status.text() == "历史 Tracker 位姿 · recorded_map"
        assert "档案时间轴" in page.status.toolTip()
        assert "地图身份未核验" in page.status.toolTip()
        assert "尚未重建相机位姿、视锥和点云" in page.status.toolTip()
        assert "拖动时间轴后重置" in page.status.toolTip()
        assert not page.hint.isVisible()
        assert not page.replay_bar.isVisible()
        assert not page.replay_open_button.isVisible() and not page.live_button.isVisible()
        assert not page.configure_button.isVisible() and not page.configure_button.isEnabled()
        assert not page.expand_button.isVisible() and not page.freeze_button.isVisible()
        assert not page.cloud_check.isEnabled()
        page.open_detail()
        assert page.detail_window is None
        assert page.layer_checks["trail"].isEnabled() and page.view_preset.isEnabled()
    finally:
        page.close()


def test_inline_spatial_status_keeps_coordinate_problems_visible(qapp):
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    backend = ArchiveSceneBackend()
    page = SpatialPage(backend, archive_embedded=True)
    page.show()
    try:
        for frames in ([tracker(), tracker("/other", frame_id="other_map")],
                       [tracker(), tracker("/unknown", frame_id="")],
                       [tracker(frame_id="")]):
            backend.update(snapshot(frames))
            page.refresh()
            assert "Tracker 坐标系缺失或不同" in page.status.text()
            assert not page.hint.isVisible()
            assert any("坐标系缺失或不同" in entity["reason"] for entity in page.model.all_entities())
        backend.update(snapshot([]), dict(topics=[dict(name="/rgb", kind="camera", count=3)]))
        page.refresh()
        assert "话题详情" in page.status.text()
        assert "尚未重建" not in page.status.text()
        assert "尚未重建" in page.status.toolTip()
    finally:
        page.close()


def test_archive_copy_simplification_keeps_live_status_and_hint(qapp):
    from types import SimpleNamespace
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    frame = ArchiveSceneBackend().update(snapshot([tracker()]))
    frame["mode"] = "live"
    backend = SimpleNamespace(mode="live", latest=lambda: frame,
                              status=lambda: dict(state="ready", message="live status"),
                              set_interest=lambda *_: None)
    page = SpatialPage(backend)
    page.show()
    try:
        page.refresh()
        assert page.status.text() == "实时监看 · recorded_map · 各实体独立时刻 · 地图身份未核验"
        assert page.status.toolTip() == "live status"
        assert page.hint.isVisible()
        assert "相机需在空间配置中设置" in page.hint.text()
    finally:
        page.close()
