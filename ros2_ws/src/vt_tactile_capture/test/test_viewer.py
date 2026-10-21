from copy import deepcopy
import csv
import json
import math
import os
from types import SimpleNamespace

import pytest

from vt_tactile_capture.viewer import ViewerModel, create_window, normalize_range


def state():
    return SimpleNamespace(
        schema_version=2, rows=20, cols=20, hand="right", session_id="test", stream_epoch=1,
        header=SimpleNamespace(stamp=SimpleNamespace(sec=10, nanosec=0)),
        host_realtime_ns=10_000_000_000, host_monotonic_ns=8_000_000_000,
        snapshot_monotonic_ns=8_000_000_000,
        source_sequence=3, adc=[0] * 400, valid_mask=[False] * 400,
        updated_mask=[False] * 400, block_realtime_ns=[0] * 160,
        block_monotonic_ns=[0] * 160, connected=True, time_valid=True,
        reason="ok", max_age_ns=0, producer_dropped_total=0,
    )


def test_range_inclusive_sentinel_and_reversed_bounds():
    assert normalize_range(0, 20) == (0, 19)
    assert normalize_range(15, 5) == (5, 15)
    assert normalize_range(-1, 999) == (0, 19)


def test_legacy_ros_state_is_cropped_for_view_and_csv_without_restamping(tmp_path):
    message = state()
    message.schema_version, message.rows, message.cols = 1, 80, 80
    message.adc = list(range(6400))
    message.valid_mask = [True] * 6400
    message.updated_mask = [True] * 6400
    message.block_realtime_ns = list(range(640))
    message.block_monotonic_ns = list(range(640))
    model = ViewerModel()
    assert model.observe(message)
    assert len(model.data) == 400 and model.data[20] == 80
    assert model.data[-1] == 1539
    assert model.message.schema_version == 2
    assert model.message.header.stamp == message.header.stamp
    assert len(model.display_block_monotonic_ns) == 160
    model.save_csv(tmp_path / "legacy_cropped.csv")
    model.save_metadata(tmp_path / "legacy_cropped.json")
    metadata = json.loads((tmp_path / "legacy_cropped.json").read_text())
    assert (metadata["rows"], metadata["cols"]) == (20, 20)
    assert len(metadata["current_valid_mask"]) == 400


def test_missing_is_not_zero_and_valid_zero_is_displayed():
    model = ViewerModel()
    message = state()
    message.valid_mask[8] = True
    message.block_realtime_ns[0] = 10_000_000_000
    assert model.observe(message)
    assert math.isnan(model.data[0])
    assert model.data[8] == 0
    assert model.display_block_realtime_ns[0] == 10_000_000_000
    assert "LIVE" in model.status(10_000_000_000)


def test_disconnect_retains_image_and_original_source_time():
    clock = [1_000_000_000]
    model = ViewerModel(monotonic_ns=lambda: clock[0])
    message = state()
    message.valid_mask[8], message.adc[8] = True, 22444
    model.observe(message)
    disconnected = deepcopy(message)
    disconnected.connected = False
    disconnected.valid_mask = [False] * 400
    disconnected.adc = [0] * 400
    disconnected.reason = "serial_disconnected"
    model.observe(disconnected)
    clock[0] += 3_000_000_000
    assert model.data[8] == 22444
    assert "RETAINED / STALE" in model.status(13_000_000_000)
    assert "source age (ROS system): 3.00s" in model.status(13_000_000_000)


def test_replay_clock_and_arrival_watchdog_are_separate():
    clock = [1_000_000_000]
    model = ViewerModel(monotonic_ns=lambda: clock[0])
    message = state()
    message.valid_mask[0] = True
    model.observe(message)
    clock[0] += 6_000_000_000
    label = model.status(10_000_000_000, use_sim_time=True)
    assert "paused/stopped" in label
    assert "source age (ROS playback): 0.00s" in label
    assert "arrival age (steady): 6.00s" in label


def test_no_valid_measurements_or_bad_time_never_says_live():
    model = ViewerModel()
    message = state()
    model.observe(message)
    assert "LIVE" not in model.status(10_000_000_000)
    message.valid_mask[0] = True
    message.time_valid = False
    model.observe(message)
    assert "LIVE" not in model.status(10_000_000_000)


def test_epoch_change_retains_until_new_data_then_clears_and_rejects_bad_shape():
    model = ViewerModel()
    message = state()
    message.valid_mask[0], message.adc[0] = True, 20001
    model.observe(message)
    new_epoch = state()
    new_epoch.stream_epoch = 2
    model.observe(new_epoch)
    assert model.data[0] == 20001
    new_epoch.valid_mask[8], new_epoch.adc[8] = True, 23000
    model.observe(new_epoch)
    assert math.isnan(model.data[0])
    assert model.data[8] == 23000
    new_epoch.adc.pop()
    assert not model.observe(new_epoch)
    assert model.rejected == 1


def test_csv_and_metadata_preserve_missing_zero_and_historical_provenance(tmp_path):
    model = ViewerModel()
    message = state()
    message.valid_mask[8] = True
    message.block_realtime_ns[0] = 10_000_000_000
    message.block_monotonic_ns[0] = 8_000_000_000
    model.observe(message)
    model.save_csv(tmp_path / "grid.csv")
    model.save_metadata(tmp_path / "grid.json")
    with (tmp_path / "grid.csv").open() as handle:
        rows = list(csv.reader(handle))
    metadata = json.loads((tmp_path / "grid.json").read_text())
    assert len(rows) == 21
    assert rows[1][1] == ""
    assert rows[1][9] == "0"
    assert metadata["current_valid_mask"][8]
    assert metadata["display_block_realtime_ns"][0] == 10_000_000_000


def test_offscreen_window_range_and_png_csv_export(tmp_path):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("pyqtgraph")
    QtWidgets = pytest.importorskip("pyqtgraph.Qt").QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    model = ViewerModel()
    message = state()
    message.valid_mask[8], message.adc[8] = True, 22000
    model.observe(message)
    window = create_window(model)
    try:
        assert isinstance(window, QtWidgets.QMainWindow)
        assert isinstance(window.hand_graphics, QtWidgets.QWidget)
        assert isinstance(window.graphics, QtWidgets.QWidget)
        window.show()
        app.processEvents()
        window.bounds[("Row", "start")].setValue(4)
        window.bounds[("Row", "end")].setValue(10)
        window.apply_range()
        assert window.view_range == (4, 10, 0, 19)
        assert window.image.axisOrder == "row-major"
        assert tuple(window.image.getLevels()) == (20000, 24000)
        window.full_range()
        window.export(tmp_path / "snapshot.png")
        assert (tmp_path / "snapshot.png").read_bytes().startswith(b"\x89PNG")
        assert (tmp_path / "snapshot.csv").is_file()
        assert (tmp_path / "snapshot.json").is_file()
        window.clear_button.click()
        assert all(math.isnan(value) for value in model.data)
        assert model.message is None
        assert "capture continues" in window.status_label.text()
    finally:
        window.timer.stop()
        window.close()


def test_hand_view_signed_reference_mapping_and_raw_export(tmp_path):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    np = pytest.importorskip("numpy")
    pytest.importorskip("pyqtgraph")
    QtWidgets = pytest.importorskip("pyqtgraph.Qt").QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    reference = tmp_path / "baseline.npz"
    np.savez(reference, adc=np.full((12, 20, 20), 18000, dtype=np.uint16),
             valid_mask=np.ones((12, 20, 20), dtype=bool))
    reference.with_suffix(".json").write_text(json.dumps({"session_epoch_frame_counts": {"test:1": 12}}))
    model = ViewerModel()
    message = state()
    message.adc = [18000] * 400
    message.valid_mask = [True] * 400
    message.adc[19 * 20 + 19] = 31000
    message.adc[4 * 20 + 9] = 17900
    original_adc = list(message.adc)
    model.observe(message)
    window = create_window(model, baseline_path=reference)
    try:
        window.show()
        window.tick()
        app.processEvents()
        assert window.tabs.currentIndex() == 0
        assert len(window.hand_scatter.points()) == 400
        assert len(window.anchor_scatter.points()) == 9
        assert window.value_mode.currentData() == "delta"
        assert tuple(window.image.getLevels()) == (-3000, 3000)
        assert not window.anchor_scatter.isVisible()
        assert not window.matrix_controls.isVisible()
        assert not window.advanced_widget.isVisible()
        assert window.minimumSizeHint().height() < 940
        assert window.height() == 940
        window.anchors_toggle.setChecked(True)
        assert window.anchor_scatter.isVisible()
        assert "13000" in window.region_widgets["thumb"][0].text()
        assert "超色标" in window.region_widgets["thumb"][2].text()
        # Reference-adjacent negative values get neutral color but stay signed.
        palm_index = next(i for i,p in enumerate(window.hand_layout.points) if (p.row,p.col)==(4,9))
        assert tuple(window.hand_cells.colors[palm_index]) == (155,174,185,255)
        window.range_preset.setCurrentIndex(1)
        assert tuple(window.image.getLevels()) == (-16000,16000)
        assert "超色标" not in window.region_widgets["thumb"][2].text()
        assert window.image.image[19, 19] == 13000
        assert window.image.image[4, 9] == -100
        assert window.image.image.shape == (20, 20)
        assert "20×20 / 400" in window.unmapped_label.text()
        window.deadband.setValue(0)
        assert tuple(window.hand_cells.colors[palm_index]) != (155,174,185,255)
        assert window.image.image[4, 9] == -100
        window.deadband.setValue(200)
        assert message.adc == original_adc
        assert model.data[19 * 20 + 19] == 31000
        window.export(tmp_path / "hand.png")
        metadata = json.loads((tmp_path / "hand.json").read_text())
        assert metadata["visualization"]["layout_status"] == "candidate"
        assert metadata["visualization"]["baseline_adc_row_major"][19 * 20 + 19] == 18000
        with (tmp_path / "hand.csv").open() as handle:
            rows = list(csv.reader(handle))
        assert rows[20][20] == "31000"  # CSV exports raw, never delta or color-clipped ADC.
        assert len(rows) == 21 and len(rows[0]) == 21
        window.mapped_range()
        assert window.tabs.currentIndex() == 1
        assert window.view_range == (0, 19, 0, 19)
        assert window.matrix_controls.isVisible()
        assert window.image.image.shape == (20, 20)
        window.full_range()
        assert window.image.image.shape == (20, 20)
        window.details_toggle.setChecked(True)
        assert window.advanced_widget.isVisible()
        reconnect = deepcopy(message)
        reconnect.stream_epoch = 2
        model.observe(reconnect)
        window.tick()
        assert window.baseline.values is None
        assert np.isnan(window.image.image).all()  # Never subtract an old-epoch reference.
        window.value_mode.setCurrentIndex(0)
        assert window.image.image[19, 19] == 31000
    finally:
        window.timer.stop()
        window.close()


def test_hand_manual_reference_finishes_in_observe_and_switches_mode(monkeypatch):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("pyqtgraph")
    QtWidgets = pytest.importorskip("pyqtgraph.Qt").QtWidgets
    from vt_tactile_capture import viewer
    clock = [1_000_000_000]
    monkeypatch.setattr(viewer.time, "monotonic_ns", lambda: clock[0])
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    model = ViewerModel(monotonic_ns=lambda: clock[0])
    message = state()
    message.adc = [18000] * 400
    message.valid_mask = [True] * 400
    model.observe(message)
    window = create_window(model)
    try:
        window.tick()
        window.start_baseline()
        for index in range(90):
            clock[0] = 1_000_000_000 + round((index + 1) / 30 * 1e9)
            current = deepcopy(message)
            current.source_sequence += index + 1
            model.observe(current)
            window.tick()
        assert not window.baseline.collecting
        assert window.baseline.values is not None
        assert window.value_mode.currentData() == "delta"
        assert window.baseline_button.isEnabled()
        assert window.image.image[0, 0] == 0
        window.start_baseline()
        assert window.baseline.collecting
        disconnected = deepcopy(message)
        disconnected.connected = False
        model.observe(disconnected)
        model.observe(current)  # Healthy again before the next rendered frame.
        window.tick()
        assert not window.baseline.collecting
        assert window.baseline.values is None
        assert "cancelled" in window.baseline.status
        window.clear_display()
        assert window.baseline.values is None
        assert model.message is None
    finally:
        window.timer.stop()
        window.close()
    assert not model.state_listeners


def test_left_stream_is_not_silently_drawn_as_right_hand():
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("pyqtgraph")
    QtWidgets = pytest.importorskip("pyqtgraph.Qt").QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    model = ViewerModel()
    message = state()
    message.hand = "left"
    message.valid_mask = [True] * 400
    message.adc = [24000] * 400
    model.observe(message)
    window = create_window(model)
    try:
        window.tick()
        assert "不匹配" in window.hand_plot.titleLabel.text
        assert tuple(window.hand_cells.colors[0]) == (65, 72, 80, 255)
        assert window.image.image[0, 0] == 24000  # Raw view still works.
    finally:
        window.timer.stop()
        window.close()


def test_two_viewer_models_filter_other_hand_and_never_share_pixel_history():
    left, right = ViewerModel(expected_hand="left"), ViewerModel(expected_hand="right")
    left_message, right_message = state(), state()
    left_message.hand = "left"
    left_message.valid_mask[0], left_message.adc[0] = True, 111
    right_message.valid_mask[0], right_message.adc[0] = True, 222
    for model, own, other in ((left, left_message, right_message), (right, right_message, left_message)):
        assert model.observe(own)
        assert not model.observe(other)
        assert model.data[0] == own.adc[0]
    left.clear_display()
    assert math.isnan(left.data[0]) and right.data[0] == 222


def test_left_default_window_is_raw_matrix_without_anatomical_claim(tmp_path):
    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    np = pytest.importorskip("numpy")
    pytest.importorskip("pyqtgraph")
    QtWidgets = pytest.importorskip("pyqtgraph.Qt").QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    model = ViewerModel(expected_hand="left")
    message = state()
    message.hand = "left"
    message.adc, message.valid_mask = list(range(400)), [True] * 400
    model.observe(message)
    window = create_window(model)
    try:
        window.tick()
        assert "左手" in window.windowTitle()
        assert "尚未选择" in window.layout_label.text()
        assert window.tabs.currentIndex() == 1 and not window.tabs.isTabEnabled(0)
        assert not window.hand_layout.points
        np.testing.assert_array_equal(window.image.image, np.arange(400).reshape(20, 20))
        window.export(tmp_path / "left_raw.png")
        metadata = json.loads((tmp_path / "left_raw.json").read_text())
        assert metadata["hand"] == "left"
        assert metadata["visualization"]["layout_status"] == "unmapped"
    finally:
        window.close()


def test_explicit_left_candidate_window_labels_unverified_and_keeps_raw_addresses():
    from pathlib import Path
    import vt_tactile_capture.hand_layout as layout_module

    os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
    pytest.importorskip("pyqtgraph")
    QtWidgets = pytest.importorskip("pyqtgraph.Qt").QtWidgets
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    layout = layout_module.load_layout(Path(layout_module.__file__).parent / "layouts/left_glove_unverified_candidate.json")
    model = ViewerModel(expected_hand="left")
    message = state()
    message.hand = "left"
    message.adc = list(range(400)); message.valid_mask = [True] * 400
    model.observe(message)
    window = create_window(model, hand_layout=layout)
    try:
        window.tick()
        assert window.tabs.isTabEnabled(0)
        assert len(window.hand_scatter.points()) == 400
        assert len(window.anchor_scatter.points()) == 0
        assert "未验证" in window.layout_label.text()
        assert "左手" in window.hand_plot.titleLabel.text
        assert window.image.image[19, 19] == 399
        assert window.baseline.expected_hand == "left"
    finally:
        window.close()
