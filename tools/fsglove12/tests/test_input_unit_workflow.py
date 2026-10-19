import json

import pytest

from vt_fsglove12.input_unit_workflow import stationary_windows_from_events


def _capture(t, step):
    return [{"event": "capture", "timestamp_s": t, "step": step, "session_id": "test"},
            {"event": "capture_complete", "timestamp_s": t + 5.02,
             "step": step, "session_id": "test"}]


def test_completed_guided_captures_only(tmp_path):
    events = [{"event": "capture", "timestamp_s": 1., "step": "neutral"},
              {"event": "clear_calibration", "timestamp_s": 2.}]
    for t, step in zip((100, 110, 120), ("neutral", "fingers_down", "thumb_up")):
        events.extend(_capture(t, step))
    path = tmp_path / "events.jsonl"
    path.write_text("\n".join(json.dumps(event) for event in events))
    windows, source = stationary_windows_from_events(path)
    assert len(windows) == 3
    assert windows[0] == {"start_s": 103.75, "end_s": 104.25, "label": "neutral",
                          "time_basis": "absolute_monotonic"}
    assert source["bytes"] == len(path.read_bytes())
    assert source["stationarity_requires_raw_validation"]


def test_incomplete_canceled_captures_not_inferred(tmp_path):
    path = tmp_path / "events.jsonl"
    events = _capture(10., "neutral") + _capture(20., "fingers_down")
    events.extend([{"event": "capture", "timestamp_s": 30., "step": "thumb_up"},
                   {"event": "cancel_capture", "timestamp_s": 31.},
                   {"event": "capture_complete", "timestamp_s": 35., "step": "thumb_up"}])
    path.write_text("\n".join(json.dumps(event) for event in events))
    with pytest.raises(ValueError, match="At least three"):
        stationary_windows_from_events(path)


def test_event_timings_match_runtime():
    from vt_fsglove12.live_runtime import CAPTURE_SECONDS, PREPARE_SECONDS
    assert PREPARE_SECONDS == 3
    assert CAPTURE_SECONDS == 2
