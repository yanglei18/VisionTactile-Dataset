"""Process-writer/controller lifecycle integration with fake input only."""
import json

from vt_fsglove12.live_recording_process import ProcessResultRecorder
from test_live_runtime import harness, wait_for


def test_process_result_writer_records_and_drains_before_complete_summary(harness):
    h = harness
    h.controller._result_recorder_factory = ProcessResultRecorder
    h.connect()
    session = h.controller._session
    assert set(session.streams) == {"events"}  # Child exclusively owns result files.
    assert session.result_recorder.snapshot()["ready"]
    assert h.controller._writers == [session.result_recorder.worker]
    h.pump(nodes=tuple(range(1, 13)))
    h.pump(nodes=tuple(range(1, 13)))
    assert h.controller.submit("disconnect")
    summary_path = session.directory / "summary.json"
    wait_for(summary_path.exists, timeout=5.)
    summary = json.loads(summary_path.read_text())
    recording = summary["result_recording"]
    assert summary["recording_complete"] and recording["recording_complete"]
    assert recording["mode"] == "bounded_process"
    selected = summary["local_recording"]["policy"]["selected_total"]
    assert recording["accepted_pairs"] == recording["written_pairs"] == selected
    assert selected <= summary["output_frames"]
    assert recording["written_pairs"] > 0
    assert not recording["pending_pairs"] and not recording["failed_pairs"]
    assert not session.result_recorder.worker.is_alive()
    rows = {name: [json.loads(row) for row in (session.directory / f"{name}.jsonl").read_text().splitlines()]
            for name in ("orientations", "poses")}
    assert [row["frame_index"] for row in rows["orientations"]] == [row["frame_index"] for row in rows["poses"]]


def test_process_result_startup_failure_never_opens_input_source(harness):
    h = harness

    def occupied_result_path(paths):
        paths["poses"].write_text("existing file must survive\n")
        return ProcessResultRecorder(paths)

    h.controller._result_recorder_factory = occupied_result_path
    h.controller.start()
    assert h.controller.submit("connect", port="FAKE_NEVER_OPEN")
    wait_for(lambda: bool(list(h.output.glob("*/summary.json"))), timeout=5.)
    summary_path = next(h.output.glob("*/summary.json"))
    summary = json.loads(summary_path.read_text())
    assert not summary["recording_complete"]
    assert not summary["hardware_accessed"]
    assert h.sources and all(source.open_thread is None for source in h.sources)
    assert (summary_path.parent / "poses.jsonl").read_text() == "existing file must survive\n"
    assert summary["result_recording"]["error"]
    assert not h.controller.latest_snapshot()["runtime"]["connected"]
