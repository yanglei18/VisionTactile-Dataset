"""Offline model and hidden Tk smoke; never launch ROS, sensors or rosbag record."""
import json
from pathlib import Path
import subprocess

import pytest

from vt_tracker_camera_calib.gui import CalibrationWindow, OfflineSession, main
from vt_tracker_camera_calib.gui_workflow import prepare_config


class FakeJob:
    def __init__(self):
        self.argv = None
        self.log_path = None
        self.code = None
        self.running = False
        self.stops = 0

    def start(self, argv, log_path):
        assert "calibrate" in argv and "record" not in argv
        self.argv, self.log_path, self.running = argv, log_path, True
        log_path.write_text("synthetic solver log\n")

    def poll(self):
        if self.code is not None:
            self.running = False
        return self.code

    def stop(self):
        self.stops += 1


@pytest.fixture
def prepared(tmp_path):
    path = prepare_config(tmp_path / "inputs", "d436", "torso", 40, 30)
    session = OfflineSession(tmp_path / "sessions", job_factory=FakeJob)
    session.select_config(path)
    bag = tmp_path / "closed bag ; inert"
    bag.mkdir()
    (bag / "metadata.yaml").write_text("synthetic metadata only\n")
    return session, bag, tmp_path / "results"


def report(output, quality="TARGET"):
    output.mkdir()
    (output / "report.json").write_text(json.dumps(dict(quality=quality, counts=dict(selected_static_pairs=40),
        metrics=dict(validation_translation_rms_m=.001, validation_rotation_rms_deg=.1))))


def test_construction_load_board_and_help_never_start_process(tmp_path, monkeypatch, capsys):
    def forbidden(*args, **kwargs):
        pytest.fail("No automatic process/hardware action is authorized")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    session = OfflineSession(tmp_path / "not-created")
    assert not session.running and not (tmp_path / "not-created").exists()
    path = prepare_config(tmp_path / "input", "d405_1", "left_wrist", 40, 30)
    session.select_config(path)
    assert session.board_image().size == (640, 420)
    assert not (tmp_path / "not-created").exists()
    with pytest.raises(SystemExit) as exit:
        main(["--help"])
    assert exit.value.code == 0 and "不连接硬件" in capsys.readouterr().out


def test_generate_import_and_export_reuse_existing_workflow(tmp_path):
    session = OfflineSession(tmp_path / "sessions", job_factory=FakeJob)
    generated = session.prepare("d405_2", "right_wrist", 40.1, 29.9, squares_x=9, squares_y=6)
    imported = session.import_configuration(generated)
    assert imported != generated and imported.read_bytes() == generated.read_bytes()
    board = session.export_board(tmp_path / "board.png")
    assert board.is_file()
    with pytest.raises(FileExistsError):
        session.export_board(board)
    assert session.job.argv is None


def test_solve_requires_closed_bag_and_new_external_output(prepared):
    session, bag, output = prepared
    with pytest.raises(ValueError, match="明确确认"):
        session.solve(bag, output)
    with pytest.raises(ValueError, match="源 bag"):
        session.solve(bag, bag / "result", closed_confirmed=True)
    assert session.job.argv is None
    session.solve(bag, output, closed_confirmed=True)
    argv = session.job.argv
    assert argv[1:4] == ["-m", "vt_tracker_camera_calib.cli", "calibrate"]
    assert argv[argv.index("--bag") + 1] == str(bag)
    snapshot = Path(argv[argv.index("--config") + 1])
    assert snapshot != session.config_path and snapshot.read_bytes() == session.config_path.read_bytes()
    with pytest.raises(ValueError, match="并发"):
        session.solve(bag, output, closed_confirmed=True)
    with pytest.raises(ValueError, match="切换配置"):
        session.select_config(snapshot)


@pytest.mark.parametrize("code,quality,expected", [(0, "TARGET", "COMPLETE"), (0, "ACCEPTABLE", "COMPLETE"),
                                                  (0, "REJECTED", "FAILED"), (2, "REJECTED", "FAILED"), (2, "TARGET", "FAILED")])
def test_only_consistent_exit_and_quality_report_is_success(prepared, code, quality, expected):
    session, bag, output = prepared
    session.solve(bag, output, closed_confirmed=True)
    report(output, quality)
    session.job.code = code
    session.poll()
    assert session.state == expected and not session.running
    assert session.report["quality"] == quality and "synthetic" in session.log


def test_zero_exit_without_report_does_not_claim_success(prepared):
    session, bag, output = prepared
    session.solve(bag, output, closed_confirmed=True)
    session.job.code = 0
    session.poll()
    assert session.state == "FAILED" and "有效质量报告" in session.detail


def test_cancel_waits_for_own_job_exit_and_never_marks_complete(prepared):
    session, bag, output = prepared
    session.solve(bag, output, closed_confirmed=True)
    session.cancel()
    assert session.running and session.state == "CANCELLING" and session.job.stops == 1
    session.job.code = 130
    session.poll()
    assert not session.running and session.state == "CANCELLED"


def test_new_config_clears_previous_report_and_log(prepared):
    session, bag, output = prepared
    session.solve(bag, output, closed_confirmed=True)
    report(output)
    session.job.code = 0
    session.poll()
    assert session.report
    session.select_config(session.config_path)
    session.poll()
    assert session.report is None and session.log == "" and session.state == "IDLE"


def test_report_and_log_reads_are_bounded(prepared, monkeypatch):
    from vt_tracker_camera_calib import gui
    session, bag, output = prepared
    session.solve(bag, output, closed_confirmed=True)
    report(output)
    monkeypatch.setattr(gui, "MAX_REPORT_BYTES", 8)
    with pytest.raises(ValueError, match="受限"):
        session.load_report(output / "report.json")
    session.job.log_path.write_text("a" * 2000)
    monkeypatch.setattr(gui, "MAX_LOG_BYTES", 32)
    session.poll()
    assert len(session.log) == 32


@pytest.fixture
def tk_root():
    tk = pytest.importorskip("tkinter", reason="GUI dependency python3-tk is optional for offline CLI tests")
    try:
        root = tk.Tk()
    except tk.TclError:
        pytest.skip("Tk display unavailable; run hidden smoke under Xvfb or a desktop session")
    root.withdraw()
    yield root
    try:
        for identifier in root.tk.call("after", "info"):
            root.after_cancel(identifier)
        root.destroy()
    except tk.TclError:
        pass


def test_hidden_tk_opens_offline_and_generates_board_without_process(tk_root, tmp_path, monkeypatch):
    def forbidden(*args, **kwargs):
        pytest.fail("Hidden Tk smoke must not execute a process")
    monkeypatch.setattr(subprocess, "Popen", forbidden)
    session = OfflineSession(tmp_path / "sessions", job_factory=FakeJob)
    window = CalibrationWindow(tk_root, session=session)
    tk_root.update()
    assert window.variables["camera"].get() == window.variables["role"].get() == ""
    assert "在线预览／专用录包尚未接入" in session.detail
    assert not (tmp_path / "sessions").exists()
    window.variables["camera"].set("d405_1")
    window.variables["role"].set("torso")
    with pytest.raises(ValueError, match="实测"):
        window.prepare_configuration()
    window.measured.set(True)
    window.prepare_configuration()
    tk_root.update()
    assert session.config_path.is_file() and window._photo is not None
    assert session.job.argv is None
    window.close()


def test_hidden_tk_solve_report_and_close_waits_for_fake_child(tk_root, prepared):
    session, bag, output = prepared
    window = CalibrationWindow(tk_root, session=session, config_path=session.config_path)
    window.variables["bag"].set(str(bag))
    window.variables["output"].set(str(output))
    window.closed_confirmed.set(True)
    window.start_solve()
    window.close()
    assert tk_root.winfo_exists() and session.running and session.job.stops == 1
    session.job.code = 130
    window._refresh()
    assert session.state == "CANCELLED" and not session.running


def test_offline_session_defaults_match_reference_board_and_pattern_dimensions(tmp_path):
    from PIL import Image
    session = OfflineSession(tmp_path / "sessions", job_factory=FakeJob)
    session.prepare("d405_1", "torso", 15., 11.25)
    assert (session.config.board.squares_x, session.config.board.squares_y) == (12, 9)
    output = session.export_board(tmp_path / "board.png")
    with Image.open(output) as board:
        assert board.size == (round(.180 * 300 / .0254), round(.135 * 300 / .0254))
    assert session.job.argv is None


def test_tk_initial_board_fields_use_shared_geometry(tk_root, tmp_path):
    from vt_capture_core.calibration_policy import board_geometry_defaults
    session = OfflineSession(tmp_path / "sessions", job_factory=FakeJob)
    window = CalibrationWindow(tk_root, session=session)
    try:
        defaults = board_geometry_defaults()
        assert int(window.variables["squares_x"].get()) == defaults["squares_x"] == 12
        assert int(window.variables["squares_y"].get()) == defaults["squares_y"] == 9
        assert float(window.variables["square"].get()) == defaults["square_length_mm"] == 15.
        assert float(window.variables["marker"].get()) == defaults["marker_length_mm"] == 11.25
        assert window.variables["dictionary"].get() == defaults["dictionary"]
        assert session.config_path is None
        assert not (tmp_path / "sessions").exists()
    finally:
        window.close()
