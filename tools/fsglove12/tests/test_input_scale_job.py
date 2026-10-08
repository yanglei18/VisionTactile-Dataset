"""Real spawn lifecycle; deterministic cancellation gates, no hardware access."""
import builtins
import json
import math
import multiprocessing as mp
import threading
from types import SimpleNamespace

from vt_fsglove12.config import load_config
from vt_fsglove12.live_stream import RawSample
import vt_fsglove12.input_scale_job as jobs


def _inputs(*, gain=1.):
    """One synthetic node, known tilt truth, three quiet two-second holds."""
    config = load_config()
    config["gyro_dps_per_lsb_by_node"] = {}
    rate = float(config["sample_rate_hz"])
    holds = [dict(label=label, start_s=1000. + start, end_s=1002. + start,
                  time_basis="absolute_monotonic")
             for label, start in (("neutral", 0.), ("fingers_down", 3.5), ("thumb_up", 7.))]
    samples, previous = [], 0.
    for index in range(round(9.2 * rate) + 1):
        stamp = index / rate
        angle = 60. * min(max(stamp - 2.2, 0.), 1.) - 45. * min(max(stamp - 5.7, 0.), 1.)
        speed = (angle - previous) * rate
        previous = angle
        radians = math.radians(angle)
        values = (round(speed / (config["gyro_dps_per_lsb"] * gain)) + 12, -8, 4,
                  0, round(math.sin(radians) / config["accel_g_per_lsb"]),
                  round(math.cos(radians) / config["accel_g_per_lsb"]))
        samples.append(RawSample(5, values, 1000. + stamp, 2000. + stamp, index % 256))
    return tuple(samples), config, holds


def _serial_guard_worker(*args):
    """Runs in the actual child; fail if the offline worker tries serial IO."""
    from vt_fsglove12.live_stream import SerialSource
    from vt_fsglove12.input_scale_job import _analyze

    original_import = builtins.__import__

    def deny_serial(name, *rest, **kwargs):
        if name == "serial" or name.startswith("serial."):
            raise AssertionError("offline scale job attempted to import serial")
        return original_import(name, *rest, **kwargs)

    def deny_source(*unused, **kwargs):
        raise AssertionError("offline scale job attempted to own a serial source")

    builtins.__import__ = deny_serial
    SerialSource.__init__ = deny_source
    _analyze(*args)


def _fit_gate_worker(samples, config, holds, directory, cancel, result,
                     entered, release, fitter_found_candidate):
    """Only the child fitter entry is gated; its real numerical path runs."""
    from vt_fsglove12 import input_scale_check
    from vt_fsglove12.input_scale_job import _analyze

    original_fit = input_scale_check.input_unit_fit.fit_input_units

    def gated_fit(*args, **kwargs):
        entered.set()
        if not release.wait(15.):
            raise RuntimeError("test fitter gate was not released")
        fitted = original_fit(*args, **kwargs)
        if fitted["recommended_nodes"]:
            fitter_found_candidate.set()
        return fitted

    input_scale_check.input_unit_fit.fit_input_units = gated_fit
    _analyze(samples, config, holds, directory, cancel, result)


def _instrument_child(monkeypatch, target, extra_args=()):
    """Preserve a real spawn context, replacing only the test child entry."""
    context = mp.get_context("spawn")
    processes = []

    def process(**kwargs):
        assert kwargs["target"] is jobs._analyze
        kwargs["target"] = target
        kwargs["args"] = (*kwargs["args"], *extra_args)
        child = context.Process(**kwargs)
        processes.append(child)
        return child

    proxy = SimpleNamespace(Event=context.Event, Pipe=context.Pipe, Process=process)
    monkeypatch.setattr(jobs, "mp", SimpleNamespace(get_context=lambda method: proxy if method == "spawn" else None))
    return processes


def _finished(job, previous_children):
    job.join(15.)
    assert not job.is_alive() and job.done.is_set()
    assert not job._thread.is_alive()
    assert not [child for child in mp.active_children() if child.pid not in previous_children]


def test_real_spawn_returns_report_and_never_owns_serial(tmp_path, monkeypatch):
    samples, config, holds = _inputs()
    children = _instrument_child(monkeypatch, _serial_guard_worker)
    previous = {child.pid for child in mp.active_children()}
    job = jobs.InputScaleJob(samples, config, holds, tmp_path / "success")
    try:
        _finished(job, previous)
        assert len(children) == 1 and children[0].exitcode == 0
        assert job.result["error"] is None
        report = job.result["report"]
        assert report["status"] == "completed" and report["nominal_supported_nodes"] == ["5"]
        assert not report["candidate_available"] and not report["hardware_accessed"]
        assert json.loads((tmp_path / "success" / "report.json").read_text()) == report
    finally:
        job.cancel()
        job.join(15.)


def test_pre_start_cancel_has_no_child_or_output(tmp_path, monkeypatch):
    samples, config, holds = _inputs()
    entered, release = threading.Event(), threading.Event()
    original_run = jobs.InputScaleJob._run

    def gated_run(self, *args):
        entered.set()
        release.wait(15.)
        original_run(self, *args)

    monkeypatch.setattr(jobs.InputScaleJob, "_run", gated_run)
    children = _instrument_child(monkeypatch, _serial_guard_worker)
    previous = {child.pid for child in mp.active_children()}
    job = jobs.InputScaleJob(samples, config, holds, tmp_path / "cancelled")
    try:
        assert entered.wait(15.)
        job.cancel()
        release.set()
        _finished(job, previous)
        assert children == [] and job.result is None
        assert not (tmp_path / "cancelled").exists()
    finally:
        job.cancel()
        release.set()
        job.join(15.)


def test_real_spawn_cancel_during_fit_discards_a_real_candidate(tmp_path, monkeypatch):
    samples, config, holds = _inputs(gain=2.)
    context = mp.get_context("spawn")
    entered, release, fitted_candidate = context.Event(), context.Event(), context.Event()
    children = _instrument_child(monkeypatch, _fit_gate_worker, (entered, release, fitted_candidate))
    previous = {child.pid for child in mp.active_children()}
    job = jobs.InputScaleJob(samples, config, holds, tmp_path / "cancelled")
    try:
        assert entered.wait(15.), "real spawned worker did not reach the fitter"
        job.cancel()
        release.set()
        _finished(job, previous)
        assert len(children) == 1 and children[0].exitcode == 0
        assert fitted_candidate.is_set(), "the unchanged fitter must have found the synthetic 2x candidate"
        assert job.result["error"] is None
        report = job.result["report"]
        assert report["status"] == "cancelled"
        assert not report["candidate_available"] and report["candidate_config"] is None
        assert report["candidate_path"] is None and report["recommended_nodes"] == []
        assert not list((tmp_path / "cancelled").glob("config.candidate*"))
        assert json.loads((tmp_path / "cancelled" / "report.json").read_text())["status"] == "cancelled"
    finally:
        job.cancel()
        release.set()
        job.join(15.)


def test_real_spawn_invalid_holds_returns_error_and_joins(tmp_path):
    samples, config, holds = _inputs()
    previous = {child.pid for child in mp.active_children()}
    job = jobs.InputScaleJob(samples, config, holds[:2], tmp_path / "invalid")
    try:
        _finished(job, previous)
        assert job.result["report"] is None
        assert "ValueError" in job.result["error"] and "three" in job.result["error"]
        assert not (tmp_path / "invalid").exists()
    finally:
        job.cancel()
        job.join(15.)


def test_real_spawn_existing_output_returns_error_without_replacement(tmp_path):
    samples, config, holds = _inputs()
    target = tmp_path / "existing"
    target.mkdir()
    (target / "raw.csv").write_bytes(b"original raw evidence\n")
    (target / "report.json").write_bytes(b'{"original": true}\n')
    before = {path.name: path.read_bytes() for path in target.iterdir()}
    previous = {child.pid for child in mp.active_children()}
    job = jobs.InputScaleJob(samples, config, holds, target)
    try:
        _finished(job, previous)
        assert job.result["report"] is None and "FileExistsError" in job.result["error"]
        assert {path.name: path.read_bytes() for path in target.iterdir()} == before
    finally:
        job.cancel()
        job.join(15.)
