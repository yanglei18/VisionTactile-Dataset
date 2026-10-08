"""Actual spawn lifecycle for offline fitting, including hard cancellation."""
import builtins
import multiprocessing as mp
import os
import threading
from types import SimpleNamespace

import numpy as np
import pytest

from vt_fsglove12 import shape_job as jobs


def _contacts():
    return dict(mano_pose_rotvec=np.zeros((4, 16, 3)).tolist(), finger_category=[2, 3, 4, 5],
                contact_labels_confirmed=True, fixture_note="SYNTHETIC TEST DATA, NOT HUMAN CONTACTS")


def _fake_worker(contacts, options, directory, cancel, result):
    result.send(dict(progress=dict(stage="optimizing")))
    result.send(dict(report=dict(status="completed", candidate_available=True, input=contacts), error=None))
    result.close()


def _blocked_worker(contacts, options, directory, cancel, result, entered):
    entered.set()
    # Intentionally ignore cancellation to test the unchanged-optimizer case.
    threading.Event().wait(30.)
    result.send(dict(report=dict(status="completed", candidate_available=True), error=None))
    result.close()


def _reported_but_alive_worker(contacts, options, directory, cancel, result, entered):
    result.send(dict(report=dict(status="completed", candidate_available=True), error=None))
    entered.set()
    threading.Event().wait(30.)
    result.close()


def _serial_guard_worker(*args):
    original = builtins.__import__

    def no_serial(name, *rest, **kwargs):
        if name == "serial" or name.startswith("serial."):
            raise AssertionError("shape job must not own serial IO")
        return original(name, *rest, **kwargs)

    builtins.__import__ = no_serial
    from vt_fsglove12.shape_job import _optimize
    _optimize(*args)


def _instrument_child(monkeypatch, target, extra_args=()):
    context = mp.get_context("spawn")
    processes = []

    def process(**kwargs):
        assert kwargs["target"] is jobs._optimize
        kwargs["target"] = target
        kwargs["args"] = (*kwargs["args"], *extra_args)
        child = context.Process(**kwargs)
        processes.append(child)
        return child

    proxy = SimpleNamespace(Event=context.Event, Pipe=context.Pipe, Process=process)
    monkeypatch.setattr(jobs, "mp", SimpleNamespace(get_context=lambda method: proxy if method == "spawn" else None))
    return processes


def _finish(job, previous):
    job.join(20.)
    assert not job.is_alive() and job.done.is_set()
    assert not [child for child in mp.active_children() if child.pid not in previous]


def test_spawn_returns_result_and_freezes_caller_snapshot(tmp_path, monkeypatch):
    children = _instrument_child(monkeypatch, _fake_worker)
    previous = {child.pid for child in mp.active_children()}
    contacts = _contacts()
    job = jobs.ShapeJob(contacts, {}, tmp_path / "job")
    contacts["mano_pose_rotvec"][0][0][0] = 999.
    _finish(job, previous)
    assert job.result["error"] is None
    assert job.result["report"]["input"]["mano_pose_rotvec"][0][0][0] == 0.
    assert len(children) == 1 and children[0].exitcode == 0


def test_cancel_before_start_has_no_child_or_files(tmp_path, monkeypatch):
    children = _instrument_child(monkeypatch, _fake_worker)
    entered, release = threading.Event(), threading.Event()
    original = jobs.ShapeJob._run

    def delayed(self, *args):
        entered.set()
        release.wait(10.)
        original(self, *args)

    monkeypatch.setattr(jobs.ShapeJob, "_run", delayed)
    previous = {child.pid for child in mp.active_children()}
    job = jobs.ShapeJob(_contacts(), {}, tmp_path / "job")
    assert entered.wait(10.)
    job.cancel()
    release.set()
    _finish(job, previous)
    assert not children and not (tmp_path / "job").exists()
    assert job.result["report"]["status"] == "cancelled"
    assert job.result["report"]["shape_profile"] is None


def test_explicit_cancel_terminates_and_reaps_noncooperating_optimizer(tmp_path, monkeypatch):
    entered = mp.get_context("spawn").Event()
    children = _instrument_child(monkeypatch, _blocked_worker, (entered,))
    monkeypatch.setattr(jobs.ShapeJob, "CANCEL_GRACE_SECONDS", .1)
    previous = {child.pid for child in mp.active_children()}
    job = jobs.ShapeJob(_contacts(), {}, tmp_path / "job")
    try:
        assert entered.wait(10.)
        job.cancel()
        _finish(job, previous)
        assert children[0].exitcode != 0
        assert job.result["error"] is None
        assert job.result["report"]["status"] == "cancelled"
        assert not job.result["report"]["candidate_available"]
    finally:
        job.cancel()
        job.join(10.)


def test_cancel_after_result_message_still_reaps_child_and_discards_candidate(tmp_path, monkeypatch):
    entered = mp.get_context("spawn").Event()
    children = _instrument_child(monkeypatch, _reported_but_alive_worker, (entered,))
    monkeypatch.setattr(jobs.ShapeJob, "CANCEL_GRACE_SECONDS", .1)
    previous = {child.pid for child in mp.active_children()}
    job = jobs.ShapeJob(_contacts(), {}, tmp_path / "job")
    try:
        assert entered.wait(10.)
        job.cancel()
        _finish(job, previous)
        assert children[0].exitcode != 0
        assert job.result["report"]["status"] == "cancelled"
        assert not job.result["report"]["candidate_available"]
    finally:
        job.cancel()
        job.join(10.)


def test_cancel_completed_job_never_exposes_candidate(tmp_path, monkeypatch):
    _instrument_child(monkeypatch, _fake_worker)
    previous = {child.pid for child in mp.active_children()}
    job = jobs.ShapeJob(_contacts(), {}, tmp_path / "job")
    _finish(job, previous)
    assert job.result["report"]["candidate_available"]
    job.cancel()
    assert job.result["report"]["status"] == "cancelled"
    assert not job.result["report"]["candidate_available"]


def test_real_spawn_rejects_bad_contacts_before_loading_or_serial(tmp_path, monkeypatch):
    children = _instrument_child(monkeypatch, _serial_guard_worker)
    previous = {child.pid for child in mp.active_children()}
    contacts = _contacts()
    contacts["contact_labels_confirmed"] = False
    job = jobs.ShapeJob(contacts, dict(checkout="missing", mano_assets="missing"), tmp_path / "job")
    _finish(job, previous)
    assert children[0].exitcode == 0
    assert "contact_labels_confirmed" in job.result["error"]
    assert job.result["report"] is None and not (tmp_path / "job").exists()


@pytest.mark.skipif(not os.environ.get("FSGLOVE12_UPSTREAM_CHECKOUT") or not os.environ.get("FSGLOVE12_MANO_ASSETS"),
                    reason="Explicit trusted upstream checkout and MANO assets required")
def test_real_spawn_official_fit_no_serial_and_no_live_optimizer(tmp_path, monkeypatch):
    children = _instrument_child(monkeypatch, _serial_guard_worker)
    previous = {child.pid for child in mp.active_children()}
    options = dict(checkout=os.environ["FSGLOVE12_UPSTREAM_CHECKOUT"],
                   mano_assets=os.environ["FSGLOVE12_MANO_ASSETS"], iterations=2, seed=0)
    job = jobs.ShapeJob(_contacts(), options, tmp_path / "real")
    _finish(job, previous)
    assert children[0].exitcode == 0 and job.result["error"] is None
    assert job.result["report"]["status"] == "completed"
    assert not job.result["report"]["hardware_accessed"]
