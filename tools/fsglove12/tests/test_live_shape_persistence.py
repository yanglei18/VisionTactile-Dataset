"""End-to-end command transactions for log-independent calibration TASKs."""
from copy import deepcopy
import json
from pathlib import Path

import pytest

from test_live_shape_workflow import workflow, candidate, complete, fill_four, fresh
from vt_fsglove12 import calibration_artifacts


def test_each_completed_group_is_durable_even_after_bulk_quota_latch(workflow):
    h, c = workflow, workflow.controller
    c._disable_recording(h.session, 'max_session_bytes', 'fixture 2 GiB quota')
    for count, category in enumerate((2, 3, 4, 5), 1):
        complete(h, category)
        state = c._shape_state(h.clock())
        assert not state['storage_error']
        path = Path(state['task_path'])
        task = calibration_artifacts.load_task(path, c._shape_expected_provenance())
        assert len(task['groups']) == count
        assert len(list((h.directory / 'calibration').glob('touch_*.json'))) == count
        assert state['saved_groups'] == [str(k) for k in range(2, category + 1)]
        assert all(v['saved'] for v in state['contacts'].values())
        assert h.session.recording_disabled.is_set()
    before = deepcopy(c._profile)
    c._command('optimize_shape', {})
    assert c._shape_error is None and c._shape_job is not None
    assert c._shape_job.contacts['source_session_id'] == h.session.session_id
    report = candidate(h)
    c._shape_candidate = None
    c._shape_job.result, c._shape_job.running = {'report': report, 'error': None}, False
    c._poll_shape()
    c._command('apply_shape_candidate', {})
    assert c._profile['shape_params'] == [.7] * 10
    assert c._profile['upstream_calibration'] == before['upstream_calibration']
    assert h.session.profile_saved and list(h.directory.glob('profile.v*.json'))
    c._command('save_shape_profile', {'path': str(h.directory / 'export.json')})
    assert c._shape_error is None and (h.directory / 'export.json').is_file()
    h.clock.advance(.1)
    fresh(h)
    assert c._tick(h.session)
    assert h.exporter.documents and not h.session.stop.is_set()
    assert h.session.recording_disabled.is_set()  # Never silently re-enable bulk logs.


def test_failed_checkpoint_keeps_rows_and_retries_to_another_directory(workflow, monkeypatch):
    h, c = workflow, workflow.controller
    real = calibration_artifacts.save_task
    attempts = []
    def fail(path, document, provenance=None):
        attempts.append(path)
        raise OSError('fixture filesystem full')
    monkeypatch.setattr(calibration_artifacts, 'save_task', fail)
    complete(h)
    rows = deepcopy(c._shape_contacts[2]['rows'])
    state = c._shape_state(h.clock())
    assert state['status'] == 'waiting' and state['can_retry_save']
    assert state['unsaved_groups'] == ['2'] and 'filesystem full' in state['storage_error']
    assert c._shape_contacts[2]['rows'] == rows
    assert not h.session.recording_disabled.is_set() and not h.session.stop.is_set()
    for _ in range(3):
        h.clock.advance(.1)
        fresh(h)
        c._tick(h.session)
        c._poll_shape()
    assert len(attempts) == 1  # No per-pose retry/write loop.
    monkeypatch.setattr(calibration_artifacts, 'save_task', real)
    alternate = h.directory / 'alternate'
    c._command('retry_calibration_save_to', {'directory': str(alternate)})
    assert c._shape_error is None and c._shape_storage_error is None
    saved = Path(c._shape_task_path)
    assert saved.parent == alternate and saved.is_file()
    assert c._shape_contacts[2]['rows'] == rows
    assert c._shape_state(h.clock())['saved_groups'] == ['2']
    complete(h, category=3)
    assert Path(c._shape_task_path).parent == alternate  # Subsequent captures stay here.


def test_failed_recapture_retains_prior_saved_revision(workflow):
    h, c = workflow, workflow.controller
    complete(h)
    path = c._shape_task_path
    rows = deepcopy(c._shape_contacts[2]['rows'])
    complete(h, count=9)
    assert c._shape_task_path == path
    assert c._shape_contacts[2]['rows'] == rows
    assert c._shape_state(h.clock())['saved_groups'] == ['2']


def test_task_export_refuses_overwrite_and_leaves_original_evidence(workflow):
    h, c = workflow, workflow.controller
    complete(h)
    path = h.directory / 'task.json'
    c._command('save_shape_task', {'path': str(path)})
    original = path.read_bytes()
    complete(h, category=3)
    c._command('save_shape_task', {'path': str(path)})
    assert c._shape_error and path.read_bytes() == original
    assert set(c._shape_contacts) == {2, 3} and not h.session.stop.is_set()
    new = h.directory / 'task2.json'
    c._command('save_shape_task', {'path': str(new)})
    assert c._shape_error is None
    assert set(json.loads(new.read_text())['groups']) == {'2', '3'}


def test_saved_full_task_can_optimize_offline_without_installation_or_sensors(workflow):
    h, c = workflow, workflow.controller
    fill_four(h)
    task_path = c._shape_task_path
    old_binding = c._shape_binding()
    c._shape_invalidate()
    c._session, c._profile = None, None
    c._command('load_shape_task', {'path': task_path})
    assert c._shape_error is None
    assert c._session is None and c._profile is None
    state = c._shape_state(h.clock())
    assert state['task_loaded'] and state['can_optimize'] and not state['can_capture']
    assert not state['can_apply'] and state['can_save_task']
    for _ in range(3):
        c._poll_shape()
        c._shape_publish()
    assert len(c._shape_contacts) == 4
    c._command('optimize_shape', {})
    assert c._shape_error is None and c._shape_job is not None
    assert c._shape_job.contacts['binding'] == list(old_binding)
    report = candidate(h)
    c._shape_candidate = None
    c._shape_job.result, c._shape_job.running = {'report': report, 'error': None}, False
    c._poll_shape()
    assert c._shape_candidate and c._shape_state(h.clock())['can_export']
    assert c._profile is None and c._session is None
    c._command('save_shape_profile', {'path': str(h.directory / 'offline_candidate.json')})
    assert c._shape_error is None and (h.directory / 'offline_candidate.json').is_file()


def test_partial_task_import_is_inspectable_but_not_optimizable_or_appendable(workflow):
    h, c = workflow, workflow.controller
    complete(h)
    path = c._shape_task_path
    c._shape_invalidate()
    c._session, c._profile = None, None
    c._command('load_shape_task', {'path': path})
    state = c._shape_state(h.clock())
    assert state['task_loaded'] and state['can_save_task']
    assert not state['can_capture'] and not state['can_optimize']
    c._command('optimize_shape', {})
    assert c._shape_error and c._shape_job is None
    assert set(c._shape_contacts) == {2}


def test_live_import_and_invalid_offline_import_preserve_current_evidence(workflow):
    h, c = workflow, workflow.controller
    complete(h)
    original = deepcopy(c._shape_contacts)
    path = c._shape_task_path
    c._command('load_shape_task', {'path': path})
    assert '未连接' in c._shape_error and c._shape_contacts == original
    c._session = None
    bad = json.loads(Path(path).read_text())
    bad['mano_model']['sha256'] = 'd' * 64
    invalid = h.directory / 'invalid.json'
    invalid.write_text(json.dumps(bad))
    c._command('load_shape_task', {'path': str(invalid)})
    assert 'MANO' in c._shape_error and c._shape_contacts == original
    assert not c._shape_task_loaded


def test_optimization_space_failure_offers_retry_without_discarding_groups(workflow, monkeypatch):
    h, c = workflow, workflow.controller
    fill_four(h)
    original = deepcopy(c._shape_contacts)
    real = calibration_artifacts.preflight
    def fail(*args, **kwargs):
        raise OSError('fixture real disk full')
    monkeypatch.setattr(calibration_artifacts, 'preflight', fail)
    c._command('optimize_shape', {})
    assert c._shape_job is None and c._shape_contacts == original
    assert c._shape_state(h.clock())['can_retry_save'] and c._shape_storage_error
    monkeypatch.setattr(calibration_artifacts, 'preflight', real)
    target = h.directory / 'new-filesystem'
    c._command('retry_calibration_save_to', {'directory': str(target)})
    c._command('optimize_shape', {})
    assert c._shape_error is None and c._shape_job.directory.parent == target


def test_retry_base_profile_without_touch_groups_is_independent_of_log_latch(workflow, monkeypatch):
    from vt_fsglove12 import live_runtime
    h, c = workflow, workflow.controller
    real = live_runtime.write_artifact
    def fail(*args, **kwargs):
        raise OSError('fixture write error')
    monkeypatch.setattr(live_runtime, 'write_artifact', fail)
    c._save_profile(h.session, deepcopy(c._profile), event='fixture')
    assert c._shape_state(h.clock())['can_retry_save']
    monkeypatch.setattr(live_runtime, 'write_artifact', real)
    c._disable_recording(h.session, 'max_session_bytes', 'quota')
    c._command('retry_calibration_save', {})
    assert c._shape_error is None and not h.session.calibration_pending
    assert h.session.profile_saved and h.session.recording_disabled.is_set()
    assert not c._shape_state(h.clock())['can_retry_save']


def test_disconnect_then_retry_command_really_saves_detached_pending_profile(workflow, monkeypatch):
    from vt_fsglove12 import live_runtime
    h, c = workflow, workflow.controller
    def fail(*args, **kwargs):
        raise OSError('fixture filesystem full')
    with monkeypatch.context() as scoped:
        scoped.setattr(live_runtime, 'write_artifact', fail)
        c._save_profile(h.session, deepcopy(c._profile), event='fixture')
    c._disconnect()
    assert c._session is None and c._shape_state(h.clock())['can_retry_save']
    alternate = h.directory / 'after-disconnect'
    c._command('retry_calibration_save_to', {'directory': str(alternate)})
    assert c._shape_error is None and c._session is None
    assert list(alternate.glob('profile.v*.json'))
    assert not c._shape_state(h.clock())['can_retry_save']
    assert c.latest_snapshot()['runtime']['calibration_storage']['pending_count'] == 0
