"""Local-log fault injection; fake sources only, no hardware or ROS process."""
from copy import deepcopy
import io
import json

import pytest

from vt_fsglove12 import live_runtime
from vt_fsglove12.live_recording import AsyncResultRecorder
from vt_fsglove12.recording_policy import SessionStorageBudget
from test_live_runtime import harness, wait_for
from test_live_runtime_high_rate import setup_direct, feed


class RawLog:
    def __init__(self, fail=None):
        self.fail = fail
        self.rows = []
        self.opens = self.writes = self.closes = 0

    def open(self):
        self.opens += 1
        if self.fail == 'open':
            raise OSError('injected raw open failure')

    def write(self, rows):
        self.writes += 1
        if self.fail == 'write':
            raise OSError('injected raw write failure')
        self.rows.extend(rows)

    def close(self):
        self.closes += 1
        if self.fail == 'close':
            raise OSError('injected raw close failure')


class Exporter:
    def __init__(self):
        self.documents, self.errors = [], []

    def publish(self, document):
        self.documents.append(deepcopy(document))

    def record_error(self, exc):
        self.errors.append(str(exc))

    def snapshot(self):
        return dict(enabled=True, sent=len(self.documents), dropped=len(self.errors),
                    last_error=self.errors[-1] if self.errors else None)


def connect_recording_fixture(h, recorder):
    h.controller.config['local_recording'] = {'min_free_bytes': 0}
    h.controller._recorder_factory = lambda _path: recorder
    exporter = h.controller._pose_exporter = Exporter()
    source = h.connect()
    return h.controller._session, source, exporter


def assert_live_after_log_fault(h, session, source):
    snapshot = h.controller.latest_snapshot()
    assert snapshot['runtime']['connected'] and snapshot['runtime']['error'] is None
    assert not source.closed and source.opened and session.error is None
    assert not session.stop.is_set()
    assert snapshot['runtime']['local_recording']['enabled'] is False
    assert snapshot['runtime']['local_recording']['warning']
    assert session.recording_disabled.is_set()
    return snapshot


def disconnect_and_summary(h, session):
    assert h.controller.submit('disconnect')
    path = session.directory / 'summary.json'
    wait_for(path.exists)
    summary = json.loads(path.read_text())
    assert not summary['recording_complete'] and not summary['raw_recording_complete']
    assert summary['local_recording']['warning'] and summary['errors']
    return summary


@pytest.mark.parametrize('failure', ['open', 'write'])
def test_raw_open_and_write_errors_preserve_current_and_future_fusion(harness, failure):
    h, raw = harness, RawLog(failure)
    session, source, exporter = connect_recording_fixture(h, raw)
    for expected in range(1, 4):
        h.pump(nodes=(3, 12))
        snapshot = assert_live_after_log_fault(h, session, source)
        assert session.fusion.counts['12'] == session.fusion.counts['3'] == expected
        assert snapshot['orientation']['nodes']['12']['sample_index'] == expected
    assert raw.writes == (1 if failure == 'write' else 0)  # No repeated disk retry.
    assert not raw.rows and session.received == 6 and session.recorded == 0
    assert len(exporter.documents) >= 3 and not exporter.errors
    assert f'raw_recording_{"open_failed" if failure == "open" else "failed"}' in snapshot['runtime']['local_recording']['warning']
    summary = disconnect_and_summary(h, session)
    assert summary['received_samples'] == 6 and summary['recorded_samples'] == 0
    assert summary['samples_per_node'] == {'3': 3, '12': 3}


@pytest.mark.parametrize('close_fails', [False, True])
def test_storage_latch_stops_raw_writes_but_not_source_or_fusion_even_if_raw_close_fails(harness, close_fails):
    h, raw = harness, RawLog('close' if close_fails else None)
    session, source, exporter = connect_recording_fixture(h, raw)
    h.pump(nodes=(12,))
    assert session.recorded == 1
    # Config/metadata already exceed this test budget. Check the real guard,
    # not a mocked connected flag; the reader sees only its latched event.
    session.storage_budget = SessionStorageBudget(session.directory,
        {'max_session_bytes': 1, 'min_free_bytes': 0})
    h.pump(nodes=(12,))
    first = assert_live_after_log_fault(h, session, source)
    writes_after_latch, recorded_after_latch = raw.writes, session.recorded
    warning = first['runtime']['local_recording']['warning']
    assert 'max_session_bytes' in warning
    for _ in range(3):
        h.pump(nodes=(12,))
        assert assert_live_after_log_fault(h, session, source)['runtime']['local_recording']['warning'] == warning
    assert raw.writes == writes_after_latch and session.recorded == recorded_after_latch
    assert raw.closes >= 1 and session.fusion.counts['12'] == 5
    assert session.received == 5 and session.node_counts['12'] == 5
    assert len(exporter.documents) >= 5 and not exporter.errors
    assert not session.storage_budget.snapshot()['active']
    disconnect_and_summary(h, session)


def test_raw_close_error_on_normal_disconnect_is_not_a_fusion_error(harness):
    h, raw = harness, RawLog('close')
    session, source, exporter = connect_recording_fixture(h, raw)
    h.pump(nodes=(12,))
    h.pump(nodes=(12,))
    assert h.controller.latest_snapshot()['runtime']['connected']
    assert h.controller.latest_snapshot()['runtime']['error'] is None
    assert session.fusion.counts['12'] == 2 and session.recorded == 2
    summary = disconnect_and_summary(h, session)
    assert session.error is None and source.closed
    assert 'raw_recording_close_failed' in summary['local_recording']['warning']
    assert summary['samples_per_node']['12'] == 2 and not exporter.errors


@pytest.mark.parametrize('failure', ['reported_error', 'submit_error'])
def test_async_writer_failures_do_not_skip_tick_or_results_export(tmp_path, monkeypatch, failure):
    controller, session, backend, clock = setup_direct(tmp_path)
    exporter = controller._pose_exporter = Exporter()
    writer = session.result_recorder = AsyncResultRecorder(
        dict(orientations=io.StringIO(), poses=io.StringIO()))
    if failure == 'reported_error':
        writer._fail(OSError('injected asynchronous disk failure'))
    else:
        def reject(*_args, **_kwargs):
            raise OSError('injected full result queue')
        monkeypatch.setattr(writer, 'submit', reject)
    try:
        for _ in range(4):
            feed(session, clock, 12)
            assert controller._tick(session)
            assert controller.latest_snapshot()['runtime']['connected']
            assert controller.latest_snapshot()['runtime']['error'] is None
            clock.advance(1. / 120.)
        assert len(backend.calls) == len(exporter.documents) == 4
        assert session.frames == session.fresh_input_results == 4 and not exporter.errors
        status = controller.latest_snapshot()['runtime']['local_recording']
        assert not status['enabled'] and 'result_recording_failed' in status['warning']
        assert writer.snapshot()['abort_requested']
    finally:
        assert writer.close(2.)
    assert not writer.snapshot()['recording_complete']


def test_profile_save_error_updates_memory_and_versions_without_claiming_files(tmp_path, monkeypatch):
    controller, session, _backend, clock = setup_direct(tmp_path)
    original_profile = dict(status='fitted', warnings=[], rejected_nodes={}, nodes={'12': {'epoch': 0}})
    saved = deepcopy(original_profile)
    attempts = []

    def fail(path, value):
        attempts.append(path)
        raise OSError('injected profile save failure')

    monkeypatch.setattr(live_runtime, 'write_artifact', fail)
    controller._save_profile(session, original_profile, event='calibrated')
    assert controller._profile['profile_version'] == session.profile_version == 1
    assert not session.profile_saved and session.profiles == []
    assert not (tmp_path / 'profile.v001.json').exists() and not (tmp_path / 'profile.json').exists()
    assert controller.latest_snapshot()['runtime']['error'] is None
    status = controller.latest_snapshot()['runtime']['calibration_storage']
    assert status['status'] == 'save_failed' and status['pending_count'] == 2
    assert 'injected profile save failure' in status['warning']
    assert controller.latest_snapshot()['runtime']['local_recording']['enabled']
    assert not session.recording_disabled.is_set()
    controller._save_profile(session, {**original_profile, 'functional_enabled': True}, event='functional_updated')
    assert controller._profile['profile_version'] == session.profile_version == 2
    assert controller._profile['functional_enabled'] is True and not session.profile_saved
    assert session.profiles == [] and len(attempts) == 3 and original_profile == saved
    feed(session, clock, 12)
    assert controller._tick(session)
    assert controller.latest_snapshot()['pose']['pose_available']
    assert controller.latest_snapshot()['runtime']['connected']
    assert controller.latest_snapshot()['runtime']['local_recording']['profile_saved'] is False
    assert controller.latest_snapshot()['runtime']['calibration_storage']['pending_count'] == 3


@pytest.mark.parametrize('calibrated', [True, False])
def test_recording_rate_selection_never_throttles_tick_or_export(tmp_path, calibrated):
    controller, session, backend, clock = setup_direct(tmp_path)
    exporter = controller._pose_exporter = Exporter()
    if not calibrated:
        controller._profile = None
    start = clock()
    for index in range(240):
        clock.value = start + index / 120.
        feed(session, clock, 12)
        assert controller._tick(session)
    assert session.frames == len(exporter.documents) == 240 and not exporter.errors
    assert session.fusion.counts['12'] == 240
    assert len(backend.calls) == (240 if calibrated else 0)
    rows = session.streams['poses'].getvalue().splitlines()
    assert len(rows) == (240 if calibrated else 2)
    status = controller.latest_snapshot()['runtime']['local_recording']['policy']
    assert status['selected_solved'] == (240 if calibrated else 0)
    assert status['selected_diagnostic'] == (0 if calibrated else 2)
    assert status['suppressed_diagnostic'] == (0 if calibrated else 238)
    assert controller.latest_snapshot()['runtime']['connected']
    assert controller.latest_snapshot()['runtime']['error'] is None
