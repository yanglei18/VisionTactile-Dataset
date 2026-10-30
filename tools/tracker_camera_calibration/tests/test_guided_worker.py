import json
from pathlib import Path
from types import SimpleNamespace

import pytest

from vt_tracker_camera_calib.config_writer import build_config_document, write_config
from vt_tracker_camera_calib.guided_worker import Request, StatusWriter, Worker, source_fingerprint


def request(tmp_path, mode='offline', bags=None):
    config = write_config(tmp_path / 'config.yaml', build_config_document(
        camera_name='d436', tracker_role='torso', square_length_mm=40, marker_length_mm=30))
    return {'schema_version': 1, 'session_id': 'test-session', 'mode': mode,
            'config_path': str(config), 'output_root': str(tmp_path),
            'bag_paths': [str(p) for p in (bags or [])], 'expected_tracker_id': None}


def test_request_rejects_duplicate_bags_and_outside_config(tmp_path):
    bag = tmp_path / 'bag'; bag.mkdir()
    data = request(tmp_path, bags=[bag, bag, bag])
    with pytest.raises(ValueError, match='distinct'):
        Request.from_document(data)
    data.update(bag_paths=[str(bag)], config_path='/tmp/not-frozen.yaml')
    with pytest.raises(ValueError, match='session'):
        Request.from_document(data)


def test_status_atomic_and_bounded(tmp_path):
    writer = StatusWriter(tmp_path / 'status.json', 'test-session')
    writer.update(state='COLLECTING', accepted=3, hint_code='HOLD_STILL')
    assert json.loads((tmp_path / 'status.json').read_text())['accepted'] == 3
    with pytest.raises(ValueError, match='limit'):
        writer.update(detail='x' * (1024 * 1024))
    assert json.loads((tmp_path / 'status.json').read_text())['detail'] == ''


def test_cancel_before_start_never_records(tmp_path):
    req = Request.from_document(request(tmp_path, mode='guided'))
    cancel = tmp_path / 'cancel'; cancel.touch()
    worker = Worker(req, tmp_path / 'status.json', cancel)
    assert worker.run() == 130
    assert json.loads((tmp_path / 'status.json').read_text())['state'] == 'CANCELLED'
    assert not (tmp_path / 'round-1').exists()


def test_duplicate_copied_source_is_not_independent(tmp_path):
    first = tmp_path / 'one'; second = tmp_path / 'two'
    for bag in (first, second):
        bag.mkdir()
        (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths: [data.mcap]\n')
        (bag / 'data.mcap').write_bytes(b'identical sensor messages')
    assert source_fingerprint(first) == source_fingerprint(second)


def test_fingerprint_rejects_escaped_storage_path(tmp_path):
    (tmp_path / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths: [../private.mcap]\n')
    with pytest.raises(ValueError, match='storage'):
        source_fingerprint(tmp_path)


@pytest.mark.parametrize('mode', ['offline', 'guided'])
def test_three_round_worker_exports_eligible_manifest(tmp_path, monkeypatch, mode):
    from dataclasses import replace
    import numpy as np
    import vt_tracker_camera_calib.guided_worker as module
    from test_handeye import synthetic_pairs
    from vt_tracker_camera_calib.bag_reader import BagObservations
    from vt_tracker_camera_calib.model import CameraIntrinsics, TimedTransform, BoardObservation
    pairs, _ = synthetic_pairs(50)
    bags = []
    for index in range(4 if mode == 'guided' else 3):
        bag = tmp_path / f'bag-{index}'
        bag.mkdir()
        (bag / 'metadata.yaml').write_text('rosbag2_bagfile_information:\n  relative_file_paths: [data.mcap]\n')
        (bag / 'data.mcap').write_bytes(bytes([index]))
        bags.append(bag)
    req = Request.from_document(request(tmp_path, mode=mode, bags=bags if mode == 'offline' else []))
    observations = {}
    for index, bag in enumerate(bags):
        offset = (index + 1) * 100_000_000_000
        tracker, board = [], []
        for p in pairs:
            stamp = p.timestamp_ns + offset
            tracker.extend(TimedTransform(stamp + delta, p.world_from_tracker, stamp + delta)
                for delta in range(-150_000_000, 151_000_000, 25_000_000))
            board.append(BoardObservation(stamp, p.camera_from_board, .2, 35, stamp, stamp))
        observations[bag] = BagObservations(CameraIntrinsics(2, 2, np.array([[100,0,1],[0,100,1],[0,0,1]]), np.zeros(5)),
            tuple(tracker), tuple(board), 'a' * 64, 50, 50, 0)
    monkeypatch.setattr(module, 'read_calibration_bag', lambda bag, config, **kw: observations[bag])
    worker = Worker(req, tmp_path / 'status.json', tmp_path / 'cancel')
    if mode == 'guided':
        monkeypatch.setattr(worker, '_collect', lambda n: bags[n - 1])
        solve = worker._solve_round
        def fail_second(n, bag):
            if n == 2:
                raise RuntimeError('temporary round 2 input error')
            return solve(n, bag)
        monkeypatch.setattr(worker, '_solve_round', fail_second)
        assert worker.run() == 1
        progress = json.loads((tmp_path / 'round-progress.json').read_text())
        assert [r['round'] for r in progress['rounds']] == [1]
        saved = Path(progress['rounds'][0]['extrinsics_path']).read_bytes()
        calls = []
        worker = Worker(req, tmp_path / 'status.json', tmp_path / 'cancel')
        monkeypatch.setattr(worker, '_collect', lambda n: calls.append(n) or bags[n - 1])
    assert worker.run() == 0
    if mode == 'guided':
        assert calls == [2, 3]
        assert Path(progress['rounds'][0]['extrinsics_path']).read_bytes() == saved
        assert json.loads((tmp_path / 'status.json').read_text())['metrics']['completed_rounds'] == [1, 2, 3]
    status = json.loads((tmp_path / 'status.json').read_text())
    assert status['state'] == 'READY'
    manifest = json.loads(Path(status['result']['manifest_path']).read_text())
    assert manifest['activation_eligible'] is True
    assert len(manifest['rounds']) == 3
    assert all(r['validation']['validation_count'] == 10 for r in manifest['rounds'])
    assert all(r['validation']['training_count'] == 40 for r in manifest['rounds'])
    assert json.loads(Path(manifest['repeatability_path']).read_text())['status'] == 'PASS'
    if mode == 'guided':
        # Re-comparing saved rounds must not collect or overwrite old reports,
        # including repeated failures. Failure must not become READY.
        original_report = (tmp_path / 'repeatability.json').read_bytes()
        compare = module.compare_extrinsics
        monkeypatch.setattr(module, 'compare_extrinsics', lambda paths: replace(
            compare(paths), status='FAIL', maximum_translation_m=.005327))
        for attempt in (2, 3):
            retry = Worker(req, tmp_path / 'status.json', tmp_path / 'cancel')
            monkeypatch.setattr(retry, '_collect', lambda n: pytest.fail('must only recompare'))
            assert retry.run() == 1
            state = json.loads((tmp_path / 'status.json').read_text())
            assert state['state'] == 'FAILED'
            assert state['hint_code'] == 'REPEATABILITY_FAILED'
            assert '5.327 mm' in state['detail']
            assert Path(state['result']['repeatability_path']).name == f'repeatability-attempt-{attempt}.json'
            assert (tmp_path / 'repeatability.json').read_bytes() == original_report
        monkeypatch.setattr(module, 'compare_extrinsics', compare)

        # The manager retires only round 1. A failed recapture must still leave
        # rounds 2 and 3 reusable. New round 1 may be chronologically LAST.
        retained = [r for r in retry.round_store.load() if r['round'] in (2, 3)]
        retry.round_store.save(retained)
        untouched = {Path(r[key]): Path(r[key]).read_bytes()
                     for r in manifest['rounds']
                     for key in ('extrinsics_path', 'report_path', 'validation_path')}
        failed = Worker(req, tmp_path / 'status.json', tmp_path / 'cancel')
        def fail_recapture(n):
            assert n == 1
            raise RuntimeError('temporary recapture input failure')
        monkeypatch.setattr(failed, '_collect', fail_recapture)
        assert failed.run() == 1
        assert [r['round'] for r in failed.round_store.load()] == [2, 3]
        retry = Worker(req, tmp_path / 'status.json', tmp_path / 'cancel')
        calls = []
        monkeypatch.setattr(retry, '_collect', lambda n: calls.append(n) or bags[3])
        assert retry.run() == 0
        assert calls == [1]
        updated = json.loads((tmp_path / 'manifest.json').read_text())
        assert [r['round'] for r in updated['rounds']] == [1, 2, 3]
        assert updated['rounds'][1:] == retained
        assert 'round-1-result-attempt-2' in updated['rounds'][0]['extrinsics_path']
        assert updated['rounds'][0]['first_stamp_ns'] > updated['rounds'][2]['last_stamp_ns']
        assert all(path.read_bytes() == data for path, data in untouched.items())
        assert (tmp_path / 'repeatability.json').read_bytes() == original_report
        assert json.loads(Path(updated['repeatability_path']).read_text())['status'] == 'PASS'


def test_failed_round_gets_new_attempt_path(tmp_path):
    req = Request.from_document(request(tmp_path, mode='guided'))
    worker = Worker(req, tmp_path / 'status.json', tmp_path / 'cancel')
    original = tmp_path / 'round-2'
    original.mkdir()
    (original / 'evidence').write_text('keep')
    assert worker._unused_path('round-2') == tmp_path / 'round-2-attempt-2'
    assert (original / 'evidence').read_text() == 'keep'


def test_single_offline_bag_never_becomes_activation_eligible(tmp_path, monkeypatch):
    import vt_tracker_camera_calib.guided_worker as module
    bag = tmp_path / 'bag'; bag.mkdir()
    req = Request.from_document(request(tmp_path, bags=[bag]))
    worker = Worker(req, tmp_path / 'status.json', tmp_path / 'cancel')
    monkeypatch.setattr(worker, '_solve_round', lambda number, bag: dict(
        extrinsics_path=str(tmp_path / 'extrinsics.yaml'), report_path=str(tmp_path / 'report.json'),
        source_sha256='a' * 64, first_stamp_ns=1, last_stamp_ns=2))
    assert worker.run() == 0
    status = json.loads((tmp_path / 'status.json').read_text())
    assert status['hint_code'] == 'CANDIDATE'
    manifest = json.loads(Path(status['result']['manifest_path']).read_text())
    assert manifest['activation_eligible'] is False
    assert manifest['repeatability_path'] is None


def test_bag_scan_observes_cancellation_before_decoding(tmp_path, monkeypatch):
    import vt_tracker_camera_calib.bag_reader as reader
    from vt_tracker_camera_calib.config import load_config
    document = request(tmp_path, mode='guided')
    monkeypatch.setattr(reader, '_open_reader', lambda *args: SimpleNamespace(has_next=lambda: True))
    monkeypatch.setattr(reader, '_message_types', lambda source: {})
    monkeypatch.setattr(reader, '_require_topics', lambda *args: None)
    def cancelled():
        raise InterruptedError('cancelled in scan')
    with pytest.raises(InterruptedError, match='scan'):
        reader.read_calibration_bag(tmp_path, load_config(document['config_path']), check_cancel=cancelled)


def test_cancel_waits_for_owned_recorder_before_terminal_state(tmp_path, monkeypatch):
    import vt_tracker_camera_calib.guided_worker as module
    events = []
    cancel_path = tmp_path / 'cancel'
    class Recorder:
        def __init__(self):
            import os
            self.pid = os.getpid()
            self.stopping = False
            self.remaining = 2
        def start(self, *args):
            from vt_tracker_camera_calib.process_lease import recorder_lease
            module.atomic_document(tmp_path / 'recorder-lease.json',
                recorder_lease(self.pid, 'test-session', owner_pid=self.pid))
            events.append('recording')
        @property
        def running(self):
            if not self.stopping:
                return True
            self.remaining -= 1
            if self.remaining == 0:
                events.append('closed')
            return self.remaining > 0
        def stop(self):
            events.append('stop-request')
            self.stopping = True
        def poll(self):
            return 0
    class Monitor:
        def __init__(self, *args):
            pass
        def start(self):
            events.append('monitor-start')
        def stop(self):
            events.append('monitor-stop')
        def request_detection(self):
            lease = json.loads((tmp_path / 'recorder-lease.json').read_text())
            assert lease['start_token'] and lease['boot_id']
            assert lease['session_id'] == 'test-session'
            cancel_path.touch()
        def snapshot(self):
            return dict(ready=False, accepted=0, target=50, hint_code='HOLD_STILL',
                        detail='', preview=None, metrics={})
    monkeypatch.setattr(module, 'ManagedJob', Recorder)
    monkeypatch.setattr(module, 'GuidedMonitor', Monitor)
    worker = Worker(Request.from_document(request(tmp_path, mode='guided')),
                    tmp_path / 'status.json', cancel_path)
    assert worker.run() == 130
    assert events == ['monitor-start', 'recording', 'stop-request', 'closed', 'monitor-stop']
    assert not (tmp_path / 'recorder-lease.json').exists()
    assert json.loads((tmp_path / 'status.json').read_text())['state'] == 'CANCELLED'
