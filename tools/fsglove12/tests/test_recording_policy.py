from copy import deepcopy
from pathlib import Path
from threading import Event, Thread
from types import SimpleNamespace

import pytest

from vt_fsglove12.recording_policy import DEFAULTS, ResultRecordingPolicy, SessionStorageBudget


def sample(index, *, epoch=0, valid=True):
    frame = dict(session_id='test', nodes={'12': dict(sample_index=index, epoch=epoch)})
    pose = dict(pose_available=valid, solver_success=valid, source_fresh=True,
                reason=None if valid else 'calibration_required', profile_version=1)
    return frame, pose


def test_healthy_120hz_is_not_throttled_or_modified():
    policy = ResultRecordingPolicy({'node_to_slot': {'12': 0}})
    for index in range(1200):
        frame, pose = sample(index + 1)
        before = deepcopy((frame, pose))
        assert policy.should_record(frame, pose, now=index / 120.)
        assert (frame, pose) == before
    state = policy.snapshot()
    assert state['selected_solved'] == state['selected_total'] == 1200
    assert state['selected_diagnostic'] == state['suppressed_diagnostic'] == 0
    assert 'not_input_or_ros_throttle' in state['semantics']


def test_invalid_new_inputs_are_one_hz_not_120hz():
    policy = ResultRecordingPolicy({})
    selected = []
    for index in range(1200):
        if policy.should_record(*sample(index + 1, valid=False), now=index / 120.):
            selected.append(index)
    assert selected == list(range(0, 1200, 120))
    state = policy.snapshot()
    assert state['selected_diagnostic'] == 10 and state['suppressed_diagnostic'] == 1190


@pytest.mark.parametrize('transition', ['reason', 'available', 'profile', 'clear', 'health', 'epoch', 'phase'])
def test_real_state_changes_can_be_immediate(transition):
    policy = ResultRecordingPolicy({})
    frame, pose = sample(1, valid=False)
    context = dict(clear_generation=0, active=dict(kind='functional', phase='neutral'))
    assert policy.should_record(frame, pose, now=0., context=context)
    if transition == 'reason':
        pose['reason'] = 'missing_or_stale_root'
    elif transition == 'available':
        pose['pose_available'] = True
    elif transition == 'profile':
        pose['profile_version'] += 1
    elif transition == 'clear':
        context['clear_generation'] += 1
    elif transition == 'health':
        frame['input_health'] = dict(enabled=True, faulted_nodes=['12'])
    elif transition == 'epoch':
        frame['nodes']['12']['epoch'] += 1
    else:
        context['active']['phase'] = 'moving'
    assert policy.should_record(frame, pose, now=.1, context=context)
    assert policy.snapshot()['selected_diagnostic'] == 2


def test_flapping_diagnostic_state_has_rolling_one_second_hard_cap():
    policy = ResultRecordingPolicy({})
    selected = []
    for index in range(600):
        frame, pose = sample(index + 1, valid=False)
        pose['reason'] = f'flap-{index % 2}'
        now = index / 120.
        if policy.should_record(frame, pose, now=now):
            selected.append(now)
    assert len(selected) <= 25
    assert all(sum(start <= stamp < start + 1. - 1e-9 for stamp in selected) <= 5 for start in selected)
    assert policy.snapshot()['suppressed_diagnostic_hard_limit'] > 0
    # Exhausting diagnostics never eats a genuinely new healthy solved frame.
    assert policy.should_record(*sample(1001), now=5.)
    assert policy.snapshot()['selected_solved'] == 1


def test_input_receive_stage_changes_are_events_but_increasing_counters_are_not():
    policy = ResultRecordingPolicy({})
    frame, pose = sample(1, valid=False)
    receive = frame['input_diagnostics'] = dict(source_kind='raw_serial', bytes_received=0,
                                              frames_decoded=0, samples_decoded=0)
    assert policy.should_record(frame, pose, now=0.)
    receive['bytes_received'] = 300
    assert policy.should_record(frame, pose, now=.1)
    receive['bytes_received'] = 600
    assert not policy.should_record(frame, pose, now=.2)
    receive.update(frames_decoded=2, samples_decoded=12)
    assert policy.should_record(frame, pose, now=.3)
    receive.update(bytes_received=1200, frames_decoded=4, samples_decoded=24)
    assert not policy.should_record(frame, pose, now=.4)
    assert policy.snapshot()['selected_diagnostic'] == 3


def test_capture_force_keeps_new_inputs_but_not_countdowns_or_duplicate_ticks():
    policy = ResultRecordingPolicy({})
    frame, pose = sample(1, valid=False)
    for index in range(240):
        frame['nodes']['12']['sample_index'] = index + 1
        assert policy.should_record(frame, pose, now=index / 120., force=True,
            context=dict(active=dict(kind='mounting', step='neutral', remaining_s=2. - index / 120.)))
    assert policy.snapshot()['selected_capture'] == 240
    for index in range(240):
        policy.should_record(frame, pose, now=2. + index / 120., force=True,
            context=dict(active=dict(kind='mounting', step='neutral', remaining_s=100. - index)))
    state = policy.snapshot()
    assert state['selected_capture'] == 240
    assert state['selected_diagnostic'] == 2  # No per-frame countdown transition.


def test_diagnostics_can_be_disabled_without_affecting_solved_or_capture():
    policy = ResultRecordingPolicy({'local_recording': {'diagnostics_enabled': False}})
    assert not policy.should_record(*sample(1, valid=False), now=0.)
    assert policy.should_record(*sample(2), now=.01)
    assert policy.should_record(*sample(3, valid=False), now=.02, force=True)
    state = policy.snapshot()
    assert state['selected_solved'] == state['selected_capture'] == 1
    assert state['suppressed_diagnostic_disabled'] == 1


def test_health_faults_and_repeated_identical_sources_cannot_claim_fullrate_results():
    policy = ResultRecordingPolicy({})
    for index in range(120):
        frame, pose = sample(index + 1)
        frame['input_health'] = dict(enabled=True, nodes={'12': dict(state='faulted', healthy=False)})
        policy.should_record(frame, pose, now=index / 120.)
    assert policy.snapshot()['selected_solved'] == 0
    assert policy.snapshot()['selected_diagnostic'] == 1
    frame, pose = sample(120)
    for index in range(120):
        frame['frame_index'] = index  # Scheduler identities are not new IMU data.
        frame['timestamp_s'] = 1. + index / 120.
        policy.should_record(frame, pose, now=frame['timestamp_s'])
    assert policy.snapshot()['selected_solved'] == 0
    assert policy.snapshot()['selected_diagnostic'] == 2


def test_missing_and_returning_same_node_is_not_new_input_and_backwards_clock_recovers():
    policy = ResultRecordingPolicy({})
    frame, pose = sample(1)
    assert policy.should_record(frame, pose, now=10.)
    assert policy.should_record({'nodes': {}}, {**pose, 'pose_available': False}, now=10.1)
    policy.should_record(frame, pose, now=10.2)
    assert policy.snapshot()['selected_solved'] == 1
    assert policy.should_record(*sample(1, valid=False), now=2.)
    assert policy.snapshot()['clock_resets'] == 1
    assert policy.should_record(*sample(2), now=2.01)


def test_old_source_snapshots_do_not_become_new_again_but_new_epoch_and_session_do():
    policy = ResultRecordingPolicy({})
    for index, source in enumerate((1, 2, 1, 2, 3)):
        policy.should_record(*sample(source), now=index / 120.)
    assert policy.snapshot()['selected_solved'] == 3
    assert policy.should_record(*sample(1, epoch=1), now=.1)
    policy.should_record(*sample(100, epoch=0), now=.11)
    assert policy.snapshot()['selected_solved'] == 4
    frame, pose = sample(1)
    frame['session_id'] = 'new-session'
    assert policy.should_record(frame, pose, now=.12)
    assert policy.snapshot()['selected_solved'] == 5


@pytest.mark.parametrize('settings', [None, [], {'unknown': 1}, {'diagnostics_enabled': 0},
    {'diagnostic_rate_hz': float('nan')}, {'diagnostic_rate_hz': 6},
    {'diagnostic_max_per_second': 1.5}, {'max_session_bytes': 0},
    {'min_free_bytes': -1}, {'storage_check_interval_s': .1}])
def test_invalid_settings_rejected(settings):
    if settings is None:
        assert ResultRecordingPolicy({'local_recording': settings}).snapshot()['settings'] == DEFAULTS
    else:
        with pytest.raises(ValueError):
            ResultRecordingPolicy({'local_recording': settings})


def budget(tmp_path, *, free=10_000, **settings):
    return SessionStorageBudget(tmp_path, {'max_session_bytes': 1000, 'min_free_bytes': 100, **settings},
                                disk_usage=lambda path: SimpleNamespace(free=free))


def test_storage_stats_only_current_known_files_and_profiles_are_cached(tmp_path, monkeypatch):
    files = {'raw.csv': 100, 'orientations.jsonl': 200, 'poses.jsonl': 300,
             'events.jsonl': 30, 'profile.v001.json': 40, 'profile.json': 50, 'config.json': 10}
    for name, size in files.items():
        (tmp_path / name).write_bytes(b' ' * size)
    history = tmp_path / 'history'
    history.mkdir()
    (history / 'raw.csv').write_bytes(b'x' * 5000)
    (tmp_path / 'unrelated.bin').write_bytes(b'x' * 5000)
    calls = []
    guard = SessionStorageBudget(tmp_path, {'max_session_bytes': 1000, 'min_free_bytes': 100},
        disk_usage=lambda path: calls.append(path) or SimpleNamespace(free=10_000))
    monkeypatch.setattr(Path, 'read_bytes', lambda *_: pytest.fail('budget must not read file contents'))
    assert guard.check(now=0.)
    first = guard.snapshot()
    assert first['session_bytes'] == sum(files.values()) and first['checked_files'] == sorted(files)
    (tmp_path / 'raw.csv').write_bytes(b' ' * 101)
    assert guard.check(now=.5) and guard.snapshot()['session_bytes'] == sum(files.values())
    assert len(calls) == 1
    assert guard.check(now=1.) and guard.snapshot()['session_bytes'] == sum(files.values()) + 1
    first['checked_files'].clear()
    assert guard.snapshot()['checked_files']


@pytest.mark.parametrize('kind', ['size', 'free', 'io'])
def test_storage_fault_latches_without_deleting_or_reenabling(tmp_path, kind):
    raw = tmp_path / 'raw.csv'
    raw.write_bytes(b'x' * (1000 if kind == 'size' else 1))
    guard = budget(tmp_path, free=99 if kind == 'free' else 10000)
    if kind == 'io':
        def fail(_):
            raise OSError('test filesystem failure')
        guard._disk_usage = fail
    assert not guard.check(now=0.)
    snapshot = guard.snapshot()
    expected = {'size': 'max_session_bytes', 'free': 'min_free_bytes', 'io': 'storage_check_failed'}[kind]
    assert snapshot['disabled_reason'] == expected and snapshot['disabled_detail']
    assert snapshot['latched'] and not snapshot['active'] and not snapshot['recording_enabled']
    assert raw.exists()
    guard._disk_usage = lambda path: SimpleNamespace(free=10000)
    raw.write_bytes(b'')  # External state changes do not automatically re-enable.
    assert not guard.check(now=5.) and guard.snapshot()['checks'] == 1
    guard.disable('later_error', 'must not overwrite first cause')
    assert guard.snapshot()['disabled_reason'] == expected


def test_reader_disable_does_not_wait_for_disk_and_check_cannot_reenable(tmp_path):
    entered, release, disabled = Event(), Event(), Event()
    def disk(_):
        entered.set()
        assert release.wait(2.)
        return SimpleNamespace(free=10000)
    guard = SessionStorageBudget(tmp_path, {'min_free_bytes': 100}, disk_usage=disk)
    outcome = []
    checker = Thread(target=lambda: outcome.append(guard.check(now=0.)))
    checker.start()
    assert entered.wait(1.)
    reader = Thread(target=lambda: (guard.disable('raw_write_failed', 'disk full'), disabled.set()))
    reader.start()
    try:
        assert disabled.wait(.5), 'reader disable was blocked behind disk I/O'
    finally:
        release.set()
        checker.join(2.)
        reader.join(2.)
    assert outcome == [False] and guard.snapshot()['disabled_reason'] == 'raw_write_failed'
    assert not guard.snapshot()['check_in_progress']


def test_storage_backward_clock_does_not_leave_stale_budget_cache(tmp_path):
    guard = budget(tmp_path)
    assert guard.check(now=20.)
    assert guard.check(now=2.)
    assert guard.snapshot()['checks'] == 2
