"""Lifecycle and durable activation tests; no ROS, hardware or child process."""
from copy import deepcopy
import json
from pathlib import Path
from types import SimpleNamespace

import pytest
import yaml

from vt_capture_manager.calibration.config import build_config
from vt_capture_manager.calibration.service import CalibrationService
from vt_capture_manager.calibration.storage import read_document, atomic_json


TRACKER_ID = 'a' * 64


def config():
    return build_config(dict(name='cam', model='D405', serial='123'), 'torso',
                        dict(squares_x=9, squares_y=6, square_length_mm=40.,
                             marker_length_mm=30., dictionary='DICT_5X5_1000'))


def extrinsic():
    return dict(schema_version=1, status='VALID', camera=dict(name='cam', model='D405', serial='123',
                frame_id='cam_color_optical_frame'), tracker=dict(role='torso', tracker_id=TRACKER_ID,
                frame_id='vive_tracker_torso'), frames=dict(world='vive_map', board='charuco_board'),
                transform=dict(semantics='parent_from_child', parent_frame='vive_tracker_torso',
                child_frame='cam_color_optical_frame', translation_m=[.1, .0, .0], quaternion_xyzw=[0., 0., 0., 1.]),
                quality=dict(grade='TARGET', sample_count=40, validation_translation_rms_m=.001,
                             validation_rotation_rms_deg=.1))


class Process:
    def __init__(self):
        self.returncode = None
        self.signals = []
        self.pid = 12345
    def poll(self):
        return self.returncode
    def send_signal(self, value):
        self.signals.append(value)
    def wait(self, timeout=None):
        self.returncode = 0
        return 0


class Factory:
    def __init__(self):
        self.calls = []
        self.process = Process()
    def __call__(self, args, **kwargs):
        self.calls.append((args, kwargs))
        return self.process


def service(tmp_path, **kwargs):
    factory = Factory()
    options = dict(can_start=lambda: (True, ''), prepare_config=lambda params: dict(config=config(), expected_tracker_id=TRACKER_ID),
                   identity=lambda document: (True, ''), apply=lambda doc, manifest: None,
                   publish=lambda doc, manifest, valid, reason: None, popen=factory)
    options.update(kwargs)
    svc = CalibrationService(tmp_path / 'calibration', 'manager-1', '/usr/bin/python3', **options)
    return svc, factory


def start(svc, **kwargs):
    params = dict(expected_manager_instance_id='manager-1', mode='guided', camera_name='cam', tracker_role='torso',
                  board=dict(squares_x=9, squares_y=6, square_length_mm=40., marker_length_mm=30., dictionary='DICT_5X5_1000'))
    params.update(kwargs)
    return svc.command('calibration.start', params, 'start-1')


def ready(svc, factory, *, eligible=True):
    request = read_document(Path(factory.calls[0][0][factory.calls[0][0].index('--request') + 1]))
    root = Path(request['output_root'])
    root.mkdir(parents=True, exist_ok=True)
    rounds = []
    for index in range(3 if eligible else 1):
        out = root / f'round-{index + 1}'
        out.mkdir()
        (out / 'extrinsics.yaml').write_text(yaml.safe_dump(extrinsic()))
        atomic_json(out / 'report.json', {'quality': 'TARGET'})
        validation = dict(status='PASS', training_count=40, validation_count=10,
                          translation_rms_m=.001, rotation_rms_deg=.1)
        atomic_json(out / 'validation.json', validation)
        rounds.append(dict(round=index + 1, extrinsics_path=str(out / 'extrinsics.yaml'),
                           report_path=str(out / 'report.json'), validation_path=str(out / 'validation.json'),
                           bag_path=str(root / f'bag-{index}'), source_sha256=str(index) * 64,
                           first_stamp_ns=(index + 1) * 1000, last_stamp_ns=(index + 1) * 1000 + 900,
                           tracker_id=TRACKER_ID, validation=validation))
    repeat = root / 'repeatability.json'
    atomic_json(repeat, dict(schema_version=1, status='PASS', run_count=3, metrics=dict(
                maximum_pairwise_translation_m=.001, maximum_pairwise_rotation_deg=.1)))
    manifest = dict(schema_version=1, session_id=request['session_id'], policy_version='guided-v1',
                    activation_eligible=eligible, rounds=rounds, repeatability_path=str(repeat) if eligible else None,
                    selected_extrinsics_path=rounds[0]['extrinsics_path'])
    atomic_json(root / 'manifest.json', manifest)
    result = dict(extrinsics_path=rounds[0]['extrinsics_path'], report_path=rounds[0]['report_path'],
                  repeatability_path=manifest['repeatability_path'], manifest_path=str(root / 'manifest.json'))
    status = dict(schema_version=1, session_id=request['session_id'], state='READY', round=len(rounds), rounds=3,
                  accepted=50, target=50, hint_code='ready', detail='', metrics={}, preview=None, result=result, updated_ns=1)
    status_path = Path(factory.calls[0][0][factory.calls[0][0].index('--status') + 1])
    atomic_json(status_path, status)
    factory.process.returncode = 0
    return request, status_path, root


def test_start_has_identity_binding_private_snapshot_and_owned_worker(tmp_path):
    svc, factory = service(tmp_path)
    assert start(svc)['accepted']
    assert svc.busy
    assert len(factory.calls) == 1
    args, options = factory.calls[0]
    assert args[1:3] == ['-m', 'vt_tracker_camera_calib.guided_worker']
    request = read_document(Path(args[args.index('--request') + 1]))
    assert request['expected_tracker_id'] == TRACKER_ID
    assert read_document(Path(request['config_path']))['camera']['serial'] == '123'
    assert options['start_new_session'] is True
    assert start(svc)['accepted']  # Idempotent retry does not start a second worker.
    assert len(factory.calls) == 1


def test_admission_and_manager_identity_are_checked_before_creating_worker(tmp_path):
    svc, factory = service(tmp_path, can_start=lambda: (False, 'recording'))
    assert start(svc)['code'] == 'calibration_busy'
    assert not factory.calls
    assert start(svc, expected_manager_instance_id='other')['code'] == 'manager_instance_mismatch'


def test_resume_keeps_session_and_archives_failed_attempt(tmp_path):
    svc, factory = service(tmp_path)
    start(svc)
    sid = svc.snapshot()['session']['session_id']
    factory.process.returncode = 1
    svc.tick()
    root = tmp_path / 'calibration' / 'sessions' / sid
    atomic_json(root / 'cancel.json', {'session_id': sid})
    atomic_json(root / 'round-progress.json', {'preserve': True})
    before = (root / 'config.yaml').read_bytes()
    factory.process = Process()
    params = dict(expected_manager_instance_id='manager-1', session_id=sid, assembly_unchanged=True)
    assert svc.command('calibration.resume', params, 'resume-1')['accepted']
    assert svc.snapshot()['session']['session_id'] == sid
    assert len(factory.calls) == 2
    assert (root / 'config.yaml').read_bytes() == before
    assert read_document(root / 'round-progress.json') == {'preserve': True}
    assert not (root / 'cancel.json').exists()
    assert len(list((root / 'attempt-history').glob('*/cancel.json'))) == 1
    assert svc.command('calibration.resume', params, 'resume-1')['accepted']
    assert len(factory.calls) == 2
    svc.close()


def test_resume_first_round_preserves_second_third_and_archives_selection(tmp_path):
    svc, factory = service(tmp_path)
    start(svc)
    sid = svc.snapshot()['session']['session_id']
    factory.process.returncode = 1
    svc.tick()
    root = tmp_path / 'calibration' / 'sessions' / sid
    progress = dict(schema_version=1, session_id=sid, config_sha256='f'*64,
                   rounds=[dict(round=n, evidence=f'original-{n}') for n in (1, 2, 3)])
    atomic_json(root / 'round-progress.json', progress)
    atomic_json(root / 'repeatability.json', dict(status='FAIL', evidence='original comparison'))
    (root / 'round-1-original').write_text('never remove original data')
    params = dict(expected_manager_instance_id='manager-1', session_id=sid,
                  assembly_unchanged=True, retry_rounds=[1])
    factory.process = Process()
    assert svc.command('calibration.resume', params, 'retry-first')['accepted']
    assert read_document(root / 'round-progress.json')['rounds'] == progress['rounds'][1:]
    assert svc.snapshot()['session']['metrics']['completed_rounds'] == [2, 3]
    assert (root / 'round-1-original').read_text() == 'never remove original data'
    assert read_document(root / 'repeatability.json')['status'] == 'FAIL'
    archived = list((root / 'attempt-history').glob('*/round-progress.json'))
    assert len(archived) == 1
    assert read_document(archived[0]) == progress
    assert read_document(archived[0].with_name('retry-selection.json'))['retry_rounds'] == [1]
    assert svc.command('calibration.resume', params, 'retry-first')['accepted']
    assert len(factory.calls) == 2
    assert len(list((root / 'attempt-history').glob('*/round-progress.json'))) == 1
    svc.close()


@pytest.mark.parametrize('selection', [[0], [True], [1, 1], [3]])
def test_bad_recapture_selection_does_not_change_checkpoint(tmp_path, selection):
    svc, factory = service(tmp_path)
    start(svc)
    sid = svc.snapshot()['session']['session_id']
    factory.process.returncode = 1
    svc.tick()
    root = tmp_path / 'calibration' / 'sessions' / sid
    progress = dict(schema_version=1, session_id=sid, rounds=[dict(round=1), dict(round=2)])
    atomic_json(root / 'round-progress.json', progress)
    before = (root / 'round-progress.json').read_bytes()
    receipt = svc.command('calibration.resume', dict(
        expected_manager_instance_id='manager-1', session_id=sid,
        assembly_unchanged=True, retry_rounds=selection), 'bad-retry')
    assert not receipt['accepted']
    assert (root / 'round-progress.json').read_bytes() == before
    assert not (root / 'attempt-history').exists()
    assert len(factory.calls) == 1
    svc.close()


@pytest.mark.parametrize('problem', ['assembly', 'identity', 'config', 'session'])
def test_resume_rejects_changed_binding_or_missing_confirmation(tmp_path, problem):
    svc, factory = service(tmp_path)
    start(svc)
    sid = svc.snapshot()['session']['session_id']
    factory.process.returncode = 1
    svc.tick()
    params = dict(expected_manager_instance_id='manager-1', session_id=sid, assembly_unchanged=True)
    if problem == 'assembly':
        params['assembly_unchanged'] = False
    elif problem == 'session':
        params['session_id'] = 'other'
    else:
        changed = config()
        if problem == 'config':
            changed['frames']['world'] = 'different_map'
        svc.prepare_config = lambda p: dict(config=changed, expected_tracker_id='b'*64 if problem == 'identity' else TRACKER_ID)
    assert not svc.command('calibration.resume', params, 'resume-1')['accepted']
    assert len(factory.calls) == 1
    svc.close()


def test_three_verified_rounds_commit_and_publish(tmp_path):
    published = []
    svc, factory = service(tmp_path, publish=lambda *args: published.append(args))
    start(svc)
    ready(svc, factory)
    svc.tick()
    snap = svc.snapshot()
    assert snap['session']['state'] == 'COMPLETE'
    assert not snap['busy']
    assert snap['active'][0]['camera_name'] == 'cam'
    assert published[0][2] is True
    assert svc.active_documents()[0][0] == extrinsic()


def test_candidate_does_not_change_active_configuration(tmp_path):
    svc, factory = service(tmp_path)
    start(svc, mode='offline', bag_paths=['/data/one'])
    ready(svc, factory, eligible=False)
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'CANDIDATE'
    assert not svc.snapshot()['active']


def test_running_recorder_defers_application_then_applies(tmp_path):
    available = [True]
    svc, factory = service(tmp_path, can_start=lambda: (available[0], 'recording'))
    start(svc)
    ready(svc, factory)
    available[0] = False
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'WAITING_TO_APPLY'
    assert not svc.snapshot()['active']
    available[0] = True
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'COMPLETE'


def test_failed_publication_persists_committed_version_and_recovers(tmp_path):
    def broken(*args):
        raise RuntimeError('publisher unavailable')
    svc, factory = service(tmp_path, publish=broken)
    start(svc)
    ready(svc, factory)
    svc.tick()
    revision = svc.snapshot()['active'][0]['revision']
    assert svc.snapshot()['session']['state'] == 'APPLIED_PENDING_PUBLICATION'
    assert read_document(tmp_path / 'calibration' / 'active' / 'cam.json')['revision'] == revision
    svc.close()
    restored, _ = service(tmp_path)
    restored.tick()
    assert restored.snapshot()['session']['state'] == 'COMPLETE'
    assert restored.snapshot()['active'][0]['revision'] == revision


def test_unclosed_worker_cannot_apply_result(tmp_path):
    svc, factory = service(tmp_path)
    start(svc)
    ready(svc, factory)
    factory.process.returncode = None
    svc.tick()
    assert not svc.snapshot()['active']
    factory.process.returncode = 0
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'COMPLETE'


@pytest.mark.parametrize('mutation', ['overlap', 'fingerprint', 'validation', 'transform', 'path_escape'])
def test_invalid_worker_evidence_never_commits(tmp_path, mutation):
    svc, factory = service(tmp_path)
    start(svc)
    _, status_path, root = ready(svc, factory)
    manifest = read_document(root / 'manifest.json')
    if mutation == 'overlap':
        manifest['rounds'][1]['first_stamp_ns'] = 1000
    elif mutation == 'fingerprint':
        manifest['rounds'][1]['source_sha256'] = manifest['rounds'][0]['source_sha256']
    elif mutation == 'validation':
        manifest['rounds'][1]['validation']['translation_rms_m'] = .1
    elif mutation == 'transform':
        doc = extrinsic()
        doc['transform']['quaternion_xyzw'] = [0., 0., 0., 0.]
        Path(manifest['selected_extrinsics_path']).write_text(yaml.safe_dump(doc))
    else:
        outside = tmp_path / 'outside.yaml'
        outside.write_text(yaml.safe_dump(extrinsic()))
        manifest['selected_extrinsics_path'] = str(outside)
        status = read_document(status_path)
        status['result']['extrinsics_path'] = str(outside)
        atomic_json(status_path, status)
    atomic_json(root / 'manifest.json', manifest)
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'FAILED'
    assert not svc.snapshot()['active']


def test_cancel_is_cooperative_and_stale_cancel_does_not_touch_new_session(tmp_path):
    svc, factory = service(tmp_path)
    start(svc)
    session_id = svc.snapshot()['session']['session_id']
    assert svc.command('calibration.cancel', {'expected_manager_instance_id': 'manager-1', 'session_id': 'foreign'}, 'x')['accepted'] is False
    assert svc.command('calibration.cancel', {'expected_manager_instance_id': 'manager-1', 'session_id': session_id}, 'c')['accepted']
    cancel = Path(factory.calls[0][0][factory.calls[0][0].index('--cancel') + 1])
    assert cancel.exists()
    assert not factory.process.signals
    assert svc.snapshot()['session']['state'] == 'CANCELLING'


def test_restart_never_signals_persisted_process_id(tmp_path):
    svc, factory = service(tmp_path)
    start(svc)
    svc._lease.close()  # Emulate OS closing file descriptors after a crash.
    restored, _ = service(tmp_path)
    assert restored.snapshot()['session']['state'] == 'INTERRUPTED'
    assert not factory.process.signals


def test_storage_rejects_symlink_and_oversized_document(tmp_path):
    source = tmp_path / 'source.json'
    source.write_text('{}')
    link = tmp_path / 'link.json'
    link.symlink_to(source)
    with pytest.raises((ValueError, OSError)):
        read_document(link)
    source.write_text(' ' * (1024 * 1024 + 1))
    with pytest.raises(ValueError, match='large'):
        read_document(source)


def test_corrupt_status_is_sticky_and_cannot_later_activate(tmp_path):
    svc, factory = service(tmp_path)
    start(svc)
    _, status_path, _ = ready(svc, factory)
    good = read_document(status_path)
    factory.process.returncode = None
    atomic_json(status_path, dict(good, session_id='other'))
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'FAILED'
    atomic_json(status_path, good)
    factory.process.returncode = 0
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'FAILED'
    assert not svc.snapshot()['active']


def test_bad_recovery_still_has_valid_snapshot_identity(tmp_path):
    root = tmp_path / 'calibration'
    root.mkdir()
    atomic_json(root / 'session.json', {'session': {'session_id': 'invalid'}})
    svc, _ = service(tmp_path)
    assert len(svc.snapshot()['session']['session_id']) == 32


def test_yaml_aliases_are_rejected_without_expansion(tmp_path):
    path = tmp_path / 'aliases.yaml'
    path.write_text('first: &a [1, 2]\nsecond: *a\n')
    with pytest.raises(ValueError, match='alias'):
        read_document(path)


def test_store_restores_history_as_new_revision_and_rejects_stale_request(tmp_path):
    from vt_capture_manager.calibration.results import ResultStore
    store = ResultStore(tmp_path / 'store')
    first = store.commit(extrinsic(), {'session_id': 'one'})
    second_doc = extrinsic()
    second_doc['transform']['translation_m'] = [.2, 0., 0.]
    second = store.commit(second_doc, {'session_id': 'two'})
    with pytest.raises(ValueError, match='changed'):
        store.restore_revision('cam', first['revision'], expected_current_revision=first['revision'])
    restored = store.restore_revision('cam', first['revision'], expected_current_revision=second['revision'])
    assert restored['revision'] not in (first['revision'], second['revision'])
    assert restored['restored_from_revision'] == first['revision']
    assert restored['previous_revision'] == second['revision']
    assert store.load_active()[0][0] == extrinsic()


def test_worker_configuration_is_inside_existing_output_directory(tmp_path):
    svc, factory = service(tmp_path)
    start(svc)
    args = factory.calls[0][0]
    request = read_document(args[args.index('--request') + 1])
    assert Path(request['output_root']).is_dir()
    assert Path(request['config_path']).is_relative_to(request['output_root'])


def test_failed_worker_keeps_its_actionable_diagnostic(tmp_path):
    svc, factory = service(tmp_path)
    start(svc)
    _, status_path, _ = ready(svc, factory)
    status = read_document(status_path)
    status.update(state='FAILED', hint_code='FAILED', detail='Board was not detected; check measured square size.')
    atomic_json(status_path, status)
    factory.process.returncode = 1
    svc.tick()
    assert svc.snapshot()['session']['detail'] == status['detail']


def test_only_one_manager_can_own_calibration_storage(tmp_path):
    svc, _ = service(tmp_path)
    with pytest.raises((RuntimeError, OSError), match='owned|locked|unavailable'):
        service(tmp_path)
    svc.close()
    restored, _ = service(tmp_path)
    restored.close()


def test_start_receipt_survives_manager_restart(tmp_path):
    svc, factory = service(tmp_path)
    original = start(svc)
    ready(svc, factory)
    svc.tick()
    svc.close()
    restored, second_factory = service(tmp_path)
    result = start(restored)
    assert result == original
    assert not second_factory.calls


def test_activation_rejects_changed_base_revision(tmp_path):
    from vt_capture_manager.calibration.results import ResultStore
    svc, factory = service(tmp_path)
    start(svc)
    ready(svc, factory)
    ResultStore(tmp_path / 'calibration').commit(extrinsic(), {'session_id': 'external-change'})
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'FAILED'
    assert 'changed' in svc.snapshot()['session']['detail']


def test_restore_requires_identity_and_creates_new_operation_revision(tmp_path):
    from vt_capture_manager.calibration.results import ResultStore
    store = ResultStore(tmp_path / 'calibration')
    first = store.commit(extrinsic(), {'session_id': 'first'})
    changed = extrinsic()
    changed['transform']['translation_m'] = [.2, 0., 0.]
    second = store.commit(changed, {'session_id': 'second'})
    verified = [False]
    svc, _ = service(tmp_path, identity=lambda doc: (verified[0], 'tracker_unavailable'))
    params = dict(expected_manager_instance_id='manager-1', camera_name='cam', revision=first['revision'], expected_active_revision=second['revision'])
    rejected = svc.command('calibration.restore', params, 'restore-no-identity')
    assert not rejected['accepted']
    assert svc.snapshot()['active'][0]['revision'] == second['revision']
    verified[0] = True
    accepted = svc.command('calibration.restore', params, 'restore-1')
    assert accepted['accepted']
    assert svc.snapshot()['session']['mode'] == 'restore'
    assert svc.snapshot()['session']['state'] == 'COMPLETE'
    active = svc.snapshot()['active'][0]
    assert active['revision'] not in (first['revision'], second['revision'])
    assert active['source_session_id'] == accepted['session_id']
    assert active['calibration_session_id'] == 'first'
    assert svc.active_documents()[0][0] == extrinsic()


def test_live_owned_recorder_lease_blocks_new_session_after_worker_exit(tmp_path):
    import os
    from vt_capture_manager.calibration.service import _process_start
    svc, factory = service(tmp_path)
    start(svc)
    session_id = svc.snapshot()['session']['session_id']
    directory = tmp_path / 'calibration' / 'sessions' / session_id
    atomic_json(directory / 'recorder-lease.json', dict(schema_version=1, session_id=session_id,
                pid=os.getpid(), start_token=_process_start(os.getpid()),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip()))
    factory.process.returncode = 1
    svc.tick()
    assert svc.snapshot()['session']['state'] == 'FAILED'
    assert svc.busy
    assert not svc.command('calibration.start', dict(expected_manager_instance_id='manager-1'), 'new')['accepted']
    (directory / 'recorder-lease.json').unlink()
    svc.tick()
    assert not svc.busy


@pytest.mark.parametrize('after_commit', [False, True])
def test_atomic_active_write_failure_preserves_truthful_commit_state(tmp_path, monkeypatch, after_commit):
    from vt_capture_manager.calibration import results
    svc, factory = service(tmp_path)
    start(svc)
    ready(svc, factory)
    original = results.atomic_json
    failed = [False]
    def failing(path, document):
        if Path(path).parent.name == 'active' and not failed[0]:
            failed[0] = True
            if after_commit:
                original(path, document)
            raise OSError('simulated filesystem flush error')
        return original(path, document)
    monkeypatch.setattr(results, 'atomic_json', failing)
    svc.tick()
    snapshot = svc.snapshot()
    if after_commit:
        assert snapshot['session']['state'] == 'COMPLETE'
        assert snapshot['session']['result']['active_revision'] == snapshot['active'][0]['revision']
    else:
        assert snapshot['session']['state'] == 'FAILED'
        assert not snapshot['active']
