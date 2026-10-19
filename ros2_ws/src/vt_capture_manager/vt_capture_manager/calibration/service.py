"""Backend-owned calibration session lifecycle with crash-safe activation."""
from __future__ import annotations

from collections import OrderedDict
from copy import deepcopy
import fcntl
import os
from pathlib import Path
import re
import signal
import subprocess
import threading
import time
import uuid

from vt_capture_core.calibration_protocol import validate_calibration_snapshot

from .results import ResultStore, validate_candidate
from .storage import atomic_json, checked_path, digest_document, private_directory, read_document

WORKER_STATES = frozenset(('PREPARING', 'COLLECTING', 'STOPPING', 'SOLVING', 'VERIFYING',
                           'COMPARING', 'READY', 'FAILED', 'CANCELLED'))
TERMINAL_STATES = frozenset(('COMPLETE', 'CANDIDATE', 'FAILED', 'CANCELLED', 'INTERRUPTED'))


def _decision(value) -> tuple[bool, str]:
    if isinstance(value, tuple) and len(value) == 2 and type(value[0]) is bool:
        return value[0], str(value[1])[:512]
    if type(value) is bool:
        return value, '' if value else 'unavailable'
    if isinstance(value, str):
        return False, value[:512]
    raise ValueError('admission callback must return bool or (bool, reason)')


def _process_start(pid):
    """Linux birth token: inspected only, never authority to signal a saved PID."""
    try:
        fields = Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()
        return fields[19] if fields[0] != 'Z' else None
    except (OSError, ValueError, IndexError):
        return None


class CalibrationService:
    """All callbacks run under the manager's admission mutex (and this RLock).

    prepare_config(params) returns {config: schema1, expected_tracker_id: 64hex}.
    can_start() and identity(document) return bool or (bool, reason).
    apply(document, active_manifest) may update in-memory consumers; publish has
    arguments (document, active_manifest, identity_valid, reason). Failures after
    durable commit remain explicitly recoverable and never revert silently.
    """
    def __init__(self, root_dir, instance_id, worker_python, *, can_start,
                 prepare_config, identity=None, apply=None, publish=None, popen=subprocess.Popen):
        self.root = private_directory(root_dir)
        if any((parent / '.git').exists() for parent in (self.root, *self.root.parents)):
            raise ValueError('calibration sessions must be stored outside a Git checkout')
        descriptor = os.open(self.root / 'owner.lock', os.O_RDWR | os.O_CREAT | os.O_NOFOLLOW, 0o600)
        self._lease = os.fdopen(descriptor, 'r+b')
        try:
            fcntl.flock(self._lease.fileno(), fcntl.LOCK_EX | fcntl.LOCK_NB)
        except OSError as exc:
            self._lease.close()
            raise RuntimeError('calibration storage is already owned by another manager') from exc
        self.instance_id, self.worker_python = instance_id, str(worker_python)
        self.can_start, self.prepare_config = can_start, prepare_config
        self.identity = identity or (lambda document: (False, 'identity_unverified'))
        self.apply = apply or (lambda document, manifest: None)
        self.publish = publish or self._missing_publisher
        self._popen, self._lock = popen, threading.RLock()
        self._store = ResultStore(self.root)
        self._session = None
        self._job, self._log = None, None
        self._receipts = OrderedDict()
        self._candidate = None
        self._closed = False
        self._last_status = None
        self._worker_error = None
        self._last_refresh = 0.
        self._recovery_pid = None
        self._recovering = False
        self._active = {}
        self._publication = {}
        self._restore_receipts()
        self._restore()

    def _restore_receipts(self):
        path = self.root / 'requests.json'
        if path.exists():
            document = read_document(path)
            for value in document.get('receipts', [])[-128:]:
                self._receipts[value['request_id']] = (value['command'], value['fingerprint'], value['result'])

    def _remember(self, command, params, request_id, result):
        self._receipts[request_id] = (command, digest_document(params), deepcopy(result))
        while len(self._receipts) > 128:
            self._receipts.popitem(last=False)
        atomic_json(self.root / 'requests.json', dict(schema_version=1, receipts=[
            dict(request_id=key, command=value[0], fingerprint=value[1], result=value[2])
            for key, value in self._receipts.items()]))

    @staticmethod
    def _missing_publisher(*args):
        raise RuntimeError('calibration publisher is unavailable')

    def _restore(self):
        try:
            for document, manifest in self._store.load_active():
                self._active[manifest['camera_name']] = (document, manifest)
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._session = self._new_session('', '', '', '')
            self._session.update(state='FAILED', hint_code='active_configuration_invalid', detail=str(exc)[:1024])
            return
        path = self.root / 'session.json'
        if not path.exists():
            return
        try:
            saved = read_document(path)
            session = saved['session']
            if session.get('schema_version') != 1 or not re.fullmatch(r'[0-9a-f]{32}', session.get('session_id', '')):
                raise ValueError('invalid persisted calibration session')
            self._session = session
            self._recovery_pid = saved.get('process')
            self._recovering = self._previous_worker_alive() or self._recorder_alive()
            if self._recovering:
                self._request_cancel()
            if session['state'] not in TERMINAL_STATES:
                committed = next((pair for pair in self._active.values()
                                  if pair[1]['source_session_id'] == session['session_id']), None)
                if committed:
                    self._adopt_commit(*committed)
                    self._session['hint_code'] = 'restoring_calibration'
                elif session['state'] == 'WAITING_TO_APPLY':
                    self._candidate = self._validate_ready()
                else:
                    self._request_cancel()
                    self._recovery_pid = saved.get('process')
                    self._recovering = self._previous_worker_alive() or self._recorder_alive()
                    self._session.update(state='INTERRUPTED', hint_code='session_interrupted',
                                         detail='Manager restarted; interrupted capture is preserved and cancelled.')
                self._persist()
        except (OSError, ValueError, KeyError, TypeError) as exc:
            self._session = self._new_session('', '', '', '')
            self._session.update(state='FAILED', hint_code='session_recovery_failed', detail=str(exc)[:1024])

    def _previous_worker_alive(self):
        value = self._recovery_pid
        return bool(isinstance(value, dict) and value.get('start')
                    and value.get('boot_id') == Path('/proc/sys/kernel/random/boot_id').read_text().strip()
                    and _process_start(value.get('pid')) == value['start'])

    def _recorder_alive(self):
        if self._session is None:
            return False
        path = self._directory() / 'recorder-lease.json'
        if not path.exists():
            return False
        try:
            lease = read_document(path)
            if lease.get('schema_version') != 1 or lease.get('session_id') != self._session['session_id']:
                return True  # Unknown ownership must never authorize another Recorder.
            boot_id = Path('/proc/sys/kernel/random/boot_id').read_text().strip()
            return bool(lease.get('boot_id') == boot_id and lease.get('start_token')
                        and _process_start(lease.get('pid')) == lease['start_token'])
        except (OSError, ValueError, TypeError):
            return True

    @staticmethod
    def _new_session(session_id, mode, camera_name, tracker_role):
        return dict(schema_version=1, session_id=session_id or uuid.uuid4().hex, mode=mode, state='PREPARING',
                    camera_name=camera_name, tracker_role=tracker_role, round=0, rounds=3,
                    accepted=0, target=50, hint_code='preparing', detail='', metrics={},
                    preview=None, result=None, updated_ns=time.time_ns())

    @property
    def busy(self):
        with self._lock:
            return bool(self._recovering or self._recorder_alive() or self._job is not None and self._job.poll() is None
                        or self._session and self._session['state'] not in TERMINAL_STATES)

    def snapshot(self):
        with self._lock:
            active = []
            for _, manifest in self._active.values():
                item = deepcopy(manifest)
                item.update(self._publication.get(manifest['camera_name'],
                            dict(published=False, identity_valid=False, reason='pending_publication')))
                active.append(item)
            return dict(enabled=not self._closed, busy=self.busy,
                        session=deepcopy(self._session), active=active)

    def active_documents(self):
        with self._lock:
            return deepcopy(list(self._active.values()))

    def command(self, command, params, request_id):
        with self._lock:
            if not isinstance(params, dict) or params.get('expected_manager_instance_id') != self.instance_id:
                return self._receipt(False, 'manager_instance_mismatch')
            if not isinstance(request_id, str) or not 1 <= len(request_id) <= 256:
                return self._receipt(False, 'invalid_request_id')
            if request_id in self._receipts:
                previous, payload, result = self._receipts[request_id]
                return deepcopy(result) if (previous, payload) == (command, digest_document(params)) else self._receipt(False, 'request_id_conflict')
            try:
                if self._closed:
                    result = self._receipt(False, 'calibration_closed')
                elif command == 'calibration.start':
                    result = self._start(params, request_id)
                elif command == 'calibration.restore':
                    result = self._restore_version(params, request_id)
                elif command in ('calibration.cancel', 'calibration.retry_publish'):
                    if self._session is None or params.get('session_id') != self._session['session_id']:
                        result = self._receipt(False, 'calibration_session_mismatch')
                    elif command == 'calibration.cancel':
                        result = self._cancel()
                    else:
                        self._refresh_active(force=True)
                        result = self._receipt(True, 'publication_retried')
                else:
                    result = self._receipt(False, 'unknown_command')
            except (ValueError, OSError, KeyError, TypeError, RuntimeError) as exc:
                result = self._receipt(False, 'calibration_request_rejected', str(exc))
            self._remember(command, params, request_id, result)
            return result

    def _receipt(self, accepted, code, detail=''):
        return dict(accepted=accepted, code=code, detail=str(detail)[:1024],
                    session_id=self._session['session_id'] if self._session else None)

    def _start(self, params, request_id):
        if self.busy:
            return self._receipt(False, 'calibration_busy', 'An earlier calibration is still active.')
        allowed, reason = _decision(self.can_start())
        if not allowed:
            return self._receipt(False, 'calibration_busy', reason)
        mode = params.get('mode', 'guided')
        if mode not in ('guided', 'offline'):
            raise ValueError('unsupported calibration mode')
        prepared = self.prepare_config(deepcopy(params))
        config, tracker_id = prepared['config'], prepared.get('expected_tracker_id', '')
        if config.get('schema_version') != 1 or config['camera']['name'] != params.get('camera_name') or config['tracker']['role'] != params.get('tracker_role'):
            raise ValueError('prepared configuration does not match selected devices')
        if mode == 'guided' and (not isinstance(tracker_id, str) or re.fullmatch(r'[0-9a-f]{64}', tracker_id) is None):
            raise ValueError('live Tracker identity is not verified')
        bags = params.get('bag_paths', [])
        if not isinstance(bags, list) or any(not isinstance(path, str) or not path or len(path) > 4096 for path in bags):
            raise ValueError('invalid offline bag paths')
        if mode == 'offline' and len(bags) not in (1, 3) or mode == 'guided' and bags:
            raise ValueError('offline calibration accepts one or three bags; guided mode records its own')
        session_id = uuid.uuid4().hex
        directory = private_directory(self.root / 'sessions' / session_id)
        atomic_json(directory / 'config.yaml', config)
        request = dict(schema_version=1, session_id=session_id, mode=mode, config_path=str(directory / 'config.yaml'),
                       output_root=str(directory), bag_paths=bags, expected_tracker_id=tracker_id)
        atomic_json(directory / 'request.json', request)
        previous = self._active.get(config['camera']['name'])
        atomic_json(directory / 'admission.json', dict(expected_active_revision=previous[1]['revision'] if previous else ''))
        session = self._new_session(session_id, mode, config['camera']['name'], config['tracker']['role'])
        self._session, self._candidate, self._last_status = session, None, None
        self._worker_error = None
        self._remember('calibration.start', params, request_id, self._receipt(True, 'calibration_started'))
        self._persist()
        args = [self.worker_python, '-m', 'vt_tracker_camera_calib.guided_worker',
                '--request', str(directory / 'request.json'), '--status', str(directory / 'status.json'),
                '--cancel', str(directory / 'cancel.json')]
        try:
            self._log = open(directory / 'worker.log', 'xb', buffering=0)
            os.chmod(directory / 'worker.log', 0o600)
            self._job = self._popen(args, stdin=subprocess.DEVNULL, stdout=self._log, stderr=subprocess.STDOUT,
                                    start_new_session=True, close_fds=True)
            self._persist()
        except (OSError, ValueError) as exc:
            self._finish('FAILED', 'worker_start_failed', str(exc))
            raise
        return self._receipt(True, 'calibration_started')

    def _restore_version(self, params, request_id):
        if self.busy:
            return self._receipt(False, 'calibration_busy')
        allowed, reason = _decision(self.can_start())
        if not allowed:
            return self._receipt(False, 'calibration_busy', reason)
        camera, revision = params.get('camera_name'), params.get('revision')
        document, _, _ = self._store.read_revision(camera, revision)
        current_path = self._store.active_root / f'{camera}.json'
        if not current_path.exists() or read_document(current_path)['revision'] != params.get('expected_active_revision'):
            return self._receipt(False, 'calibration_revision_changed')
        valid, reason = _decision(self.identity(document))
        if not valid:
            return self._receipt(False, 'calibration_identity_unverified', reason)
        session_id = uuid.uuid4().hex
        private_directory(self.root / 'sessions' / session_id)
        self._session = self._new_session(session_id, 'restore', camera, document['tracker']['role'])
        self._session.update(state='APPLYING', hint_code='restoring_calibration',
                             result=dict(quality_grade=document['quality']['grade']))
        self._remember('calibration.restore', params, request_id, self._receipt(True, 'calibration_restored'))
        self._persist()
        try:
            manifest = self._store.restore_revision(camera, revision,
                expected_current_revision=params['expected_active_revision'], operation_session_id=session_id)
            self._adopt_commit(document, manifest)
            self._refresh_active(force=True)
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
            committed = next((pair for pair in self._store.load_active() if pair[1]['source_session_id'] == session_id), None)
            if committed:
                self._adopt_commit(*committed)
                self._session.update(hint_code='publication_pending', detail=str(exc)[:1024])
                self._persist()
            else:
                self._finish('FAILED', 'activation_failed', str(exc))
                return self._receipt(False, 'calibration_restore_failed', str(exc))
        return self._receipt(True, 'calibration_restored')

    def _directory(self):
        return checked_path(self.root / 'sessions' / self._session['session_id'], within=self.root)

    def _persist(self):
        process = None
        if self._job is not None and self._job.poll() is None:
            process = dict(pid=self._job.pid, start=_process_start(self._job.pid),
                           boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip())
        atomic_json(self.root / 'session.json', dict(schema_version=1, session=self._session, process=process))

    def _request_cancel(self):
        atomic_json(self._directory() / 'cancel.json', dict(schema_version=1, session_id=self._session['session_id']))

    def _cancel(self):
        if self._session['state'] in TERMINAL_STATES:
            return self._receipt(True, 'already_finished')
        if self._session['state'] == 'APPLIED_PENDING_PUBLICATION':
            return self._receipt(False, 'calibration_already_applied', 'Committed calibration cannot be cancelled.')
        self._request_cancel()
        if self._job is None or self._job.poll() is not None:
            self._finish('CANCELLED', 'cancelled')
        else:
            self._session.update(state='CANCELLING', hint_code='cancelling', updated_ns=time.time_ns())
            self._persist()
        return self._receipt(True, 'calibration_cancelling')

    def _worker_status(self):
        path = self._directory() / 'status.json'
        if not path.exists():
            return None
        document = read_document(path)
        if (document.get('schema_version') != 1 or document.get('session_id') != self._session['session_id']
                or document.get('state') not in WORKER_STATES):
            raise ValueError('worker status does not match calibration session')
        for key in ('round', 'rounds', 'accepted', 'target', 'updated_ns'):
            if type(document.get(key)) is not int or document[key] < 0:
                raise ValueError(f'invalid worker status {key}')
        if document['rounds'] != 3 or document['round'] > 3 or document['target'] > 10000 or document['accepted'] > 10000:
            raise ValueError('worker progress is outside bounds')
        if not isinstance(document.get('metrics'), dict) or not isinstance(document.get('hint_code'), str):
            raise ValueError('worker diagnostics must be structured')
        if not isinstance(document.get('detail'), str) or len(document['detail']) > 4096:
            raise ValueError('worker detail is too large')
        projected = dict(self._session)
        projected.update({key: document[key] for key in ('state', 'round', 'rounds', 'accepted', 'target', 'hint_code', 'detail', 'metrics', 'preview', 'result', 'updated_ns') if key in document})
        validate_calibration_snapshot(dict(enabled=True, busy=True, session=projected, active=[]))
        return document

    def _validate_ready(self):
        request = read_document(self._directory() / 'request.json')
        config = read_document(request['config_path'], within=self._directory())
        return validate_candidate(self._session['result'], output_root=Path(request['output_root']),
                                  session_id=self._session['session_id'], config=config,
                                  tracker_id=request['expected_tracker_id'])

    def tick(self):
        with self._lock:
            if self._closed:
                return
            if self._recovering:
                self._recovering = self._previous_worker_alive() or self._recorder_alive()
            if self._job is not None and self._worker_error is not None:
                if self._job.poll() is not None:
                    self._finish('FAILED', 'invalid_worker_result', self._worker_error)
            if self._job is not None and self._worker_error is None:
                try:
                    status = self._worker_status()
                    if status is not None and status != self._last_status:
                        self._last_status = status
                        if self._session['state'] != 'CANCELLING':
                            self._session.update({key: deepcopy(status[key]) for key in (
                                'state', 'round', 'rounds', 'accepted', 'target', 'hint_code', 'detail', 'metrics',
                                'preview', 'result', 'updated_ns') if key in status})
                            self._persist()
                    exit_code = self._job.poll()
                    if exit_code is not None:
                        self._release_job()
                        if self._session['state'] == 'CANCELLING':
                            self._finish('CANCELLED', 'cancelled')
                        elif status is not None and status['state'] in ('FAILED', 'CANCELLED'):
                            self._finish(status['state'], status['hint_code'], status['detail'])
                        elif exit_code != 0 or status is None or status['state'] != 'READY':
                            self._finish('FAILED', 'worker_failed', f'Calibration worker exited with code {exit_code}.')
                        elif status['state'] == 'READY':
                            self._candidate = self._validate_ready()
                            self._session['result']['quality_grade'] = self._candidate[0]['quality']['grade']
                            if not self._candidate[1]['activation_eligible']:
                                self._finish('CANDIDATE', 'candidate_saved')
                            else:
                                self._session.update(state='WAITING_TO_APPLY', hint_code='waiting_to_apply')
                                self._persist()
                        else:
                            self._finish(status['state'], status['hint_code'], status['detail'])
                except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
                    self._worker_error = str(exc)[:1024]
                    if self._job is not None and self._job.poll() is None:
                        self._request_cancel()
                    self._finish('FAILED', 'invalid_worker_result', str(exc), release=self._job is None or self._job.poll() is not None)
            if self._session and self._session['state'] == 'WAITING_TO_APPLY':
                self._try_activate()
            self._refresh_active()

    def _try_activate(self):
        try:
            allowed, reason = _decision(self.can_start())
            if not allowed:
                self._session.update(hint_code='waiting_for_idle', detail=reason)
                return
            document, acceptance = self._candidate or self._validate_ready()
            valid, reason = _decision(self.identity(document))
            if not valid:
                self._session.update(hint_code='waiting_for_identity', detail=reason)
                return
            admission = read_document(self._directory() / 'admission.json')
            current_path = self._store.active_root / f"{document['camera']['name']}.json"
            current_revision = read_document(current_path)['revision'] if current_path.exists() else ''
            if current_revision != admission['expected_active_revision']:
                raise ValueError('active calibration changed during this session')
            manifest = self._store.commit(document, acceptance)
            self._adopt_commit(document, manifest)
            self._refresh_active(force=True)
        except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
            # A committed pointer may exist even if a subsequent fsync/persist failed.
            committed = next((pair for pair in self._store.load_active()
                              if pair[1]['source_session_id'] == self._session['session_id']), None)
            if committed:
                self._adopt_commit(*committed)
                self._session.update(hint_code='publication_pending', detail=str(exc)[:1024])
            else:
                self._finish('FAILED', 'activation_failed', str(exc))

    def _adopt_commit(self, document, manifest):
        """Project the durable commit into memory, including recovery after fsync errors."""
        self._active[manifest['camera_name']] = (document, manifest)
        self._publication.pop(manifest['camera_name'], None)
        result = self._session.get('result') or {}
        result.update(active_revision=manifest['revision'], topic=manifest['topic'],
                      quality_grade=manifest['quality_grade'], extrinsics_path=manifest['extrinsics_path'])
        self._session.update(state='APPLIED_PENDING_PUBLICATION', hint_code='applying_calibration', detail='', result=result)
        self._persist()

    def _refresh_active(self, *, force=False):
        now = time.monotonic()
        if not force and now - self._last_refresh < 1.:
            return
        self._last_refresh = now
        for camera_name, (document, manifest) in self._active.items():
            previous = self._publication.get(camera_name)
            try:
                valid, reason = _decision(self.identity(document))
                revision = manifest['revision']
                changed = previous is None or (previous.get('identity_valid'), previous.get('reason'), previous.get('revision')) != (valid, reason, revision)
                if force or changed or not previous.get('published'):
                    self.apply(deepcopy(document), deepcopy(manifest))
                    self.publish(deepcopy(document), deepcopy(manifest), valid, reason)
                    self._publication[camera_name] = dict(published=True, identity_valid=valid, reason=reason, revision=revision)
                if (self._session and self._session['state'] == 'APPLIED_PENDING_PUBLICATION'
                        and self._session['session_id'] == manifest['source_session_id']):
                    self._finish('COMPLETE', 'calibration_applied')
            except (OSError, ValueError, KeyError, TypeError, RuntimeError) as exc:
                self._publication[camera_name] = dict(published=False, identity_valid=False, reason=str(exc)[:1024], revision=manifest['revision'])
                if self._session and self._session['session_id'] == manifest['source_session_id']:
                    self._session.update(state='APPLIED_PENDING_PUBLICATION', hint_code='publication_pending', detail=str(exc)[:1024])
                    self._persist()

    def _release_job(self):
        if self._job is not None:
            self._job.wait(timeout=0)
            self._job = None
        if self._log is not None:
            self._log.close()
            self._log = None

    def _finish(self, state, hint_code, detail='', *, release=True):
        if release:
            self._release_job()
        self._session.update(state=state, hint_code=hint_code, detail=str(detail)[:1024], updated_ns=time.time_ns(), preview=None)
        self._persist()

    def close(self):
        with self._lock:
            if self._closed:
                return
            if self._job is not None and self._job.poll() is None:
                self._request_cancel()
                try:
                    self._job.wait(timeout=5.)
                except subprocess.TimeoutExpired:
                    # Only the child handle started by this object is signalled.
                    self._job.send_signal(signal.SIGINT)
                    try:
                        self._job.wait(timeout=2.)
                    except subprocess.TimeoutExpired:
                        self._session.update(state='INTERRUPTED', hint_code='session_interrupted')
                        self._persist()
                if self._job.poll() is not None:
                    self._finish('CANCELLED', 'cancelled')
            self._closed = True
            self._lease.close()
