"""Owned, headless three-round calibration session.

Run with ``python -m vt_tracker_camera_calib.guided_worker --request ...
--status ... --cancel ...``. Files are the process boundary: no Qt dependencies,
no configuration activation, and no calibration publisher live in this worker.
"""
from __future__ import annotations

import argparse
from dataclasses import dataclass, replace
import hashlib
import os
from pathlib import Path
import re
import shutil
import signal
import sys
import threading
import time

import yaml

from .bag_reader import read_calibration_bag
from .config import load_config
from .export import export_result
from .guided_live import GuidedMonitor
from .guided_quality import (POLICY_VERSION, collection_target, solve_with_independent_validation,
                             split_validation_pairs)
from .gui_workflow import ManagedJob, external_output_path, record_command
from .pairing import pair_static_observations
from .repeatability import compare_extrinsics, write_repeatability_report
from .session_io import MAX_DOCUMENT_BYTES, atomic_document, read_document
from .process_lease import clear_recorder_lease, process_start_token
from .round_store import RoundStore

@dataclass(frozen=True)
class Request:
    session_id: str
    mode: str
    config_path: Path
    output_root: Path
    bag_paths: tuple[Path, ...]
    expected_tracker_id: str | None = None

    @classmethod
    def from_document(cls, document):
        if not isinstance(document, dict) or document.get('schema_version') != 1:
            raise ValueError('request schema_version must be 1')
        session_id = document.get('session_id')
        if not isinstance(session_id, str) or not re.fullmatch(r'[A-Za-z0-9_-]{1,128}', session_id):
            raise ValueError('invalid session_id')
        mode = document.get('mode')
        if mode not in {'guided', 'offline'}:
            raise ValueError('mode must be guided or offline')
        for key in ('config_path', 'output_root'):
            if not isinstance(document.get(key), str) or not Path(document[key]).is_absolute():
                raise ValueError(f'{key} must be absolute')
        root = external_output_path(Path(document['output_root']))
        config = Path(document['config_path']).resolve()
        if not config.is_relative_to(root):
            raise ValueError('frozen config must belong to this session')
        if not root.is_dir():
            raise ValueError('session directory does not exist')
        bags = document.get('bag_paths', [])
        if not isinstance(bags, list) or any(not isinstance(p, str) or not Path(p).is_absolute() for p in bags):
            raise ValueError('bag_paths must be a list of absolute paths')
        bags = tuple(Path(p).resolve() for p in bags)
        if len(set(bags)) != len(bags):
            raise ValueError('source bags must be distinct')
        if mode == 'offline' and len(bags) not in {1, 3} or mode == 'guided' and bags:
            raise ValueError('offline requires one or three bags; guided manages its own bags')
        identity = document.get('expected_tracker_id') or None
        if identity is not None and (not isinstance(identity, str) or not re.fullmatch('[0-9a-f]{64}', identity)):
            raise ValueError('expected_tracker_id must be a 64-character identity fingerprint')
        load_config(config)
        return cls(session_id, mode, config, root, bags, identity)


class StatusWriter:
    def __init__(self, path, session_id):
        self.path = Path(path)
        self.completed_rounds = []
        self._value = dict(schema_version=1, session_id=session_id, state='PREPARING',
                           round=0, rounds=3, accepted=0, target=50,
                           hint_code='WAITING_INPUT', detail='', metrics={}, preview=None,
                           result=None, updated_ns=time.time_ns())

    def update(self, **fields):
        candidate = {**self._value, **fields, 'updated_ns': time.time_ns()}
        candidate['metrics'] = {**candidate.get('metrics', {}),
                                'completed_rounds': list(self.completed_rounds)}
        atomic_document(self.path, candidate)
        self._value = candidate


def source_fingerprint(path, check_cancel=lambda: None):
    """Hash storage content, independent of directory/file renaming.

    The original metadata-selected files are checked before and after each read.
    Copied bags remain the same source even when operators rename their folders.
    """
    path = Path(path).resolve()
    metadata = path / 'metadata.yaml'
    if not metadata.is_file() or metadata.is_symlink() or metadata.stat().st_size > MAX_DOCUMENT_BYTES:
        raise ValueError('invalid bag metadata')
    document = yaml.safe_load(metadata.read_text())
    files = document['rosbag2_bagfile_information']['relative_file_paths']
    if not isinstance(files, list) or not files:
        raise ValueError('bag metadata has no storage files')
    digest = hashlib.sha256()
    for name in files:
        check_cancel()
        if (not isinstance(name, str) or Path(name).is_absolute()
                or '..' in Path(name).parts or (path / name).is_symlink()):
            raise ValueError('invalid bag storage path')
        source = path / name
        if not source.resolve().is_relative_to(path):
            raise ValueError('escaped bag storage path')
        before = source.stat()
        if not source.is_file():
            raise ValueError('invalid bag storage file')
        part = hashlib.sha256()
        with source.open('rb') as stream:
            while block := stream.read(4 * 1024 * 1024):
                check_cancel()
                part.update(block)
        after = source.stat()
        if (before.st_ino, before.st_size, before.st_mtime_ns) != (after.st_ino, after.st_size, after.st_mtime_ns):
            raise ValueError('bag storage changed while reading')
        digest.update(part.digest())
    return digest.hexdigest()


class Worker:
    def __init__(self, request, status_path, cancel_path):
        self.request = request
        self.config = load_config(request.config_path)
        # Commercial guided acceptance cannot be weakened by a legacy config.
        self.config = replace(self.config, acceptance=replace(self.config.acceptance,
            min_pairs=max(40, self.config.acceptance.min_pairs),
            maximum_translation_rms_m=min(.010, self.config.acceptance.maximum_translation_rms_m),
            maximum_rotation_rms_deg=min(1., self.config.acceptance.maximum_rotation_rms_deg)))
        self.status = StatusWriter(status_path, request.session_id)
        self.cancel_path = Path(cancel_path)
        self.cancel_event = threading.Event()
        self.tracker_id = request.expected_tracker_id
        self.round_store = RoundStore(request, lambda bag: source_fingerprint(bag, self.check_cancel))

    def _unused_path(self, name):
        path = self.request.output_root / name
        attempt = 2
        while path.exists():
            original = Path(name)
            path = self.request.output_root / f'{original.stem}-attempt-{attempt}{original.suffix}'
            attempt += 1
        return path

    def cancel(self, *_):
        self.cancel_event.set()

    def check_cancel(self):
        if self.cancel_event.is_set() or self.cancel_path.exists():
            raise InterruptedError('Calibration cancelled')

    def _collect(self, number):
        directory = self._unused_path(f'round-{number}')
        directory.mkdir(mode=0o700)
        bag = directory / 'bag'
        recorder = ManagedJob()
        lease_path = self.request.output_root / 'recorder-lease.json'
        monitor = GuidedMonitor(self.config, self.tracker_id)
        try:
            if shutil.disk_usage(directory).free < 1024 ** 3:
                raise RuntimeError('Insufficient free space for calibration recording')
            monitor.start()
            self.check_cancel()
            command = [sys.executable, '-m', 'vt_tracker_camera_calib.recorder_runner',
                       '--lease', str(lease_path), '--session-id', self.request.session_id,
                       '--owner-pid', str(os.getpid()), '--owner-start-token', process_start_token(os.getpid()),
                       '--', *record_command(self.config, bag)]
            recorder.start(command, directory / 'recorder.log')
            started = time.monotonic()
            while True:
                self.check_cancel()
                if not recorder.running:
                    raise RuntimeError(f'Recorder exited before collection completed ({recorder.poll()})')
                if time.monotonic() - started > 1800:
                    raise RuntimeError('Calibration round exceeded 30 minutes; retry with broader poses')
                if shutil.disk_usage(directory).free < 512 * 1024 ** 2:
                    raise RuntimeError('Recording stopped before free disk space was exhausted')
                monitor.request_detection()
                snapshot = monitor.snapshot()
                if snapshot['detail']:
                    raise RuntimeError(snapshot['detail'])
                ready = snapshot.pop('ready')
                self.status.update(state='COLLECTING', round=number, **snapshot)
                if ready:
                    self.tracker_id = monitor.tracker_id
                    break
                self.cancel_event.wait(.1)
        finally:
            # A terminal worker state is not emitted while its Recorder is alive.
            recorder.stop()
            self.status.update(state='STOPPING', hint_code='STOPPING_RECORDING', preview=None)
            try:
                while recorder.running:
                    self.cancel_event.wait(.1) if not self.cancel_event.is_set() else time.sleep(.1)
            finally:
                try:
                    if recorder.poll() is not None:
                        clear_recorder_lease(lease_path)
                finally:
                    monitor.stop()
        self.check_cancel()
        if recorder.poll() not in {0, -signal.SIGINT, 130}:
            raise RuntimeError(f'Recorder did not close successfully ({recorder.poll()})')
        if not (bag / 'metadata.yaml').is_file():
            raise RuntimeError('Recorder closed without readable bag metadata')
        return bag

    def _solve_round(self, number, bag):
        self.check_cancel()
        self.status.update(state='SOLVING', round=number, hint_code='SOLVING', preview=None)
        fingerprint = source_fingerprint(bag, self.check_cancel)
        observations = read_calibration_bag(bag, self.config, check_cancel=self.check_cancel)
        self.check_cancel()
        if self.tracker_id not in {None, observations.tracker_id}:
            raise ValueError('Recorded Tracker identity differs from selected session identity')
        self.tracker_id = observations.tracker_id
        pairs = pair_static_observations(observations.board_observations,
                                         observations.tracker_samples, self.config.pairing)
        self.status.update(state='VERIFYING', accepted=len(pairs), hint_code='VERIFYING')
        solution, validation = solve_with_independent_validation(pairs, self.config.acceptance)
        self.check_cancel()
        if source_fingerprint(bag, self.check_cancel) != fingerprint:
            raise ValueError('Source bag changed during calibration')
        training, _ = split_validation_pairs(pairs, self.config.acceptance)
        result = export_result(output_directory=self._unused_path(f'round-{number}-result'),
            config_path=self.request.config_path, bag_path=bag, config=self.config,
            observations=observations, selected_pairs=training, solution=solution)
        validation_path = result / 'independent-validation.json'
        atomic_document(validation_path, validation)
        if validation['status'] != 'PASS':
            raise ValueError('Independent validation failed; keep the board fixed and repeat collection')
        clocks = [value.timestamp_ns for value in observations.tracker_samples]
        return dict(round=number, round_id=f'{self.request.session_id}:{number}',
                    extrinsics_path=str(result / 'extrinsics.yaml'), report_path=str(result / 'report.json'),
                    validation_path=str(validation_path), bag_path=str(bag), source_sha256=fingerprint,
                    first_stamp_ns=min(clocks), last_stamp_ns=max(clocks),
                    tracker_id=self.tracker_id, validation=validation)

    def run(self):
        self.status.update(target=collection_target(self.config.acceptance))
        try:
            self.check_cancel()
            rounds = self.round_store.load() if self.request.mode == 'guided' else []
            self.status.completed_rounds = [r['round'] for r in rounds]
            self.status.update(detail='')
            count = 3 if self.request.mode == 'guided' else len(self.request.bag_paths)
            for number in range(1, count + 1):
                self.check_cancel()
                if any(r['round'] == number for r in rounds):
                    continue
                bag = self._collect(number) if self.request.mode == 'guided' else self.request.bag_paths[number - 1]
                candidate = self._solve_round(number, bag)
                for previous in rounds:
                    if previous['source_sha256'] == candidate['source_sha256']:
                        raise ValueError('Independent rounds must use distinct recorded source data')
                    if not (previous['last_stamp_ns'] < candidate['first_stamp_ns']
                            or candidate['last_stamp_ns'] < previous['first_stamp_ns']):
                        raise ValueError('Independent round source time windows must not overlap')
                rounds.append(candidate)
                if self.request.mode == 'guided':
                    rounds = self.round_store.save(rounds)
                    self.status.completed_rounds = [r['round'] for r in rounds]
                    self.status.update(result=dict(extrinsics_path=candidate['extrinsics_path'],
                        report_path=candidate['report_path']))
            eligible, repeatability_path = False, None
            selected = rounds[0]
            if len(rounds) == 3:
                self.status.update(state='COMPARING', hint_code='COMPARING', preview=None)
                comparison = compare_extrinsics([r['extrinsics_path'] for r in rounds])
                repeatability_path = self._unused_path('repeatability.json')
                write_repeatability_report(repeatability_path, comparison)
                if comparison.status != 'PASS':
                    self.status.update(state='FAILED', hint_code='REPEATABILITY_FAILED',
                        detail=(f'Three-round repeatability failed: maximum translation '
                            f'{comparison.maximum_translation_m * 1000:.3f} mm '
                            f'(limit {comparison.threshold_translation_m * 1000:.3f} mm), '
                            f'rotation {comparison.maximum_rotation_deg:.3f} deg '
                            f'(limit {comparison.threshold_rotation_deg:.3f} deg).'),
                        result=dict(repeatability_path=str(repeatability_path)),
                        preview=None)
                    return 1
                selected = next(r for r in rounds if '/'.join(Path(r['extrinsics_path']).parts[-2:]) == comparison.recommended_input)
                eligible = True
            self.check_cancel()
            manifest_path = self.request.output_root / 'manifest.json'
            atomic_document(manifest_path, dict(schema_version=1, session_id=self.request.session_id,
                policy_version=POLICY_VERSION, activation_eligible=eligible, rounds=rounds,
                repeatability_path=str(repeatability_path) if repeatability_path else None,
                selected_extrinsics_path=selected['extrinsics_path']))
            result = dict(extrinsics_path=selected['extrinsics_path'], report_path=selected['report_path'],
                          repeatability_path=str(repeatability_path) if repeatability_path else None,
                          manifest_path=str(manifest_path))
            self.status.update(state='READY', hint_code='READY' if eligible else 'CANDIDATE',
                result=result, preview=None, metrics={'activation_eligible': eligible,
                'tracker_id': self.tracker_id, 'training_target': self.config.acceptance.min_pairs,
                'validation_target': collection_target(self.config.acceptance) // 5})
            return 0
        except InterruptedError:
            self.status.update(state='CANCELLED', hint_code='CANCELLED', preview=None, detail='')
            return 130
        except Exception as error:
            self.status.update(state='FAILED', hint_code='FAILED', preview=None, detail=str(error)[:2048])
            return 1


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--request', type=Path, required=True)
    parser.add_argument('--status', type=Path, required=True)
    parser.add_argument('--cancel', type=Path, required=True)
    args = parser.parse_args(argv)
    request = Request.from_document(read_document(args.request))
    for path in (args.status, args.cancel):
        if not path.is_absolute() or not path.resolve().is_relative_to(request.output_root):
            raise ValueError('status and cancel paths must belong to this session')
    worker = Worker(request, args.status, args.cancel)
    signal.signal(signal.SIGINT, worker.cancel)
    signal.signal(signal.SIGTERM, worker.cancel)
    return worker.run()


if __name__ == '__main__':
    raise SystemExit(main())
