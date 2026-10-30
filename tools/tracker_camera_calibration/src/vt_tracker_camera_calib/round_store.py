"""Durable per-round results; a failed retry never overwrites an accepted round."""
from __future__ import annotations

import hashlib
import math
from pathlib import Path
import yaml

from .session_io import atomic_document, MAX_DOCUMENT_BYTES
from .transforms import Transform


def digest(path):
    return hashlib.sha256(Path(path).read_bytes()).hexdigest()


def read_document(path):
    path = Path(path)
    if path.is_symlink() or path.stat().st_size > MAX_DOCUMENT_BYTES:
        raise ValueError('Invalid round document')
    value = yaml.safe_load(path.read_text(encoding='utf-8'))
    if not isinstance(value, dict):
        raise ValueError('Round document must be a mapping')
    return value


class RoundStore:
    def __init__(self, request, fingerprint):
        self.request = request
        self.root = request.output_root.resolve()
        self.path = self.root / 'round-progress.json'
        self.config_sha = digest(request.config_path)
        self.config = read_document(request.config_path)
        self.fingerprint = fingerprint

    def _artifact(self, path):
        path = Path(path)
        if path.is_symlink() or not path.resolve().is_relative_to(self.root):
            raise ValueError('Round artifact must belong to the calibration session')
        return path

    def _validate(self, entry):
        if type(entry.get('round')) is not int or not 1 <= entry['round'] <= 3:
            raise ValueError('Invalid saved round number')
        for key in ('extrinsics_path', 'report_path', 'validation_path'):
            path = self._artifact(entry[key])
            if digest(path) != entry['artifact_sha256'][key]:
                raise ValueError('Saved round artifact changed; not reusing it')
        doc = read_document(entry['extrinsics_path'])
        if doc.get('status') != 'VALID' or doc.get('provenance', {}).get('config_sha256') != self.config_sha:
            raise ValueError('Saved round configuration or quality differs')
        for branch, keys in (('camera', ('name', 'serial', 'model', 'frame_id')),
                             ('tracker', ('role', 'frame_id')), ('frames', ('world', 'board'))):
            if any(doc[branch][key] != self.config[branch][key] for key in keys):
                raise ValueError('Saved round device/frame binding differs')
        if self.request.expected_tracker_id and doc['tracker']['tracker_id'] != self.request.expected_tracker_id:
            raise ValueError('Saved round Tracker identity differs')
        if doc['tracker']['tracker_id'] != entry['tracker_id']:
            raise ValueError('Saved round identity changed')
        transform = doc['transform']
        Transform.from_quaternion_xyzw(transform['translation_m'], transform['quaternion_xyzw'])
        validation = read_document(entry['validation_path'])
        if validation != entry['validation'] or validation.get('status') != 'PASS':
            raise ValueError('Saved independent validation did not pass')
        if validation.get('training_count', 0) < 40 or validation.get('validation_count', 0) < 10:
            raise ValueError('Saved round has insufficient independent poses')
        for key, maximum in (('translation_rms_m', .010), ('rotation_rms_deg', 1.)):
            value = validation.get(key)
            if type(value) not in (float, int) or not math.isfinite(value) or not 0 <= value <= maximum:
                raise ValueError('Saved round validation is outside acceptance thresholds')

    @staticmethod
    def check_independent(rounds):
        for i, entry in enumerate(rounds):
            for previous in rounds[:i]:
                if previous['round'] == entry['round'] or previous['source_sha256'] == entry['source_sha256']:
                    raise ValueError('Independent rounds must use distinct recorded source data')
                if not (previous['last_stamp_ns'] < entry['first_stamp_ns']
                        or entry['last_stamp_ns'] < previous['first_stamp_ns']):
                    raise ValueError('Independent round source time windows must not overlap')

    def save(self, rounds):
        self.check_independent(rounds)
        stored = []
        for entry in sorted(rounds, key=lambda value: value['round']):
            value = dict(entry)
            value['artifact_sha256'] = {key: digest(self._artifact(value[key]))
                for key in ('extrinsics_path', 'report_path', 'validation_path')}
            self._validate(value)
            stored.append(value)
        atomic_document(self.path, dict(schema_version=1, session_id=self.request.session_id,
            config_sha256=self.config_sha, rounds=stored))
        return stored

    def load(self):
        if not self.path.exists():
            return self._adopt_legacy()
        doc = read_document(self.path)
        if (doc.get('schema_version') != 1 or doc.get('session_id') != self.request.session_id
                or doc.get('config_sha256') != self.config_sha):
            raise ValueError('Saved round checkpoint does not match this session')
        rounds = doc.get('rounds')
        if not isinstance(rounds, list) or len(rounds) > 3:
            raise ValueError('Invalid saved round checkpoint')
        for entry in rounds:
            self._validate(entry)
        self.check_independent(rounds)
        return rounds

    def _adopt_legacy(self):
        """Recover already-solved v1 rounds without refitting or touching originals."""
        rounds = []
        for number in range(1, 4):
            result = self.root / f'round-{number}-result'
            validation_path = result / 'independent-validation.json'
            if not validation_path.is_file():
                continue
            validation = read_document(self._artifact(validation_path))
            if validation.get('status') != 'PASS':
                continue
            extrinsic_path = self._artifact(result / 'extrinsics.yaml')
            doc = read_document(extrinsic_path)
            bag = self._artifact(self.root / f'round-{number}' / 'bag')
            metadata = read_document(bag / 'metadata.yaml')['rosbag2_bagfile_information']
            first = metadata['starting_time']['nanoseconds_since_epoch']
            last = first + metadata['duration']['nanoseconds']
            entry = dict(round=number, round_id=f'{self.request.session_id}:{number}',
                extrinsics_path=str(extrinsic_path), report_path=str(result / 'report.json'),
                validation_path=str(validation_path), validation=validation, bag_path=str(bag),
                source_sha256=self.fingerprint(bag), first_stamp_ns=first, last_stamp_ns=last,
                tracker_id=doc['tracker']['tracker_id'], recovered_from_legacy=True,
                interval_scope='bag_recording_envelope')
            rounds.append(entry)
        return self.save(rounds) if rounds else []
