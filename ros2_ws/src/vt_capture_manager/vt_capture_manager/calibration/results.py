"""Verify worker evidence before immutable storage and active-pointer commit."""
from __future__ import annotations

from copy import deepcopy
import math
from pathlib import Path
import re
import time
import uuid

from vt_capture_core.spatial.transforms import RigidTransform

from .storage import atomic_json, checked_path, digest_document, private_directory, read_document


def validate_extrinsic(document: dict, expected: dict | None = None, tracker_id: str = '') -> None:
    if document.get('schema_version') != 1 or document.get('status') != 'VALID':
        raise ValueError('extrinsic schema 1 and status VALID required')
    camera, tracker, transform = document['camera'], document['tracker'], document['transform']
    if (not isinstance(camera.get('name'), str)
            or re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,127}', camera['name']) is None):
        raise ValueError('invalid camera name')
    for record, keys in ((camera, ('serial', 'model', 'frame_id')), (tracker, ('role', 'tracker_id', 'frame_id'))):
        if any(not isinstance(record.get(key), str) or not record[key] or len(record[key]) > 256 for key in keys):
            raise ValueError('extrinsic identity is incomplete')
    if transform['semantics'] != 'parent_from_child' or transform['parent_frame'] != tracker['frame_id'] or transform['child_frame'] != camera['frame_id']:
        raise ValueError('extrinsic transform frame mismatch')
    RigidTransform(transform['translation_m'], transform['quaternion_xyzw'])
    if tracker_id and tracker['tracker_id'] != tracker_id:
        raise ValueError('calibration Tracker identity changed')
    if expected is not None:
        for key in ('name', 'serial', 'model', 'frame_id'):
            if camera[key] != expected['camera'][key]:
                raise ValueError(f'calibration camera {key} mismatch')
        for key in ('role', 'frame_id'):
            if tracker[key] != expected['tracker'][key]:
                raise ValueError(f'calibration Tracker {key} mismatch')
        if document['frames']['world'] != expected['frames']['world']:
            raise ValueError('calibration world frame mismatch')
    digest_document(document)


def _metric(value, maximum, label):
    if type(value) not in (int, float) or not math.isfinite(value) or not 0 <= value <= maximum:
        raise ValueError(f'{label} is outside acceptance thresholds')


def validate_candidate(result: dict, *, output_root: Path, session_id: str, config: dict, tracker_id: str) -> tuple[dict, dict]:
    """Recheck all acceptance evidence, independently of the worker READY flag."""
    def artifact(path):
        return read_document(path, within=output_root)
    manifest = artifact(result['manifest_path'])
    if (manifest.get('schema_version') != 1 or manifest.get('session_id') != session_id
            or manifest.get('policy_version') != 'guided-v1'
            or type(manifest.get('activation_eligible')) is not bool):
        raise ValueError('unsupported calibration acceptance manifest')
    if checked_path(result['extrinsics_path'], within=output_root) != checked_path(manifest['selected_extrinsics_path'], within=output_root):
        raise ValueError('selected calibration does not match manifest')
    selected = artifact(result['extrinsics_path'])
    validate_extrinsic(selected, config, tracker_id)
    rounds = manifest.get('rounds')
    if not isinstance(rounds, list) or not 1 <= len(rounds) <= 3:
        raise ValueError('calibration requires one to three independent rounds')
    intervals, hashes, outputs, indices = [], set(), set(), set()
    for entry in rounds:
        round_document = artifact(entry['extrinsics_path'])
        validate_extrinsic(round_document, config, tracker_id)
        if round_document['tracker']['tracker_id'] != selected['tracker']['tracker_id']:
            raise ValueError('rounds do not share the same Tracker identity')
        artifact(entry['report_path'])
        validation = artifact(entry['validation_path'])
        if validation != entry['validation']:
            raise ValueError('independent validation differs from its manifest')
        if validation.get('status') != 'PASS':
            raise ValueError('independent validation did not pass')
        if type(validation.get('training_count')) is not int or validation['training_count'] < 40:
            raise ValueError('insufficient solve poses')
        if type(validation.get('validation_count')) is not int or validation['validation_count'] < 10:
            raise ValueError('insufficient independent validation poses')
        _metric(validation.get('translation_rms_m'), .010, 'validation translation')
        _metric(validation.get('rotation_rms_deg'), 1., 'validation rotation')
        stamp_first, stamp_last = entry.get('first_stamp_ns'), entry.get('last_stamp_ns')
        if type(stamp_first) is not int or type(stamp_last) is not int or not 0 < stamp_first < stamp_last:
            raise ValueError('invalid round timestamp interval')
        intervals.append((stamp_first, stamp_last))
        fingerprint = entry.get('source_sha256')
        if not isinstance(fingerprint, str) or not re.fullmatch(r'[0-9a-f]{64}', fingerprint) or fingerprint in hashes:
            raise ValueError('rounds must have distinct source fingerprints')
        hashes.add(fingerprint)
        output = checked_path(entry['extrinsics_path'], within=output_root)
        if output in outputs or type(entry.get('round')) is not int or entry['round'] in indices:
            raise ValueError('round outputs and indices must be distinct')
        outputs.add(output)
        indices.add(entry['round'])
    if checked_path(result['extrinsics_path'], within=output_root) not in outputs:
        raise ValueError('selected calibration is not an accepted round')
    intervals.sort()
    if any(left[1] >= right[0] for left, right in zip(intervals, intervals[1:])):
        raise ValueError('independent rounds must not overlap')
    if manifest['activation_eligible']:
        if len(rounds) != 3:
            raise ValueError('automatic activation requires three independent rounds')
        repeatability = artifact(manifest['repeatability_path'])
        if (repeatability.get('status') != 'PASS' or repeatability.get('run_count') != 3
                or result.get('repeatability_path') != manifest['repeatability_path']):
            raise ValueError('three-round repeatability did not pass')
        metrics = repeatability.get('metrics', {})
        _metric(metrics.get('maximum_pairwise_translation_m'), .005, 'repeatability translation')
        _metric(metrics.get('maximum_pairwise_rotation_deg'), .5, 'repeatability rotation')
    return selected, manifest


class ResultStore:
    """The active JSON reference is the sole durable commit point."""
    def __init__(self, root: Path):
        self.root = private_directory(root)
        self.active_root = private_directory(self.root / 'active')
        self.versions = private_directory(self.root / 'versions')

    def load_active(self) -> list[tuple[dict, dict]]:
        results = []
        for path in sorted(self.active_root.glob('*.json')):
            manifest = read_document(path, within=self.root)
            document = read_document(manifest['extrinsics_path'], within=self.versions)
            validate_extrinsic(document)
            if (manifest.get('schema_version') != 1 or manifest.get('sha256') != digest_document(document)
                    or manifest.get('camera_name') != document['camera']['name'] or path.stem != document['camera']['name']):
                raise ValueError('active calibration manifest does not match its immutable result')
            results.append((document, manifest))
        return results

    def read_revision(self, camera_name: str, revision: str) -> tuple[dict, dict, dict]:
        """Return a verified historical document, activation manifest and acceptance."""
        if not isinstance(camera_name, str) or not isinstance(revision, str):
            raise ValueError('invalid calibration revision')
        if not re.fullmatch(r'[A-Za-z_][A-Za-z0-9_]{0,127}', camera_name) or not re.fullmatch(r'[0-9a-f]{32}', revision):
            raise ValueError('invalid calibration revision')
        source = self.versions / camera_name / revision
        manifest = read_document(source / 'manifest.json', within=self.versions)
        document = read_document(source / 'extrinsics.yaml', within=self.versions)
        acceptance = read_document(source / 'acceptance.json', within=self.versions)
        validate_extrinsic(document)
        if (manifest['sha256'] != digest_document(document) or document['camera']['name'] != camera_name
                or manifest['revision'] != revision):
            raise ValueError('historical calibration integrity mismatch')
        return document, manifest, acceptance

    def restore_revision(self, camera_name: str, revision: str, *, expected_current_revision: str,
                         operation_session_id: str | None = None) -> dict:
        """Restore through a new audit revision under the caller's admission lock."""
        document, _, acceptance = self.read_revision(camera_name, revision)
        current = read_document(self.active_root / f'{camera_name}.json')
        if current['revision'] != expected_current_revision:
            raise ValueError('active calibration changed before restoration')
        if operation_session_id is not None:
            acceptance = deepcopy(acceptance)
            acceptance['calibration_session_id'] = acceptance.get('calibration_session_id', acceptance['session_id'])
            acceptance['session_id'] = operation_session_id
        return self.commit(document, acceptance, restored_from_revision=revision)

    def commit(self, document: dict, acceptance: dict, *, restored_from_revision: str | None = None) -> dict:
        validate_extrinsic(document)
        name = document['camera']['name']
        current_path = self.active_root / f'{name}.json'
        previous_revision = read_document(current_path)['revision'] if current_path.exists() else ''
        revision = uuid.uuid4().hex
        destination = private_directory(self.versions / name / revision)
        extrinsics_path = destination / 'extrinsics.yaml'
        # JSON is a strict YAML subset, so schema 1 readers remain compatible.
        atomic_json(extrinsics_path, document)
        atomic_json(destination / 'acceptance.json', acceptance)
        manifest = dict(schema_version=1, camera_name=name, revision=revision,
                        sha256=digest_document(document), extrinsics_path=str(extrinsics_path),
                        acceptance_path=str(destination / 'acceptance.json'),
                        source_session_id=acceptance['session_id'],
                        calibration_session_id=acceptance.get('calibration_session_id', acceptance['session_id']),
                        activated_ns=time.time_ns(),
                        topic=f'/calibration/{name}/tracker_from_camera',
                        tracker_role=document['tracker']['role'], tracker_id=document['tracker']['tracker_id'],
                        camera_serial=document['camera']['serial'], quality_grade=document['quality']['grade'])
        manifest['previous_revision'] = previous_revision
        if restored_from_revision is not None:
            manifest['restored_from_revision'] = restored_from_revision
        atomic_json(destination / 'manifest.json', manifest)
        atomic_json(self.active_root / f'{name}.json', manifest)
        return deepcopy(manifest)
