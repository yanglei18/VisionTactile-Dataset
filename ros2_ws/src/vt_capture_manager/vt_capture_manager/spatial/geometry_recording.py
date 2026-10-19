"""Additive geometry sidecars for owned recordings; never edits bag contents.

The manifest's complete flag means its recorder terminal state was observed.
Geometry completeness is a separate field. Writes run outside recorder locks.
"""
from collections import OrderedDict, deque
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import threading
import time
import uuid

from vt_capture_core.spatial.transforms import RigidTransform


class GeometryRecorder:
    def __init__(self, scene_store, *, event=None):
        self.scene_store, self.event = scene_store, event
        self.errors = deque(maxlen=32)
        self._condition = threading.Condition()
        self._pending = OrderedDict()
        self._active = False
        self._stop = False
        self._sessions = {}
        self._thread = threading.Thread(target=self._run, name='spatial-geometry-recording', daemon=True)
        self._thread.start()

    def observe(self, trial, context):
        if (not context.get('session_id') or not context.get('output_path')
                or trial.get('start_request_id') != context.get('start_request_id')
                or trial.get('controller_instance_id') != context.get('controller_instance_id')
                or (trial.get('session_id') and trial['session_id'] != context['session_id'])
                or context.get('state') not in (3, 4, 5, 6)):
            return
        # Freeze bindings at observation time: a queued terminal write must not
        # pick up a calibration activated after that Recorder already stopped.
        geometry_snapshot = self.scene_store.geometry_snapshot()
        with self._condition:
            if self._stop:
                return
            key = (trial['id'], context['session_id'])
            if key not in self._pending and len(self._pending) >= 16:
                self._report('geometry recording queue full')
                return
            self._pending[key] = (deepcopy(trial), deepcopy(context), geometry_snapshot)
            self._condition.notify_all()

    def _report(self, message):
        self.errors.append(str(message)[:512])
        if self.event:
            self.event('warning', 'Spatial geometry sidecar: '+str(message)[:512], key='spatial-geometry')

    @staticmethod
    def _bytes(value):
        result = json.dumps(value, ensure_ascii=False, sort_keys=True, allow_nan=False, indent=2).encode()+b'\n'
        if len(result) > 512*1024:
            raise ValueError('geometry document exceeds 512 KiB')
        return result

    @staticmethod
    def _write(path, content, *, immutable=False):
        if path.is_symlink():
            raise ValueError('refusing symlink geometry artifact')
        if immutable:
            try:
                fd = os.open(path, os.O_CREAT | os.O_EXCL | os.O_WRONLY | os.O_NOFOLLOW, 0o600)
            except FileExistsError:
                if not path.is_file() or path.stat().st_size != len(content) or path.read_bytes() != content:
                    raise ValueError('geometry artifact collision')
                return
            with os.fdopen(fd, 'wb') as stream:
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            return
        temporary = path.with_name(path.name+'.'+uuid.uuid4().hex+'.tmp')
        try:
            with open(temporary, 'xb') as stream:
                os.chmod(temporary, 0o600)
                stream.write(content)
                stream.flush()
                os.fsync(stream.fileno())
            os.replace(temporary, path)
            fd = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
            try:
                os.fsync(fd)
            finally:
                os.close(fd)
        finally:
            temporary.unlink(missing_ok=True)

    def _record(self, trial, context, geometry_snapshot):
        cfg, scene, geometries = geometry_snapshot
        bag = Path(context['output_path'])
        if not bag.is_absolute() or any(part.is_symlink() for part in (bag, *bag.parents)):
            raise ValueError('bag output path must be absolute and have no symlinks')
        directory = bag.with_name(bag.name+'.spatial')
        key = str(directory)
        identity = (trial['id'], context['controller_instance_id'], context['session_id'], context['start_request_id'])
        if key not in self._sessions:
            # Never adopt an unrelated or partially written sidecar after restart.
            # It remains inspectable; missing geometry is explicitly unavailable.
            directory.mkdir(mode=0o700)
            manifest = dict(schema_version=1, kind='vt.spatial.geometry', trial_id=trial['id'],
                controller_instance_id=context['controller_instance_id'], session_id=context['session_id'],
                start_request_id=context['start_request_id'], manager_instance_id=self.scene_store.manager_instance_id,
                bag_path=str(bag), world_frame=cfg.world_frame, config_revision=cfg.config_revision,
                display_from_world=getattr(cfg, 'display_from_world', RigidTransform.identity()).as_dict(),
                camera_config=deepcopy(list(cfg.cameras)), reference_camera=getattr(cfg, 'reference_camera', None),
                alignment_config=deepcopy(getattr(cfg, 'alignment_document', None)),
                geometry_status='incomplete', complete=False,
                recorder_state=context['state'], revisions=[], epochs=[], identity_intervals=[], created_realtime_ns=time.time_ns(),
                notes=['Geometry is a derived sidecar; raw bags are unchanged.',
                       'Recorder completion does not establish geometric validity.'])
            self._sessions[key] = dict(identity=identity, manifest=manifest, seen=set(), bytes=0)
        session = self._sessions[key]
        if identity != session['identity']:
            raise ValueError('sidecar session ownership changed')
        manifest = session['manifest']
        if manifest['complete']:
            return
        epoch = {k: scene[k] for k in ('scene_epoch', 'time_epoch', 'coordinate_epoch', 'sync_epoch', 'map_identity')}
        if not manifest['epochs'] or any(manifest['epochs'][-1][k] != v for k, v in epoch.items()):
            if len(manifest['epochs']) >= 512:
                raise ValueError('sidecar epoch limit exceeded')
            manifest['epochs'].append(dict(epoch, observed_realtime_ns=scene['generated_realtime_ns']))
        ready = bool(cfg.cameras)
        for camera in cfg.cameras:
            name = camera['id']
            geometry = geometries[name]
            ready &= bool(geometry.get('color_intrinsics') and geometry.get('depth_intrinsics')
                          and geometry.get('depth_to_color') and geometry.get('extrinsic')
                          and geometry.get('depth_units_m'))
            self._identity_interval(manifest, name, geometry, scene)
            # Polling the same verified camera refreshes a lease, not geometry.
            # Revision numbers retain A -> invalid B -> A segment boundaries.
            revision_key = (name, geometry.get('geometry_sha256'), geometry.get('geometry_revision'),
                            geometry.get('identity_revision', 0), scene['time_epoch'],
                            scene['coordinate_epoch'], scene['scene_epoch'])
            if revision_key in session['seen']:
                continue
            content = self._bytes(geometry)
            digest = hashlib.sha256(content).hexdigest()
            if len(manifest['revisions']) >= 512 or session['bytes']+len(content) > 32*1024*1024:
                raise ValueError('geometry sidecar revision/byte limit exceeded')
            filename = f'geometry-{digest}.json'
            self._write(directory/filename, content, immutable=True)
            manifest['revisions'].append(dict(camera_id=name, geometry_revision=geometry.get('geometry_revision'),
                geometry_sha256=geometry.get('geometry_sha256'), identity_revision=geometry.get('identity_revision', 0),
                artifact=filename, artifact_sha256=digest,
                observed_realtime_ns=scene['generated_realtime_ns'], time_epoch=scene['time_epoch'],
                coordinate_epoch=scene['coordinate_epoch'], scene_epoch=scene['scene_epoch']))
            session['seen'].add(revision_key)
            session['bytes'] += len(content)
        manifest.update(updated_realtime_ns=time.time_ns(), recorder_state=context['state'],
                        geometry_status='available' if ready else 'incomplete',
                        complete=context['state'] in (5, 6) and not context.get('recorder_alive', True))
        self._write(directory/'manifest.json', self._bytes(manifest))

    @staticmethod
    def _identity_interval(manifest, name, geometry, scene):
        identity = geometry.get('identity', {})
        observed = identity.get('observed_ns')
        now = scene['generated_realtime_ns']
        revision = geometry.get('identity_revision', 0)
        generations = {key: scene[key] for key in ('time_epoch', 'coordinate_epoch', 'scene_epoch')}
        intervals = manifest['identity_intervals']
        previous = next((value for value in reversed(intervals) if value['camera_id'] == name), None)
        verified = (identity.get('verified') is True and identity.get('serial') == geometry.get('config', {}).get('serial')
                    and type(observed) is int and 0 <= now-observed <= 5_000_000_000)
        # A missed invalid -> verified transition is conservatively bounded at
        # the last observed verification, not bridged by its former lease.
        if previous is not None and previous['identity_revision'] != revision:
            invalidated = identity.get('invalidated_ns')
            boundary = invalidated if type(invalidated) is int else previous['last_observed_ns']
            previous['end_realtime_ns'] = max(previous['start_realtime_ns'], min(previous['end_realtime_ns'], boundary))
        if not verified:
            if previous is not None:
                invalidated = identity.get('invalidated_ns')
                boundary = invalidated if type(invalidated) is int else now
                previous['end_realtime_ns'] = max(previous['start_realtime_ns'], min(previous['end_realtime_ns'], boundary))
            return
        key = dict(camera_id=name, identity_revision=revision, serial=identity['serial'],
                   geometry_revision=geometry.get('geometry_revision'), geometry_sha256=geometry.get('geometry_sha256'),
                   **generations)
        same = previous is not None and all(previous.get(k) == v for k, v in key.items())
        if same and previous['start_realtime_ns'] <= observed <= previous['end_realtime_ns']:
            previous['last_observed_ns'] = max(previous['last_observed_ns'], observed)
            previous['end_realtime_ns'] = max(previous['end_realtime_ns'], observed+5_000_000_000)
            return
        if len(intervals) >= 512:
            raise ValueError('identity interval limit exceeded')
        intervals.append(dict(key, verified=True, start_realtime_ns=observed,
                              last_observed_ns=observed, end_realtime_ns=observed+5_000_000_000))

    def _run(self):
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._pending or self._stop)
                if not self._pending:
                    return
                _, item = self._pending.popitem(last=False)
                self._active = True
            try:
                self._record(*item)
            except (OSError, ValueError, TypeError, KeyError) as exc:
                self._report(exc)
            finally:
                with self._condition:
                    self._active = False
                    self._condition.notify_all()

    def flush(self, timeout=5.):
        with self._condition:
            if not self._condition.wait_for(lambda: not self._pending and not self._active, timeout):
                raise TimeoutError('geometry sidecar writer did not flush')

    def close(self):
        with self._condition:
            self._stop = True
            self._condition.notify_all()
        self._thread.join(5.)
        if self._thread.is_alive():
            raise RuntimeError('geometry sidecar writer did not stop')
