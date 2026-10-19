"""Local log selection and a latched storage budget, never an input filter.

Healthy solved results with actual new source identities are retained at their
real solve rate (including 120 Hz). Only diagnostic heartbeats are limited:
1 Hz normally, state transitions immediately, at most five in any second.
Explicit capture ``force`` retains new input, not repeated render heartbeats.
Neither decision changes acquisition, VQF, pose publication, or ROS cadence.

Storage checks inspect metadata, not file contents or historical directories.
Defaults are a 2 GiB current-session budget and 2 GiB minimum free space, checked
at most once per second. This is a cached warning/stop budget, not a filesystem
reservation: writes between checks can overshoot. Nothing is deleted, rotated,
or automatically re-enabled. The runtime owns closing writers after a latch.
"""
from __future__ import annotations

from collections import deque
from copy import deepcopy
import math
import os
from pathlib import Path
import shutil
import stat
from threading import RLock


DEFAULTS = dict(diagnostics_enabled=True, diagnostic_rate_hz=1.,
    diagnostic_max_per_second=5, max_session_bytes=2 * 1024 ** 3,
    min_free_bytes=2 * 1024 ** 3, storage_check_interval_s=1.)
SEMANTICS = "local_recording_only_not_input_or_ros_throttle"
_CONTEXT_KEYS = frozenset(('profile_version', 'clear_generation', 'profile_status',
    'functional_enabled', 'session_id', 'kind', 'step', 'finger', 'phase', 'capture_id', 'active'))
_FIXED_FILES = ('raw.csv', 'orientations.jsonl', 'poses.jsonl', 'events.jsonl',
    'config.json', 'input_contract.json', 'rest.json', 'session_metadata.json', 'summary.json')


def _settings(value):
    if value is None:
        value = {}
    if not isinstance(value, dict) or set(value) - set(DEFAULTS):
        raise ValueError('local_recording must contain supported recording settings')
    result = {**DEFAULTS, **value}
    if type(result['diagnostics_enabled']) is not bool:
        raise ValueError('local_recording.diagnostics_enabled must be boolean')
    for key in ('diagnostic_max_per_second', 'max_session_bytes', 'min_free_bytes'):
        if type(result[key]) is not int or result[key] < (0 if key == 'min_free_bytes' else 1):
            raise ValueError(f'local_recording.{key} must be a nonnegative/positive integer')
    for key in ('diagnostic_rate_hz', 'storage_check_interval_s'):
        number = result[key]
        if type(number) not in (int, float) or not math.isfinite(number) or number <= 0:
            raise ValueError(f'local_recording.{key} must be finite and positive')
    if result['diagnostic_rate_hz'] > result['diagnostic_max_per_second']:
        raise ValueError('diagnostic_rate_hz cannot exceed diagnostic_max_per_second')
    if result['storage_check_interval_s'] < 1.:
        raise ValueError('storage checks must be cached for at least one second')
    return result


def _time(now):
    if type(now) not in (int, float) or not math.isfinite(now):
        raise ValueError('now must be a finite monotonic timestamp')
    return float(now)


def _context(value):
    # Progress fields (remaining_s, sample counts, render timestamps) are not
    # recording state changes. Callers should pass stable capture metadata.
    if isinstance(value, dict):
        return tuple((key, _context(item)) for key, item in sorted(value.items()) if key in _CONTEXT_KEYS)
    if isinstance(value, (list, tuple)):
        return tuple(_context(item) for item in value)
    return value if value is None or type(value) in (str, int, float, bool) else str(type(value).__name__)


class ResultRecordingPolicy:
    def __init__(self, config):
        if not isinstance(config, dict):
            raise ValueError('config must be an object')
        self.settings = _settings(config.get('local_recording'))
        self._mapped = set(map(str, config.get('node_to_slot', {})))
        self._source_ids = {}
        self._session_id = None
        self._last_now = self._last_diagnostic_at = None
        self._last_recorded_state = None
        self._diagnostic_times = deque()
        self._counts = dict(selected_total=0, selected_solved=0, selected_capture=0,
            selected_diagnostic=0, suppressed_diagnostic=0,
            suppressed_diagnostic_disabled=0, suppressed_diagnostic_hard_limit=0,
            clock_resets=0)
        self._last_decision = None

    def should_record(self, frame, pose, *, now, context=None, force=False):
        now = _time(now)
        frame, pose = frame or {}, pose or {}
        if self._last_now is not None and now < self._last_now:
            self._diagnostic_times.clear()
            self._last_diagnostic_at = None
            self._counts['clock_resets'] += 1
        self._last_now = now
        session = frame.get('session_id')
        if session is not None and session != self._session_id:
            self._source_ids.clear()
            self._session_id = session
        nodes = {str(node): data for node, data in frame.get('nodes', {}).items()
                 if not self._mapped or str(node) in self._mapped}
        identities = {node: (data.get('epoch'), data['sample_index']) for node, data in nodes.items()
                      if type(data.get('sample_index')) is int and data['sample_index'] > 0}
        new_input = False
        for node, identity in identities.items():
            previous = self._source_ids.get(node)
            advanced = (previous is None or
                        identity[0] == previous[0] and identity[1] > previous[1] or
                        identity[0] != previous[0] and
                        (type(identity[0]) is not int or type(previous[0]) is not int or identity[0] > previous[0]))
            if advanced:
                self._source_ids[node] = identity
                new_input = True
        health = frame.get('input_health') or pose.get('input_health') or {}
        blocked = set(map(str, health.get('faulted_nodes', ()))) | set(map(str, health.get('recovering_nodes', ())))
        blocked.update(str(node) for node, item in health.get('nodes', {}).items()
                       if item.get('healthy') is False)
        if self._mapped:
            blocked.intersection_update(self._mapped)
        blocked = blocked if health.get('enabled') else set()
        receive = frame.get('input_diagnostics') or {}
        input_stage = None
        if receive.get('source_kind') == 'raw_serial':
            input_stage = ('decoded' if receive.get('frames_decoded', 0) or receive.get('samples_decoded', 0)
                           else 'waiting_frame' if receive.get('bytes_received', 0) else 'no_bytes')
        state = (pose.get('reason'), bool(pose.get('pose_available')), bool(pose.get('solver_success')),
            pose.get('profile_version'), pose.get('clear_generation'), frame.get('session_id'),
            input_stage, tuple(sorted(blocked)), tuple(sorted((node, data.get('epoch')) for node, data in nodes.items())),
            _context(context))
        solved = bool(pose.get('pose_available') and pose.get('solver_success')
                      and pose.get('source_fresh') is not False and not blocked)
        if new_input and (solved or force):
            kind = 'selected_solved' if solved else 'selected_capture'
            self._counts[kind] += 1
            self._counts['selected_total'] += 1
            self._last_recorded_state = state
            self._last_decision = kind
            return True
        self._last_decision = 'suppressed_diagnostic'
        if not self.settings['diagnostics_enabled']:
            self._counts['suppressed_diagnostic_disabled'] += 1
        else:
            while self._diagnostic_times and now - self._diagnostic_times[0] >= 1. - 1e-9:
                self._diagnostic_times.popleft()
            state_changed = state != self._last_recorded_state
            due = self._last_diagnostic_at is None or now - self._last_diagnostic_at >= 1. / self.settings['diagnostic_rate_hz'] - 1e-9
            if len(self._diagnostic_times) >= self.settings['diagnostic_max_per_second']:
                self._counts['suppressed_diagnostic_hard_limit'] += 1
            elif state_changed or due:
                self._diagnostic_times.append(now)
                self._last_diagnostic_at, self._last_recorded_state = now, state
                self._counts['selected_diagnostic'] += 1
                self._counts['selected_total'] += 1
                self._last_decision = 'selected_diagnostic'
                return True
        self._counts['suppressed_diagnostic'] += 1
        return False

    def snapshot(self):
        return dict(**self._counts, last_decision=self._last_decision,
                    settings=dict(self.settings), semantics=SEMANTICS)


class SessionStorageBudget:
    def __init__(self, directory, settings=None, *, disk_usage=shutil.disk_usage):
        self.directory = Path(directory)
        self.settings = _settings(settings)
        self._disk_usage = disk_usage
        self._lock = RLock()
        self._state = dict(active=True, recording_enabled=True, latched=False,
            disabled_reason=None, disabled_detail=None, disabled_at_s=None,
            last_checked_at_s=None, session_bytes=None, free_bytes=None,
            checked_files=[], checks=0, check_in_progress=False)

    def disable(self, reason, detail=None, *, now=None):
        """First reason wins, including a concurrent writer failure."""
        if now is not None:
            now = _time(now)
        with self._lock:
            if self._state['active']:
                self._state.update(active=False, recording_enabled=False, latched=True,
                    disabled_reason=str(reason), disabled_detail=None if detail is None else str(detail),
                    disabled_at_s=now if now is not None else self._state['last_checked_at_s'])
        return False

    def check(self, *, now):
        now = _time(now)
        with self._lock:
            if not self._state['active']:
                return False
            last = self._state['last_checked_at_s']
            if self._state['check_in_progress'] or (last is not None and 0 <= now - last < self.settings['storage_check_interval_s']):
                return True
            self._state.update(last_checked_at_s=now, check_in_progress=True)
            self._state['checks'] += 1
        # Do not make acquisition-thread disable() wait for filesystem I/O.
        try:
            paths = {name: self.directory / name for name in _FIXED_FILES}
            with os.scandir(self.directory) as entries:
                for entry in entries:
                    if entry.name.startswith('profile') and entry.name.endswith('.json'):
                        paths[entry.name] = self.directory / entry.name
            total, checked = 0, []
            for name, path in paths.items():
                try:
                    info = path.stat(follow_symlinks=False)
                except FileNotFoundError:
                    continue  # Optional file or a metadata atomic replacement.
                if not stat.S_ISREG(info.st_mode):
                    raise OSError(f'expected a regular session recording file: {name}')
                total += info.st_size
                checked.append(name)
            free = self._disk_usage(self.directory).free
            if type(free) is not int or free < 0:
                raise OSError('filesystem returned an invalid free-space count')
            with self._lock:
                self._state.update(session_bytes=total, free_bytes=free, checked_files=sorted(checked))
                if total >= self.settings['max_session_bytes']:
                    self.disable('max_session_bytes', f'{total} bytes >= {self.settings["max_session_bytes"]}', now=now)
                elif free < self.settings['min_free_bytes']:
                    self.disable('min_free_bytes', f'{free} free bytes < {self.settings["min_free_bytes"]}', now=now)
        except OSError as exc:
            self.disable('storage_check_failed', f'{type(exc).__name__}: {exc}', now=now)
        finally:
            with self._lock:
                self._state['check_in_progress'] = False
        with self._lock:
            return self._state['active']

    def snapshot(self):
        with self._lock:
            return dict(**deepcopy(self._state), settings=dict(self.settings), semantics=SEMANTICS)
