"""Bounded latest-only surface preview, isolated from measurement processing.

Only preview requests may be superseded. This transport never carries raw
samples or measured poses for recording. Each surface keeps its source pose
and calibration context; receiving it cannot relabel an older result as fresh.
"""
from __future__ import annotations

import multiprocessing as mp
import os
import pickle
import time


class _LatestSlot:
    """One fixed-capacity message with nonblocking readers and writers."""
    def __init__(self, context, capacity):
        self.data = context.RawArray('B', capacity)
        self.size = context.RawValue('I', 0)
        self.revision = context.RawValue('Q', 0)
        self.lock = context.Lock()

    def publish(self, value):
        data = pickle.dumps(value, protocol=pickle.HIGHEST_PROTOCOL)
        if len(data) > len(self.data):
            raise ValueError('Preview message exceeds fixed mailbox capacity')
        if not self.lock.acquire(False):
            return False
        try:
            memoryview(self.data).cast('B')[:len(data)] = data
            self.size.value = len(data)
            self.revision.value += 1
            return True
        finally:
            self.lock.release()

    def take(self, after):
        if self.revision.value == after or not self.lock.acquire(False):
            return after, None
        try:
            revision = self.revision.value
            data = bytes(memoryview(self.data).cast('B')[:self.size.value]) if revision != after else None
        finally:
            self.lock.release()
        return revision, pickle.loads(data) if data else None


def _renderer(assets_root, hand):
    # Import/model construction happens only in the display process.
    import torch
    from .mano import ManoBackend
    torch.set_num_threads(1)
    return ManoBackend(assets_root, hand, compact_live_geometry=True)


def _run(assets_root, hand, incoming, outgoing, stop, owner_pid, factory):
    backend = factory(assets_root, hand)
    revision, next_render = 0, 0.
    completed = superseded = 0
    while not stop.is_set() and os.getppid() == owner_pid:
        delay = next_render - time.monotonic()
        if delay > 0:
            stop.wait(min(delay, .03))
            continue
        current, request = incoming.take(revision)
        if request is None:
            stop.wait(.002)
            continue
        superseded += max(0, current - revision - 1)
        revision = current
        started = time.perf_counter()
        try:
            pose = request['pose']
            geometry = backend.render_mano(pose['mano_pose_rotvec'], pose.get('shape_params'))
            if pose.get('pose_postprocessing'):
                geometry['provenance'] = {**geometry.get('provenance', {}),
                    'pose_postprocessing': pose['pose_postprocessing'], 'pose_algorithm': pose.get('algorithm'),
                    'pose_input': 'upstream_axis_layer_composed_functional_constraint'}
            if pose.get('display_only') or pose.get('held_slots') or pose.get('unknown_slots'):
                geometry = {**geometry, 'display_only': True,
                    'provenance': {**geometry.get('provenance', {}), 'display_only': True,
                                   'shape_preview': pose.get('shape_preview_mode', 'current')}}
            error = ''
        except Exception as exc:
            geometry, error = None, f'{type(exc).__name__}: {exc}'
        completed += 1
        outgoing.publish({**request, 'geometry': geometry, 'error': error,
                          'render_ms': (time.perf_counter() - started) * 1000.,
                          'completed': completed, 'superseded': superseded})
        next_render = time.monotonic() + 1. / 30.


class GeometryPreviewWorker:
    def __init__(self, assets_root, hand, *, renderer_factory=_renderer):
        context = mp.get_context('spawn')
        self.incoming = _LatestSlot(context, 256 * 1024)
        self.outgoing = _LatestSlot(context, 2 * 1024 * 1024)
        self._stop = context.Event()
        self._revision = self._submitted = self._busy = 0
        self._last = {}
        self._process = context.Process(target=_run,
            args=(str(assets_root), hand, self.incoming, self.outgoing, self._stop,
                  os.getpid(), renderer_factory), name=f'{hand}-surface-preview', daemon=True)
        self._process.start()

    def submit(self, request):
        accepted = self.incoming.publish(request)
        self._submitted += int(accepted)
        self._busy += int(not accepted)
        return accepted

    def poll(self):
        self._revision, result = self.outgoing.take(self._revision)
        if result is not None:
            self._last = {key: result[key] for key in ('render_ms', 'completed', 'superseded')}
        return result

    def snapshot(self):
        return dict(submitted=self._submitted, mailbox_busy=self._busy,
                    alive=self._process.is_alive(), capacity=1, **self._last)

    def close(self):
        self._stop.set()
        self._process.join(.2)
        if self._process.is_alive():
            self._process.terminate()
            self._process.join(.2)
