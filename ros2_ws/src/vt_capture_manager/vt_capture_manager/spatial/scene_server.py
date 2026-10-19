"""A separate read-only, bounded scene endpoint backed by a producer cache."""
from copy import deepcopy
import os
import threading
from vt_capture_core.spatial.scene_protocol import encode_scene_message, decode_scene_message, MAX_SCENE_BYTES
from ..server import SnapshotServer


class SceneServer(SnapshotServer):
    def __init__(self, store, path):
        super().__init__(store, path, max_message_bytes=MAX_SCENE_BYTES)
        self.store = store
        self._cache_lock = threading.Lock()
        self._cache = store.snapshot()
        self._cache_error = None
        self._producer = None

    def start(self):
        super().start()
        self._producer = threading.Thread(target=self._produce, name='spatial-scene-cache', daemon=True)
        self._producer.start()
        return self

    def _produce(self):
        while not self._stop.wait(1/self.store.config.scene_hz):
            self._refresh_cache()

    def _refresh_cache(self):
        try:
            scene = self.store.snapshot()
            # Validate before exchanging caches, not inside the socket thread.
            encode_scene_message(dict(protocol='vt.scene', version=1, request_id='cache', ok=True, error=None, scene=scene))
            with self._cache_lock:
                self._cache = scene
                self._cache_error = None
        except Exception as exc:
            # Derived preview failure must neither kill its producer silently
            # nor keep presenting the last healthy scene as current live data.
            with self._cache_lock:
                self._cache_error = str(exc)[:512] or 'Scene assembly failed'

    def _reply(self, payload):
        request_id = 'invalid-request'
        error_code = 'invalid_request'
        try:
            request = decode_scene_message(payload)
            request_id = request['request_id']
            if 'command' not in request:
                raise ValueError('scene endpoint accepts requests only')
            error_code = 'scene_unavailable'
            response = dict(protocol='vt.scene', version=1, request_id=request_id, ok=True, error=None)
            command = request['command']
            if command == 'hello':
                response['hello'] = dict(manager_instance_id=self.store.manager_instance_id, manager_pid=os.getpid(),
                                         scene_epoch=self.store.scene_epoch, capabilities=['tracker', 'camera', 'cloud'],
                                         cloud=self.store.cloud_supervisor.description() if self.store.cloud_supervisor else None)
            elif command == 'describe':
                response['description'] = self.store.description()
            else:
                params = request['params']
                self.store.set_interest(params.get('viewer_id', 'anonymous'), params.get('interest', []))
                with self._cache_lock:
                    if self._cache_error is not None:
                        raise ValueError(self._cache_error)
                    scene = deepcopy(self._cache)
                if scene['sequence'] <= params.get('after_sequence', 0):
                    response['no_change'] = True
                else:
                    response['scene'] = scene
            return encode_scene_message(response)
        except (ValueError, KeyError, TypeError, RuntimeError, AttributeError, OverflowError) as exc:
            return encode_scene_message(dict(protocol='vt.scene', version=1, request_id=request_id,
                ok=False, error=dict(code=error_code, message=str(exc)[:512] or 'Invalid request')))

    def close(self):
        super().close()
        if self._producer is not None:
            self._producer.join(2.)
