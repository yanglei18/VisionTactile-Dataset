"""Owned offline decoder transport shared by topic and spatial playback.

A decoder process is one session. Request identifiers increase for its entire
lifetime, including cancelled requests. A late reply can therefore be drained
without being mistaken for the next seek. Every send/read/drain uses the same
deadline and cancellation callback; buffered replies do not bypass either.
"""
from __future__ import annotations

import json
import os
from pathlib import Path
import selectors
import subprocess
import threading
import time


MAX_REPLY_BYTES = 4 * 1024 * 1024
MAX_REQUEST_BYTES = 65536
FRAME_TIMEOUT_S = 15.
METADATA_TIMEOUT_S = 120.


def decode_json(data):
    def constant(value):
        raise ValueError("nonfinite JSON value: " + value)

    def pairs(items):
        result = {}
        for key, value in items:
            if key in result:
                raise ValueError("duplicate JSON key: " + key)
            result[key] = value
        return result

    return json.loads(data, parse_constant=constant, object_pairs_hook=pairs)


class ReplayDecoder:
    """Single-reader, bounded JSON-lines IPC; close is safe from the UI thread."""

    def __init__(self, worker, path, cancelled=lambda: False):
        from .workstation_settings import repository_root
        if worker not in {"spatial_replay_worker", "topic_replay_worker"}:
            raise ValueError("Unsupported playback worker")
        root = repository_root()
        script = '''set -e
source /opt/ros/jazzy/setup.bash
if [ -f "$1/ros2_ws/install/setup.bash" ]; then source "$1/ros2_ws/install/setup.bash"; fi
export PYTHONPATH="$1/ros2_ws/src/vt_capture_core:$1/ros2_ws/src/vt_capture_manager:$1/ros2_ws/src/vt_tactile_capture:$1/tools/multisensor_alignment/src${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m "$3" "$2"
'''
        environment = dict(os.environ)
        for key in ("PYTHONPATH", "PYTHONHOME", "VIRTUAL_ENV", "QT_PLUGIN_PATH", "QT_QPA_PLATFORM_PLUGIN_PATH"):
            environment.pop(key, None)
        self._process = subprocess.Popen(
            ("/bin/bash", "-c", script, "vt-replay", str(root),
             str(Path(path).expanduser().absolute()), "vt_multisensor_alignment." + worker),
            cwd=root, env=environment, stdin=subprocess.PIPE, stdout=subprocess.PIPE,
            stderr=subprocess.PIPE, bufsize=0, start_new_session=True)
        self._buffer = bytearray()
        self._stderr = bytearray()
        self._stderr_lock = threading.Lock()
        self._close_lock = threading.Lock()
        self._closed = False
        self._request_id = 0
        self._cancelled = cancelled
        self._error_thread = threading.Thread(target=self._read_errors, name="replay-errors", daemon=True)
        try:
            os.set_blocking(self._process.stdin.fileno(), False)
            self._error_thread.start()
        except BaseException:
            self.close()
            raise

    def set_cancelled(self, callback):
        self._cancelled = callback

    def _remaining(self, deadline):
        if self._closed or self._cancelled():
            raise InterruptedError("Playback load cancelled")
        remaining = deadline - time.monotonic()
        if remaining <= 0:
            raise TimeoutError("Playback decoder timed out")
        return remaining

    def _read_errors(self):
        try:
            while chunk := self._process.stderr.read(1024):
                with self._stderr_lock:
                    self._stderr.extend(chunk)
                    del self._stderr[:-4096]
        except (OSError, ValueError):
            pass

    def metadata(self):
        return self._reply(time.monotonic() + METADATA_TIMEOUT_S).get("metadata")

    def request(self, fields):
        deadline = time.monotonic() + FRAME_TIMEOUT_S
        self._remaining(deadline)
        self._request_id += 1
        request = json.dumps(dict(fields, request_id=self._request_id), allow_nan=False).encode() + b"\n"
        if len(request) > MAX_REQUEST_BYTES:
            raise ValueError("Playback request exceeds 64 KiB")
        self._write(request, deadline)
        return self._reply(deadline, self._request_id)

    def _write(self, data, deadline):
        with selectors.DefaultSelector() as selector:
            selector.register(self._process.stdin, selectors.EVENT_WRITE)
            view = memoryview(data)
            try:
                while view:
                    remaining = self._remaining(deadline)
                    if selector.select(min(.05, remaining)):
                        try:
                            view = view[os.write(self._process.stdin.fileno(), view):]
                        except BlockingIOError:
                            continue
            except BaseException:
                if len(view) != len(data):
                    # An incomplete JSON line cannot be reused by another seek.
                    self.close()
                raise

    def _reply(self, deadline, request_id=None):
        while True:
            response = self._read_reply(deadline)
            if not isinstance(response, dict):
                raise ValueError("Invalid playback response")
            if request_id is not None:
                received = response.get("request_id")
                if type(received) is not int or received <= 0 or received > request_id:
                    raise ValueError("Invalid playback response request identifier")
                if received < request_id:
                    # Success and failure replies from obsolete selections are
                    # equally irrelevant. Draining cannot extend the deadline.
                    continue
            if response.get("ok") is not True:
                raise ValueError(str(response.get("error", "Invalid playback response"))[:1600])
            return response

    def _read_reply(self, deadline):
        with selectors.DefaultSelector() as selector:
            selector.register(self._process.stdout, selectors.EVENT_READ)
            while True:
                remaining = self._remaining(deadline)
                newline = self._buffer.find(b"\n")
                if newline >= 0:
                    line = self._buffer[:newline]
                    del self._buffer[:newline + 1]
                    return decode_json(line)
                if len(self._buffer) >= MAX_REPLY_BYTES:
                    raise ValueError("Playback response exceeds 4 MiB")
                if not selector.select(min(.05, remaining)):
                    continue
                chunk = os.read(self._process.stdout.fileno(), min(65536, MAX_REPLY_BYTES - len(self._buffer)))
                if not chunk:
                    self._error_thread.join(timeout=.05)
                    with self._stderr_lock:
                        detail = self._stderr.decode("utf-8", errors="replace")[-1600:].strip()
                    raise RuntimeError("Playback decoder exited" + (": " + detail if detail else ""))
                self._buffer.extend(chunk)

    def close(self):
        with self._close_lock:
            if self._closed:
                return
            self._closed = True
            process = self._process
            if process.poll() is None:
                try:
                    process.terminate()
                except ProcessLookupError:
                    pass

        def reap():
            try:
                try:
                    process.wait(timeout=.5)
                except subprocess.TimeoutExpired:
                    process.kill()
                    process.wait(timeout=1.)
            finally:
                for stream in (process.stdin, process.stdout, process.stderr):
                    if stream:
                        stream.close()
        threading.Thread(target=reap, name="replay-reaper", daemon=True).start()
