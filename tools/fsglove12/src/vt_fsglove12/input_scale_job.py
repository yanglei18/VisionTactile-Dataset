"""Bounded, spawn-isolated calibration analysis; never owns a serial port.

Spawning and transferring the frozen raw snapshot happens on a launcher thread,
not the acquisition/solve thread. Cancellation is cooperative. The controller
still checks the capture generation before committing any returned evidence.
"""
from __future__ import annotations

import multiprocessing as mp
import threading


def _analyze(samples, config, holds, directory, cancel, result):
    try:
        from .input_scale_check import run_input_scale_check
        report = run_input_scale_check(samples, config, holds, directory,
                                       cancelled=cancel.is_set)
        result.send(dict(report=report, error=None))
    except Exception as exc:
        result.send(dict(report=None, error=f"{type(exc).__name__}: {exc}"))
    finally:
        result.close()


class InputScaleJob:
    """One frozen analysis, with a thread-compatible join/is_alive interface."""

    def __init__(self, samples, config, holds, directory):
        self.result = None
        self.done = threading.Event()
        self._context = mp.get_context("spawn")
        self._cancel = self._context.Event()
        self._thread = threading.Thread(target=self._run,
            args=(samples, config, holds, str(directory)),
            name="fsglove12-input-scale-launcher", daemon=True)
        self._thread.start()

    def _run(self, samples, config, holds, directory):
        receiver, sender = self._context.Pipe(duplex=False)
        process = None
        try:
            if self._cancel.is_set():
                return
            process = self._context.Process(target=_analyze,
                args=(samples, config, holds, directory, self._cancel, sender),
                name="fsglove12-input-scale", daemon=False)
            process.start()
            sender.close()
            # Drain before joining: a full result pipe must never deadlock exit.
            while process.is_alive() or receiver.poll():
                if receiver.poll(.05):
                    self.result = receiver.recv()
                    break
            if self.result is None:
                self.result = dict(report=None, error="尺度检查进程未返回结果。")
        except Exception as exc:
            self.result = dict(report=None, error=f"{type(exc).__name__}: {exc}")
        finally:
            sender.close()
            receiver.close()
            if process is not None and process.pid is not None:
                process.join()
            self.done.set()

    def cancel(self):
        self._cancel.set()

    def is_alive(self):
        return self._thread.is_alive()

    def join(self, timeout=None):
        self._thread.join(timeout)
