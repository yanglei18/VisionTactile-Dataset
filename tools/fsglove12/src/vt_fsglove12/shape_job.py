"""Spawn-isolated, bounded official shape fitting; no serial ownership."""
from __future__ import annotations

from copy import deepcopy
import multiprocessing as mp
import threading
import time

from . import host_clock


def _optimize(contacts, options, directory, cancel, result):
    try:
        from .shape_calibration import optimize_contacts

        def progress(snapshot):
            result.send(dict(progress=snapshot))

        report = optimize_contacts(contacts, options["checkout"], options["mano_assets"], directory,
            iterations=options.get("iterations", 500), seed=options.get("seed", 0),
            cancelled=cancel.is_set, progress=progress, hand=options.get("hand"))
        result.send(dict(report=report, error=None))
    except Exception as exc:
        result.send(dict(report=None, error=f"{type(exc).__name__}: {exc}"))
    finally:
        result.close()


class ShapeJob:
    """Frozen inputs, launcher thread, spawn worker, and safe cancellation.

    Cancellation first requests cooperation. The upstream optimizer has no
    per-iteration cancellation API; after a one-second grace period an explicitly
    cancelled child is terminated and reaped. A cancelled result can never expose
    a candidate, even when cancellation races with the worker's final message.
    """
    CANCEL_GRACE_SECONDS = 1.

    def __init__(self, contacts, options, directory):
        self.result = None
        self.progress = dict(stage="starting")
        self.done = threading.Event()
        self._context = mp.get_context("spawn")
        self._cancel = self._context.Event()
        self._thread = threading.Thread(target=self._run,
            args=(deepcopy(contacts), deepcopy(options), str(directory)),
            name="fsglove12-shape-launcher", daemon=True)
        self._thread.start()

    def _run(self, contacts, options, directory):
        from .shape_calibration import cancelled_result
        receiver, sender = self._context.Pipe(duplex=False)
        process, cancel_started = None, None
        try:
            if self._cancel.is_set():
                return
            process = self._context.Process(target=_optimize,
                args=(contacts, options, directory, self._cancel, sender),
                name="fsglove12-shape", daemon=False)
            process.start()
            sender.close()
            while process.is_alive() or receiver.poll():
                if self._cancel.is_set():
                    cancel_started = host_clock.monotonic() if cancel_started is None else cancel_started
                    self.progress = dict(stage="cancelling")
                    if host_clock.monotonic() - cancel_started >= self.CANCEL_GRACE_SECONDS:
                        if process.is_alive():
                            process.terminate()
                        break
                if receiver.poll(.05):
                    try:
                        message = receiver.recv()
                    except EOFError:
                        break
                    if "progress" in message:
                        self.progress = message["progress"]
                    else:
                        self.result = message
                        break
            if self.result is None and not self._cancel.is_set():
                self.result = dict(report=None, error="Shape optimization process returned no result.")
        except Exception as exc:
            self.result = dict(report=None, error=f"{type(exc).__name__}: {exc}")
        finally:
            sender.close()
            receiver.close()
            if process is not None and process.pid is not None:
                # Keep cancellation effective during child teardown too: a
                # child may send its result before it has finished exiting.
                while process.is_alive():
                    if self._cancel.is_set():
                        cancel_started = host_clock.monotonic() if cancel_started is None else cancel_started
                        if host_clock.monotonic() - cancel_started >= self.CANCEL_GRACE_SECONDS:
                            process.terminate()
                    process.join(.05)
                process.join()
            if self._cancel.is_set():
                self.result = dict(report=cancelled_result(), error=None)
                self.progress = dict(stage="cancelled")
            elif self.result and self.result["error"]:
                self.progress = dict(stage="failed")
            self.done.set()

    def cancel(self):
        self._cancel.set()
        if self.done.is_set():
            from .shape_calibration import cancelled_result
            self.result = dict(report=cancelled_result(), error=None)
            self.progress = dict(stage="cancelled")

    def is_alive(self):
        return self._thread.is_alive()

    def join(self, timeout=None):
        self._thread.join(timeout)
