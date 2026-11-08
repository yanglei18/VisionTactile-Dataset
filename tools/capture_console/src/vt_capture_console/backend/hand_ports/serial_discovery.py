"""Serial discovery and explicit protocol identification outside the UI thread.

One daemon worker keeps only the latest immutable result. The provider lists
metadata; it must never open a device. Only ``identify`` requests a bounded
protocol probe of idle ports. Neither periodic nor manual refresh opens ports.
"""
from __future__ import annotations

from dataclasses import dataclass, replace
import math
import os
import threading

from .port_identity import port_identity


@dataclass(frozen=True)
class PortClassification:
    path: str
    role: str
    detail: str = ""


@dataclass(frozen=True)
class PortSnapshot:
    choices: tuple[tuple[str, str], ...] = ()
    error: str | None = None
    classifications: tuple[PortClassification, ...] = ()
    identifying: bool = False
    identities: tuple[tuple[str, str | None], ...] = ()


def _choices(entries):
    by_path = {}
    for entry in entries:
        path, label = entry["path"], entry.get("label", entry["path"])
        if not isinstance(path, str) or not isinstance(label, str):
            raise ValueError("Serial port paths and labels must be strings")
        if path:
            by_path.setdefault(os.path.normcase(path), (path, label or path))
    return tuple(by_path[key] for key in sorted(by_path))


def _probe_port(*args, **kwargs):
    from .port_probe import probe_port
    return probe_port(*args, **kwargs)


class SerialPortDiscovery:
    """Periodically enumerate metadata; coalesce manual requests into one wakeup.

    ``start`` is idempotent; a stopped instance is never restarted. ``refresh``
    only wakes its worker and ``latest`` never waits for an enumeration. Failed
    scans preserve the last successful choices until a later scan recovers.
    """

    def __init__(self, provider, *, interval_s=1.0, probe=None):
        if not callable(provider):
            raise TypeError("A serial port listing provider is required")
        if isinstance(interval_s, bool) or not math.isfinite(interval_s) or interval_s <= 0:
            raise ValueError("The discovery interval must be finite and positive")
        self._provider = provider
        self._interval_s = interval_s
        self._lock = threading.Lock()
        self._wake = threading.Event()
        self._stop = threading.Event()
        self._thread = None
        self._snapshot = PortSnapshot()
        self._probe = probe
        self._identification_request = None

    def identify(self, *, excluded=(), baudrates=(115200,)):
        """Queue one explicit probe pass; never queue another while it runs."""
        excluded = frozenset(port_identity(port) for port in excluded if port)
        baudrates = tuple(dict.fromkeys(baudrates))
        if not 1 <= len(baudrates) <= 2 or any(type(rate) is not int or rate <= 0 for rate in baudrates):
            raise ValueError("Identification requires one or two positive baud rates")
        with self._lock:
            if self._stop.is_set() or self._thread is None or self._snapshot.identifying:
                return False
            self._identification_request = (excluded, baudrates)
            self._snapshot = replace(self._snapshot, identifying=True)
            self._wake.set()
            return True

    def _set_classification(self, result):
        # Caller holds _lock. One immutable latest value per present port.
        results = {item.path: item for item in self._snapshot.classifications}
        if results.get(result.path) == result:
            return
        results[result.path] = result
        self._snapshot = replace(self._snapshot, classifications=tuple(results[key] for key in sorted(results)))

    def start(self):
        with self._lock:
            if self._thread is not None or self._stop.is_set():
                return
            self._thread = threading.Thread(target=self._run, name="hand-viewer-port-discovery", daemon=True)
            self._thread.start()

    def refresh(self):
        with self._lock:
            if not self._stop.is_set():
                self._wake.set()

    def latest(self):
        with self._lock:
            return self._snapshot

    def stop(self):
        with self._lock:
            self._stop.set()
            self._wake.set()

    def join(self, timeout=0):
        with self._lock:
            thread = self._thread
        if thread is None:
            return True
        if thread is threading.current_thread():
            return False
        thread.join(timeout)
        return not thread.is_alive()

    def _enumerate(self):
        try:
            entries = tuple(self._provider())
            choices, error = _choices(entries), None
            identities = {entry["path"]: entry.get("identity") for entry in entries}
            if any(value is not None and not isinstance(value, str) for value in identities.values()):
                raise ValueError("Port identity must be a string or None")
        except Exception as exc:
            choices, error = None, (str(exc) or type(exc).__name__)[:2048]
        with self._lock:
            if self._stop.is_set():
                return False
            retained = dict(choices) if choices is not None else dict(self._snapshot.choices)
            previous = dict(self._snapshot.choices)
            previous_identities = dict(self._snapshot.identities)
            identities = identities if choices is not None else previous_identities
            # Without a stable identity, evidence is only a temporary observation
            # of this port, not a remembered device binding (the UI labels it so).
            classifications = tuple(item for item in self._snapshot.classifications
                                    if item.path in retained and retained[item.path] == previous.get(item.path)
                                    and identities.get(item.path) == previous_identities.get(item.path))
            self._snapshot = replace(self._snapshot, choices=tuple(retained.items()),
                                     error=error, classifications=classifications,
                                     identities=tuple((path, identities.get(path)) for path in retained))
        return error is None

    def _identify(self, request):
        excluded, baudrates = request
        probe = self._probe or _probe_port
        for path, _label in self.latest().choices:
            if self._stop.is_set():
                return
            if port_identity(path) in excluded:
                # Keep any earlier probe result; never infer a type from the
                # selected input or from isolated decoded display samples.
                continue
            with self._lock:
                if self._stop.is_set():
                    return
                self._set_classification(PortClassification(path, "identifying"))
            try:
                result = probe(path, baudrates=baudrates, cancel=self._stop)
                classification = PortClassification(path, result.role, result.detail[:2048])
            except Exception as exc:
                classification = PortClassification(path, "error", (str(exc) or type(exc).__name__)[:2048])
            with self._lock:
                if self._stop.is_set():
                    return
                self._set_classification(classification)

    def _run(self):
        while not self._stop.is_set():
            self._wake.clear()
            enumerated = self._enumerate()
            with self._lock:
                request, self._identification_request = self._identification_request, None
            if request is not None:
                if enumerated:
                    self._identify(request)
                    # Never apply a completed probe to an unplugged device.
                    self._enumerate()
                with self._lock:
                    if self._stop.is_set():
                        return
                    self._snapshot = replace(self._snapshot, identifying=False)
            self._wake.wait(self._interval_s)
