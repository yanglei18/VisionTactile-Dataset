"""One startup-only GC boundary for an isolated compute/benchmark process.

This freezes ALL currently tracked objects, not selectively model objects.
Call only after model/library initialization and before constructing a runtime,
loading recorded profiles/frames, or starting acquisition/publication threads.
Reference counting and automatic GC of newly allocated objects remain enabled.
Cycles containing frozen objects can survive until close/unfreeze; callers must
not treat this as a leak cure or use it again at session/calibration boundaries.
"""
from __future__ import annotations

import gc
import multiprocessing as mp
import os
import threading


_MODULE_PID = os.getpid()
_STARTUP_USED = False
_ACTIVE_OWNER = None
_SCOPES = frozenset({"compute_child", "offline_benchmark"})


class StartupGcFreeze:
    """Own exactly one process-wide startup freeze, never another owner's thaw.

    ``compute_child`` requires an actual multiprocessing child. The explicit
    ``offline_benchmark`` scope is for a fresh dedicated benchmark interpreter,
    not an interactive/in-process benchmark. Both require the main Python
    thread with no other Python threads. Caller ordering is part of the API:
    the helper cannot identify historical frames hidden in arbitrary objects.

    Skip incompatible/pre-existing policy rather than change its enabled state
    or thresholds. A successful start first collects initialization garbage,
    then freezes once. Close must follow cooperative worker shutdown; it only
    unfreezes and never forces a shutdown/hot-loop collection. No other code
    may call freeze/unfreeze while this token owns the process-wide policy.

    ``allow_bootstrap_permanent`` is an explicit fresh-interpreter commitment:
    construct the token BEFORE loading models/application data. CPython 3.12
    itself puts immortal objects in its permanent generation. When authorized,
    accept only that captured bootstrap count, unchanged through model loading.
    This is not proof of object identity and must not be used to adopt a prior
    application's freeze. Close also moves bootstrap immortals back to gen2;
    their immortal status remains and GC excludes them again on collection.
    Exact prior generation membership is not restored.
    """

    def __init__(self, *, scope, allow_bootstrap_permanent=False):
        self.scope, self._pid = scope, os.getpid()
        self._allow_bootstrap_permanent = allow_bootstrap_permanent is True
        self._started = self._active = self._applied = self._closed = False
        self._skip_reason = self._close_refused_reason = None
        self._collect_called = self._freeze_called = self._unfreeze_called = False
        self._collected = self._frozen_at_start = None
        self._enabled_before = gc.isenabled()
        self._thresholds_before = gc.get_threshold()
        self._preexisting_frozen = gc.get_freeze_count()

    def _stage_issue(self):
        if os.getpid() != self._pid or os.getpid() != _MODULE_PID:
            return "foreign_process"
        if self.scope not in _SCOPES:
            return "unsupported_scope"
        if self.scope == "compute_child" and mp.parent_process() is None:
            return "not_a_compute_child"
        if threading.current_thread() is not threading.main_thread():
            return "not_main_thread"
        if threading.active_count() != 1:
            return "python_workers_already_started"
        if not gc.isenabled() or gc.get_threshold()[0] == 0:
            return "automatic_gc_not_enabled"
        permitted = self._preexisting_frozen if self._allow_bootstrap_permanent else 0
        if gc.get_freeze_count() != permitted:
            return "preexisting_frozen_objects"
        return None

    def start(self):
        global _STARTUP_USED, _ACTIVE_OWNER
        if self._started:
            return self
        self._started = True
        if self._closed:
            self._skip_reason = "closed_before_start"
            return self
        self._skip_reason = self._stage_issue()
        if self._skip_reason is None and _STARTUP_USED:
            self._skip_reason = "startup_boundary_already_used"
        if self._skip_reason is not None:
            return self
        # Claim the only boundary before collecting: finalizers must not cause
        # a second helper to freeze objects during this startup operation.
        _STARTUP_USED = True
        self._collect_called = True
        self._collected = gc.collect(2)
        self._skip_reason = self._stage_issue()
        if (self._skip_reason is None and
                (gc.isenabled() != self._enabled_before or gc.get_threshold() != self._thresholds_before)):
            self._skip_reason = "gc_policy_changed_during_startup"
        if self._skip_reason is not None:
            return self
        gc.freeze()
        self._freeze_called = self._applied = self._active = True
        self._frozen_at_start = gc.get_freeze_count()
        _ACTIVE_OWNER = self
        return self

    def close(self):
        global _ACTIVE_OWNER
        if not self._active:
            self._closed = True
            return
        if os.getpid() != self._pid or _ACTIVE_OWNER is not self:
            self._close_refused_reason = "foreign_process_or_owner"
            return
        gc.unfreeze()
        self._unfreeze_called = True
        self._active, self._closed = False, True
        _ACTIVE_OWNER = None

    def snapshot(self):
        return dict(scope=self.scope, owner_pid=self._pid,
            applied=self._applied, active=self._active, closed=self._closed,
            skip_reason=self._skip_reason, close_refused_reason=self._close_refused_reason,
            automatic_gc_enabled_before=self._enabled_before, automatic_gc_enabled_after=gc.isenabled(),
            thresholds_before=self._thresholds_before, thresholds_after=gc.get_threshold(),
            allow_bootstrap_permanent=self._allow_bootstrap_permanent,
            preexisting_frozen_objects=self._preexisting_frozen,
            frozen_objects_at_start=self._frozen_at_start, frozen_objects_now=gc.get_freeze_count(),
            startup_collect_called=self._collect_called, startup_collected_objects=self._collected,
            freeze_called=self._freeze_called, unfreeze_called=self._unfreeze_called,
            freeze_scope="all_tracked_objects_at_model_ready_boundary",
            cleanup_restores_generation_membership=False,
            dynamic_session_and_frames_must_be_allocated_after_start=True)

    def __enter__(self):
        return self.start()

    def __exit__(self, *_):
        self.close()
