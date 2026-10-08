"""GC policy tests run only in fresh interpreters, never freeze pytest itself."""
import os
from pathlib import Path
import subprocess
import sys
import textwrap


def isolated(code):
    source = Path(__file__).resolve().parents[1] / "src"
    env = {**os.environ, "PYTHONPATH": os.pathsep.join(filter(None, (str(source), os.environ.get("PYTHONPATH"))))}
    program = "import gc, weakref\nfrom vt_fsglove12.live_gc import StartupGcFreeze\n" + textwrap.dedent(code)
    result = subprocess.run([sys.executable, "-c", program], env=env, capture_output=True,
                            text=True, timeout=10.)
    assert result.returncode == 0, result.stdout + result.stderr


def test_startup_freeze_preserves_gc_policy_and_refcount_ownership():
    isolated("""
        class Value:
            pass
        value = Value()
        ref = weakref.ref(value)
        enabled, thresholds = gc.isenabled(), gc.get_threshold()
        policy = StartupGcFreeze(scope="offline_benchmark", allow_bootstrap_permanent=True)
        with policy:
            status = policy.snapshot()
            assert status["applied"] and status["active"]
            assert status["startup_collect_called"] and status["freeze_called"]
            assert status["frozen_objects_at_start"] > 0
            assert gc.isenabled() == enabled and gc.get_threshold() == thresholds
            del value
            assert ref() is None  # Freeze does not replace reference counting.
        status = policy.snapshot()
        assert status["closed"] and status["unfreeze_called"] and not status["active"]
        assert gc.get_freeze_count() == 0
        assert gc.isenabled() == enabled and gc.get_threshold() == thresholds
        policy.close()
        assert gc.get_freeze_count() == 0
    """)


def test_initialization_cycles_collect_before_freeze_and_new_cycles_still_collect():
    isolated("""
        class Value:
            pass
        old = Value()
        old.self = old
        old_ref = weakref.ref(old)
        del old
        policy = StartupGcFreeze(scope="offline_benchmark", allow_bootstrap_permanent=True)
        with policy:
            assert old_ref() is None
            value = Value()
            value.self = value
            ref = weakref.ref(value)
            del value
            # Ordinary new allocations cause automatic GC; do not alter its
            # thresholds or manually collect to make this assertion pass.
            # Do not dereference the weakref inside the allocation loop: GC
            # can run at that call boundary while its result is temporarily
            # on the evaluation stack, keeping the very cycle under test live.
            retained = [[] for _ in range(20_000)]
            assert ref() is None
            assert policy.snapshot()["active"] and gc.isenabled()
    """)


def test_frozen_and_cross_boundary_cycles_defer_until_unfreeze():
    isolated("""
        class Value:
            pass
        old = Value()
        old.self = old
        old_ref = weakref.ref(old)
        holder = Value()
        holder_ref = weakref.ref(holder)
        policy = StartupGcFreeze(scope="offline_benchmark", allow_bootstrap_permanent=True)
        with policy:
            later = Value()  # Models can acquire references to dynamic objects.
            later.holder = holder
            holder.later = later
            later_ref = weakref.ref(later)
            del old, holder, later
            gc.collect(2)  # A test assertion, never a production hot-loop call.
            assert old_ref() is not None
            assert holder_ref() is not None and later_ref() is not None
        gc.collect(2)
        assert old_ref() is holder_ref() is later_ref() is None
    """)


def test_preexisting_owner_is_not_unfrozen_and_second_startup_is_skipped():
    isolated("""
        gc.freeze()  # Simulate an unrelated policy owner in this subprocess.
        before = gc.get_freeze_count()
        skipped = StartupGcFreeze(scope="offline_benchmark")
        with skipped:
            assert skipped.snapshot()["skip_reason"] == "preexisting_frozen_objects"
        assert gc.get_freeze_count() == before
        assert not skipped.snapshot()["startup_collect_called"]
        assert not skipped.snapshot()["unfreeze_called"]
        gc.unfreeze()  # Only the original owner thaws its objects.
        gc.collect(2)  # Restore the interpreter's own immortal baseline.
        first = StartupGcFreeze(scope="offline_benchmark", allow_bootstrap_permanent=True)
        with first:
            assert first.snapshot()["applied"]
        second = StartupGcFreeze(scope="offline_benchmark", allow_bootstrap_permanent=True)
        with second:
            assert second.snapshot()["skip_reason"] == "startup_boundary_already_used"
        assert not second.snapshot()["startup_collect_called"]
        assert not second.snapshot()["unfreeze_called"]
    """)


def test_foreign_scope_main_process_and_active_python_worker_are_skipped():
    isolated("""
        import threading
        baseline = gc.get_freeze_count()
        for scope, reason in (("gui", "unsupported_scope"), ("compute_child", "not_a_compute_child")):
            with StartupGcFreeze(scope=scope) as policy:
                assert policy.snapshot()["skip_reason"] == reason
                assert not policy.snapshot()["startup_collect_called"]
        release = threading.Event()
        thread = threading.Thread(target=release.wait)
        thread.start()
        try:
            with StartupGcFreeze(scope="offline_benchmark") as policy:
                assert policy.snapshot()["skip_reason"] == "python_workers_already_started"
        finally:
            release.set()
            thread.join()
        assert gc.get_freeze_count() == baseline
    """)


def test_disabled_gc_is_left_untouched_without_collecting_or_freezing():
    isolated("""
        from unittest.mock import patch
        baseline = gc.get_freeze_count()
        # Simulate the unsupported inherited policy without ever disabling GC.
        with patch("vt_fsglove12.live_gc.gc.isenabled", return_value=False):
            with StartupGcFreeze(scope="offline_benchmark") as policy:
                assert policy.snapshot()["skip_reason"] == "automatic_gc_not_enabled"
                assert not policy.snapshot()["startup_collect_called"]
        assert gc.isenabled() and gc.get_freeze_count() == baseline
    """)


def test_compute_scope_activates_only_inside_spawn_child():
    isolated("""
        import multiprocessing as mp
        code = (
            "import gc; from vt_fsglove12.live_gc import StartupGcFreeze; "
            "policy = StartupGcFreeze(scope='compute_child', allow_bootstrap_permanent=True); policy.start(); "
            "assert policy.snapshot()['applied']; policy.close(); "
            "assert policy.snapshot()['unfreeze_called']; assert gc.get_freeze_count() == 0"
        )
        process = mp.get_context("spawn").Process(target=exec, args=(code,))
        process.start()
        process.join(5.)
        assert not process.is_alive() and process.exitcode == 0
    """)


def test_exception_cleanup_unfreezes_and_foreign_pid_cannot_unfreeze():
    isolated("""
        from unittest.mock import patch
        policy = StartupGcFreeze(scope="offline_benchmark", allow_bootstrap_permanent=True)
        try:
            with policy:
                with patch("vt_fsglove12.live_gc.os.getpid", return_value=-1):
                    policy.close()
                    assert policy.snapshot()["close_refused_reason"] == "foreign_process_or_owner"
                    assert gc.get_freeze_count() > 0
                    assert not policy.snapshot()["unfreeze_called"]
                raise ValueError("injected runtime construction failure")
        except ValueError:
            pass
        assert policy.snapshot()["unfreeze_called"]
        assert gc.get_freeze_count() == 0
    """)


def test_foreign_freeze_after_bootstrap_capture_is_not_adopted_or_unfrozen():
    isolated("""
        policy = StartupGcFreeze(scope="offline_benchmark", allow_bootstrap_permanent=True)
        gc.freeze()  # A different owner freezes after our early bootstrap token.
        frozen = gc.get_freeze_count()
        with policy:
            assert policy.snapshot()["skip_reason"] == "preexisting_frozen_objects"
        assert gc.get_freeze_count() == frozen
        assert not policy.snapshot()["startup_collect_called"]
        assert not policy.snapshot()["unfreeze_called"]
        gc.unfreeze()  # Cleanup belongs only to that different owner.
    """)
