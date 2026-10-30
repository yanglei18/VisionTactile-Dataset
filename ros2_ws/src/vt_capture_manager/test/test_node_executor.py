"""Fair dispatch is a scheduling boundary, never a data or control throttle."""
import sys
import os
import threading
import time
from types import SimpleNamespace

import pytest

from vt_capture_manager import node as manager_node


@pytest.mark.parametrize('failure', [None, RuntimeError('callback failed')])
@pytest.mark.parametrize('entrypoint', ['spin_once', 'spin'])
def test_executor_yields_after_dispatch_without_changing_workers_or_timeout(monkeypatch, failure, entrypoint):
    calls = []

    class Executor:
        def __init__(self, *, num_threads, context):
            calls.append(('init', num_threads, context))

        def add_node(self, node):
            calls.append(('node', node))

        def spin_once(self, timeout_sec=None):
            return self._spin_once_impl(timeout_sec=timeout_sec)

        def spin(self):
            # Jazzy's production loop bypasses the public spin_once method.
            return self._spin_once_impl()

        def _spin_once_impl(self, timeout_sec=None):
            calls.append(('spin', timeout_sec))
            if failure is not None:
                raise failure
            return 'dispatched'

    monkeypatch.setitem(sys.modules, 'rclpy.executors', SimpleNamespace(MultiThreadedExecutor=Executor))
    monkeypatch.setattr(manager_node, 'time', SimpleNamespace(sleep=lambda value: calls.append(('yield', value))))
    node = SimpleNamespace(context=object())
    executor = manager_node.build_executor(node)
    invoke = (lambda: executor.spin_once(timeout_sec=.012)) if entrypoint == 'spin_once' else executor.spin
    if failure is None:
        assert invoke() == 'dispatched'
    else:
        with pytest.raises(RuntimeError, match='callback failed'):
            invoke()
    timeout = .012 if entrypoint == 'spin_once' else None
    assert calls == [('init', 4, node.context), ('node', node), ('spin', timeout), ('yield', 0)]


@pytest.mark.skipif(os.environ.get('VT_CAPTURE_MANAGER_ROS_TEST') != '1',
                    reason='opt-in synthetic ROS test requires an isolated ROS domain')
def test_real_production_spin_loop_reaches_fair_dispatch_boundary(monkeypatch):
    import rclpy
    from rclpy.context import Context
    from rclpy.node import Node

    context = Context()
    rclpy.init(context=context)
    node = Node('manager_fair_dispatch_test', context=context, enable_rosout=False,
                start_parameter_services=False)
    executor = manager_node.build_executor(node)
    yields, ticks = [], []
    ready = threading.Event()
    failures = []

    def yield_cpu(delay):
        yields.append(delay)
        time.sleep(delay)

    def tick():
        ticks.append(1)
        if len(ticks) >= 5:
            ready.set()

    monkeypatch.setattr(manager_node, 'time', SimpleNamespace(sleep=yield_cpu))
    node.create_timer(.01, tick)

    def spin():
        try:
            executor.spin()
        except Exception as exc:
            failures.append(exc)

    thread = threading.Thread(target=spin, daemon=True)
    thread.start()
    try:
        assert ready.wait(3.), 'production spin did not dispatch synthetic timers'
        assert yields and all(delay == 0 for delay in yields)
    finally:
        executor.shutdown(timeout_sec=2.)
        thread.join(timeout=2.)
        node.destroy_node()
        context.shutdown()
    assert not thread.is_alive() and not failures
