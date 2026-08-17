from __future__ import annotations

import threading

import pytest
from rclpy.context import Context
from rclpy.node import Node

from vt_vive_tracker_gui.runtime import RosRuntime


class FakeExecutor:
    def __init__(self, events):
        self.events = events
        self.spinning = threading.Event()
        self.shutdown_requested = threading.Event()

    def spin(self):
        thread = threading.current_thread()
        self.events.append(
            ("executor_spin", thread.name, thread.daemon)
        )
        self.spinning.set()
        self.shutdown_requested.wait(timeout=2.0)

    def shutdown(self):
        self.events.append("executor_shutdown")
        self.shutdown_requested.set()
        return True


class FakeNode:
    def __init__(self, events, error=None):
        self.events = events
        self.error = error

    def destroy_node(self):
        self.events.append("node_destroy")
        if self.error is not None:
            raise self.error
        return True


class FailingExecutor(FakeExecutor):
    def shutdown(self):
        self.events.append("executor_shutdown")
        self.shutdown_requested.set()
        raise RuntimeError("executor shutdown failed")


def test_runtime_starts_executor_thread_and_stops_in_owned_order():
    events = []
    executor = FakeExecutor(events)
    runtime = RosRuntime(
        executor=executor,
        node=FakeNode(events),
        shutdown_context=lambda: events.append("context"),
    )

    runtime.start()
    assert executor.spinning.wait(timeout=1.0)
    runtime.stop()
    runtime.stop()

    assert events[0] == ("executor_spin", "vive-gui-ros", False)
    assert sum(
        event[0] == "executor_spin"
        for event in events
        if isinstance(event, tuple)
    ) == 1
    assert events[-3:] == [
        "executor_shutdown",
        "node_destroy",
        "context",
    ]


def test_start_is_idempotent():
    events = []
    executor = FakeExecutor(events)
    runtime = RosRuntime(
        executor=executor,
        node=FakeNode(events),
        shutdown_context=lambda: events.append("context"),
    )

    runtime.start()
    runtime.start()
    assert executor.spinning.wait(timeout=1.0)
    runtime.stop()

    assert sum(
        event[0] == "executor_spin"
        for event in events
        if isinstance(event, tuple)
    ) == 1


def test_join_timeout_reports_error_after_owned_cleanup(monkeypatch):
    events = []
    executor = FakeExecutor(events)
    runtime = RosRuntime(
        executor=executor,
        node=FakeNode(events),
        shutdown_context=lambda: events.append("context"),
    )
    runtime.start()
    assert executor.spinning.wait(timeout=1.0)

    monkeypatch.setattr(runtime._thread, "join", lambda timeout: None)
    monkeypatch.setattr(runtime._thread, "is_alive", lambda: True)

    with pytest.raises(RuntimeError, match="vive-gui-ros.*2 seconds"):
        runtime.stop()
    runtime.stop()

    assert events[-3:] == [
        "executor_shutdown",
        "node_destroy",
        "context",
    ]
    assert events.count("node_destroy") == 1
    assert events.count("context") == 1


def test_executor_shutdown_error_still_cleans_node_and_context_once():
    events = []
    executor = FailingExecutor(events)
    runtime = RosRuntime(
        executor=executor,
        node=FakeNode(events),
        shutdown_context=lambda: events.append("context"),
    )
    runtime.start()
    assert executor.spinning.wait(timeout=1.0)

    with pytest.raises(RuntimeError, match="executor shutdown failed"):
        runtime.stop()
    runtime.stop()

    assert events[-3:] == [
        "executor_shutdown",
        "node_destroy",
        "context",
    ]
    assert events.count("executor_shutdown") == 1
    assert events.count("node_destroy") == 1
    assert events.count("context") == 1


def test_node_destroy_error_still_shuts_context_once():
    events = []
    executor = FakeExecutor(events)
    runtime = RosRuntime(
        executor=executor,
        node=FakeNode(events, RuntimeError("node destroy failed")),
        shutdown_context=lambda: events.append("context"),
    )
    runtime.start()
    assert executor.spinning.wait(timeout=1.0)

    with pytest.raises(RuntimeError, match="node destroy failed"):
        runtime.stop()
    runtime.stop()

    assert events[-3:] == [
        "executor_shutdown",
        "node_destroy",
        "context",
    ]
    assert events.count("node_destroy") == 1
    assert events.count("context") == 1


def test_context_shutdown_error_does_not_repeat_owned_cleanup():
    events = []
    executor = FakeExecutor(events)

    def fail_context_shutdown():
        events.append("context")
        raise RuntimeError("context shutdown failed")

    runtime = RosRuntime(
        executor=executor,
        node=FakeNode(events),
        shutdown_context=fail_context_shutdown,
    )
    runtime.start()
    assert executor.spinning.wait(timeout=1.0)

    with pytest.raises(RuntimeError, match="context shutdown failed"):
        runtime.stop()
    runtime.stop()

    assert events[-3:] == [
        "executor_shutdown",
        "node_destroy",
        "context",
    ]
    assert events.count("node_destroy") == 1
    assert events.count("context") == 1


def test_stopping_runtime_does_not_destroy_separate_publisher_node():
    runtime_context = Context()
    publisher_context = Context()
    runtime_context.init(initialize_logging=False)
    publisher_context.init(initialize_logging=False)
    runtime_node = Node(
        "gui_runtime_test_node",
        context=runtime_context,
        enable_rosout=False,
        start_parameter_services=False,
    )
    publisher_node = Node(
        "separate_test_publisher",
        context=publisher_context,
        enable_rosout=False,
        start_parameter_services=False,
    )
    runtime = RosRuntime.from_node(
        runtime_node,
        shutdown_context=runtime_context.shutdown,
    )

    try:
        runtime.start()
        runtime.stop()

        assert not runtime_context.ok()
        assert publisher_context.ok()
        assert publisher_node.get_clock().now().nanoseconds >= 0
        assert publisher_node.get_name() == "separate_test_publisher"
    finally:
        if publisher_node.context.ok():
            publisher_node.destroy_node()
            publisher_context.shutdown()
