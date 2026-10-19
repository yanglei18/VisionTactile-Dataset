import importlib
import signal
from types import SimpleNamespace

import pytest
import rclpy
from rclpy.executors import ExternalShutdownException


@pytest.mark.parametrize("module_name,node_class", [
    ("node", "TactileNode"), ("replay", "TactileReplayNode"),
])
@pytest.mark.parametrize("exception", [KeyboardInterrupt, ExternalShutdownException])
def test_capture_main_expected_shutdown_ignores_repeated_signals(monkeypatch, module_name, node_class, exception):
    module = importlib.import_module("vt_tactile_capture." + module_name)
    events = []
    before = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}

    def destroy():
        assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN
        assert signal.getsignal(signal.SIGTERM) == signal.SIG_IGN
        signal.raise_signal(signal.SIGINT)
        events.append("destroy")

    def spin(_):
        raise exception()

    monkeypatch.setattr(rclpy, "init", lambda **_: None)
    monkeypatch.setattr(rclpy, "ok", lambda: True)
    monkeypatch.setattr(rclpy, "spin", spin)
    monkeypatch.setattr(rclpy, "shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(module, node_class, lambda: SimpleNamespace(destroy_node=destroy))
    module.main([])
    assert events == ["destroy", "shutdown"]
    assert before == {number: signal.getsignal(number) for number in before}


@pytest.mark.parametrize("module_name,node_class", [
    ("node", "TactileNode"), ("replay", "TactileReplayNode"),
])
def test_capture_main_does_not_hide_startup_errors(monkeypatch, module_name, node_class):
    module = importlib.import_module("vt_tactile_capture." + module_name)
    events = []

    def fail():
        raise ValueError("real startup error")

    monkeypatch.setattr(rclpy, "init", lambda **_: None)
    monkeypatch.setattr(rclpy, "ok", lambda: True)
    monkeypatch.setattr(rclpy, "shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(module, node_class, fail)
    with pytest.raises(ValueError, match="real startup error"):
        module.main([])
    assert events == ["shutdown"]


def test_viewer_main_signals_quit_qt_and_cleanup_is_shielded(monkeypatch):
    pytest.importorskip("pyqtgraph")
    QtWidgets = pytest.importorskip("PyQt5.QtWidgets")
    from rclpy.signals import SignalHandlerOptions
    from vt_tactile_capture import viewer

    events = []
    before = {number: signal.getsignal(number) for number in (signal.SIGINT, signal.SIGTERM)}

    def init(**kwargs):
        assert kwargs["signal_handler_options"] == SignalHandlerOptions.NO

    def execute():
        signal.raise_signal(signal.SIGINT)
        signal.raise_signal(signal.SIGTERM)

    def destroy():
        assert signal.getsignal(signal.SIGINT) == signal.SIG_IGN
        signal.raise_signal(signal.SIGINT)
        events.append("destroy")

    app = SimpleNamespace(exec=execute, quit=lambda: events.append("quit"))
    monkeypatch.setattr(QtWidgets, "QApplication", lambda *_: app)
    monkeypatch.setattr(rclpy, "init", init)
    monkeypatch.setattr(rclpy, "ok", lambda: True)
    monkeypatch.setattr(rclpy, "shutdown", lambda: events.append("shutdown"))
    monkeypatch.setattr(viewer, "create_viewer_node", lambda _: SimpleNamespace(
        destroy_node=destroy, declare_parameter=lambda _name, value: SimpleNamespace(value=value)))
    monkeypatch.setattr(viewer, "create_window", lambda *_, **__: SimpleNamespace(show=lambda: None))
    viewer.main([])
    assert events == ["quit", "quit", "destroy", "shutdown"]
    assert before == {number: signal.getsignal(number) for number in before}
