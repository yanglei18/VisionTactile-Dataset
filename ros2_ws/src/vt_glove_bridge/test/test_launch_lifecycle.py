"""Exercise launch lifecycle with synthetic processes; no ROS nodes or devices."""
import asyncio
import importlib.util
from pathlib import Path
import sys

import pytest

from launch import LaunchContext, LaunchDescription, LaunchService
from launch.actions import EmitEvent, ExecuteProcess, RegisterEventHandler
from launch.event_handlers import OnExecutionComplete, OnProcessExit
from launch.events import ExecutionComplete
from launch.events.process import ShutdownProcess


@pytest.fixture
def glove_launch():
    path = Path(__file__).parents[1] / "launch/glove.launch.py"
    spec = importlib.util.spec_from_file_location("glove_lifecycle_launch_test", path)
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def configuration(tmp_path, **changes):
    context = LaunchContext()
    context.launch_configurations.update(
        hand="right", launch_gui="true", glove_python=sys.executable,
        fsglove_source=str(tmp_path), mano_assets=str(tmp_path), tactile_overlay="true")
    context.launch_configurations.update(changes)
    return context


def test_gui_completion_stops_only_its_session_even_when_included(glove_launch, tmp_path):
    context = configuration(tmp_path)
    actions = glove_launch._start(context)
    handlers = [action.event_handler for action in actions if isinstance(action, RegisterEventHandler)]
    processes = [action for action in actions if isinstance(action, ExecuteProcess)]
    bridge, display, gui = processes
    unrelated = ExecuteProcess(cmd=[sys.executable, "-c", "pass"])
    assert len(handlers) == 2
    assert all(isinstance(action, RegisterEventHandler) for action in actions[:2])
    assert actions[2:] == processes
    for completed in (bridge, gui):
        event = ExecutionComplete(action=completed)
        matching = [handler for handler in handlers if handler.matches(event)]
        assert len(matching) == 1
        shutdowns = matching[0].handle(event, context)
        assert all(isinstance(action, EmitEvent) and isinstance(action.event, ShutdownProcess)
                   for action in shutdowns)
        assert {peer for peer in processes
                if any(action.event.process_matcher(peer) for action in shutdowns)} == set(processes) - {completed}
        assert not any(action.event.process_matcher(unrelated) for action in shutdowns)
    # The optional read-only tactile overlay is not required for pose calibration.
    assert not any(handler.matches(ExecutionComplete(action=display)) for handler in handlers)


def test_bridge_only_launch_keeps_its_existing_independent_lifecycle(glove_launch, tmp_path):
    actions = glove_launch._start(configuration(tmp_path, launch_gui="false"))
    assert len(actions) == 2
    assert all(isinstance(action, ExecuteProcess) for action in actions)
    assert [action.node_executable for action in actions] == [
        "vt_glove_bridge_node", "vt_tactile_display_bridge"]


_PROCESS = """
from pathlib import Path
import signal
import sys
import time

root, role, required, code = Path(sys.argv[1]), sys.argv[2], sys.argv[3], int(sys.argv[4])
def interrupted(signum, frame):
    (root / (role + '-interrupted')).touch()
    sys.exit(0)
signal.signal(signal.SIGINT, interrupted)
(root / (role + '-ready')).touch()
if role == 'left_gui':
    while not ((root / 'required-completed').exists()
               and all((root / (peer + '-ready')).exists()
                       for peer in ('left_gui', 'left_bridge', 'left_display'))):
        time.sleep(.01)
    time.sleep(.1)
    (root / 'left-gui-survived').touch()
elif role == required:
    while not all((root / (peer + '-ready')).exists()
                  for peer in ('bridge', 'display', 'gui', 'left_gui', 'left_bridge', 'left_display')):
        time.sleep(.01)
    sys.exit(code)
else:
    while True:
        time.sleep(.05)
"""


@pytest.mark.parametrize("required,exit_code,spawn_failure", [
    ("gui", 0, False), ("gui", 7, False), ("bridge", 7, False),
    ("bridge", 0, True), ("gui", 0, True),
])
def test_real_launch_cleans_peers_on_close_error_and_spawn_failure(
        glove_launch, tmp_path, monkeypatch, required, exit_code, spawn_failure):
    """Two hands share a LaunchService but run only tiny local Python fixtures."""
    monkeypatch.setenv("ROS_LOG_DIR", str(tmp_path / "logs"))
    processes = {}

    def command(role):
        if role == required and spawn_failure:
            return [str(tmp_path / "deliberately-missing-executable")]
        return [sys.executable, "-c", _PROCESS, str(tmp_path), role, required, str(exit_code)]

    def node(**kwargs):
        role = "bridge" if kwargs["executable"] == "vt_glove_bridge_node" else "display"
        if kwargs["namespace"] == "/gloves/left":
            role = "left_" + role
        process = ExecuteProcess(cmd=command(role), name="synthetic_" + role, output="log")
        processes[role] = process
        return process

    monkeypatch.setattr(glove_launch, "Node", node)
    monkeypatch.setattr(glove_launch, "gui_command",
                        lambda **kwargs: command("left_gui" if kwargs["hand"] == "left" else "gui"))
    actions = glove_launch._start(configuration(tmp_path))
    processes["gui"] = next(action for action in actions if isinstance(action, ExecuteProcess)
                            and action not in processes.values())
    left_actions = glove_launch._start(configuration(tmp_path, hand="left"))
    processes["left_gui"] = next(action for action in left_actions if isinstance(action, ExecuteProcess)
                                 and action not in processes.values())
    exited = {}

    def completed(_event, _context):
        (tmp_path / "required-completed").touch()

    def record_exit(event, _context):
        exited[event.action] = event.returncode

    service = LaunchService()
    service.include_launch_description(LaunchDescription([
        RegisterEventHandler(OnExecutionComplete(target_action=processes[required], on_completion=completed)),
        RegisterEventHandler(OnProcessExit(on_exit=record_exit)),
        *actions, *left_actions,
    ]))

    async def run_bounded():
        task = asyncio.create_task(service.run_async())
        try:
            await asyncio.wait_for(asyncio.shield(task), timeout=8.)
        finally:
            if not task.done():
                await service.shutdown()
                await asyncio.wait_for(task, timeout=8.)

    asyncio.run(run_bounded())
    assert (tmp_path / "left-gui-survived").is_file()
    assert exited[processes["left_gui"]] == 0
    for role in ("left_bridge", "left_display"):
        assert (tmp_path / (role + "-interrupted")).is_file()
        assert exited[processes[role]] == 0
    for role, process in processes.items():
        if role == required:
            if spawn_failure:
                assert process not in exited  # No ProcessExited event is emitted on spawn failure.
            else:
                assert exited[process] == exit_code
        else:
            assert process in exited  # No session peer is left running.
    if not spawn_failure:
        for role in {"gui", "bridge", "display"} - {required}:
            assert (tmp_path / (role + "-interrupted")).is_file()
            assert exited[processes[role]] == 0
