"""Exercise the actual manager entry boundary without importing ROS."""
import os
import time

from vt_capture_manager import node


def test_manager_entry_keeps_log_retention_alive_through_its_body(tmp_path, monkeypatch):
    path = tmp_path / 'manager.log'
    fd = os.open(path, os.O_WRONLY | os.O_CREAT | os.O_APPEND, 0o600)
    with os.fdopen(fd, 'w') as output:
        def run(args, ros_args):
            assert args.log_path == str(path)
            assert ros_args == ['--ros-args', '--disable-stdout-logs']
            os.write(output.fileno(), b'x' * (6 * 1024 * 1024))
            end = time.monotonic() + 3
            while path.stat().st_size > 5 * 1024 * 1024 and time.monotonic() < end:
                time.sleep(.01)
            assert path.stat().st_size == 1024 * 1024
            return 'manager-result'
        monkeypatch.setattr(node, '_run_manager', run)
        with monkeypatch.context() as context:
            context.setattr(node.sys, 'stdout', output)
            assert node.main(['--log-path', str(path), '--ros-args', '--disable-stdout-logs']) == 'manager-result'
        # Context exit closed retention, not the inherited output stream.
        os.write(output.fileno(), b'after-manager')
    assert path.read_bytes().endswith(b'after-manager')


def test_command_line_without_log_path_does_not_start_retention(monkeypatch):
    from vt_capture_core import process_log
    def unexpected(*args, **kwargs):
        raise AssertionError('CLI requested no log service')
    monkeypatch.setattr(process_log, 'ProcessLogRetention', unexpected)
    monkeypatch.setattr(node, '_run_manager', lambda args, ros_args: 'cli')
    assert node.main([]) == 'cli'
