import json
import os
from pathlib import Path
import subprocess
import sys


def test_lease_is_durable_before_exec_and_pid_is_unchanged(tmp_path):
    from vt_tracker_camera_calib.process_lease import process_start_token
    lease_path = tmp_path / 'recorder-lease.json'
    script = 'import json,os,pathlib; d=json.loads(pathlib.Path(os.environ["TEST_LEASE"]).read_text()); assert d["pid"]==os.getpid(); print(d["pid"])'
    process = subprocess.Popen([sys.executable, '-m', 'vt_tracker_camera_calib.recorder_runner',
        '--lease', str(lease_path), '--session-id', 'runner-test', '--owner-pid', str(os.getpid()),
        '--owner-start-token', process_start_token(os.getpid()), '--', sys.executable, '-c', script],
        stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, env={**os.environ, 'TEST_LEASE': str(lease_path)})
    stdout, stderr = process.communicate(timeout=10)
    assert process.returncode == 0, stderr
    assert int(stdout) == process.pid
    lease = json.loads(lease_path.read_text())
    assert lease['owner_pid'] == os.getpid()
    assert lease['start_token']
    assert lease['boot_id'] == Path('/proc/sys/kernel/random/boot_id').read_text().strip()


def test_runner_refuses_replaced_or_departed_owner(tmp_path):
    lease_path = tmp_path / 'recorder-lease.json'
    completed = subprocess.run([sys.executable, '-m', 'vt_tracker_camera_calib.recorder_runner',
        '--lease', str(lease_path), '--session-id', 'runner-test', '--owner-pid', str(os.getpid()),
        '--owner-start-token', '0', '--', sys.executable, '-c', 'raise RuntimeError("must not execute")'],
        capture_output=True, text=True, timeout=10)
    assert completed.returncode != 0
    assert 'owner changed' in completed.stderr
    assert not lease_path.exists()
