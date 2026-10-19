"""Linux process identity evidence; stored PIDs are never signal authority."""
from __future__ import annotations
import os
from pathlib import Path
import time


def process_start_token(pid: int) -> str:
    return Path(f'/proc/{pid}/stat').read_text().rsplit(')', 1)[1].split()[19]


def recorder_lease(pid: int, session_id: str, *, owner_pid: int) -> dict:
    """Persist only identity evidence; saved PIDs never grant signalling authority."""
    return dict(schema_version=1, session_id=session_id, pid=pid,
                start_token=process_start_token(pid),
                boot_id=Path('/proc/sys/kernel/random/boot_id').read_text().strip(),
                owner_pid=owner_pid, owner_start_token=process_start_token(owner_pid),
                created_ns=time.time_ns())


def clear_recorder_lease(path: Path) -> None:
    path.unlink(missing_ok=True)
    descriptor = os.open(path.parent, os.O_RDONLY | os.O_DIRECTORY)
    try:
        os.fsync(descriptor)
    finally:
        os.close(descriptor)
