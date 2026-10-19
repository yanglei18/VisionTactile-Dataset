"""Write a durable identity lease before replacing this process with Recorder.

The worker owns this Popen handle and process group. exec preserves their PID and
birth token. If the worker dies before exec, the lease still identifies Recorder
for read-only recovery checks; no unrecorded Recorder can run in that gap.
"""
from __future__ import annotations

import argparse
import os
from pathlib import Path

from .process_lease import process_start_token, recorder_lease
from .session_io import atomic_document


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--lease', type=Path, required=True)
    parser.add_argument('--session-id', required=True)
    parser.add_argument('--owner-pid', type=int, required=True)
    parser.add_argument('--owner-start-token', required=True)
    parser.add_argument('command', nargs=argparse.REMAINDER)
    args = parser.parse_args(argv)
    command = args.command[1:] if args.command[:1] == ['--'] else args.command
    if not command or not args.lease.is_absolute():
        raise ValueError('absolute lease path and Recorder command are required')

    def check_owner():
        if os.getppid() != args.owner_pid or process_start_token(args.owner_pid) != args.owner_start_token:
            raise RuntimeError('Recorder owner changed before launch')

    check_owner()
    atomic_document(args.lease, recorder_lease(os.getpid(), args.session_id, owner_pid=args.owner_pid))
    check_owner()
    os.execvp(command[0], command)


if __name__ == '__main__':
    main()
