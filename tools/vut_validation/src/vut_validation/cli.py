from __future__ import annotations

import argparse
from dataclasses import asdict
import json
from pathlib import Path
import sys
import time

from .backend import PyVUTBackend
from .metrics import ValidationSession
from .model import ValidationThresholds
from .preflight import check_mode, enumerate_vive


def build_parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(prog="vt-vut-validate")
    commands = parser.add_subparsers(
        dest="command",
        required=True,
    )

    preflight = commands.add_parser("preflight")
    preflight.add_argument(
        "--mode",
        choices=("TRACKER_USB", "DONGLE_USB"),
        required=True,
    )

    run = commands.add_parser("run")
    run.add_argument(
        "--mode",
        choices=("TRACKER_USB", "DONGLE_USB"),
        required=True,
    )
    run.add_argument("--duration", type=float, default=300.0)
    run.add_argument("--min-hz", type=float, default=30.0)
    run.add_argument("--max-gap-ms", type=float, default=100.0)
    run.add_argument("--expected-trackers", type=int, default=1)
    run.add_argument("--output", required=True)
    return parser


def main(
    argv=None,
    backend_factory=PyVUTBackend,
    sleep=time.sleep,
) -> int:
    args = build_parser().parse_args(argv)
    if args.command == "preflight":
        errors = check_mode(args.mode, enumerate_vive())
        if errors:
            print("\n".join(errors), file=sys.stderr)
            return 2
        print(f"preflight PASS: mode={args.mode}")
        return 0

    output = Path(args.output)
    if (
        not output.is_absolute()
        or args.duration <= 0
        or args.expected_trackers <= 0
    ):
        print(
            "absolute output and positive values required",
            file=sys.stderr,
        )
        return 2

    thresholds = ValidationThresholds(
        duration_s=args.duration,
        min_hz=args.min_hz,
        max_gap_ms=args.max_gap_ms,
    )
    session = ValidationSession(thresholds)
    backend = backend_factory(args.mode)
    try:
        backend.start(session.add)
        sleep(args.duration + 0.25)
    finally:
        backend.stop()

    report = session.finish(args.expected_trackers)
    output.parent.mkdir(parents=True, exist_ok=True)
    output.write_text(
        json.dumps(asdict(report), indent=2, sort_keys=True) + "\n"
    )
    print(
        f"validation {'PASS' if report.passed else 'FAIL'}: "
        f"{output}"
    )
    return 0 if report.passed else 1


def entrypoint() -> None:
    raise SystemExit(main())
