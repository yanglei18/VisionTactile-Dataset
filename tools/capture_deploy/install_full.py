#!/usr/bin/env python3
"""Automatic Ubuntu workstation installation; GUI by default, CLI for deployment."""
from __future__ import annotations

import argparse
import json
from pathlib import Path
import sys


if not getattr(sys, "frozen", False):
    sys.path.insert(0, str(Path(__file__).resolve().parents[1] / "tracker_deploy/src"))


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--delivery", type=Path, help="Combined delivery directory")
    parser.add_argument("--headless", action="store_true", help="Run the same installer without its graphical view")
    parser.add_argument("--check-only", action="store_true", help="Validate package bytes without installing")
    parser.add_argument("--smoke-test", action="store_true", help=argparse.SUPPRESS)
    args = parser.parse_args(argv)
    if args.delivery is not None:
        directory = args.delivery
    elif getattr(sys, "frozen", False):
        directory = Path(sys.executable).parent
    else:
        directory = Path(__file__).resolve().parents[3]
    if args.check_only:
        from oneclick.delivery import inspect_delivery
        try:
            delivery = inspect_delivery(directory)
        except (ValueError, OSError) as exc:
            print(str(exc), file=sys.stderr)
            return 1
        print(json.dumps({"valid": True, "installation_id": delivery.installation_id,
                          "source_files": len(delivery.files), "hardware_accessed": False}))
        return 0
    if args.headless:
        from oneclick.engine import Installer
        try:
            result = Installer(directory, notify=lambda event: print(
                json.dumps(event, ensure_ascii=False), flush=True)).run()
        except (ValueError, RuntimeError, OSError) as exc:
            print(json.dumps({"kind": "error", "message": str(exc)}, ensure_ascii=False), file=sys.stderr)
            return 1
        print(json.dumps({"kind": "complete", **result}, ensure_ascii=False))
        return 0
    from oneclick.gui import run
    return run(directory, smoke_test=args.smoke_test)


if __name__ == "__main__":
    raise SystemExit(main())
