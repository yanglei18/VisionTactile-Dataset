#!/usr/bin/env python3
"""Run deployment commands directly from the trusted project checkout."""
from pathlib import Path
import sys

sys.path.insert(0, str(Path(__file__).resolve().parent / "src"))

from vt_tracker_deploy.cli import main

if __name__ == "__main__":
    raise SystemExit(main())
