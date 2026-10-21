#!/usr/bin/env python3
"""Internal pkexec entry point for the fixed system installation protocol."""
from pathlib import Path
import sys

# -I ignores user PYTHONPATH/site packages. Import only our explicitly selected
# trusted delivery, never the calling process's working directory.
sys.path.insert(0, str(Path(__file__).resolve().parent))
from oneclick.system import main  # noqa: E402

if __name__ == "__main__":
    raise SystemExit(main())
