"""PyInstaller entry point: the installed application owns all runtime behavior."""
from vt_capture_console.tactile_app import main


if __name__ == "__main__":
    raise SystemExit(main())
