"""Frozen entry point; dispatch spawned workers before importing the desktop."""
import multiprocessing


if __name__ == "__main__":
    multiprocessing.freeze_support()
    from vt_hand_tactile_viewer.app import main

    raise SystemExit(main())
