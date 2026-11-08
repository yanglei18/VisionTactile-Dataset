"""Frozen left-hand entry point. Spawned workers must dispatch first."""
import multiprocessing


if __name__ == "__main__":
    multiprocessing.freeze_support()
    from vt_hand_tactile_viewer.app import left_main

    raise SystemExit(left_main())
