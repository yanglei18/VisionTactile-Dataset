"""Run the standalone left-hand display."""
import multiprocessing

if __name__ == "__main__":
    multiprocessing.freeze_support()
    from .app import left_main
    raise SystemExit(left_main())
