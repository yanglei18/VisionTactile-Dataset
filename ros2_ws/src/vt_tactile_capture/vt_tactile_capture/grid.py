"""Physical tactile grid, separate from the unchanged 80x80 wire addressing.

The user-confirmed active area is raw rows/columns [0, 20). Consumers use
schema 2 with 400 cells and 160 stride-8 block clocks. Legacy schema-1 rolling
states can be cropped explicitly; no resampling, reordering or re-timestamping.
"""
from types import SimpleNamespace
from numbers import Integral

import numpy as np


ROWS = COLS = 20
SHAPE = (ROWS, COLS)
CELL_COUNT = ROWS * COLS
COLUMN_GROUPS = 8
BLOCK_COUNT = ROWS * COLUMN_GROUPS
SCHEMA_VERSION = 2
LAYOUT_ID = "row20_col_group_stride8_v2"
WIRE_ROWS = WIRE_COLS = 80

_FIELDS = (
    "schema_version", "hand", "session_id", "stream_epoch", "source_sequence",
    "header", "host_realtime_ns", "host_monotonic_ns", "snapshot_monotonic_ns",
    "clock_domain", "timing_source", "layout_id", "unit", "rows", "cols",
    "connected", "time_valid", "reason", "adc", "valid_mask", "updated_mask",
    "block_realtime_ns", "block_monotonic_ns", "max_age_ns", "producer_dropped_total",
    "transport_reset_generation", "transport_health_generation", "transport_health_only",
)


def normalize_state(message):
    """Accept v2 20x20 or explicitly crop legacy v1 80x80, without mutation.

    Current states are returned unchanged for zero-copy use. A legacy state is
    copied to a ROS-free namespace with new independent arrays. Consumers still
    validate payload dtypes, health, clocks, and ownership as appropriate.
    """
    version = message.schema_version
    shape = (message.rows, message.cols)
    if isinstance(version, (bool, np.bool_)) or not isinstance(version, Integral):
        raise ValueError("Invalid tactile schema version")
    if any(isinstance(value, (bool, np.bool_)) or not isinstance(value, Integral)
           for value in shape):
        raise ValueError("Tactile grid dimensions must be integers")
    if version == SCHEMA_VERSION and shape == SHAPE:
        return message
    if version != 1 or shape != (WIRE_ROWS, WIRE_COLS):
        raise ValueError("Expected schema 2 / 20x20 or legacy schema 1 / 80x80 tactile state")
    values = {key: getattr(message, key) for key in _FIELDS if hasattr(message, key)}
    for name in ("adc", "valid_mask", "updated_mask"):
        if name not in values and name == "updated_mask":
            continue
        if name not in values:
            raise ValueError(f"Legacy state is missing {name}")
        array = np.asarray(values[name])
        if array.shape != (WIRE_ROWS * WIRE_COLS,):
            raise ValueError(f"Legacy {name} must contain 6400 elements")
        values[name] = array.reshape(WIRE_ROWS, WIRE_COLS)[:ROWS, :COLS].copy().reshape(-1)
    for name in ("block_monotonic_ns", "block_realtime_ns"):
        if name not in values:
            continue
        array = np.asarray(values[name])
        if array.shape != (WIRE_ROWS * COLUMN_GROUPS,):
            raise ValueError(f"Legacy {name} must contain 640 elements")
        values[name] = array[:BLOCK_COUNT].copy()
    values.update(schema_version=SCHEMA_VERSION, rows=ROWS, cols=COLS, layout_id=LAYOUT_ID)
    return SimpleNamespace(**values)
