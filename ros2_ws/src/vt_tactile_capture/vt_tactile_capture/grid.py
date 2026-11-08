"""The native 20x20 tactile grid with one host receive timestamp per row."""
from numbers import Integral

ROWS = COLS = 20
SHAPE = (ROWS, COLS)
CELL_COUNT = ROWS * COLS
CELL_ROW_INDICES = tuple(row for row in range(ROWS) for _ in range(COLS))
SCHEMA_VERSION = 3
LAYOUT_ID = "row20_col20_v3"


def normalize_state(message):
    """Validate the current grid without copying or cropping it.

    Previous schemas describe another wire format and are not accepted.
    Consumers additionally validate payload values, lengths and source clocks.
    """
    version = message.schema_version
    shape = (message.rows, message.cols)
    if isinstance(version, bool) or not isinstance(version, Integral):
        raise ValueError("Invalid tactile schema version")
    if any(isinstance(value, bool) or not isinstance(value, Integral) for value in shape):
        raise ValueError("Tactile grid dimensions must be integers")
    if version != SCHEMA_VERSION or shape != SHAPE or message.layout_id != LAYOUT_ID:
        raise ValueError("Expected schema 3 / 20x20 / row20_col20_v3 tactile state")
    return message
