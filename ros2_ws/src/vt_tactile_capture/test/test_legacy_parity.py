"""Check standalone Glove.py and ROS crop parity without Qt or a serial port."""
import ast
from pathlib import Path
import struct
from types import SimpleNamespace

import numpy as np
import pytest

from vt_tactile_capture.assembler import Assembler
from vt_tactile_capture.protocol import Parser
from vt_tactile_capture.serial_source import Batch


@pytest.mark.parametrize("row,group", [(0, 0), (0, 7), (17, 3), (79, 0), (79, 7)])
def test_existing_glove_method_and_ros_core_address_adc_parity(row, group):
    source = Path(__file__).resolve().parents[4] / "Glove.py"
    tree = ast.parse(source.read_text())
    cls = next(node for node in tree.body if isinstance(node, ast.ClassDef) and node.name == "HeatmapViewer")
    method = next(node for node in cls.body if isinstance(node, ast.FunctionDef) and node.name == "process_frame")
    namespace = {"FRAME_SIZE": 26, "HEADER1": 0xA5, "HEADER2": 0x5A,
                 "ROWS": 20, "COLS": 20, "WIRE_ROWS": 80}
    # Only execute the existing pure numeric method, never HeatmapViewer.__init__.
    exec(compile(ast.Module(body=[method], type_ignores=[]), str(source), "exec"), namespace)
    old = SimpleNamespace(
        data=np.full((20, 20), np.nan), bad_frame_count=0, frame_count=0,
        status_label=SimpleNamespace(setText=lambda text: None),
    )
    values = (0, 1, 255, 256, 32767, 32768, 40000, 65534, 65535, 23232)
    payload = b"\xa5\x5a" + struct.pack(">HBB10H", row, 201, group, *values)
    assert namespace["process_frame"](old, payload)
    blocks = Parser().feed(payload)
    grid = Assembler()
    grid.apply(Batch(session_id="legacy-parity", stream_epoch=1, sequence=1,
                     connected=True, time_valid=True, reason="ok",
                     host_realtime_ns=1_700_000_000_000_000_000,
                     host_monotonic_ns=1_000_000_000, blocks=blocks))
    state = grid.snapshot(1_001_000_000, 100_000_000)
    mask = np.asarray(state.valid_mask).reshape(20, 20)
    actual = np.asarray(state.adc).reshape(20, 20)
    np.testing.assert_array_equal(mask, np.isfinite(old.data))
    np.testing.assert_array_equal(actual[mask], old.data[mask])
    assert blocks[0].status_raw == 201
    assert old.bad_frame_count == 0  # Out-of-ROI packets are still valid wire packets.
    assert int(mask.sum()) == (len(range(group, 20, 8)) if row < 20 else 0)
