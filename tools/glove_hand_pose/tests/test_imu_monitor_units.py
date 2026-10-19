"""Standalone waveform tool uses the same per-node unit fields; no real ports."""
import importlib.util
import os
from pathlib import Path

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")
os.environ["PYQTGRAPH_QT_LIB"] = "PyQt5"
import pytest

pytest.importorskip("pyqtgraph")
from PyQt5 import QtWidgets

SOURCE = Path(__file__).resolve().parents[3] / "Ploy_IMU.py"
if not SOURCE.is_file():
    pytest.skip("standalone monitor is not part of the installed wheel", allow_module_level=True)
spec = importlib.util.spec_from_file_location("imu_monitor_unit_test", SOURCE)
monitor = importlib.util.module_from_spec(spec)
spec.loader.exec_module(monitor)


def test_monitor_legacy_units_and_node_overrides():
    legacy = monitor.node_unit_scales()
    assert legacy[4] == (.00875,) * 3 + (.000122,) * 3
    config = {"gyro_dps_per_lsb": .00875,
              "gyro_dps_per_lsb_by_node": {"5": .0175, "6": .0175}}
    scales = monitor.node_unit_scales(config)
    assert scales[0] == legacy[0]
    assert scales[4] == scales[5] == (.0175,) * 3 + (.000122,) * 3
    assert scales[6:] == legacy[6:]


@pytest.mark.parametrize("mapping", [{"05": .0175}, {5: .0175}, {"13": .0175},
                                     {"5": True}, {"5": float("nan")}, {"5": 0}, []])
def test_monitor_rejects_invalid_node_scale(mapping):
    with pytest.raises(ValueError):
        monitor.node_unit_scales({"gyro_dps_per_lsb_by_node": mapping})


def test_waveform_values_change_by_node_but_raw_records_do_not(monkeypatch):
    app = QtWidgets.QApplication.instance() or QtWidgets.QApplication([])
    monkeypatch.setattr(monitor.list_ports, "comports", lambda: [])
    config = {"gyro_dps_per_lsb_by_node": {"5": .0175}}
    window = monitor.ImuMainWindow(config=config)
    raw = (100, -200, 300, 8197, 0, 0)
    rows = [(1., 100., 3, raw), (1., 100., 5, raw)]
    try:
        window._receive_records(rows)
        assert window.values[2][0][-1] == pytest.approx(.875)
        assert window.values[4][0][-1] == pytest.approx(1.75)
        assert window.values[4][1][-1] == pytest.approx(-3.5)
        assert rows[1][3] == raw
        assert window.worker is None
    finally:
        window.close()
        app.processEvents()
