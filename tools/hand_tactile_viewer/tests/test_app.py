"""Entrypoint and installation checks without opening a device."""
import importlib
import json
from pathlib import Path

import pytest

from vt_hand_tactile_viewer import app


def test_functional_dependency_check_executes_both_original_qmt_functions():
    report = app._check_functional_numerics()
    assert report["status"] == "ok"
    assert report["projection"]["function"] == "qmt.quatProject"
    assert report["relative_heading"]["recovered_offset_rad"] == pytest.approx(.2, abs=1e-10)
    for value in (report["projection"], report["relative_heading"]):
        assert Path(value["source_file"]).is_file()
        assert len(value["source_sha256"]) == 64


@pytest.mark.parametrize("function", ["projection", "heading"])
def test_importable_but_broken_functional_dependency_is_not_reported_ready(monkeypatch, function):
    if function == "projection":
        import qmt
        monkeypatch.setattr(qmt, "quatProject", lambda *args, **kwargs:
                            {"projAngle": float("nan"), "resAngle": 0.})
    else:
        heading = importlib.import_module("qmt.functions.heading_correction")
        monkeypatch.setattr(heading, "estimateDelta1d", lambda *args:
                            (float("nan"), 1., 0.))
    with pytest.raises(RuntimeError, match="self-check failed"):
        app._check_functional_numerics()


def test_missing_frozen_source_files_fail_the_calibration_provenance_check(monkeypatch):
    import inspect
    monkeypatch.setattr(inspect, "getsourcefile", lambda function: None)
    with pytest.raises(RuntimeError, match="source provenance"):
        app._check_functional_numerics()


def test_missing_private_resources_write_error_report_without_starting_devices(tmp_path):
    report_path = tmp_path / "report" / "self-check.json"
    assert app.main(["--self-check", str(report_path), "--resources", str(tmp_path / "absent")]) == 1
    report = json.loads(report_path.read_text(encoding="utf-8"))
    assert report["status"] == "error"
    assert report["issues"][0]["type"] == "FileNotFoundError"
    assert report["details"]["serial_device"] == "not_opened"
    assert "solver_process" not in report["details"]


@pytest.mark.parametrize("arguments, message", [
    (["--smoke-seconds=-1"], "finite and nonnegative"),
    (["--smoke-seconds=nan"], "finite and nonnegative"),
    (["--smoke-seconds=inf"], "finite and nonnegative"),
    (["--screenshot=unused.png"], "requires --smoke-seconds"),
])
def test_invalid_smoke_arguments_fail_before_gui_start(arguments, message):
    with pytest.raises(SystemExit, match=message):
        app.main(arguments)
