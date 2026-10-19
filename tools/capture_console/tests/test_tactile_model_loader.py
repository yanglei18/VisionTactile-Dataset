"""Static-model workers are bounded asynchronous children, never device jobs."""
from copy import deepcopy
import json
from pathlib import Path
import sys

import numpy as np
import pytest
from PySide6.QtCore import QProcess, QTimer
from PySide6.QtTest import QSignalSpy, QTest

from vt_capture_console.ui.widgets import tactile_model_loader as module


def bundle(hand="right"):
    vertices = np.zeros((778, 3))
    vertices[1] = [.01, 0, 0]
    vertices[2] = [0, .01, 0]
    return {"schema_version": 1, "neutral_geometry": {
        "vertices_m": vertices.tolist(), "faces": [[0, 1, 2]],
        "joints_m": np.zeros((21, 3)).tolist(), "provenance": {"hand": hand}},
        "model_axes": {"pitch_axis": [-1, 0, 0], "roll_axis": [0, 1, 0],
                       "palm_up_axis": [0, 0, 1 if hand == "right" else -1], "hand": hand}}


@pytest.fixture
def configuration(tmp_path):
    models = tmp_path / "trusted assets/models"
    models.mkdir(parents=True)
    for hand in ("RIGHT", "LEFT"):
        (models / f"MANO_{hand}.pkl").touch()
    return {"process_groups": [{"id": hand + "_glove", "enabled": False,
            "parameters": {"hand": hand, "glove_python": sys.executable,
                           "mano_assets": str(models.parent), "port": "/dev/MUST_NOT_OPEN"}}
            for hand in ("right", "left")]}


@pytest.fixture
def loader(qapp, configuration):
    value = module.TactileModelLoader(lambda: configuration)
    yield value
    value.shutdown()
    QTest.qWait(50)
    value.deleteLater()
    qapp.processEvents()


def command(monkeypatch, source):
    monkeypatch.setattr(module, "_worker_arguments", lambda hand, assets: ["-I", "-c", source])


def wait_for(spy, timeout=5000):
    if not spy.count():
        assert spy.wait(timeout), "worker did not emit its result"


@pytest.mark.parametrize("hand", ["right", "left"])
def test_validation_matches_hand_and_returns_owned_readonly_arrays(hand):
    raw = bundle(hand)
    value = module.validate_bundle(raw, hand)
    raw["neutral_geometry"]["vertices_m"][1][0] = 999
    for name in ("vertices_m", "faces", "joints_m"):
        assert not value["neutral_geometry"][name].flags.writeable
    assert value["neutral_geometry"]["vertices_m"][1, 0] == .01
    assert value["model_axes"]["hand"] == hand


@pytest.mark.parametrize("change", [
    lambda value: value.update(schema_version=True),
    lambda value: value["neutral_geometry"].update(vertices_m=[[0, 0, 0]]),
    lambda value: value["neutral_geometry"]["vertices_m"][1].__setitem__(0, float("nan")),
    lambda value: value["neutral_geometry"]["vertices_m"][1].__setitem__(0, 99),
    lambda value: value["neutral_geometry"].update(faces=[[0, 1, 778]]),
    lambda value: value["neutral_geometry"].update(faces=[[0, 1, 1]]),
    lambda value: value["neutral_geometry"].update(faces=[[0., 1., 2.]]),
    lambda value: value["neutral_geometry"].update(faces=[[0, 1, 2]] * 4097),
    lambda value: value["neutral_geometry"].update(joints_m=[[0, 0, 0]]),
    lambda value: value["neutral_geometry"].update(provenance={"hand": "left"}),
    lambda value: value["model_axes"].update(pitch_axis=[0, 0, 0]),
    lambda value: value["model_axes"].update(roll_axis=[-1, 0, 0]),
    lambda value: value["model_axes"].update(palm_up_axis=[0, 0, -1]),
])
def test_invalid_geometry_is_rejected(change):
    value = bundle()
    change(value)
    with pytest.raises(ValueError):
        module.validate_bundle(value, "right")


def test_fixed_worker_command_preserves_venv_and_paths_without_shell(configuration):
    assets = configuration["process_groups"][0]["parameters"]["mano_assets"]
    assert module._worker_arguments("right", assets) == ["-I", "-m", "vt_fsglove12.tactile_preview_export",
                                                        "--mano-assets", assets, "--hand", "right"]


def test_missing_configuration_reports_actionable_error_without_process(qapp):
    loader = module.TactileModelLoader(lambda: {"process_groups": []})
    failed = QSignalSpy(loader.failed)
    assert not loader.available("right")
    loader.request("right")
    assert failed.count() == 1 and "手套 Python" in failed.at(0)[1]
    assert not loader.findChildren(QProcess)
    loader.shutdown()


def test_static_bundle_uses_same_geometry_without_a_model_process(loader, configuration, tmp_path):
    path = tmp_path / "内置模型 with spaces.json"
    document = bundle()
    path.write_text(json.dumps(document), encoding="utf-8")
    parameters = configuration["process_groups"][0]["parameters"]
    parameters.update(preview_bundle=str(path), glove_python="/not-installed", mano_assets="/not-installed")
    ready, failed = QSignalSpy(loader.ready), QSignalSpy(loader.failed)
    loader.request("right")
    assert ready.count() == 1 and failed.count() == 0
    actual = loader.get("right")
    expected = module.validate_bundle(document, "right")
    for name in ("vertices_m", "faces", "joints_m"):
        np.testing.assert_array_equal(actual["neutral_geometry"][name], expected["neutral_geometry"][name])
        assert not actual["neutral_geometry"][name].flags.writeable
    assert not loader.findChildren(QProcess)
    loader.request("right")
    assert loader.get("right") is actual and ready.count() == 2
    document["neutral_geometry"]["vertices_m"][1][0] = .012
    path.write_text(json.dumps(document), encoding="utf-8")
    assert loader.get("right") is None
    loader.request("right")
    assert loader.get("right")["neutral_geometry"]["vertices_m"][1, 0] == .012


@pytest.mark.parametrize("content", [b"not-json", b"\xff", b"x" * (module.MAX_OUTPUT_BYTES + 1),
                                    json.dumps(bundle("left")).encode(), b"null"])
def test_bad_static_model_is_reported_without_falling_back_to_external_runtime(
        loader, configuration, tmp_path, content):
    path = tmp_path / "right_hand.json"
    path.write_bytes(content)
    configuration["process_groups"][0]["parameters"]["preview_bundle"] = str(path)
    failed = QSignalSpy(loader.failed)
    loader.request("right")
    assert failed.count() == 1 and "静态手模型" in str(failed.at(0)[1])
    assert loader.get("right") is None and not loader.findChildren(QProcess)


def test_switching_to_static_model_cancels_only_inflight_model_export(loader, configuration, tmp_path, monkeypatch):
    command(monkeypatch, "import time; time.sleep(30)")
    loader.request("right")
    process = loader._jobs["right"].process
    assert process.waitForStarted(5000)
    path = tmp_path / "right_hand.json"
    path.write_text(json.dumps(bundle()), encoding="utf-8")
    configuration["process_groups"][0]["parameters"]["preview_bundle"] = str(path)
    ready = QSignalSpy(loader.ready)
    loader.request("right")
    assert ready.count() == 1 and not loader._jobs
    assert process.waitForFinished(5000)
    assert loader.get("right") is ready.at(0)[1]


def test_missing_model_never_starts_sensor_or_worker(loader, configuration):
    model = Path(configuration["process_groups"][0]["parameters"]["mano_assets"]) / "models/MANO_RIGHT.pkl"
    model.unlink()
    assert not loader.available("right")
    assert loader.available("left")
    assert "MANO_RIGHT.pkl" in loader.error("right")
    loader.request("right")
    assert not loader.findChildren(QProcess)


def test_success_is_async_shared_inflight_and_cached(loader, monkeypatch, configuration):
    document = json.dumps(bundle())
    command(monkeypatch, "import time; time.sleep(.15); print(" + repr(document) + ")")
    ready, failed = QSignalSpy(loader.ready), QSignalSpy(loader.failed)
    assert loader.available("right")
    loader.request("right")
    first = loader._jobs["right"].process
    loader.request("right")
    assert loader._jobs["right"].process is first
    assert ready.count() == 0
    ticked = []
    QTimer.singleShot(0, lambda: ticked.append(True))
    wait_for(ready)
    assert ticked and failed.count() == 0
    assert loader.get("right") is ready.at(0)[1]
    assert loader.error("right") == ""
    loader.request("right")
    assert ready.count() == 2 and not loader._jobs
    model = Path(configuration["process_groups"][0]["parameters"]["mano_assets"]) / "models/MANO_RIGHT.pkl"
    model.write_bytes(b"changed model identity")
    assert loader.get("right") is None


def test_stale_configuration_result_is_not_published(loader, monkeypatch, configuration):
    document = json.dumps(bundle())
    command(monkeypatch, "import time; time.sleep(.1); print(" + repr(document) + ")")
    ready, failed = QSignalSpy(loader.ready), QSignalSpy(loader.failed)
    loader.request("right")
    finished = QSignalSpy(loader._jobs["right"].process.finished)
    configuration["process_groups"][0]["parameters"]["mano_assets"] = "/missing"
    wait_for(finished)
    assert ready.count() == 0 and loader.get("right") is None
    assert not loader._jobs
    assert failed.count() == 1
    assert failed.at(0)[0] == "right" and "重新加载模型" in failed.at(0)[1]


def test_changed_configuration_failure_can_retry_current_model(loader, monkeypatch, configuration):
    document = json.dumps(bundle())
    command(monkeypatch, "import time; time.sleep(.1); print(" + repr(document) + ")")
    ready, failed = QSignalSpy(loader.ready), QSignalSpy(loader.failed)
    loader.request("right")
    model = Path(configuration["process_groups"][0]["parameters"]["mano_assets"]) / "models/MANO_RIGHT.pkl"
    model.write_bytes(b"new model revision")
    wait_for(failed)
    assert ready.count() == 0 and "重新加载模型" in failed.at(0)[1]
    loader.request("right")
    wait_for(ready)
    assert ready.count() == 1 and failed.count() == 1
    assert loader.get("right") is ready.at(0)[1]
    assert loader.error("right") == ""


@pytest.mark.parametrize("source, expected", [
    ("print('not json')", "数据无效"),
    ("import sys; print('bad resource', file=sys.stderr); sys.exit(2)", "bad resource"),
    ("print('x' * 600000)", "安全大小"),
])
def test_worker_errors_are_bounded_and_do_not_cache(loader, monkeypatch, source, expected):
    command(monkeypatch, source)
    failed, ready = QSignalSpy(loader.failed), QSignalSpy(loader.ready)
    loader.request("right")
    wait_for(failed)
    assert failed.count() == 1 and expected in failed.at(0)[1]
    assert ready.count() == 0 and loader.get("right") is None
    assert not loader._jobs


def test_timeout_kills_only_owned_export_worker(loader, monkeypatch):
    command(monkeypatch, "import time; time.sleep(30)")
    loader._timeout_ms = 40
    failed = QSignalSpy(loader.failed)
    loader.request("right")
    process = loader._jobs["right"].process
    finished = QSignalSpy(process.finished)
    wait_for(failed)
    wait_for(finished)
    assert "60 秒" in failed.at(0)[1] and not loader._jobs


def test_shutdown_cancels_without_publishing_and_never_restarts(loader, monkeypatch):
    command(monkeypatch, "import time; time.sleep(30)")
    ready, failed = QSignalSpy(loader.ready), QSignalSpy(loader.failed)
    loader.request("right")
    loader.shutdown()
    loader.request("left")
    assert loader._closed and not loader._jobs
    assert not loader.available("right")
    assert ready.count() == failed.count() == 0


def test_singleton_is_owned_by_app(qapp, configuration):
    first = module.model_loader(lambda: configuration)
    assert module.model_loader() is first
    assert first.parent() is qapp
