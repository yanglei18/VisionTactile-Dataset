"""Adapter contract tests plus an optional real-upstream integration check."""

import json
from types import SimpleNamespace

import numpy as np
import pytest

from vt_glove_pose import qmt_reference as reference


def inputs(count=240):
    t = np.arange(count)/120.
    gp = np.column_stack((10+4*np.sin(t), 4*np.cos(t), np.ones(count)))
    gd = gp + [5, 0, 0]
    ap = np.tile([.3, .4, np.sqrt(.75)], (count, 1))
    ad = ap.copy()
    return gp, gd, ap, ad


def install_spy(monkeypatch):
    calls = []

    def upstream(ap, ad, gp, gd, *, estSettings, debug, plot):
        # Consume RNG just as upstream currently does, and mutate input copies
        # after retaining evidence to verify adapter isolation.
        np.random.random()
        calls.append(dict(ap=ap.copy(), ad=ad.copy(), gp=gp.copy(), gd=gd.copy(),
                          settings=dict(estSettings), debug=debug, plot=plot))
        objective = 5. if "x0" not in estSettings else 3.
        return (np.array([[1.], [0.], [0.]]), np.array([[1.], [0.], [0.]]),
                {"optimVarsAxis": {"f": objective, "ftraj": np.array([7., objective])}})

    monkeypatch.setattr(reference, "_load_qmt", lambda: SimpleNamespace(jointAxisEstHingeOlsson=upstream))
    return calls


def test_calls_upstream_with_si_units_and_training_only(monkeypatch):
    calls = install_spy(monkeypatch)
    data = inputs()
    result = reference.fit_qmt_reference(*data, sample_rate_hz=120)
    train = np.arange(240)//60 % 2 == 0
    assert len(calls) == 1
    actual = calls[0]
    np.testing.assert_array_equal(actual["gp"], np.deg2rad(data[0][train]))
    np.testing.assert_array_equal(actual["gd"], np.deg2rad(data[1][train]))
    np.testing.assert_array_equal(actual["ap"], data[2][train]*9.80665)
    np.testing.assert_array_equal(actual["ad"], data[3][train]*9.80665)
    assert actual["settings"] == {"quiet": True}
    assert actual["debug"] and not actual["plot"]
    assert result["engine"] == "upstream_qmt_jointAxisEstHingeOlsson"
    assert result["offline_only"] and not result["usable_as_live_calibration"]
    assert not result["product_accuracy_verified"]
    json.dumps(result, allow_nan=False)


def test_multiple_starts_select_only_upstream_training_objective(monkeypatch):
    calls = install_spy(monkeypatch)
    result = reference.fit_qmt_reference(*inputs(), sample_rate_hz=120, n_starts=4, seed=2**32-1)
    assert len(calls) == 4
    assert result["optimization"]["selected_start"] == 1
    assert result["optimization"]["selected_upstream_train_objective"] == 3.
    assert not result["optimization"]["holdout_used_for_selection"]
    assert "x0" not in calls[0]["settings"]
    assert calls[1]["settings"]["x0"].shape == (4, 1)


def test_holdout_corruption_changes_metrics_not_upstream_input_or_selected_axes(monkeypatch):
    calls = install_spy(monkeypatch)
    data = inputs()
    first = reference.fit_qmt_reference(*data, sample_rate_hz=120, n_starts=2)
    bad = [v.copy() for v in data]
    mask = np.arange(240)//60 % 2 == 1
    bad[0][mask] = [40, 50, 60]
    bad[1][mask] = [0, 0, 0]
    bad[2][mask] = [1, 0, 0]
    bad[3][mask] = [0, 1, 0]
    second = reference.fit_qmt_reference(*bad, sample_rate_hz=120, n_starts=2)
    assert first["metrics"]["holdout"] != second["metrics"]["holdout"]
    assert second["reference_status"] == "fixed_metric_mismatch"
    for key in ("axis_prox", "axis_dist", "optimization"):
        assert first[key] == second[key]
    assert first["metrics"]["train"] == second["metrics"]["train"]
    for i in range(2):
        for key in ("ap", "ad", "gp", "gd"):
            np.testing.assert_array_equal(calls[i][key], calls[i+2][key])


def test_preserves_inputs_and_global_random_state(monkeypatch):
    install_spy(monkeypatch)
    data = inputs()
    before = [v.copy() for v in data]
    np.random.seed(92)
    state = np.random.get_state()
    reference.fit_qmt_reference(*data, sample_rate_hz=120, n_starts=3)
    after = np.random.get_state()
    assert state[0] == after[0]
    np.testing.assert_array_equal(state[1], after[1])
    assert state[2:] == after[2:]
    for a, b in zip(data, before):
        np.testing.assert_array_equal(a, b)


def test_missing_optional_qmt_dependency_has_actionable_error(monkeypatch):
    def unavailable(name):
        raise ModuleNotFoundError(name)
    monkeypatch.setattr(reference.importlib, "import_module", unavailable)
    with pytest.raises(RuntimeError, match="isolated environment"):
        reference.fit_qmt_reference(*inputs(), sample_rate_hz=120)


@pytest.mark.parametrize("kwargs", [{"n_starts": 0}, {"n_starts": True},
    {"sample_rate_hz": float("nan")}, {"holdout_block_s": 0}, {"seed": -1}])
def test_invalid_settings_never_call_upstream(monkeypatch, kwargs):
    calls = install_spy(monkeypatch)
    with pytest.raises(ValueError):
        reference.fit_qmt_reference(*inputs(), **{"sample_rate_hz": 120, **kwargs})
    assert calls == []


@pytest.mark.parametrize("damage", ["nonfinite", "complex", "length", "acceleration_limit", "too_short"])
def test_invalid_inputs_never_call_upstream(monkeypatch, damage):
    calls = install_spy(monkeypatch)
    data = list(inputs())
    if damage == "nonfinite":
        data[0][0, 0] = np.nan
    elif damage == "complex":
        data[1] = data[1].astype(complex)
    elif damage == "length":
        data[2] = data[2][:-1]
    elif damage == "acceleration_limit":
        data[3][0, 0] = 101
    else:
        data = [v[:60] for v in data]
    with pytest.raises(ValueError):
        reference.fit_qmt_reference(*data, sample_rate_hz=120)
    assert calls == []


def test_cli_refuses_to_overwrite_existing_evidence(tmp_path):
    output = tmp_path/"report.json"
    output.write_text("original", encoding="utf-8")
    with pytest.raises(SystemExit):
        reference.main(["--session", "unused", "--config", "unused", "--output", str(output)])
    assert output.read_text(encoding="utf-8") == "original"


def test_actual_upstream_optional_integration():
    pytest.importorskip("qmt")
    from test_hinge_joint_axes import snapshots, line_error
    data, jp, jd = snapshots(seed=22)
    result = reference.fit_qmt_reference(*data, sample_rate_hz=120, n_starts=4)
    assert line_error(result["axis_prox"], jp) < 1.
    assert line_error(result["axis_dist"], jd) < 1.
    assert result["metrics"]["holdout"]["gyro"]["p95_abs_dps"] < .3
    assert result["optimization"]["failed_starts"] == []
    source = reference.describe_qmt()
    assert source["function_spdx_license"] == "LicenseRef-Unspecified"
    assert len(source["function_source_sha256"]) == 64
    json.dumps(result, allow_nan=False)
