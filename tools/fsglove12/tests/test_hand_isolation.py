"""Two independent 12-node hands: software contracts, never hardware validation."""
from copy import deepcopy
import json
from types import SimpleNamespace

import numpy as np
import pytest
from scipy.spatial.transform import Rotation

from test_guided_calibration import sample_joints
from test_pose_export import inputs
from test_upstream_backend import make_data, synthetic_state
from vt_fsglove12.cli import main
from vt_fsglove12.config import PARENTS, load_config
from vt_fsglove12.guided_calibration import model_motion_axes
from vt_fsglove12.live_display import demonstration_root, display_basis
from vt_fsglove12.live_process import ProcessLiveController
from vt_fsglove12.pose_export import build_state
from vt_fsglove12.upstream_backend import OfficialFSGloveBackend


def test_default_is_right_and_left_template_is_explicitly_unverified():
    right, left = load_config(), load_config(hand="left")
    assert right["hand"] == "right" and left["hand"] == "left"
    assert left["node_to_slot"] == right["node_to_slot"]
    assert not left["mounting_confirmed"] and not left["units_verified"] and not left["sample_rate_verified"]
    assert left["input_contract"]["scale_source"] == "configured_assumption"
    assert left["input_contract"]["sensor_model"] == "unknown"
    assert "Unverified" in left["provenance"]
    left["node_to_slot"]["12"] = 1
    assert load_config(hand="left")["node_to_slot"]["12"] == 0


def test_config_owns_hand_and_explicit_conflict_is_rejected(tmp_path, capsys):
    path = tmp_path / "left.json"
    path.write_text(json.dumps(load_config(hand="left")))
    assert load_config(path)["hand"] == "left"
    assert main(["layout", "--config", str(path)]) == 0
    assert json.loads(capsys.readouterr().out)["hand"] == "left"
    assert main(["layout", "--config", str(path), "--hand", "right"]) == 2
    assert "conflicts" in capsys.readouterr().err
    assert main(["layout", "--hand", "left"]) == 0
    assert json.loads(capsys.readouterr().out)["hand"] == "left"


@pytest.mark.parametrize("hand", ["right", "left"])
def test_official_fit_and_restore_pass_hand_and_profiles_cannot_cross(hand):
    calls = []
    core = SimpleNamespace(provenance={"kind": "synthetic_contract_test_no_model"})

    def calibrator(**kwargs):
        calls.append(kwargs)
        return synthetic_state()

    core.create_calibrator = calibrator
    original, captures = make_data(core)
    rest = {**original.rest, "provenance": {"hand": hand, "kind": "synthetic_test"}}
    backend = OfficialFSGloveBackend(load_config(hand=hand), rest, checkout=None, mano_assets_root=None, core=core)
    profile = backend.fit_calibration(captures, session_id="test-session")
    assert profile["hand"] == hand and calls[-1]["side"] == hand
    assert backend._calibrator(profile) is backend._cached_calibrator
    assert len(calls) == 1
    backend.reset_calibration()
    backend._calibrator(json.loads(json.dumps(profile)))
    assert len(calls) == 2 and calls[-1]["side"] == hand
    foreign = {**profile, "hand": "left" if hand == "right" else "right"}
    with pytest.raises(ValueError, match="hand"):
        backend._calibrator(foreign)
    assert original._cached_calibrator is None
    assert original._functional_pipeline is not backend._functional_pipeline


def test_known_model_side_mismatch_rejected_before_loading_dependencies():
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16,
                provenance={"hand": "right"})
    with pytest.raises(ValueError, match="model hand"):
        OfficialFSGloveBackend(load_config(hand="left"), rest, checkout="MUST_NOT_LOAD", mano_assets_root="MUST_NOT_LOAD")


def test_process_controller_derives_left_from_config_and_keeps_separate_mailboxes(tmp_path):
    rest = dict(parents=list(PARENTS), rest_global_xyzw=[[0., 0., 0., 1.]] * 16)
    left = ProcessLiveController(load_config(hand="left"), rest, {}, output_root=tmp_path / "left",
        mano_assets="NOT_LOADED", upstream_checkout="NOT_LOADED", ros_socket=str(tmp_path / "left.sock"))
    right = ProcessLiveController(load_config(), rest, {}, output_root=tmp_path / "right",
        mano_assets="NOT_LOADED", upstream_checkout="NOT_LOADED", ros_socket=str(tmp_path / "right.sock"))
    assert left._args[3]["hand"] == "left" and right._args[3]["hand"] == "right"
    assert left._mailbox is not right._mailbox and left._commands is not right._commands
    assert left._root_node == right._root_node == "12" and left._process is right._process is None
    with pytest.raises(ValueError, match="hand"):
        ProcessLiveController(load_config(hand="left"), rest, {}, output_root=tmp_path,
                              mano_assets="NOT_LOADED", hand="right")


@pytest.mark.parametrize("hand", ["right", "left"])
def test_export_hand_frame_and_non12_palm_come_from_config(hand):
    config = load_config(hand=hand)
    config["node_to_slot"]["11"] = config["node_to_slot"].pop("12")
    frame, pose, geometry = inputs()
    frame["nodes"]["11"] = frame["nodes"].pop("12")
    result = build_state(frame, pose, geometry, config, root_node="11", solve_completed_monotonic_ns=1_030_000_000)
    assert result["hand"] == hand and result["frame_id"] == f"glove_{hand}_reference"
    assert result["reference_node_id"] == 11 and result["time_valid"]
    assert result["host_realtime_ns"] == frame["nodes"]["11"]["host_realtime_ns"]
    with pytest.raises(ValueError, match="configured palm"):
        build_state(frame, pose, geometry, config, root_node="12", solve_completed_monotonic_ns=1)


def test_left_geometry_axes_and_display_rotation_preserve_chirality():
    mirror = np.diag([-1., 1., 1.])
    right = model_motion_axes(sample_joints())
    left = model_motion_axes(sample_joints() @ mirror, hand="left")
    for name in ("pitch_axis", "roll_axis", "palm_up_axis"):
        np.testing.assert_allclose(left[name], mirror @ right[name])
    basis = display_basis(left)
    np.testing.assert_allclose(basis @ basis.T, np.eye(3), atol=1e-12)
    assert np.linalg.det(basis) > .999
    np.testing.assert_allclose(basis @ left["palm_up_axis"], [0., 0., 1.])
    for step in ("fingers_down", "thumb_up"):
        before = Rotation.from_quat(demonstration_root(step, right)).as_matrix()
        after = Rotation.from_quat(demonstration_root(step, left)).as_matrix()
        np.testing.assert_allclose(after, mirror @ before @ mirror, atol=1e-12)


def test_cli_left_missing_model_names_the_left_asset_and_does_not_create_output(tmp_path, capsys):
    destination = tmp_path / "never.json"
    assert main(["export-rest", "--hand", "left", "--mano-assets", str(tmp_path),
                 "--output", str(destination)]) == 2
    assert "MANO_LEFT.pkl" in capsys.readouterr().err
    assert not destination.exists()
