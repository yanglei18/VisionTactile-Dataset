"""Sensor-ID permutation parity; synthetic inputs do not establish hand accuracy."""
from copy import deepcopy
from importlib.resources import files
import os

import numpy as np
import pytest

from test_upstream_backend import make_data, official_core
from test_upstream_functional_hand import batch_data
from vt_fsglove12.config import load_config
from vt_fsglove12.live_stream import OnlineFusion, RawSample
from vt_fsglove12.pose_export import build_state
from vt_fsglove12.upstream_backend import OfficialFSGloveBackend


# Physical-role correspondence supplied for the new right glove, including
# the still-unmapped wrist sensor. These are sensor IDs, not model slot IDs.
NEW_TO_OLD = {"1": "6", "2": "5", "3": "4", "4": "3", "5": "2", "6": "1",
              "7": "8", "8": "7", "9": "10", "10": "9", "11": "12", "12": "11"}
OLD_TO_NEW = {old: new for new, old in NEW_TO_OLD.items()}


def numbering_config(source):
    config = load_config(files("vt_fsglove12").joinpath("configs/right_new_imu.json"))
    # Isolate numbering from the separately reviewed scale/timing increment.
    for field in ("gyro_dps_per_lsb", "accel_g_per_lsb", "sample_rate_hz", "output_rate_hz"):
        config[field] = source[field]
    for field in ("gyro_dps_per_lsb_by_node", "accel_g_per_lsb_by_node"):
        config[field] = {OLD_TO_NEW[node]: value for node, value in source.get(field, {}).items()}
    # Compare numbering under the same estimator settings, independently of
    # the new glove's explicitly configured bias-estimation range.
    config.pop("vqf", None)
    if "vqf" in source:
        config["vqf"] = deepcopy(source["vqf"])
    return config


def renamed_frame(frame):
    result = deepcopy(frame)
    result["nodes"] = {OLD_TO_NEW[node]: data for node, data in result["nodes"].items()}
    return result


def test_numbering_preserves_all_twelve_raw_fusion_states_including_guards_and_gaps():
    old_config = load_config()
    old_config["gyro_dps_per_lsb_by_node"] = {"5": .0175, "12": .035}
    old_config["accel_g_per_lsb_by_node"] = {"6": .000244, "11": .000244}
    new_config = numbering_config(old_config)
    assert new_config["node_to_slot"] == {
        new: old_config["node_to_slot"][old] for new, old in NEW_TO_OLD.items()
        if old in old_config["node_to_slot"]
    }
    assert "12" not in new_config["node_to_slot"]
    old_fusion, new_fusion = OnlineFusion(old_config), OnlineFusion(new_config)
    for index in range(150):
        stamp_ns = 1_000_000_000 + round(index * 1e9 / old_config["sample_rate_hz"])
        if index >= 100:
            stamp_ns += 1_300_000_000  # Exercise the unchanged gap/reset path.
        old_rows, new_rows = [], []
        for node in range(1, 13):
            values = (node * 9 + index % 11, node * 3 - index % 7,
                      index % 13 - node, node * 2, 10 - node, 8192 + node)
            if index == 60 and node == 6:
                values = (32764, *values[1:])  # Existing guard, not a new policy.
            row = RawSample(node, values, stamp_ns / 1_000_000_000,
                            (stamp_ns + 1_700_000_000_000_000_000) / 1_000_000_000,
                            0, stamp_ns, stamp_ns + 1_700_000_000_000_000_000)
            old_rows.append(row)
            new_rows.append(RawSample(int(OLD_TO_NEW[str(node)]), row.values,
                                      row.host_monotonic_s, row.host_wall_s, row.status,
                                      row.host_monotonic_ns, row.host_realtime_ns))
        old_fusion.update(old_rows)
        new_fusion.update(sorted(new_rows, key=lambda row: row.node_id))
        if index in (0, 60, 61, 99, 100, 149):
            old = old_fusion.snapshot(stamp_ns / 1e9)
            new = new_fusion.snapshot(stamp_ns / 1e9)
            assert len(old["nodes"]) == len(new["nodes"]) == 12
            for new_node, old_node in NEW_TO_OLD.items():
                # Includes original counts, SI inputs, integer clocks, epochs,
                # quaternions, VQF bias/state and existing input-guard decisions.
                assert new["nodes"][new_node] == old["nodes"][old_node]
            assert new_fusion.input_guard_rejections == old_fusion.input_guard_rejections
    assert old_fusion.input_guard_rejections > 0
    assert old_fusion.epochs["12"] > 0


def test_numbering_preserves_official_calibration_all_fingers_geometry_and_export(batch_data, official_core):
    from vt_fsglove12.mano import ManoBackend

    old, old_profile, neutral, fingers, thumb = batch_data
    new = OfficialFSGloveBackend(numbering_config(old.config), old.rest,
                                checkout="unused", mano_assets_root="unused", core=official_core)
    _, captures = make_data(official_core)
    new_profile = new.fit_calibration(
        {key: [renamed_frame(row) for row in rows] for key, rows in captures.items()},
        session_id=old_profile["session_id"])
    assert new_profile["upstream_state"] == old_profile["upstream_state"]
    assert new_profile["calibration_inputs"] == old_profile["calibration_inputs"]
    assert new_profile["measured_calibration_slots"] == old_profile["measured_calibration_slots"]
    assert new_profile["nodes"]["11"] == old_profile["nodes"]["12"]
    assert new_profile["config_sha256"] != old_profile["config_sha256"]
    with pytest.raises(ValueError, match="configuration.*digest"):
        new.solve(renamed_frame(neutral[0]), old_profile)

    old_fit = old.fit_functional_hand(neutral, fingers, thumb, old_profile,
                                      session_id=old_profile["session_id"])
    new_fit = new.fit_functional_hand(
        [renamed_frame(row) for row in neutral], [renamed_frame(row) for row in fingers],
        [renamed_frame(row) for row in thumb], new_profile, session_id=new_profile["session_id"])
    assert old_fit["errors"] == new_fit["errors"] == {}
    assert set(new_fit["entries"]) == {"thumb", "index", "middle", "ring", "little"}
    old_profile = {**old_profile, "functional_fingers": old_fit["entries"], "functional_enabled": True}
    new_profile = {**new_profile, "functional_fingers": new_fit["entries"], "functional_enabled": True}
    model = ManoBackend(os.environ["FSGLOVE12_MANO_ASSETS"], hand="right")
    for source in (fingers[8], thumb[10]):
        old_frame = deepcopy(source)
        # Auxiliary input is deliberately newer than the palm. It must neither
        # become the exported reference nor appear among measured model nodes.
        old_frame["nodes"]["11"] = deepcopy(old_frame["nodes"]["12"])
        for node, data in old_frame["nodes"].items():
            data["host_monotonic_ns"] = 10_000_000_000 + int(node)
            data["host_realtime_ns"] = 1_700_000_000_000_000_000 + int(node)
        old_frame["nodes"]["11"]["host_monotonic_ns"] += 1000
        new_frame = renamed_frame(old_frame)
        poses = [old.solve(old_frame, old_profile), new.solve(new_frame, new_profile)]
        for key in ("mano_pose_rotvec", "anatomical_euler_rad", "root_quat_xyzw",
                    "segment_observed_mask", "joint_observed_mask", "inferred_mask", "joint_quality",
                    "joint_display_available_mask", "joint_valid_mask"):
            np.testing.assert_array_equal(poses[0][key], poses[1][key])
        assert poses[0]["pose_available"] and poses[1]["pose_available"]
        geometry = [model.render_mano(pose["mano_pose_rotvec"], pose["shape_params"]) for pose in poses]
        for key in ("vertices_m", "joints_m"):
            np.testing.assert_array_equal(geometry[0][key], geometry[1][key])
        packets = [build_state(frame, pose, mesh, backend.config, root_node=root,
                               solve_completed_monotonic_ns=11_000_000_000)
                   for frame, pose, mesh, backend, root in
                   ((old_frame, poses[0], geometry[0], old, "12"),
                    (new_frame, poses[1], geometry[1], new, "11"))]
        assert packets[1]["reference_node_id"] == 11
        assert packets[1]["source_node_ids"] == list(range(1, 12))
        assert packets[0]["source_node_ids"] == [*range(1, 11), 12]
        for key in ("host_realtime_ns", "host_monotonic_ns", "mano_pose_rotvec", "joint_positions_m",
                    "root_quat_xyzw", "pose_valid", "time_valid"):
            assert packets[0][key] == packets[1][key]
