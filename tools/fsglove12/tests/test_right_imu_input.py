"""Declared STM32 units and nominal timing; no physical accuracy certification."""
from importlib.resources import files

import numpy as np
from scipy.spatial.transform import Rotation

from test_right_imu_layout import NEW_TO_OLD, OLD_TO_NEW
from vt_fsglove12.config import load_config
from vt_fsglove12.input_contract import effective_input_scales, raw_to_si
from vt_fsglove12.live_stream import OnlineFusion, RawSample


def new_input_config():
    return load_config(files("vt_fsglove12").joinpath("configs/right_new_imu.json"))


def sample(node, values, index):
    mono = 1_000_000_000 + round(index * 1_000_000_000 / 104)
    wall = 1_700_000_000_000_000_000 + mono
    return RawSample(node, tuple(values), mono / 1_000_000_000,
                     wall / 1_000_000_000, 0, mono, wall)


def test_reported_counts_have_declared_physical_units_without_changing_raw_values():
    config = new_input_config()
    gyro_scales, accel_scales = effective_input_scales(config)
    raw = (1000, -2000, 3000, 4096, -4096, 0)
    for node in range(1, 13):
        gyro, accel = raw_to_si(raw, gyro_dps_per_lsb=gyro_scales[str(node)],
                               accel_g_per_lsb=accel_scales[str(node)])
        np.testing.assert_allclose(gyro, np.deg2rad([35., -70., 105.]), rtol=1e-14)
        np.testing.assert_allclose(accel, np.asarray([.999424, -.999424, 0.]) * 9.80665,
                                   rtol=1e-14)
    assert raw == (1000, -2000, 3000, 4096, -4096, 0)
    assert not config["units_verified"] and not config["sample_rate_verified"]


def test_nominal_period_is_independent_of_output_target_and_host_read_batching():
    config = new_input_config()
    slow_output = {**config, "output_rate_hz": 30.}
    fast, slow = OnlineFusion(config), OnlineFusion(slow_output)
    assert fast.period == slow.period == 1. / 104.
    # 52 constant-rate samples represent half a second at the declared 104 Hz.
    # This is a synthetic rotation about gravity, not a new real-hand reference.
    rows = [sample(11, (0, 0, 1000, 0, 0, 4096), i) for i in range(52)]
    for start in range(0, len(rows), 6):
        group = rows[start:start + 6]
        received = group[-1]
        # A delayed host read timestamps several samples together. Every
        # original sample still advances the existing filter by one period.
        fast.update([RawSample(row.node_id, row.values, received.host_monotonic_s,
                               received.host_wall_s, row.status,
                               received.host_monotonic_ns, received.host_realtime_ns)
                     for row in group])
    for row in rows:
        slow.update([row])
    a = fast.snapshot(rows[-1].host_monotonic_s)["nodes"]["11"]
    b = slow.snapshot(rows[-1].host_monotonic_s)["nodes"]["11"]
    assert a["sample_index"] == b["sample_index"] == len(rows)
    assert a["quat_xyzw"] == b["quat_xyzw"]
    assert a["fusion_state"] == b["fusion_state"]
    np.testing.assert_allclose(Rotation.from_quat(a["quat_xyzw"]).as_rotvec(),
                               [0., 0., np.deg2rad(17.5)], atol=1e-12)


def test_equivalent_physical_input_preserves_all_node_fusion_states():
    target = new_input_config()
    source = load_config()
    # Hold the physical sample period and estimator settings equal; numbering,
    # counts and units differ. Default-estimator compatibility is tested separately.
    source.update(sample_rate_hz=104., output_rate_hz=104.)
    if "vqf" in target:
        source["vqf"] = dict(target["vqf"])
    old, new = OnlineFusion(source), OnlineFusion(target)
    for index in range(220):
        original, converted = [], []
        for node in range(1, 13):
            values = (4 * (node * 3 + index % 13 - 6),
                      4 * (node - index % 7), 4 * (index % 9 - 4),
                      2 * (node * 3 + index % 5), 2 * (node - index % 11),
                      8192 + 2 * node)
            original.append(sample(node, values, index))
            # Exact representable synthetic counts only. Production raw rows
            # are never rewritten, clipped or resampled by this adaptation.
            converted.append(sample(int(OLD_TO_NEW[str(node)]),
                                    [v // (4 if i < 3 else 2) for i, v in enumerate(values)], index))
        old.update(original)
        new.update(sorted(converted, key=lambda row: row.node_id))
    before = old.snapshot(original[-1].host_monotonic_s)["nodes"]
    after = new.snapshot(converted[-1].host_monotonic_s)["nodes"]
    raw_by_node = {str(row.node_id): list(row.values) for row in converted}
    for new_node, old_node in NEW_TO_OLD.items():
        for key in ("gyro_rad_s", "accel_m_s2", "quat_xyzw", "fusion_state", "input_filter",
                    "epoch", "sample_index", "host_monotonic_ns", "host_realtime_ns"):
            assert after[new_node][key] == before[old_node][key], (new_node, key)
        assert after[new_node]["raw_values"] == raw_by_node[new_node]
