#!/usr/bin/env python3
"""Audit external FSGlove core loading against the whole original source.

This offline verification uses synthetic 16-IMU orientations and synthetic
shape-optimizer fixtures, not measured human hand poses or contact labels.
It does not touch a serial port, start a GUI, or change the upstream checkout.
"""

from __future__ import annotations

import argparse
import builtins
from contextlib import redirect_stdout, redirect_stderr
import hashlib
import io
import json
from pathlib import Path
import re
import time
from types import SimpleNamespace

import numpy as np
from scipy.spatial.transform import Rotation

from vt_fsglove12.upstream_core import load_upstream_core


def whole_original_calibration(core):
    """Execute the unchanged entire calibration.py with scoped imports only."""
    bindings = {
        "modules.datamodels": core.datamodels,
        "modules.enumerates": core.enumerates,
        "modules.utils": core.utils,
        "manotorch.manolayer": SimpleNamespace(ManoLayer=core.calibration.ManoLayer, MANOOutput=core.calibration.MANOOutput),
        "manotorch.axislayer": SimpleNamespace(AxisLayerFK=core.calibration.AxisLayerFK),
    }

    def scoped_import(name, globals=None, locals=None, fromlist=(), level=0):
        if name in bindings:
            return bindings[name]
        return builtins.__import__(name, globals, locals, fromlist, level)

    filename = core.provenance["sources"]["calibration"]["path"]
    namespace = {"__name__": "_official_whole_file_baseline", "__builtins__": dict(vars(builtins), __import__=scoped_import)}
    exec(compile(Path(filename).read_text(), filename, "exec", dont_inherit=True), namespace)
    return SimpleNamespace(**namespace)


def verify(upstream: Path, assets: Path, *, frames: int = 64, iterations: int = 500, seed: int = 42):
    if frames < 1 or iterations < 1:
        raise ValueError("frames and iterations must be positive")
    started = time.monotonic()
    core = load_upstream_core(upstream, assets)
    torch = core.torch
    mount = Rotation.from_euler("XYZ", np.linspace(-.3, .4, 48).reshape(16, 3))
    base = mount.as_quat()
    first = (Rotation.from_euler("x", -60, degrees=True) * mount).as_quat()
    second = (Rotation.from_euler("x", 60, degrees=True) * mount).as_quat()
    extracted = core.create_calibrator(base, first, second, seed=seed)
    original_module = whole_original_calibration(core)
    with core.seeded(seed):
        original = original_module.Raw2MANOCalibrator(original_module.CalibrationQuatSet(
            base_pose=base, z_rot_pose_1=first, z_rot_pose_2=second), side="right")

    rng = np.random.default_rng(seed)
    max_difference = 0.
    sampled_raw = []
    inference_times = []
    for _ in range(frames):
        raw = mount * Rotation.from_rotvec(rng.uniform(-.5, .5, (16, 3)))
        sampled_raw.append(raw.as_quat())
        for anatomical in (False, True):
            for world in (False, True):
                tick = time.monotonic()
                actual = extracted.get_mano_param(raw, ana_repr=anatomical, map_to_world=world)
                inference_times.append(time.monotonic() - tick)
                expected = original.get_mano_param(raw, ana_repr=anatomical, map_to_world=world)
                if not torch.isfinite(actual).all():
                    raise RuntimeError("Original official inference produced non-finite results")
                difference = float(torch.max(torch.abs(actual - expected)))
                max_difference = max(max_difference, difference)
    if max_difference != 0:
        raise RuntimeError(f"Port differs from whole original source: {max_difference}")

    local_raw = np.stack(sampled_raw[:min(3, frames)])
    label_pose = np.zeros((len(local_raw), 16, 3))
    with core.seeded(seed):
        extracted.correct_local_inst_error(local_raw, label_pose)
    with core.seeded(seed):
        original.correct_local_inst_error(local_raw, label_pose)
    local_difference = float(np.max(np.abs(extracted._local_installation_error - original._local_installation_error)))
    if local_difference != 0:
        raise RuntimeError(f"Local-installation correction differs from source: {local_difference}")

    model = core.calibration.ManoLayer(rot_mode="axisang", use_pca=False, side="right", center_idx=None, flat_hand_mean=True)
    pose = extracted.get_mano_param(Rotation.from_quat(sampled_raw[-1])).reshape(1, 48)
    with torch.no_grad():
        geometry = model(pose, torch.zeros(1, 10))
    if geometry.verts.shape != (1, 778, 3) or not torch.isfinite(geometry.verts).all():
        raise RuntimeError("Original official output could not drive real MANO vertices")

    # Intentionally artificial labels exercise the exact official loss and
    # Adam path. They do NOT represent valid recorded fingertip contacts.
    smoke_joints = torch.zeros((4, 16, 3))
    smoke_categories = torch.tensor([2, 3, 4, 5])
    stdout, stderr = io.StringIO(), io.StringIO()
    with redirect_stdout(stdout), redirect_stderr(stderr):
        beta = core.optimize_shape(smoke_joints, smoke_categories, "right", seed=seed, iter_num=iterations)
    match = re.search(r"loss dropped from ([^ ]+) to ([^\s]+)", stdout.getvalue())
    if match is None or not torch.isfinite(beta).all():
        raise RuntimeError("Original official shape optimizer did not produce finite result/loss report")
    begin_loss, final_loss = map(float, match.groups())
    if not np.isfinite([begin_loss, final_loss]).all() or final_loss >= begin_loss:
        raise RuntimeError(f"Synthetic official-optimizer health loss did not decrease: {begin_loss} -> {final_loss}")

    model_file = core.require_model("right")
    return {
        "status": "passed",
        "scope": "Original-public-core source parity and synthetic numerical health, not physical hand accuracy",
        "provenance": core.provenance,
        "model": {"path": str(model_file), "sha256": hashlib.sha256(model_file.read_bytes()).hexdigest()},
        "parity_16_imu": {"frames": frames, "mode_combinations": 4, "reference": "Unchanged entire official calibration.py executed with scoped dependency imports", "max_absolute_difference": max_difference},
        "local_installation_correction": {"frames": len(local_raw), "max_absolute_difference": local_difference},
        "geometry": {"vertices_shape": list(geometry.verts.shape), "joints_shape": list(geometry.joints.shape), "all_finite": True},
        "official_shape_optimizer_smoke": {
            "fixture": "SYNTHETIC zero poses with artificial categories; NOT recorded fingertip contacts or user calibration",
            "iterations": iterations, "seed": seed, "begin_loss": begin_loss,
            "final_loss_before_last_update": final_loss, "beta": beta.tolist(),
            "function": "official modules/utils.py:get_optimized_shape_params",
            "optimizer_and_loss_modified": False,
        },
        "inference_timing_diagnostic_ms": {"median": float(np.median(inference_times) * 1000), "p95": float(np.percentile(inference_times, 95) * 1000), "note": "Includes anatomical-only and MANO modes; not a live-system benchmark"},
        "elapsed_s": time.monotonic() - started,
        "not_validated": ["Physical mounting", "Real three-pose root calibration", "Real fingertip-contact shape calibration", "Accuracy of 12-channel inferred joints", "Unpublished paper joint optimizer"],
    }


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--upstream", required=True, type=Path)
    parser.add_argument("--mano-assets", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--frames", type=int, default=64)
    parser.add_argument("--iterations", type=int, default=500)
    parser.add_argument("--seed", type=int, default=42)
    args = parser.parse_args()
    report = verify(args.upstream, args.mano_assets, frames=args.frames, iterations=args.iterations, seed=args.seed)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(report, indent=2, ensure_ascii=False) + "\n", encoding="utf-8")
    print(json.dumps({"status": report["status"], "report": str(args.output), "parity_max_difference": report["parity_16_imu"]["max_absolute_difference"], "shape_begin_loss": report["official_shape_optimizer_smoke"]["begin_loss"], "shape_final_loss": report["official_shape_optimizer_smoke"]["final_loss_before_last_update"]}, ensure_ascii=False))


if __name__ == "__main__":
    main()
