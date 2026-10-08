#!/usr/bin/env python3
"""Render three explicit model-only poses, without reading any glove device.

Requires the optional MANO/Polyscope environment and a trusted local MANO asset.
Screenshots and a provenance report are written to a NEW directory. These are
model integration checks, not reconstructed IMU poses or evidence of accuracy.
"""
from __future__ import annotations

import argparse
from datetime import datetime, timezone
import hashlib
import importlib.metadata
import json
from pathlib import Path

import numpy as np
from scipy.spatial.transform import Rotation

from vt_fsglove12.mano import ManoBackend


def model_poses():
    neutral = np.zeros((16, 3))
    curled = neutral.copy()
    for proximal in (1, 4, 7, 10):
        curled[proximal, 2] = .7
        curled[proximal + 1, 2] = 1.
        curled[proximal + 2, 2] = .65
    curled[14, 1:] = [-.15, .4]
    curled[15, 2] = .6
    return [
        ("model_only_neutral", neutral, Rotation.identity()),
        ("model_only_curled", curled, Rotation.identity()),
        ("model_only_rotated", neutral, Rotation.from_euler("XYZ", [.35, -.4, .5])),
    ]


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--mano-assets", required=True, type=Path)
    parser.add_argument("--output-dir", required=True, type=Path)
    parser.add_argument("--model-source-url", help="Recorded as metadata only; never fetched")
    parser.add_argument("--backend", choices=("openGL3_egl", "openGL3_glfw"), default="openGL3_egl")
    args = parser.parse_args()
    if args.output_dir.exists():
        parser.error("output-dir already exists; choose a new directory (nothing overwritten)")

    backend = ManoBackend(args.mano_assets)
    rest = backend.export_rest()
    poses = model_poses()
    geometries = [backend.render(angles, root.as_quat()) for _, angles, root in poses]
    vertices = [np.asarray(g["vertices_m"]) for g in geometries]
    joints = [np.asarray(g["joints_m"]) for g in geometries]
    if any(not np.isfinite(v).all() or v.shape != (778, 3) for v in vertices):
        raise RuntimeError("MANO did not produce finite 778-vertex meshes")
    if any(not np.allclose(j[0], 0., atol=1e-6) for j in joints):
        raise RuntimeError("MANO wrist is not the local origin")
    if np.allclose(vertices[0], vertices[1], atol=1e-5):
        raise RuntimeError("Preset curl did not change model geometry")
    root_error = float(np.abs(vertices[2] - poses[2][2].apply(vertices[0])).max())
    if root_error > 5e-6:
        raise RuntimeError(f"Model root rotation mismatch: {root_error:g} m")

    import polyscope as ps

    # No preference files or persistent desktop settings are read/written.
    ps.set_use_prefs_file(False)
    ps.set_program_name("MANO integration check — MODEL PRESETS, NOT IMU")
    ps.set_window_size(960, 960)
    ps.init(args.backend)
    try:
        ps.set_up_dir("neg_x_up")
        ps.set_ground_plane_mode("none")
        ps.set_background_color((.95, .96, .97))
        args.output_dir.mkdir(parents=True, exist_ok=False)
        mesh = ps.register_surface_mesh(
            "Model-only preset — not IMU reconstruction", vertices[0],
            np.asarray(geometries[0]["faces"], dtype=int),
            color=(.29, .72, .64), smooth_shade=True,
        )
        # Same camera in all three images, so root rotation remains visible.
        low, high = vertices[0].min(axis=0), vertices[0].max(axis=0)
        center = (low + high) / 2
        span = float(np.linalg.norm(high - low))
        ps.look_at(center + span * np.array([.15, -1.8, .65]), center)
        files = []
        for (name, angles, root), points in zip(poses, vertices):
            mesh.update_vertex_positions(points)
            destination = args.output_dir / f"{name}.png"
            # No show() loop: EGL rendering completes without opening a window.
            ps.screenshot(str(destination), transparent_bg=False)
            if not destination.is_file() or destination.stat().st_size == 0:
                raise RuntimeError(f"Screenshot was not written: {destination}")
            files.append({
                "name": name, "screenshot": destination.name,
                "anatomical_euler_rad": angles.tolist(),
                "root_quat_xyzw": root.as_quat().tolist(),
            })
        versions = {name: importlib.metadata.version(name) for name in (
            "numpy", "scipy", "vqf", "torch", "manotorch", "chumpy", "polyscope",
        )}
        report = {
            "status": "model_integration_passed", "created_utc": datetime.now(timezone.utc).isoformat(),
            "input_kind": "explicit_model_presets_not_imu",
            "hardware_accessed": False, "glove_accuracy_verified": False,
            "yaw_drift_corrected": False, "upstream_fsglove_solver_executed": False,
            "model_file": str(backend.model_file),
            "model_source_url": args.model_source_url,
            "model_sha256": hashlib.sha256(backend.model_file.read_bytes()).hexdigest(),
            "model_bytes": backend.model_file.stat().st_size,
            "render_backend": args.backend, "headless": ps.is_headless(),
            "vertex_count": 778, "joint_count": 21, "anatomical_slot_count": 16,
            "root_rotation_max_abs_error_m": root_error,
            "provenance": rest["provenance"], "dependency_versions": versions, "poses": files,
            "license_note": "Third-party hosting does not change MANO terms; assets remain local, not redistributed.",
        }
        with (args.output_dir / "verification_report.json").open("x", encoding="utf-8") as stream:
            json.dump(report, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
        print(json.dumps({"status": report["status"], "input_kind": report["input_kind"],
                          "report": str(args.output_dir / "verification_report.json")}, indent=2))
    finally:
        ps.shutdown()


if __name__ == "__main__":
    main()
