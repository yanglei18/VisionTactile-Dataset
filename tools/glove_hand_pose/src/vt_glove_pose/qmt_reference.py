"""Thin, optional adapter that actually calls upstream qmt; offline only.

Install qmt into a separate environment; normal application imports do not
require it. No upstream implementation is copied here. Although qmt is mostly
MIT, joint_axis_est_hinge_olsson.py is LicenseRef-Unspecified. This reference
runner is not a statement of permission to redistribute/productize that code.

Example (from tools/glove_hand_pose, using an isolated Python with qmt):
  PYTHONPATH=src python -m vt_glove_pose.qmt_reference --session EXPORT_DIR \
      --config configs/right_glove_20260930_empirical.json --output REPORT.json
"""

from __future__ import annotations

import argparse
import contextlib
import hashlib
import importlib
import importlib.metadata
import inspect
import io
import json
from pathlib import Path
import platform
import subprocess
import warnings

import numpy as np

from .config import load_config, pair_gyro_scales, write_json_exclusive
from .hinge_axes import _gyros, _integer, _scalar
from .hinge_joint_axes import evaluate_joint_hinge_axes
from .pair_capture import load_pair_capture


STANDARD_GRAVITY_M_S2 = 9.80665
UPSTREAM_URL = "https://github.com/dlaidig/qmt"
SOURCE_RELATIVE_PATH = "qmt/functions/joint_axis_est_hinge_olsson.py"


def _load_qmt():
    try:
        return importlib.import_module("qmt")
    except ImportError as exc:
        raise RuntimeError("Optional qmt reference dependency unavailable; install qmt in an isolated environment") from exc


def _sha256(path):
    digest = hashlib.sha256()
    with Path(path).open("rb") as stream:
        for chunk in iter(lambda: stream.read(1024*1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def describe_qmt(*, upstream_checkout=None):
    """Record the *executed* installed function, not merely a repository URL."""
    qmt = _load_qmt()
    source = Path(inspect.getsourcefile(qmt.jointAxisEstHingeOlsson)).resolve()
    source_text = source.read_text(encoding="utf-8")
    spdx = next((line.split("SPDX-License-Identifier:", 1)[1].strip()
                 for line in source_text.splitlines()[:20]
                 if "SPDX-License-Identifier:" in line), "not_found")
    result = dict(package="qmt", package_version=importlib.metadata.version("qmt"),
                  upstream_url=UPSTREAM_URL, function="qmt.jointAxisEstHingeOlsson",
                  signature=str(inspect.signature(qmt.jointAxisEstHingeOlsson)),
                  installed_function_source=str(source), function_source_sha256=_sha256(source),
                  function_spdx_license=spdx, package_general_license="MIT_with_file_level_exceptions",
                  license_warning="This function has no specified license; not bundled or copied into this project.",
                  execution_scope="isolated_offline_scientific_reference_not_product_distribution",
                  python_version=platform.python_version(), numpy_version=np.__version__,
                  scipy_version=importlib.metadata.version("scipy"),
                  environment_packages={d.metadata["Name"]: d.version
                                        for d in importlib.metadata.distributions() if d.metadata.get("Name")})
    if upstream_checkout is not None:
        checkout = Path(upstream_checkout).resolve()
        commit = subprocess.run(["git", "-C", str(checkout), "rev-parse", "HEAD"],
                                check=True, capture_output=True, text=True).stdout.strip()
        repository_source = checkout/SOURCE_RELATIVE_PATH
        result["upstream_checkout"] = str(checkout)
        result["upstream_commit"] = commit
        result["upstream_function_source_sha256"] = _sha256(repository_source)
        result["installed_function_identical_to_checkout"] = (
            result["function_source_sha256"] == result["upstream_function_source_sha256"])
        result["pinned_source_url"] = f"{UPSTREAM_URL}/blob/{commit}/{SOURCE_RELATIVE_PATH}"
    return result


def _inputs(prox_gyro_dps, dist_gyro_dps, prox_acc_g, dist_acc_g):
    names = ("prox_gyro_dps", "dist_gyro_dps", "prox_acc_g", "dist_acc_g")
    arrays = [_gyros(value, name) for name, value in zip(
        names, (prox_gyro_dps, dist_gyro_dps, prox_acc_g, dist_acc_g))]
    if len({len(v) for v in arrays}) != 1:
        raise ValueError("all gyro and acceleration arrays must have equal lengths")
    if any(np.any(np.abs(v) > 100.) for v in arrays[2:]):
        raise ValueError("acceleration components must not exceed 100 g")
    return arrays


def fit_qmt_reference(prox_gyro_dps, dist_gyro_dps, prox_acc_g, dist_acc_g, *,
                      sample_rate_hz, n_starts=1, seed=0, holdout_block_s=.5):
    """Call the unmodified upstream solver on training rows only.

    n_starts=1 preserves upstream defaults except quiet=True. Optional extra
    starts change x0 only and are selected using upstream's training objective.
    No settings are tuned to held-out outcomes. Upstream sample selection is
    off (its default), so joining nonadjacent training blocks does not create
    fictitious moving-average neighbors. Evaluation uses our pre-existing
    fixed metrics; it is not presented as qmt's own acceptance test.

    dps->rad/s and g->m/s^2 are explicit at this boundary. The caller must
    remove gyro bias from a separate, explicitly declared neutral recording.
    Calls are intended for a single-threaded offline process: upstream touches
    NumPy's global RNG, whose prior state is restored after every call.
    """
    gp, gd, ap, ad = _inputs(prox_gyro_dps, dist_gyro_dps, prox_acc_g, dist_acc_g)
    rate = _scalar(sample_rate_hz, "sample_rate_hz", 10., 2000.)
    block_s = _scalar(holdout_block_s, "holdout_block_s", .01, 60.)
    starts = _integer(n_starts, "n_starts", 1, 16)
    seed = _integer(seed, "seed", 0, 2**32-1)
    block = max(1, int(round(rate*block_s)))
    train = (np.arange(len(gp))//block) % 2 == 0
    if min(train.sum(), (~train).sum()) < 60:
        raise ValueError("at least 60 samples are required in each fixed split")
    qmt = _load_qmt()
    rng = np.random.default_rng(seed)
    fits, failures = [], []
    for index in range(starts):
        settings = {"quiet": True}
        if index:
            settings["x0"] = rng.uniform(-np.pi, np.pi, (4, 1))
        random_state = np.random.get_state()
        stdout = io.StringIO()
        try:
            np.random.seed((seed+index) % 2**32)
            with warnings.catch_warnings(record=True) as caught, contextlib.redirect_stdout(stdout):
                warnings.simplefilter("always")
                jp, jd, debug = qmt.jointAxisEstHingeOlsson(
                    ap[train]*STANDARD_GRAVITY_M_S2, ad[train]*STANDARD_GRAVITY_M_S2,
                    np.deg2rad(gp[train]), np.deg2rad(gd[train]),
                    estSettings=settings, debug=True, plot=False)
            jp, jd = np.asarray(jp, dtype=float).reshape(-1), np.asarray(jd, dtype=float).reshape(-1)
            if (jp.shape != (3,) or jd.shape != (3,) or not np.all(np.isfinite([jp, jd]))
                    or not np.allclose([np.linalg.norm(jp), np.linalg.norm(jd)], 1., atol=1e-7, rtol=0)):
                raise ValueError("upstream returned invalid/non-unit axes")
            optim = debug["optimVarsAxis"]
            objective = float(np.asarray(optim["f"]).reshape(-1)[0])
            if not np.isfinite(objective):
                raise ValueError("upstream returned a nonfinite objective")
            fits.append(dict(index=index, axis_prox=jp, axis_dist=jd, objective=objective,
                             objective_trace_length=int(np.asarray(optim["ftraj"]).size),
                             warnings=sorted(set(str(w.message) for w in caught)),
                             stdout=stdout.getvalue()[-4000:]))
        except (ValueError, FloatingPointError, np.linalg.LinAlgError) as exc:
            failures.append(dict(start=index, error_type=type(exc).__name__, message=str(exc)))
        finally:
            np.random.set_state(random_state)
    if not fits:
        raise RuntimeError(f"All upstream qmt starts failed: {failures}")
    selected = min(fits, key=lambda fit: fit["objective"])
    jp, jd = selected["axis_prox"], selected["axis_dist"]
    metrics = {name: evaluate_joint_hinge_axes(jp, jd, gp[mask], gd[mask], ap[mask], ad[mask])
               for name, mask in (("train", train), ("holdout", ~train), ("all_samples", np.ones(len(gp), bool)))}
    reasons = []
    for name in ("train", "holdout"):
        if metrics[name]["gyro"]["inlier_fraction"] < .9:
            reasons.append(name+"_gyro_less_than_90_percent_within_2_dps")
        accel = metrics[name]["acceleration"]
        if accel["eligible_samples"] < 60 or accel["eligible_fraction"] < .2:
            reasons.append(name+"_insufficient_acceleration_coverage")
        elif accel["eligible_inlier_fraction"] < .9:
            reasons.append(name+"_acceleration_less_than_90_percent_within_0.08_g")
    result = dict(schema_version=1, engine="upstream_qmt_jointAxisEstHingeOlsson",
                  axis_prox=jp.tolist(), axis_dist=jd.tolist(),
                  axis_sign_convention="unmodified_upstream_output_not_anatomical_flexion",
                  offline_only=True, usable_as_live_calibration=False,
                  product_accuracy_verified=False, physical_angle_ground_truth_available=False,
                  reference_status="fixed_metric_mismatch" if reasons else "fixed_metrics_only_not_product_validation",
                  reason_codes=reasons, metrics=metrics,
                  metric_scope="existing_project_fixed_metrics_not_qmt_acceptance_criteria",
                  split=dict(method="fixed_alternating_blocks_even_train", block_samples=block,
                             train_samples=int(train.sum()), holdout_samples=int((~train).sum()),
                             sample_rate_hz=rate, statistically_independent_sessions=False),
                  units=dict(adapter_gyro="deg/s", upstream_gyro="rad/s", adapter_acceleration="g",
                             upstream_acceleration="m/s^2", standard_gravity_m_s2=STANDARD_GRAVITY_M_S2),
                  upstream_settings=dict(quiet=True, w0=50., useSampleSelection=False,
                                         loss="upstream_default_squared", x0="upstream_zero_default_first_start"),
                  optimization=dict(n_starts=starts, seed=seed, selected_start=selected["index"],
                                    selection_subset="train_only", holdout_used_for_selection=False,
                                    selected_upstream_train_objective=selected["objective"],
                                    start_train_objectives=[dict(start=f["index"], objective=f["objective"],
                                                                 objective_trace_length=f["objective_trace_length"])
                                                            for f in fits], failed_starts=failures,
                                    upstream_warnings=sorted(set(w for f in fits for w in f["warnings"])),
                                    upstream_stdout=[f["stdout"] for f in fits if f["stdout"]]))
    return result


def _read_capture_pair(session, neutral_name, motion_name, config):
    finger = config["fingers"]["thumb"]
    options = dict(proximal=finger["proximal"], distal=finger["distal"],
                   sample_rate_hz=config["sample_rate_hz"], gyro_dps_per_lsb=config["gyro_dps_per_lsb"],
                   gyro_dps_per_lsb_by_node=config.get("gyro_dps_per_lsb_by_node"), max_pairs=30000)
    neutral, motion = [load_pair_capture(session/name, **options) for name in (neutral_name, motion_name)]
    for capture in (neutral, motion):
        if capture["metadata"].get("dropped_pair_candidates", 0) or capture["near_limit_mask"].any():
            raise ValueError("reference input contains dropped pairs or near-limit sensor values")
    minimum = max(240, int(2*config["sample_rate_hz"]))
    if len(neutral["prox_gyro_dps"]) < minimum:
        raise ValueError("neutral must contain at least two nominal seconds")
    bp, bd = [neutral[key].mean(axis=0) for key in ("prox_gyro_dps", "dist_gyro_dps")]
    data = (motion["prox_gyro_dps"]-bp, motion["dist_gyro_dps"]-bd,
            motion["prox_acc_raw"]*config["accel_g_per_lsb"],
            motion["dist_acc_raw"]*config["accel_g_per_lsb"])
    evidence = dict(neutral_file=neutral_name, motion_file=motion_name,
                    neutral_sha256=_sha256(session/neutral_name), motion_sha256=_sha256(session/motion_name),
                    neutral_metadata=neutral["metadata"], motion_metadata=motion["metadata"],
                    declared_neutral_bias_prox_dps=bp.tolist(), declared_neutral_bias_dist_dps=bd.tolist(),
                    neutral_scope="separate_recorded_user_declared_pose_not_verified_anatomical_zero")
    return data, evidence


def _repeatability(first, second):
    return {name+"_line_angle_deg": float(np.degrees(np.arccos(np.clip(
        abs(np.dot(first[name], second[name])), 0., 1.)))) for name in ("axis_prox", "axis_dist")}


def run_qmt_reference(session, config_path, *, upstream_checkout=None, supplemental_starts=4):
    """Reproduce the user's two declared neutral/motion pairs without hardware."""
    session, config_path = Path(session).resolve(), Path(config_path).resolve()
    config = load_config(config_path)
    starts = _integer(supplemental_starts, "supplemental_starts", 1, 16)
    source = describe_qmt(upstream_checkout=upstream_checkout)
    datasets, runs = [], []
    for neutral, motion in (("001_neutral.csv", "002_motion.csv"), ("003_neutral.csv", "004_motion.csv")):
        data, evidence = _read_capture_pair(session, neutral, motion, config)
        default = fit_qmt_reference(*data, sample_rate_hz=config["sample_rate_hz"])
        supplemental = (fit_qmt_reference(*data, sample_rate_hz=config["sample_rate_hz"], n_starts=starts)
                        if starts > 1 else default)
        datasets.append(data)
        runs.append(dict(source=evidence, upstream_default=default, supplemental_multistart=supplemental))
    comparison = {}
    for key in ("upstream_default", "supplemental_multistart"):
        first, second = (run[key] for run in runs)
        transfer = []
        for src, dst in ((0, 1), (1, 0)):
            fit = runs[src][key]
            transfer.append(dict(fit_recording=src+1, evaluate_recording=dst+1,
                                 metrics=evaluate_joint_hinge_axes(fit["axis_prox"], fit["axis_dist"], *datasets[dst]),
                                 selection_used_target_recording=False))
        comparison[key] = dict(repeatability=_repeatability(first, second), cross_recording=transfer)
    thumb = config["fingers"]["thumb"]
    historical_hashes = [
        "aad1cf27970646181322402d808b5eb47e98b5c39f2cc551243f85c08340ed19",
        "6977cd7a599b951af2bcbb973d483e713927803c9321255e4bcfa811548fabd9",
        "9c1e487bd66d16d8e589561be1eecb49d59021f428614439573915a3912afc57",
        "9a03962d1589b41ebe82ea124b88e85a7485ae5d22c5136eb721a0663e4e5526"]
    observed_hashes = [run["source"][key] for run in runs for key in ("neutral_sha256", "motion_sha256")]
    prior_context = None
    if (observed_hashes == historical_hashes and config["sample_rate_hz"] == 120.
            and config["accel_g_per_lsb"] == .000122
            and pair_gyro_scales(config, thumb["proximal"], thumb["distal"]) == (.0175, .0175)
            and (thumb["proximal"], thumb["distal"]) == (5, 6)):
        prior_context = dict(source="previous_joint_gyro_accel_experiment_not_rerun_here",
                             input_hashes_match_previous_experiment=True,
                             holdout_gyro_inlier_fraction=[.4825, .5141666666666667],
                             holdout_gyro_rms_dps=[4.152642285925175, 4.374599641685584])
    return dict(schema_version=1, kind="vt_glove_pose.actual_upstream_qmt_reference", source=source,
                session=str(session), config_path=str(config_path), config_sha256=_sha256(config_path),
                configuration=config, effective_thumb_gyro_dps_per_lsb=list(pair_gyro_scales(
                    config, thumb["proximal"], thumb["distal"])),
                scale_provenance=config.get("gyro_scale_provenance"),
                offline_only=True, live_calibration_changed=False, production_acceptance="not_evaluated",
                hardware_synchronization_verified=False, units_firmware_verified=False,
                runs=runs, comparison=comparison,
                prior_independent_candidate_context=prior_context,
                limitations=["Within-recording blocks are not statistically independent sessions.",
                             "Cross-recording transfer does not identify the cause of model mismatch.",
                             "Unverified empirical scales and software pairing do not verify firmware or synchronization.",
                             "No physical-angle truth, anatomical validity or product precision is established.",
                             "The function's LicenseRef-Unspecified is distinct from qmt's general MIT license."])


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--session", required=True)
    parser.add_argument("--config", required=True)
    parser.add_argument("--output", required=True)
    parser.add_argument("--upstream-checkout")
    parser.add_argument("--supplemental-starts", type=int, default=4)
    args = parser.parse_args(argv)
    output = Path(args.output)
    if output.exists():
        parser.error("output already exists; reference evidence is never overwritten")
    report = run_qmt_reference(args.session, args.config, upstream_checkout=args.upstream_checkout,
                               supplemental_starts=args.supplemental_starts)
    output.parent.mkdir(parents=True, exist_ok=True)
    write_json_exclusive(output, report)
    print(json.dumps(dict(output=str(output.resolve()), qmt_version=report["source"]["package_version"],
                          default_holdout=[r["upstream_default"]["metrics"]["holdout"] for r in report["runs"]],
                          default_repeatability=report["comparison"]["upstream_default"]["repeatability"])))


if __name__ == "__main__":
    main()
