"""Frozen, explicitly labelled contacts for the unchanged FSGlove beta fit.

This module never owns hardware. ``optimize_contacts`` is an offline operation;
live callers must use ShapeJob so loading MANO and fitting cannot stall capture.
Distances are residuals on the fitting contacts, not physical accuracy evidence.
"""
from __future__ import annotations

from contextlib import redirect_stdout
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import re
import sys
from uuid import uuid4

import numpy as np


ALGORITHM = "upstream_fsglove_touch_shape"
CONTACT_SOURCE = "user_explicit_not_inferred_from_imu"
MAX_SAMPLES_PER_FINGER = 120
MAX_SAMPLES = 4 * MAX_SAMPLES_PER_FINGER
MAX_ITERATIONS = 2000
FINGERS = {2: "index", 3: "middle", 4: "ring", 5: "little"}


def _array(value, shape, name):
    array = np.asarray(value, dtype=float)
    if array.shape != shape or not np.isfinite(array).all():
        raise ValueError(f"{name} must be a finite array of shape {shape}")
    return array


def _sha(value, name):
    if not isinstance(value, str) or re.fullmatch(r"[0-9a-f]{64}", value) is None:
        raise ValueError(f"{name} must be a SHA-256 digest")
    return value


def model_provenance(mano_assets, hand="right"):
    """Identify the supplied trusted model without deserializing or changing it."""
    if hand not in ("right", "left"):
        raise ValueError("hand must be right or left")
    path = Path(mano_assets).expanduser().resolve() / "models" / f"MANO_{hand.upper()}.pkl"
    if not path.is_file():
        raise ValueError(f"MANO model is missing: {path}. Obtain licensed assets separately; no download is performed.")
    with path.open("rb") as stream:
        digest = hashlib.file_digest(stream, "sha256").hexdigest() if hasattr(hashlib, "file_digest") else hashlib.sha256(stream.read()).hexdigest()
    return dict(path=str(path), sha256=digest, hand=hand)


def _validate_upstream(actual, expected):
    if not isinstance(actual, dict) or not isinstance(expected, dict):
        raise ValueError("shape profile requires upstream source provenance")
    sources = expected.get("sources")
    if (not expected.get("upstream_commit") or not isinstance(sources, dict) or not sources
            or actual.get("upstream_commit") != expected["upstream_commit"]
            or actual.get("upstream_project") != expected.get("upstream_project")
            or actual.get("numerical_ast_modified") is not False):
        raise ValueError("shape profile uses a different or unverified upstream implementation")
    actual_sources = actual.get("sources")
    if not isinstance(actual_sources, dict) or set(actual_sources) != set(sources):
        raise ValueError("shape profile uses a different upstream implementation")
    for name, record in sources.items():
        expected_sha = _sha(record.get("sha256"), f"expected upstream {name}")
        if not isinstance(actual_sources[name], dict) or actual_sources[name].get("sha256") != expected_sha:
            raise ValueError("shape profile uses a different upstream implementation")


def validate_shape_profile(document, provenance):
    """Return an independent validated CLI-compatible shape profile.

    Source identity is the recorded commit and source bytes, never local paths.
    Supply ``provenance['mano_model']`` to also require the same MANO model bytes.
    Historical CLI profiles lacking a model digest cannot pass that stronger
    check; they must be regenerated, not silently labelled model-verified.
    """
    if not isinstance(document, dict):
        raise ValueError("shape profile must be a JSON object")
    if document.get("algorithm") != ALGORITHM or document.get("shape_calibrated") is not True:
        raise ValueError("shape profile is not an upstream-shape result")
    if document.get("contact_labels_source") != CONTACT_SOURCE:
        raise ValueError("shape profile requires explicit user-observed contact labels")
    if document.get("physical_accuracy_verified", False) is not False:
        raise ValueError("touch-shape fitting does not verify physical accuracy")
    _validate_upstream(document.get("upstream"), provenance)
    params = _array(document.get("shape_params"), (10,), "shape_params")
    actual_model, expected_model = document.get("mano_model"), provenance.get("mano_model")
    if actual_model is not None:
        if not isinstance(actual_model, dict) or actual_model.get("hand") not in ("right", "left"):
            raise ValueError("shape profile requires left/right MANO model provenance")
        _sha(actual_model.get("sha256"), "shape MANO model")
    if expected_model is not None:
        if (not isinstance(expected_model, dict) or expected_model.get("hand") not in ("right", "left")):
            raise ValueError("expected MANO model provenance is invalid")
        _sha(expected_model.get("sha256"), "expected MANO model")
        if actual_model is None:
            raise ValueError("shape profile has no MANO model fingerprint; rerun touch-shape calibration with this model")
        if any(actual_model[key] != expected_model[key] for key in ("hand", "sha256")):
            raise ValueError("shape profile uses a different MANO model")
    if "source_sha256" in document:
        _sha(document["source_sha256"], "contact source")
    result = deepcopy(document)
    result["shape_params"] = params.tolist()
    # Reject non-JSON or non-finite metadata as well as non-finite beta arrays.
    json.dumps(result, allow_nan=False)
    return result


def validate_contacts(contacts):
    """Validate the bounded snapshot before loading optional model dependencies."""
    if not isinstance(contacts, dict):
        raise ValueError("contacts must be a JSON object")
    if contacts.get("contact_labels_confirmed") is not True:
        raise ValueError("contact_labels_confirmed=true requires explicitly observed fingertip contact labels")
    if contacts.get("contact_labels_source", CONTACT_SOURCE) != CONTACT_SOURCE:
        raise ValueError("contact labels must be explicitly user-observed, not inferred from IMU")
    if "mano_model" in contacts:
        model = contacts["mano_model"]
        if not isinstance(model, dict) or model.get("hand") not in ("right", "left"):
            raise ValueError("contacts require valid left/right MANO model provenance")
        _sha(model.get("sha256"), "contact MANO model")
    poses = np.asarray(contacts.get("mano_pose_rotvec"), dtype=float)
    labels = np.asarray(contacts.get("finger_category"))
    if (poses.ndim != 3 or poses.shape[1:] != (16, 3) or not 4 <= len(poses) <= MAX_SAMPLES
            or not np.isfinite(poses).all() or labels.shape != (len(poses),)
            or labels.dtype.kind not in "iu" or not np.isin(labels, tuple(FINGERS)).all()
            or set(labels.tolist()) != set(FINGERS)):
        raise ValueError("contacts require finite [N,16,3] MANO poses, at most 480 samples, and all four integer categories 2..5")
    if any(np.count_nonzero(labels == category) > MAX_SAMPLES_PER_FINGER for category in FINGERS):
        raise ValueError("contacts allow at most 120 samples per finger")
    before = _array(contacts.get("before_shape_params", [0.] * 10), (10,), "before_shape_params")
    result = deepcopy(contacts)
    result.update(mano_pose_rotvec=poses.tolist(), finger_category=labels.tolist(),
                  before_shape_params=before.tolist())
    json.dumps(result, allow_nan=False)
    return result


def _write_json(path, document):
    # Link a fully written sibling into place exclusively. Interruption can
    # leave a .partial file, never a truncated loadable shape/report JSON.
    temporary = path.with_name(f".{path.name}.{uuid4().hex}.partial")
    try:
        with temporary.open("x", encoding="utf-8") as stream:
            json.dump(document, stream, indent=2, ensure_ascii=False, allow_nan=False)
            stream.write("\n")
            stream.flush()
            os.fsync(stream.fileno())
        os.link(temporary, path)
    finally:
        temporary.unlink(missing_ok=True)


def cancelled_result():
    return dict(status="cancelled", candidate_available=False, shape_profile=None,
                shape_path=None, physical_accuracy_verified=False, hardware_accessed=False)


def _gap_statistics(values):
    return dict(mean_gap_mm=float(np.mean(values)), median_gap_mm=float(np.median(values)),
                rms_gap_mm=float(np.sqrt(np.mean(np.square(values)))), max_gap_mm=float(np.max(values)))


def _contact_metrics(core, poses, labels, before, after, cancelled, *, hand="right"):
    """Evaluate real ManoLayer joints in metres with the live geometry settings.

    Wrist centering is the same as ManoBackend.render_mano. Translation does
    not change the tip-to-tip residual used by the official contact objective.
    """
    torch = core.torch
    model = core.utils.ManoLayer(rot_mode="axisang", use_pca=False, side=hand,
                                center_idx=0, flat_hand_mean=True).eval()
    distances, previews = [], {}
    with torch.no_grad():
        for betas in (before, after):
            if cancelled():
                return None, None
            output = model(torch.tensor(poses.reshape(len(poses), 48), dtype=torch.float32),
                           torch.tensor(np.repeat(betas[None], len(poses), axis=0), dtype=torch.float32))
            joints = np.asarray(output.joints.detach().cpu().numpy(), dtype=float)
            if joints.shape != (len(poses), 21, 3) or not np.isfinite(joints).all():
                raise ValueError("MANO returned invalid contact-report geometry")
            gap = np.linalg.norm(joints[:, 4] - joints[np.arange(len(labels)), labels * 4], axis=1) * 1000.
            distances.append(gap)
    pairs = {}
    for category, name in FINGERS.items():
        chosen = labels == category
        index = int(np.flatnonzero(chosen)[0])
        pairs[name] = dict(finger_category=category, sample_count=int(chosen.sum()),
                           before=_gap_statistics(distances[0][chosen]), after=_gap_statistics(distances[1][chosen]))
        previews[name] = dict(sample_index=index, finger_category=category,
                              mano_pose_rotvec=poses[index].tolist(),
                              contact_label_source=CONTACT_SOURCE,
                              pose_source="actual_captured_mano_pose_not_a_synthetic_gesture")
    metrics = dict(kind="training_contact_fit_residual_not_physical_accuracy", units="mm",
                   geometry_units="metres", mm_per_metre=1000., thumb_joint=4,
                   opposed_tip_joint="finger_category * 4", per_pair=pairs,
                   before=_gap_statistics(distances[0]), after=_gap_statistics(distances[1]),
                   warnings=["Residuals reuse the fitted contacts; no held-out anatomical accuracy was measured.",
                             "IMU pose error and inferred missing joints can affect fitted MANO shape.",
                             "The unchanged official optimizer starts from seeded random beta, not the displayed before shape."])
    return metrics, previews


def optimize_contacts(contacts, checkout, mano_assets, directory, *, iterations=500, seed=0,
                      cancelled=lambda: False, progress=None, hand=None):
    """Run the unchanged official beta optimizer once on a frozen snapshot.

    All files are exclusive in a newly created directory. The original contacts
    are retained even if fitting is cancelled; cancelled runs expose no usable
    candidate. No iterations, pose correction, contact labels, or loss terms
    are substituted into the upstream numerical routine.
    """
    contacts = validate_contacts(contacts)
    model_hand = contacts.get("mano_model", {}).get("hand")
    hand = (model_hand or "right") if hand is None else hand
    if hand not in ("right", "left") or (model_hand is not None and model_hand != hand):
        raise ValueError("contact MANO model hand does not match requested hand")
    if type(iterations) is not int or not 1 <= iterations <= MAX_ITERATIONS:
        raise ValueError("iterations must be an integer in [1, 2000]")
    if type(seed) is not int or not 0 <= seed < 2**32:
        raise ValueError("seed must be an integer in [0, 2**32)")
    if cancelled():
        return cancelled_result()
    directory = Path(directory).expanduser().resolve()
    directory.mkdir(parents=True, exist_ok=False)
    contacts_path, shape_path, report_path = (directory / name for name in ("contacts.json", "shape.json", "report.json"))
    _write_json(contacts_path, contacts)

    def stage(name):
        if progress is not None:
            progress(dict(stage=name))

    def stopped():
        if not cancelled():
            return None
        result = cancelled_result()
        result.update(contacts_path=str(contacts_path), report_path=str(report_path))
        _write_json(report_path, result)
        return result

    stage("loading_model")
    if (result := stopped()) is not None:
        return result
    from .upstream_core import load_upstream_core
    core = load_upstream_core(checkout, mano_assets)
    core.torch.set_num_threads(1)
    if (result := stopped()) is not None:
        return result
    model = model_provenance(mano_assets, hand=hand)
    provenance = {**core.provenance, "mano_model": model}
    if "upstream" in contacts:
        _validate_upstream(contacts["upstream"], core.provenance)
    if "mano_model" in contacts and contacts["mano_model"] != model:
        # Paths may move without changing model identity.
        supplied = contacts["mano_model"]
        if not isinstance(supplied, dict) or any(supplied.get(k) != model[k] for k in ("hand", "sha256")):
            raise ValueError("captured contacts use a different MANO model")
    if (result := stopped()) is not None:
        return result
    poses = np.asarray(contacts["mano_pose_rotvec"], dtype=float)
    labels = np.asarray(contacts["finger_category"], dtype=np.int64)
    before = np.asarray(contacts["before_shape_params"], dtype=float)
    stage("optimizing")
    if (result := stopped()) is not None:
        return result
    with redirect_stdout(sys.stderr):
        betas = core.optimize_shape(core.torch.tensor(poses, dtype=core.torch.float32),
                                    core.torch.tensor(labels, dtype=core.torch.long), hand,
                                    iter_num=iterations, seed=seed)
    if (result := stopped()) is not None:
        return result
    betas = _array(betas, (10,), "official optimizer shape_params")
    stage("evaluating_contacts")
    if (result := stopped()) is not None:
        return result
    metrics, previews = _contact_metrics(core, poses, labels, before, betas, cancelled, hand=hand)
    if (result := stopped()) is not None:
        return result
    profile = validate_shape_profile(dict(algorithm=ALGORITHM, shape_params=betas.tolist(),
        shape_calibrated=True, upstream=deepcopy(core.provenance), mano_model=model,
        source_sha256=hashlib.sha256(contacts_path.read_bytes()).hexdigest(),
        iterations=iterations, seed=seed, physical_accuracy_verified=False,
        contact_labels_source=CONTACT_SOURCE), provenance)
    report = dict(status="completed", candidate_available=True, shape_profile=profile,
        shape_path=str(shape_path), contacts_path=str(contacts_path), report_path=str(report_path),
        metrics=metrics, before_shape_params=before.tolist(), after_shape_params=betas.tolist(),
        preview_samples=previews, sample_count=len(poses), physical_accuracy_verified=False,
        hardware_accessed=False, numerical_algorithm_modified=False)
    if (result := stopped()) is not None:
        return result
    _write_json(shape_path, profile)
    _write_json(report_path, report)
    stage("completed")
    return report
