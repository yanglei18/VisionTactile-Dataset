"""Prepare an isolated workbench configuration from explicit tested hand bundles.

Migration is a one-time import, not a recurring synchronizer. The source and
existing candidate edits are never overwritten. No hardware or Hub settings
are opened/copied, and the new directory becomes visible only when complete.
"""
from __future__ import annotations

import argparse
from copy import deepcopy
import fcntl
import hashlib
import json
import os
from pathlib import Path
import shutil
import stat
import sys
import tempfile

from vt_capture_core.workstation import validate_configuration
from vt_capture_core.hand_preflight import prepare_hand_runtimes
from vt_fsglove12.hand_service_resources import validate_bundle_manifest

from .hand_runtime_probe import probe_hand_runtime
from .workstation_settings import atomic_json, private_directory


MAX_CONFIG_BYTES = 1024 * 1024
REPORT_NAME = "migration.json"
REPORT_KIND = "visiontactile_candidate_migration"


def _digest(value):
    return hashlib.sha256(value).hexdigest()


def _json_bytes(value):
    return (json.dumps(value, ensure_ascii=False, indent=2, allow_nan=False) + "\n").encode()


def _read(path):
    descriptor = os.open(path, os.O_RDONLY | os.O_NONBLOCK | os.O_NOFOLLOW)
    with os.fdopen(descriptor, "rb") as stream:
        info = os.fstat(stream.fileno())
        if not stat.S_ISREG(info.st_mode) or info.st_size > MAX_CONFIG_BYTES:
            raise ValueError("Configuration must be a regular file of at most 1 MiB")
        payload = stream.read(MAX_CONFIG_BYTES + 1)
    if len(payload) > MAX_CONFIG_BYTES:
        raise ValueError("Configuration exceeds 1 MiB")
    value = json.loads(payload)
    if not isinstance(value, dict):
        raise ValueError("Configuration must be a JSON object")
    return value, payload


def default_candidate_settings(repo_root, *, home=None):
    root = str(Path(repo_root).expanduser().resolve())
    return Path(home or Path.home()) / ".config/visiontactile/candidates" / _digest(root.encode())[:12]


def hand_socket_namespace(settings_dir):
    """Short, candidate-specific endpoints even when a checkout path is long."""
    identity = _digest(str(Path(settings_dir).expanduser().resolve()).encode())[:12]
    return Path(f"/tmp/vt-candidates-{os.getuid()}") / identity


def _location(repo_root, settings_dir):
    root = Path(repo_root).expanduser().resolve(strict=True)
    if not (root / "tools/capture_deploy/start_manager.sh").is_file():
        raise ValueError("Candidate repo_root does not contain the workbench deployment entry")
    requested = Path(settings_dir).expanduser().absolute()
    if requested.is_symlink():
        raise ValueError("Candidate settings directory must not be a symbolic link")
    destination = requested.resolve()
    if len(os.fsencode(destination / "private-desktop.sock")) > 107:
        raise ValueError("Candidate settings path is too long for a Unix socket; choose a shorter private directory")
    return root, destination


def _selected_hands(hands, root, verify_runtime):
    if not isinstance(hands, dict) or set(hands) - {"left", "right"}:
        raise ValueError("Explicit hand selections must contain only left/right")
    selected = {}
    for hand, value in hands.items():
        if not isinstance(value, dict) or set(value) != {"resources", "glove_python"}:
            raise ValueError("Each hand selection requires resources and glove_python")
        selected[hand] = _verify_hand_choice(value, hand, root, verify_runtime, motion=True)
    return selected


def _verify_hand_choice(value, hand, root, verify_runtime, *, motion):
    bundle, _ = validate_bundle_manifest(value["resources"], hand=hand)
    evidence = dict(resources=str(bundle), glove_python="",
                    manifest_sha256=_digest((bundle / "manifest.json").read_bytes()))
    if motion:
        python = Path(value.get("glove_python", "")).expanduser().absolute()
        if not python.is_file() or not os.access(python, os.X_OK):
            raise ValueError(f"{hand} numerical Python is not executable")
        evidence.update(glove_python=str(python), runtime_probe=verify_runtime(python, bundle, hand, root))
    return evidence


def verify_candidate_runtimes(configuration, repo_root, verify_runtime=None):
    """Use the shared launch/apply verifier for currently enabled choices."""
    configuration = validate_configuration(configuration)
    proof = prepare_hand_runtimes(configuration["process_groups"], candidate_root=repo_root,
                                  verify_runtime=verify_runtime or probe_hand_runtime)
    return proof.evidence


def _build_configuration(source, selected, destination):
    draft = deepcopy(source)
    profiles = draft.get("process_groups", [])
    if not isinstance(profiles, list):
        raise ValueError("process_groups must be an array")
    endpoints = hand_socket_namespace(destination)
    changes = []
    for profile in profiles:
        if not isinstance(profile, dict):
            raise ValueError("Invalid process profile")
        preset, parameters = profile.get("preset"), profile.get("parameters", {})
        if preset not in {"glove", "tactile"}:
            continue
        if not isinstance(parameters, dict) or parameters.get("hand") not in selected:
            raise ValueError("Every existing hand profile needs an explicitly selected tested resource bundle")
        hand = parameters["hand"]
        replacement = selected[hand]
        before = deepcopy(parameters)
        # Do not reinterpret a legacy installation/zero as if it belonged to
        # the selected bundle. This is an explicit implementation replacement.
        if preset == "glove":
            for key in ("config", "rest", "fsglove_source", "mano_assets"):
                parameters.pop(key, None)
            parameters.update(launch_gui="false", control_socket=str(endpoints / f"{hand}-control.sock"),
                socket_path=str(endpoints / f"{hand}-pose.sock"))
            parameters["output_root"] = parameters.get("output_root") or str(destination / "hand-sessions" / hand)
        else:
            parameters.update(display_bridge="true", display_socket=str(endpoints / f"{hand}-tactile.sock"))
        parameters.update(resources=replacement["resources"], glove_python=replacement["glove_python"])
        profile["parameters"] = parameters
        changes.append(dict(profile_id=profile.get("id"), hand=hand, preset=preset,
                            previous_parameters=before, parameters=deepcopy(parameters)))
    # In particular, validate only AFTER explicit replacement: old enabled
    # hand profiles can legitimately predate the service's required rest field.
    return validate_configuration(draft), changes


def _selection_identity(selected):
    if not isinstance(selected, dict) or set(selected) - {"left", "right"}:
        raise ValueError("Invalid candidate hand selection record")
    if any(not isinstance(value, dict) or not {"resources", "glove_python", "manifest_sha256"} <= set(value)
           for value in selected.values()):
        raise ValueError("Incomplete candidate hand selection record")
    return {hand: {key: value[key] for key in ("resources", "glove_python", "manifest_sha256")}
            for hand, value in selected.items()}


def _existing(destination, report, verify_runtime):
    private_directory(destination)
    previous, _ = _read(destination / REPORT_NAME)
    if (previous.get("schema_version") != 1 or previous.get("kind") != REPORT_KIND
            or previous.get("repo_root") != report["repo_root"]
            or previous.get("source") != report["source"]
            or _selection_identity(previous.get("hands", {})) != _selection_identity(report["hands"])):
        raise ValueError("Existing candidate has another source or resource selection; use a new candidate directory")
    _, source_backup = _read(destination / "source-workstation.json")
    if _digest(source_backup) != previous["source"]["sha256"]:
        raise ValueError("Candidate source backup no longer matches its migration record")
    configuration, payload = _read(destination / "workstation.json")
    validate_configuration(configuration)
    # Keep intentional settings edits, but verify their actual resource and
    # interpreter choices too, so an old-checkout Python cannot sneak back in.
    verify_candidate_runtimes(configuration, report["repo_root"], verify_runtime)
    return dict(status="existing", settings_dir=str(destination), configuration=configuration,
                user_edits_preserved=_digest(payload) != previous["config_sha256"], report=previous)


def prepare_candidate(source_path, *, repo_root, settings_dir=None, hands, verify_runtime=None, check_only=False):
    """Atomically import one candidate, preserving source and subsequent edits."""
    verify_runtime = verify_runtime or probe_hand_runtime
    root, destination = _location(repo_root, settings_dir or default_candidate_settings(repo_root))
    source_path = Path(source_path).expanduser().absolute()
    if source_path.parent.resolve() == destination:
        raise ValueError("Source settings and candidate settings must be separate directories")
    source, source_bytes = _read(source_path)
    selected = _selected_hands(hands, root, verify_runtime)
    configuration, changes = _build_configuration(source, selected, destination)
    report = dict(schema_version=1, kind=REPORT_KIND, repo_root=str(root),
        source=dict(path=str(source_path.resolve()), sha256=_digest(source_bytes)),
        hands=selected, changes=changes, config_sha256=_digest(_json_bytes(configuration)),
        source_preserved=True, hardware_accessed=False, calibration_profiles_copied=False,
        hub_autostart_settings_copied=False)
    if any(len(_json_bytes(document)) > MAX_CONFIG_BYTES for document in (configuration, report)):
        raise ValueError("Prepared candidate configuration or migration report exceeds 1 MiB")
    if destination.exists():
        return _existing(destination, report, verify_runtime)
    if check_only:
        return dict(status="proposed", settings_dir=str(destination), configuration=configuration,
                    user_edits_preserved=False, report=report)
    parent = private_directory(destination.parent)
    lock_path = parent / f".{destination.name}.migration.lock"
    lock = os.open(lock_path, os.O_CREAT | os.O_RDWR | os.O_NOFOLLOW, 0o600)
    staging = None
    try:
        info = os.fstat(lock)
        if not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid() or info.st_mode & 0o077:
            raise ValueError("Candidate migration lock is not a private regular file")
        try:
            fcntl.flock(lock, fcntl.LOCK_EX | fcntl.LOCK_NB)
        except BlockingIOError as exc:
            raise ValueError("Candidate migration is already in progress") from exc
        if destination.exists():
            return _existing(destination, report, verify_runtime)
        staging = Path(tempfile.mkdtemp(prefix=f".{destination.name}.prepare-", dir=parent))
        atomic_json(staging / "workstation.json", configuration)
        atomic_json(staging / REPORT_NAME, report)
        backup = os.open(staging / "source-workstation.json", os.O_CREAT | os.O_EXCL | os.O_WRONLY, 0o600)
        with os.fdopen(backup, "wb") as stream:
            stream.write(source_bytes)
            stream.flush()
            os.fsync(stream.fileno())
        # All public entry files are complete before the directory is visible.
        os.rename(staging, destination)
        staging = None
        descriptor = os.open(parent, os.O_RDONLY | os.O_DIRECTORY)
        try:
            os.fsync(descriptor)
        finally:
            os.close(descriptor)
    finally:
        if staging is not None:
            shutil.rmtree(staging)
        os.close(lock)
    return dict(status="created", settings_dir=str(destination), configuration=configuration,
                user_edits_preserved=False, report=report)


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--source", required=True, type=Path)
    parser.add_argument("--repo-root", required=True, type=Path)
    parser.add_argument("--settings-dir", type=Path)
    parser.add_argument("--hand", action="append", nargs=3, metavar=("HAND", "RESOURCES", "PYTHON"), default=[])
    parser.add_argument("--check-only", action="store_true")
    args = parser.parse_args(argv)
    selections = {}
    for hand, resources, interpreter in args.hand:
        if hand in selections:
            parser.error("each hand may be selected only once")
        selections[hand] = dict(resources=resources, glove_python=interpreter)
    try:
        result = prepare_candidate(args.source, repo_root=args.repo_root, settings_dir=args.settings_dir,
                                   hands=selections, check_only=args.check_only)
    except (OSError, ValueError, KeyError) as exc:
        parser.exit(2, str(exc) + "\n")
    print(json.dumps(dict(status=result["status"], settings_dir=result["settings_dir"],
        user_edits_preserved=result["user_edits_preserved"], hardware_accessed=False), ensure_ascii=False))
    return 0


if __name__ == "__main__":
    sys.exit(main())
