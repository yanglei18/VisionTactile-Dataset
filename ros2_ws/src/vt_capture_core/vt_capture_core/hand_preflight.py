"""Shared hardware-free admission checks for hand configuration and launch.

Preparation loads resource bytes and runs an isolated numerical probe. Its
proof uses only file metadata at the final launch boundary, never model IO.
"""
from __future__ import annotations
from copy import deepcopy
from dataclasses import dataclass
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat

from .hand_runtime_probe import probe_hand_runtime
from .workstation import parse_profiles

MAX_FILES = 4096
MAX_MANIFEST_BYTES = 2 * 1024 * 1024


def _profile_bytes(profiles):
    return json.dumps(profiles, sort_keys=True, ensure_ascii=False, allow_nan=False).encode()


def _identity(path):
    resolved = Path(path).resolve(strict=True)
    info = resolved.stat()
    if not stat.S_ISREG(info.st_mode):
        raise ValueError(f"Hand preflight expected a regular file: {path}")
    return (str(resolved), info.st_dev, info.st_ino, info.st_size, info.st_mtime_ns, info.st_ctime_ns)


@dataclass(frozen=True)
class HandPreflightProof:
    _profiles: object
    _configuration_bytes: bytes
    _files: tuple
    _evidence: dict

    @property
    def evidence(self):
        """Return a detached report; editing it cannot alter the proof."""
        return deepcopy(self._evidence)

    def assert_current(self):
        """Reject stale preparation with bounded metadata checks only."""
        if _profile_bytes(self._profiles) != self._configuration_bytes:
            raise ValueError("Hand configuration changed after preflight; verify it again")
        for path, expected in self._files:
            try:
                actual = _identity(path)
            except (OSError, ValueError) as exc:
                raise ValueError(f"Hand preflight resource is no longer available: {path}") from exc
            if actual != expected:
                raise ValueError(f"Hand runtime or resource changed after preflight: {path}")


def _bind(files, path):
    name = str(Path(path).expanduser().absolute())
    stamp = _identity(name)
    if name in files and files[name] != stamp:
        raise ValueError(f"Hand resource changed during preflight: {name}")
    files[name] = stamp
    if len(files) > MAX_FILES:
        raise ValueError("Hand preflight file inventory exceeds its limit")


def _bind_sources(files, root):
    root = Path(root)
    if not root.is_dir():
        raise ValueError(f"Hand numerical source directory is missing: {root}")
    for path in root.rglob("*.py"):
        _bind(files, path)


def _verify_bundle(bundle, hand, files):
    # This canonical resource module itself uses only the standard library.
    # Do not import it for camera/tracker or ordinary tactile-only recipes.
    from vt_fsglove12.hand_service_resources import validate_bundle_manifest
    requested = Path(bundle).expanduser().absolute()
    manifest_path = requested / "manifest.json"
    _bind(files, manifest_path)
    with manifest_path.open("rb") as stream:
        raw = stream.read(MAX_MANIFEST_BYTES + 1)
    if len(raw) > MAX_MANIFEST_BYTES:
        raise ValueError("Hand resource manifest exceeds its limit")
    manifest = json.loads(raw)
    if not isinstance(manifest, dict) or not isinstance(manifest.get("files"), dict):
        raise ValueError("Invalid hand resource manifest")
    root = requested.resolve(strict=True)
    for name in manifest["files"]:
        relative = PurePosixPath(name)
        if relative.is_absolute() or ".." in relative.parts or "\\" in name:
            raise ValueError("Invalid resource manifest path")
        path = requested / name
        path.resolve(strict=True).relative_to(root)
        _bind(files, path)
    verified, _ = validate_bundle_manifest(requested, hand=hand)
    return verified, hashlib.sha256(raw).hexdigest()


def prepare_hand_runtimes(process_groups, *, candidate_root=None, target_group_id=None,
                          verify_runtime=None):
    """Prepare enabled hands or one requested device without opening hardware.

    Candidate mode requires bundles and imports from ``candidate_root``.
    Ordinary mode preserves explicit model/config/rest/source paths. Pure
    tactile mode never probes its unused numerical interpreter. Optional
    ``verify_runtime`` retains migration's injectable numerical verifier.
    """
    before = _profile_bytes(process_groups)
    profiles = parse_profiles(list(process_groups))
    try:
        root = Path(candidate_root).expanduser().resolve(strict=True) if candidate_root is not None else None
    except OSError as exc:
        raise ValueError(f"Candidate source directory is unavailable: {candidate_root}") from exc
    target = None
    if target_group_id is not None:
        target = next((group for group in profiles if group["id"] == target_group_id), None)
        if target is None:
            raise ValueError(f"Unknown hand preflight group: {target_group_id}")
    selected = [group for group in profiles if group["enabled"] and group["preset"] in {"glove", "tactile"}
                and (target is None or group["id"] == target["id"])]
    selected_hands = {group["parameters"]["hand"] for group in selected}
    # Check agreement with enabled sibling displays without probing unrelated
    # hands or an unused motion interpreter when starting only tactile.
    choices = {}
    for group in profiles:
        if not group["enabled"] or group["preset"] not in {"glove", "tactile"}:
            continue
        p, hand = group["parameters"], group["parameters"]["hand"]
        if hand not in selected_hands:
            continue
        bundle = p.get("resources", "")
        if root is not None and not bundle:
            raise ValueError(f"Enabled {hand} candidate profile requires an explicit resource bundle")
        if bundle:
            canonical = str(Path(bundle).resolve())
            if hand in choices and choices[hand] != canonical:
                raise ValueError("One hand has conflicting display resources")
            choices[hand] = canonical

    files, evidence, bundles = {}, {}, {}
    probe = verify_runtime or probe_hand_runtime
    try:
        for group in selected:
            p, hand = group["parameters"], group["parameters"]["hand"]
            report = evidence.setdefault(hand, dict(resources="", glove_python=""))
            bundle = p.get("resources", "")
            if bundle:
                key = (hand, str(Path(bundle).absolute()))
                if key not in bundles:
                    bundles[key] = _verify_bundle(bundle, hand, files)
                verified, digest = bundles[key]
                report.update(resources=str(verified), manifest_sha256=digest)
            if group["preset"] == "tactile":
                # Display resources belong to this hand; tactile configuration
                # semantics remain with the tactile reader's existing loader.
                continue
            python = Path(p["glove_python"]).expanduser().absolute()
            if not python.is_file() or not os.access(python, os.X_OK):
                raise ValueError(f"{hand} numerical Python is not executable: {python}")
            _bind(files, python)
            explicit = None
            if bundle:
                if any(p.get(key) for key in ("config", "rest", "mano_assets", "fsglove_source")):
                    raise ValueError("Use either a resource bundle or explicit model/configuration paths")
                upstream = Path(bundle) / "upstream"
                if upstream.is_dir():
                    _bind_sources(files, upstream)
            else:
                explicit = {key: p[key] for key in ("config", "rest", "mano_assets", "fsglove_source")}
                for key in ("config", "rest"):
                    _bind(files, explicit[key])
                _bind(files, Path(explicit["mano_assets"]) / "models" / f"MANO_{hand.upper()}.pkl")
                _bind_sources(files, explicit["fsglove_source"])
            if root is not None:
                source = root / "tools/fsglove12/src/vt_fsglove12"
                if source.is_dir():
                    _bind_sources(files, source)
            options = {}
            if explicit is not None:
                options["explicit"] = explicit
            if p.get("output_rate_hz"):
                options["output_rate_hz"] = p["output_rate_hz"]
            result = probe(python, Path(bundle) if bundle else None, hand, root, **options)
            checked = result.get("checked_files", {})
            if not isinstance(checked, dict) or len(checked) > MAX_FILES:
                raise ValueError("Invalid numerical source identity evidence")
            for path, identity in checked.items():
                if (not isinstance(path, str) or not Path(path).is_absolute()
                        or not isinstance(identity, (tuple, list)) or len(identity) != 6):
                    raise ValueError("Invalid numerical source identity evidence")
                if path in files and files[path] != tuple(identity):
                    raise ValueError(f"Hand runtime changed during preflight: {path}")
                files[path] = tuple(identity)
                if len(files) > MAX_FILES:
                    raise ValueError("Hand preflight file inventory exceeds its limit")
            report.update(glove_python=str(python), runtime_probe=result)
        proof = HandPreflightProof(process_groups, before, tuple(files.items()), deepcopy(evidence))
        proof.assert_current()
        return proof
    except (OSError, ImportError, KeyError, TypeError) as exc:
        raise ValueError(f"Hand runtime preflight failed: {exc}") from exc
