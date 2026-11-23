"""Durable hand provenance separate from bounded UI metadata previews."""
from copy import deepcopy
import hashlib
import json
from pathlib import Path
import re

MAX_EVIDENCE_BYTES = 8 * 1024 * 1024
PHASES = {"before_start", "after_stop"}


def _encoded(value):
    return (json.dumps(value, sort_keys=True, separators=(",", ":"), allow_nan=False) + "\n").encode()


def _validate_identity(trial_id, phase):
    if not re.fullmatch(r"[0-9a-f]{32}", trial_id) or phase not in PHASES:
        raise ValueError("Invalid hand evidence archive identity")


def freeze(db, trial_id, phase, evidence):
    """Commit full evidence in the same transaction as its trial metadata."""
    _validate_identity(trial_id, phase)
    raw = _encoded(evidence)
    if len(raw) > MAX_EVIDENCE_BYTES:
        raise ValueError("Hand trial evidence exceeds its archive size limit")
    if (not isinstance(evidence, dict) or evidence.get("schema_version") != 1
            or evidence.get("observation_phase") != phase or evidence.get("boundary_exact") is not False
            or not isinstance(evidence.get("hands"), dict) or set(evidence["hands"]) - {"left", "right"}):
        raise ValueError("Invalid hand trial evidence contract")
    summary = deepcopy(evidence)
    for hand, row in summary["hands"].items():
        artifacts = row.get("artifacts", {})
        if not isinstance(artifacts, dict) or set(artifacts) - {"config", "rest", "profile"}:
            raise ValueError("Unexpected hand artifact name")
        for name, artifact in artifacts.items():
            document = artifact.pop("document", None)
            if not isinstance(document, dict):
                raise ValueError("Hand artifact must retain its original JSON document")
            checksum = hashlib.sha256(_encoded(document)[:-1]).hexdigest()
            if checksum != artifact.get("sha256"):
                raise ValueError("Hand artifact does not match its active canonical hash")
            artifact["relative_path"] = f"{trial_id}/hands/{phase}/{hand}/{name}.json"
    relative = f"{trial_id}/hands/{phase}/evidence.json"
    summary["archive"] = dict(relative_path=relative, sha256=hashlib.sha256(raw).hexdigest(),
                              hash_semantics="file_bytes_sha256", complete_originals_retained=True)
    # No truncation is permissible for the authoritative observation summary.
    if len(_encoded(summary)) > 96 * 1024:
        raise ValueError("Hand evidence summary exceeds its metadata bound")
    db.execute("INSERT INTO hand_trial_evidence(trial_id,phase,document_json) VALUES(?,?,?)",
               (trial_id, phase, raw.decode()))
    return summary


def write_archives(store, trial_id):
    with store._mutex:
        rows = list(store._db.execute("SELECT phase,document_json FROM hand_trial_evidence WHERE trial_id=?", (trial_id,)))
    for row in rows:
        phase = row["phase"]
        _validate_identity(trial_id, phase)
        evidence = json.loads(row["document_json"])
        # Every component is validated/private; do not follow an output_path
        # supplied by a recorder into an unrelated data or source directory.
        from .storage import private_directory
        root = private_directory(store.manifests / trial_id)
        hands = private_directory(root / "hands")
        destination = private_directory(hands / phase)
        for hand, detail in evidence["hands"].items():
            if hand not in {"left", "right"}:
                raise ValueError("Invalid archived hand")
            for name, artifact in detail.get("artifacts", {}).items():
                if name not in {"config", "rest", "profile"}:
                    raise ValueError("Invalid archived artifact")
                directory = private_directory(destination / hand)
                store._write_immutable(directory / f"{name}.json", _encoded(artifact["document"]))
        store._write_immutable(destination / "evidence.json", row["document_json"].encode())
