"""Offline verification subprocess; never enumerate, open, or initialize hardware."""
from __future__ import annotations

import importlib
import json
from pathlib import Path
import sys


def verify(root: Path, capture_sha256: str) -> None:
    # These two modules are the publisher's existing identity/role policy. The
    # path is derived from our trusted checkout, never from the imported archive.
    repository = Path(__file__).resolve().parents[4]
    policy_source = repository / "ros2_ws/src/vt_vive_tracker"
    if not all((policy_source / "vt_vive_tracker" / name).is_file()
               for name in ("identity.py", "roles.py")):
        raise ImportError("Tracker verification requires the project's trusted checkout")
    sys.path[:0] = [
        str(root / "source"),
        str(policy_source),
    ]
    import hid

    def forbid_hardware(*args, **kwargs):
        raise RuntimeError("Deployment verification does not access hardware")

    hid.enumerate = forbid_hardware
    hid.Device = forbid_hardware
    for name in (
        "pyvut.live_hid", "pyvut.live_probe", "pyvut.pose_decoder",
        "pyvut.live_bootstrap_runner", "pyvut.live_models",
    ):
        importlib.import_module(name)
    from pyvut.bootstrap_capture import load_live_capture
    from pyvut.live_bootstrap_bundle import load_private_live_bundle, verify_bundle_matches_evidence
    from pyvut.live_bootstrap_evidence import extract_live_bootstrap_evidence
    from vt_vive_tracker.identity import tracker_id
    from vt_vive_tracker.roles import load_role_map

    bundle = load_private_live_bundle(root / "inputs/bundle.json")
    roles = load_role_map(root / "inputs/roles.yaml")
    if set(roles.by_tracker_id) != {tracker_id(address) for address in bundle.tracker_addresses}:
        raise ValueError("Tracker roles do not match bundle identities")
    transcript = load_live_capture(root / "inputs/capture.pcapng", capture_sha256)
    evidence = extract_live_bootstrap_evidence(transcript)
    verify_bundle_matches_evidence(bundle, evidence, capture_sha256)


def main() -> int:
    try:
        verify(Path(sys.argv[1]), sys.argv[2])
    except ImportError:
        print(json.dumps({"ok": False, "code": "runtime_dependency"}))
        return 1
    except Exception:
        # Third-party exceptions may contain private addresses or credentials.
        print(json.dumps({"ok": False, "code": "runtime_validation"}))
        return 1
    print(json.dumps({"ok": True}))
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
