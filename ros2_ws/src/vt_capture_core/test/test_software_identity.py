"""Source-bound v8 identity, independent of hardware, Qt and ROS."""
from copy import deepcopy
import json
from pathlib import Path
import subprocess
import sys

import pytest

from vt_capture_core import (IdentityControlRequest, IdentityControlResponse, IdentityControlSnapshot,
                             ProtocolError, decode_message, encode_message)
from vt_capture_core import software_identity as identity


def software(**changes):
    return dict(source_root="/candidate", source_revision="a" * 40,
                source_sha256="b" * 64, configuration_path="/candidate/settings.json") | changes


def tree(root):
    for name in identity.ROOTS:
        directory = root / name
        directory.mkdir(parents=True)
        (directory / "runtime.py").write_text("VALUE = 1\n")
    entry = root / "tools/capture_deploy/start_manager.sh"
    entry.parent.mkdir(parents=True)
    entry.write_text("#!/bin/sh\nexit 0\n")
    return root


@pytest.fixture(autouse=True)
def isolated_cache():
    identity._source_identity.cache_clear()
    yield
    identity._source_identity.cache_clear()


def test_source_anchor_and_equal_bytes_do_not_merge_different_worktrees(tmp_path):
    first, second = tree(tmp_path / "first"), tree(tmp_path / "second")
    config = tmp_path / "settings.json"
    a, b = identity.runtime_identity(first, config), identity.runtime_identity(second, config)
    assert identity.source_root(first / "ros2_ws/src/runtime.py") == first
    assert identity.source_root(tmp_path / "elsewhere/runtime.py") is None
    assert a["source_sha256"] == b["source_sha256"]
    assert a["source_root"] != b["source_root"]
    with pytest.raises(ValueError, match="来源"):
        identity.require_identity(a, b)


def test_configuration_location_is_bound_without_hashing_user_recordings(tmp_path):
    root = tree(tmp_path / "repo")
    a = identity.runtime_identity(root, tmp_path / "first.json")
    b = identity.runtime_identity(root, tmp_path / "second.json")
    assert a["source_sha256"] == b["source_sha256"]
    assert a["configuration_path"] != b["configuration_path"]
    with pytest.raises(ValueError):
        identity.require_identity(a, b)


def test_loaded_process_keeps_old_identity_but_new_process_observes_source_edit(tmp_path):
    root = tree(tmp_path / "repo")
    before = identity.runtime_identity(root)
    (root / "ros2_ws/src/runtime.py").write_text("VALUE = 2\n")
    assert identity.runtime_identity(root) == before
    package = str(Path(identity.__file__).resolve().parents[1])
    program = ("import json,sys;sys.path.insert(0,sys.argv[1]);"
               "from vt_capture_core.software_identity import runtime_identity;"
               "print(json.dumps(runtime_identity(sys.argv[2])))")
    result = subprocess.run([sys.executable, "-I", "-c", program, package, str(root)],
                            capture_output=True, text=True, timeout=10, check=True)
    after = json.loads(result.stdout)
    assert after["source_root"] == before["source_root"]
    assert after["source_sha256"] != before["source_sha256"]


def test_runtime_checksum_excludes_tests_and_private_runtime_resources(tmp_path):
    root = tree(tmp_path / "repo")
    before = identity.runtime_identity(root)
    for name in ("ros2_ws/src/test/test_local.py", ".runtime/model.json", "recordings/raw.json"):
        path = root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_text("private runtime contents")
    identity._source_identity.cache_clear()  # Model a fresh process with the same executable sources.
    assert identity.runtime_identity(root) == before


def test_identity_rejects_source_symlink_outside_selected_tree(tmp_path):
    root = tree(tmp_path / "repo")
    external = tmp_path / "other.py"
    external.write_text("VALUE = 10")
    (root / "ros2_ws/src/escape.py").symlink_to(external)
    with pytest.raises(ValueError):
        identity.runtime_identity(root)


def test_incomplete_tree_never_produces_candidate_identity(tmp_path):
    with pytest.raises(ValueError, match="Incomplete runtime source tree"):
        identity.runtime_identity(tmp_path)


@pytest.mark.parametrize("changes", [
    dict(source_root="relative"), dict(configuration_path="relative"),
    dict(source_revision="not-a-revision"), dict(source_sha256="short"),
    dict(source_root=""), dict(source_sha256=""), dict(unrecognized="field"),
])
def test_invalid_identity_fields_rejected(changes):
    with pytest.raises(ProtocolError):
        identity.validate_identity(software(**changes))


def test_unknown_identity_is_valid_for_readonly_snapshot_but_cannot_be_pinned():
    unknown = identity.empty_identity()
    identity.validate_identity(unknown)
    with pytest.raises(ValueError):
        identity.require_identity(unknown, unknown)


def test_v8_snapshot_and_dto_round_trip_require_software():
    state = IdentityControlSnapshot("live", "manager", software=software())
    response = IdentityControlResponse(request_id="reply", ok=True, snapshot=state)
    wire = response.to_dict()
    assert decode_message(encode_message(response)) == wire
    invalid = deepcopy(wire)
    invalid["snapshot"].pop("software")
    with pytest.raises(ProtocolError):
        encode_message(invalid)


def test_v8_mutation_requires_explicit_nonempty_source_binding():
    for expected in (None, identity.empty_identity()):
        value = dict(version=8, request_id="write", command="batch.create", params={"label": "session"})
        if expected is not None:
            value["expected_software"] = expected
        with pytest.raises(ProtocolError):
            encode_message(value)
    request = IdentityControlRequest("write", "batch.create", params={"label": "session"},
                                     expected_software=software())
    assert decode_message(encode_message(request)) == request.to_dict()


@pytest.mark.parametrize("version", range(1, 8))
def test_legacy_requests_remain_unchanged_and_reject_new_fields(version):
    request = dict(version=version, request_id="read", command="snapshot")
    assert decode_message(encode_message(request)) == request
    with pytest.raises(ProtocolError):
        encode_message(dict(request, expected_software=software()))


@pytest.mark.parametrize("command", ["snapshot", "ping"])
def test_v8_readonly_discovery_does_not_require_source_binding(command):
    request = IdentityControlRequest("read", command)
    assert decode_message(encode_message(request)) == request.to_dict()
