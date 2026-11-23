"""Admission is shared, preserves recipe modes and rejects stale resources."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import subprocess
import sys

import pytest

from vt_capture_core import hand_preflight as module


@pytest.fixture
def hand_bundle(tmp_path):
    def make(hand):
        root = tmp_path / hand
        content = {"config.json": b"{}", "rest.json": b"{}", "hand.json": b"{}",
                   f"mano/models/MANO_{hand.upper()}.pkl": b"test only, never unpickle"}
        if hand == "left":
            content["tactile_layout.json"] = b"{}"
        for name, raw in content.items():
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(raw)
        (root / "manifest.json").write_text(json.dumps(dict(schema_version=1,
            product="hand-motion-touch", files={name: hashlib.sha256(raw).hexdigest() for name, raw in content.items()})))
        return root
    return make


def group(hand, preset="glove", **parameters):
    return dict(id=f"{hand}_{preset}", label=f"{hand} {preset}", preset=preset, enabled=True,
                parameters=dict(hand=hand, port=f"/dev/unused-{hand}-{preset}", **parameters))


def probe(*args, **kwargs):
    return dict(hand=args[2], source_root=str(args[3] or ""), hardware_accessed=False, controller_started=False)


def test_both_hands_share_one_probe_each_and_detached_reports(tmp_path, hand_bundle):
    profiles, calls = [], []
    for hand in ("right", "left"):
        bundle = str(hand_bundle(hand))
        profiles += [group(hand, resources=bundle, glove_python=sys.executable),
                     group(hand, "tactile", resources=bundle)]
    def verify(*args, **kwargs):
        calls.append((args, kwargs))
        return probe(*args, **kwargs)
    proof = module.prepare_hand_runtimes(tuple(profiles), candidate_root=tmp_path, verify_runtime=verify)
    assert [call[0][2] for call in calls] == ["right", "left"]
    assert all(call[0][3] == tmp_path for call in calls)
    assert set(proof.evidence) == {"right", "left"}
    report = proof.evidence
    report["right"]["resources"] = "tampered"
    assert proof.evidence["right"]["resources"] != "tampered"
    proof.assert_current()


def test_tactile_target_does_not_probe_an_enabled_motion_interpreter(tmp_path, hand_bundle):
    bundle = str(hand_bundle("left"))
    profiles = [group("left", resources=bundle, glove_python="/removed/python"),
                group("left", "tactile", resources=bundle)]
    proof = module.prepare_hand_runtimes(profiles, candidate_root=tmp_path,
        target_group_id="left_tactile", verify_runtime=lambda *a, **k: pytest.fail("numerical probe"))
    assert proof.evidence["left"]["glove_python"] == ""
    assert "runtime_probe" not in proof.evidence["left"]


def test_disabled_other_hand_resources_are_not_opened(tmp_path, hand_bundle):
    disabled = group("left", resources="/removed/bundle", glove_python="/removed/python")
    disabled["enabled"] = False
    profiles = [disabled, group("right", "tactile", resources=str(hand_bundle("right")))]
    assert set(module.prepare_hand_runtimes(profiles, candidate_root=tmp_path).evidence) == {"right"}


def test_ordinary_tactile_reader_and_nonhand_recipes_have_no_optional_imports(monkeypatch):
    def unexpected(*a, **k):
        pytest.fail("bundle verifier must not run")
    monkeypatch.setattr(module, "_verify_bundle", unexpected)
    profiles = [group("left", "tactile"), dict(id="vive", label="VIVE", preset="trackers", enabled=True,
                parameters=dict(backend="vivehub", frame_id="vive_hub_map", hub_bindings_path="/tmp/bindings",
                                hub_reader_path="/tmp/reader", hub_socket_path="/tmp/hub.sock"))]
    assert module.prepare_hand_runtimes(profiles).evidence == {"left": {"resources": "", "glove_python": ""}}
    assert module.prepare_hand_runtimes(profiles, target_group_id="vive").evidence == {}


def test_explicit_resources_are_forwarded_without_candidate_restriction(tmp_path):
    source = tmp_path / "external-upstream"
    source.mkdir()
    (source / "solver.py").write_text("# external pinned source")
    assets = tmp_path / "assets"
    (assets / "models").mkdir(parents=True)
    (assets / "models/MANO_LEFT.pkl").write_bytes(b"never unpickle")
    config, rest = tmp_path / "config.json", tmp_path / "rest.json"
    config.write_text("{}")
    rest.write_text("{}")
    explicit = dict(config=str(config), rest=str(rest), mano_assets=str(assets), fsglove_source=str(source))
    profiles = [group("left", glove_python=sys.executable, output_rate_hz="120", **explicit)]
    calls = []
    def verify(*args, **kwargs):
        calls.append((args, kwargs))
        return probe(*args, **kwargs)
    proof = module.prepare_hand_runtimes(profiles, verify_runtime=verify)
    assert calls == [((Path(sys.executable), None, "left", None), dict(explicit=explicit, output_rate_hz="120"))]
    proof.assert_current()
    (source / "solver.py").write_text("# modified source")
    with pytest.raises(ValueError, match="changed after preflight"):
        proof.assert_current()
    with pytest.raises(ValueError, match="explicit resource bundle"):
        module.prepare_hand_runtimes(profiles, candidate_root=tmp_path, verify_runtime=verify)


@pytest.mark.parametrize("change", ["profile", "manifest", "model", "delete", "symlink", "python", "source"])
def test_prepared_proof_rejects_changes_without_reprobing(tmp_path, hand_bundle, change, monkeypatch):
    bundle = hand_bundle("right")
    interpreter = tmp_path / "python"
    interpreter.symlink_to(sys.executable)
    source = tmp_path / "tools/fsglove12/src/vt_fsglove12"
    source.mkdir(parents=True)
    (source / "solver.py").write_text("original")
    alias = tmp_path / "bundle-alias"
    alias.symlink_to(bundle, target_is_directory=True)
    profiles = [group("right", resources=str(alias), glove_python=str(interpreter))]
    proof = module.prepare_hand_runtimes(profiles, candidate_root=tmp_path, verify_runtime=probe)
    if change == "profile":
        profiles[0]["parameters"]["port"] = "/dev/changed"
    elif change == "manifest":
        (bundle / "manifest.json").write_text("{}")
    elif change == "model":
        (bundle / "mano/models/MANO_RIGHT.pkl").write_bytes(b"changed")
    elif change == "delete":
        (bundle / "rest.json").unlink()
    elif change == "symlink":
        alias.unlink()
        alias.symlink_to(tmp_path / "missing", target_is_directory=True)
    elif change == "python":
        interpreter.unlink()
        interpreter.write_text("different executable")
        interpreter.chmod(0o700)
    else:
        (source / "solver.py").write_text("modified")
    monkeypatch.setattr(module, "probe_hand_runtime", lambda *a, **k: pytest.fail("must not probe"))
    monkeypatch.setattr(Path, "open", lambda *a, **k: pytest.fail("must not read model bytes"))
    with pytest.raises(ValueError, match="changed|no longer"):
        proof.assert_current()


def test_changes_during_probe_cannot_receive_a_valid_proof(tmp_path, hand_bundle):
    bundle = hand_bundle("right")
    profiles = [group("right", resources=str(bundle), glove_python=sys.executable)]
    def verify(*args, **kwargs):
        (bundle / "rest.json").write_text('{"changed":true}')
        return probe(*args, **kwargs)
    with pytest.raises(ValueError, match="changed after preflight"):
        module.prepare_hand_runtimes(profiles, candidate_root=tmp_path, verify_runtime=verify)


def test_same_hand_conflicts_are_detected_before_target_launch(tmp_path, hand_bundle):
    profiles = [group("right", resources=str(hand_bundle("right")), glove_python=sys.executable),
                group("right", "tactile", resources=str(hand_bundle("left")))]
    with pytest.raises(ValueError, match="conflicting display resources"):
        module.prepare_hand_runtimes(profiles, target_group_id="right_tactile")


def test_isolated_probe_rejects_foreign_checkout_before_loading_model(tmp_path, hand_bundle):
    # Real interpreter process, fake package. A foreign editable installation
    # must fail before the package body (or any numerical code) is executed.
    from vt_capture_core.hand_runtime_probe import _PROBE
    package = tmp_path / "foreign/vt_fsglove12"
    package.mkdir(parents=True)
    (package / "__init__.py").write_text("raise RuntimeError('PACKAGE EXECUTED')")
    root = tmp_path / "candidate"
    script = "import sys; sys.path.insert(0, " + repr(str(package.parent)) + ");\n" + _PROBE
    result = subprocess.run([sys.executable, "-I", "-c", script, str(root), str(hand_bundle("left")), "left"],
                            capture_output=True, text=True, timeout=10)
    assert result.returncode != 0
    assert "another checkout" in result.stderr
    assert "PACKAGE EXECUTED" not in result.stderr
