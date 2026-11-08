"""Explicit configuration replacement is atomic and never touches acquisition."""
from copy import deepcopy
import hashlib
import json
import os
from pathlib import Path
import tempfile

import pytest

from vt_capture_console.backend import candidate_migration as migration


@pytest.fixture
def setup(tmp_path):
    root = tmp_path / "checkout"
    (root / "tools/capture_deploy").mkdir(parents=True)
    (root / "tools/capture_deploy/start_manager.sh").write_text("fixture")
    hands = {}
    for hand in ("left", "right"):
        bundle = tmp_path / hand
        documents = {"config.json": {"hand": hand}, "rest.json": {}, "hand.json": {}}
        if hand == "left":
            documents["tactile_layout.json"] = {}
        contents = {key: json.dumps(value).encode() for key, value in documents.items()}
        contents[f"mano/models/MANO_{hand.upper()}.pkl"] = b"fixture model, never unpickled"
        for name, content in contents.items():
            path = bundle / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        (bundle / "manifest.json").write_text(json.dumps(dict(schema_version=1, product="hand-motion-touch",
            files={key: hashlib.sha256(value).hexdigest() for key, value in contents.items()})))
        python = tmp_path / f"{hand}-python"
        python.write_text("fixture interpreter, never run")
        python.chmod(0o700)
        hands[hand] = dict(resources=str(bundle), glove_python=str(python))
    config = dict(quality_enabled=False, scene_config_path="/existing/scene.json", process_groups=[
        dict(id="camera", label="Camera", preset="cameras", enabled=True, startup_timeout_s=30.,
             parameters=dict(config_path="/existing/camera.yaml", output_root="/existing/data")),
        dict(id="vive", label="VIVE Hub", preset="trackers", enabled=True, startup_timeout_s=30.,
             parameters=dict(backend="vivehub", hub_bindings_path="/existing/vive.json", frame_id="vive_hub_map",
                             hub_reader_path="/existing/reader", hub_socket_path="/existing/hub.sock")),
        *[dict(id=f"{hand}_motion", label=hand, preset="glove", enabled=hand == "right",
               parameters=dict(hand=hand, port=f"/dev/serial/by-id/{hand}", glove_python="/old/python",
                   config=f"/old/{hand}.json", fsglove_source="/old/upstream", mano_assets="/old/mano"))
          for hand in ("left", "right")],
        *[dict(id=f"{hand}_touch", label=hand, preset="tactile", enabled=hand == "right",
               parameters=dict(hand=hand, port=f"/dev/serial/by-id/{hand}-touch", config="/existing/touch.yaml"))
          for hand in ("left", "right")],
    ])
    source = tmp_path / "legacy-workstation.json"
    source.write_text(json.dumps(config, indent=3) + "\n")
    # Keep the control socket short independently of pytest's descriptive path.
    with tempfile.TemporaryDirectory(prefix="vt-migration-") as short:
        yield dict(root=root, source=source, config=config, hands=hands, destination=Path(short) / "candidate")


def fake_probe(python, bundle, hand, root):
    return dict(hand=hand, source_root=str(root), hardware_accessed=False, controller_started=False)


def prepare(setup, **kwargs):
    return migration.prepare_candidate(setup["source"], repo_root=setup["root"],
        settings_dir=setup["destination"], hands=setup["hands"], verify_runtime=kwargs.pop("verify_runtime", fake_probe), **kwargs)


def test_explicit_bundles_replace_old_paths_preserving_every_other_sensor(setup):
    original = setup["source"].read_bytes()
    result = prepare(setup)
    target = setup["destination"]
    config = result["configuration"]
    assert result["status"] == "created"
    assert setup["source"].read_bytes() == original
    assert (target / "source-workstation.json").read_bytes() == original
    assert config["scene_config_path"] == setup["config"]["scene_config_path"]
    assert config["process_groups"][:2] == setup["config"]["process_groups"][:2]
    for group in config["process_groups"][2:]:
        p, hand = group["parameters"], group["parameters"]["hand"]
        assert group["enabled"] == (hand == "right")
        assert p["resources"] == setup["hands"][hand]["resources"]
        assert p["glove_python"] == setup["hands"][hand]["glove_python"]
        assert p["port"].startswith(f"/dev/serial/by-id/{hand}")
        if group["preset"] == "glove":
            assert not {"config", "rest", "fsglove_source", "mano_assets"} & set(p)
            assert p["launch_gui"] == "false"
            assert Path(p["output_root"]).is_relative_to(target)
            sockets = (p["socket_path"], p["control_socket"])
        else:
            assert p["config"] == "/existing/touch.yaml"
            sockets = (p["display_socket"],)
        assert all(len(os.fsencode(socket)) <= 107 for socket in sockets)
    assert set(p.name for p in target.iterdir()) == {"workstation.json", "source-workstation.json", "migration.json"}
    assert target.stat().st_mode & 0o777 == 0o700
    assert all(p.stat().st_mode & 0o777 == 0o600 for p in target.iterdir())
    assert result["report"]["source"]["sha256"] == hashlib.sha256(original).hexdigest()
    assert result["report"]["hardware_accessed"] is False
    assert result["report"]["hub_autostart_settings_copied"] is False


def test_tactile_only_migration_does_not_create_a_motion_owner(setup):
    source = deepcopy(setup["config"])
    source["process_groups"] = [group for group in source["process_groups"] if group["preset"] == "tactile"]
    setup["source"].write_text(json.dumps(source))
    result = prepare(setup)
    assert len(result["configuration"]["process_groups"]) == 2
    assert all(group["preset"] == "tactile" for group in result["configuration"]["process_groups"])


def test_idempotent_preparation_keeps_user_edits_and_rechecks_runtime(setup):
    prepare(setup)
    assert not prepare(setup)["user_edits_preserved"]
    path = setup["destination"] / "workstation.json"
    document = json.loads(path.read_text())
    document["process_groups"][2]["parameters"]["port"] = "/dev/serial/by-id/user-chosen"
    path.write_text(json.dumps(document))
    content = path.read_bytes()
    probes = []
    def probe(*args):
        probes.append(args)
        return fake_probe(*args)
    result = prepare(setup, verify_runtime=probe)
    assert result["status"] == "existing" and result["user_edits_preserved"]
    assert len(probes) == 3  # both requested bundles, then enabled current motion
    assert path.read_bytes() == content


def test_changed_source_is_not_silently_resynchronized(setup):
    prepare(setup)
    path = setup["destination"] / "workstation.json"
    before = path.read_bytes()
    setup["source"].write_text(setup["source"].read_text() + "\n")
    with pytest.raises(ValueError, match="another source"):
        prepare(setup)
    assert path.read_bytes() == before


@pytest.mark.parametrize("failure", ["manifest", "runtime", "hand", "source", "validation"])
def test_failed_preparation_never_exposes_a_partial_candidate(setup, failure):
    original = setup["source"].read_bytes()
    probe = fake_probe
    if failure == "manifest":
        (Path(setup["hands"]["left"]["resources"]) / "hand.json").write_text("changed")
    elif failure == "runtime":
        def probe(*args):
            raise ValueError("Wrong numerical environment")
    elif failure == "hand":
        setup["hands"].pop("left")
    elif failure == "source":
        setup["source"].write_text("[]")
        original = setup["source"].read_bytes()
    else:
        setup["config"]["unexpected"] = True
        setup["source"].write_text(json.dumps(setup["config"]))
        original = setup["source"].read_bytes()
    with pytest.raises(ValueError):
        prepare(setup, verify_runtime=probe)
    assert not setup["destination"].exists()
    assert setup["source"].read_bytes() == original


def test_write_failure_removes_staging_and_keeps_source(setup, monkeypatch):
    original = setup["source"].read_bytes()
    write = migration.atomic_json
    def fail_second(path, value):
        if path.name == "migration.json":
            raise OSError("disk full fixture")
        write(path, value)
    monkeypatch.setattr(migration, "atomic_json", fail_second)
    with pytest.raises(OSError, match="disk full"):
        prepare(setup)
    assert not setup["destination"].exists()
    assert not list(setup["destination"].parent.glob("*.prepare-*"))
    assert setup["source"].read_bytes() == original


def test_check_only_does_not_create_configuration_or_parent(setup):
    setup["destination"] = setup["destination"] / "nested"
    result = prepare(setup, check_only=True)
    assert result["status"] == "proposed"
    assert not setup["destination"].parent.exists()


def test_foreign_directory_and_source_directory_are_never_overwritten(setup):
    setup["destination"].mkdir(mode=0o700)
    marker = setup["destination"] / "keep"
    marker.write_text("user file")
    with pytest.raises(FileNotFoundError):
        prepare(setup)
    assert marker.read_text() == "user file"
    setup["destination"] = setup["source"].parent
    with pytest.raises(ValueError, match="separate"):
        prepare(setup)


def test_candidate_paths_are_scoped_and_socket_lengths_fail_explicitly(setup):
    a = migration.default_candidate_settings(setup["root"], home="/home/test")
    b = migration.default_candidate_settings(setup["root"].parent, home="/home/test")
    assert a != b and migration.hand_socket_namespace(a) != migration.hand_socket_namespace(b)
    setup["destination"] = setup["destination"] / ("x" * 110)
    with pytest.raises(ValueError, match="too long"):
        prepare(setup)


def test_cli_duplicate_hands_fail_without_writes(setup):
    with pytest.raises(SystemExit) as error:
        migration.main(["--source", str(setup["source"]), "--repo-root", str(setup["root"]),
            "--settings-dir", str(setup["destination"]), "--hand", "left", "/bundle", "/python",
            "--hand", "left", "/bundle", "/python"])
    assert error.value.code == 2 and not setup["destination"].exists()


def test_runtime_preflight_only_checks_enabled_motion_once_per_hand(setup):
    configuration = prepare(setup)["configuration"]
    calls = []
    def probe(*args):
        calls.append(args)
        return fake_probe(*args)
    result = migration.verify_candidate_runtimes(configuration, setup["root"], probe)
    assert list(result) == ["right"]
    assert len(calls) == 1 and calls[0][2] == "right"


def test_tactile_preflight_needs_no_numeric_runtime_or_disabled_motion_bundle(setup):
    configuration = prepare(setup)["configuration"]
    for group in configuration["process_groups"]:
        if group["preset"] == "glove":
            group["enabled"] = False
            group["parameters"].update(resources="/removed/motion-bundle", glove_python="/removed/python")
        elif group["preset"] == "tactile":
            group["enabled"] = True
            group["parameters"].pop("glove_python", None)
    def unexpected_probe(*args):
        pytest.fail("Pure tactile must not start a numerical runtime")
    result = migration.verify_candidate_runtimes(configuration, setup["root"], unexpected_probe)
    assert set(result) == {"left", "right"}
    assert all(not value["glove_python"] and "runtime_probe" not in value for value in result.values())
    # The same path is used by idempotent migration; intentional tactile-only
    # edits do not need to restore the discarded motion environment.
    path = setup["destination"] / "workstation.json"
    path.write_text(json.dumps(configuration))
    before = path.read_bytes()
    assert prepare(setup)["user_edits_preserved"]
    assert path.read_bytes() == before


def test_runtime_preflight_rejects_conflicting_or_missing_active_bundles(setup):
    configuration = prepare(setup)["configuration"]
    touch = next(group for group in configuration["process_groups"] if group["id"] == "right_touch")
    touch["parameters"]["resources"] = setup["hands"]["left"]["resources"]
    with pytest.raises(ValueError, match="conflicting display resources"):
        migration.verify_candidate_runtimes(configuration, setup["root"], fake_probe)
    touch["parameters"].pop("resources")
    with pytest.raises(ValueError, match="explicit resource bundle"):
        migration.verify_candidate_runtimes(configuration, setup["root"], fake_probe)


def test_tactile_preflight_still_checks_actual_hand_and_manifest(setup):
    configuration = prepare(setup)["configuration"]
    configuration["process_groups"] = [group for group in configuration["process_groups"]
                                        if group["id"] == "right_touch"]
    manifest = Path(setup["hands"]["right"]["resources"]) / "hand.json"
    manifest.write_text("changed after preparation")
    with pytest.raises(ValueError):
        migration.verify_candidate_runtimes(configuration, setup["root"], fake_probe)


def test_corrupted_source_backup_cannot_be_certified_as_idempotent(setup):
    prepare(setup)
    backup = setup["destination"] / "source-workstation.json"
    backup.write_text(backup.read_text() + "\n")
    with pytest.raises(ValueError, match="source backup"):
        prepare(setup)


def test_expanded_report_must_fit_the_same_reader_limit_before_writes(setup, monkeypatch):
    monkeypatch.setattr(migration, "MAX_CONFIG_BYTES", setup["source"].stat().st_size + 100)
    with pytest.raises(ValueError, match="report exceeds"):
        prepare(setup)
    assert not setup["destination"].exists()
