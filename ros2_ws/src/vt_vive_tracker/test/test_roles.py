import os
import stat
import sys
from pathlib import Path
from types import ModuleType, SimpleNamespace

import pytest
import yaml

from vt_vive_tracker.identity import tracker_id
from vt_vive_tracker.role_map_cli import main
from vt_vive_tracker.roles import EXPECTED_ROLES, load_role_map


IDS = {
    "left_wrist": "a" * 64,
    "right_wrist": "b" * 64,
    "torso": "c" * 64,
}


def write_map(path: Path, roles=IDS, mode=0o600) -> Path:
    path.write_text(
        yaml.safe_dump({"schema_version": 1, "roles": roles}),
        encoding="utf-8",
    )
    path.chmod(mode)
    return path


def test_role_map_requires_exact_roles_and_has_stable_reverse_lookup(tmp_path):
    role_map = load_role_map(write_map(tmp_path / "roles.yaml"))

    assert frozenset(role_map.by_role) == EXPECTED_ROLES
    assert role_map.tracker_id_for_role("left_wrist") == "a" * 64
    assert role_map.role_for_tracker_id("b" * 64) == "right_wrist"
    with pytest.raises(TypeError):
        role_map.by_role["torso"] = "d" * 64


@pytest.mark.parametrize(
    "roles",
    [
        {"left_wrist": "a" * 64, "right_wrist": "b" * 64},
        {**IDS, "head": "d" * 64},
    ],
)
def test_role_map_rejects_wrong_role_set(tmp_path, roles):
    with pytest.raises(ValueError, match="exactly"):
        load_role_map(write_map(tmp_path / "roles.yaml", roles))


@pytest.mark.parametrize("invalid", ["A" * 64, "a" * 63, "g" * 64])
def test_role_map_rejects_non_lowercase_sha256(tmp_path, invalid):
    roles = {**IDS, "left_wrist": invalid}
    with pytest.raises(ValueError, match="lowercase SHA-256"):
        load_role_map(write_map(tmp_path / "roles.yaml", roles))


def test_role_map_rejects_duplicate_tracker_ids(tmp_path):
    roles = {**IDS, "right_wrist": IDS["left_wrist"]}
    with pytest.raises(ValueError, match="unique"):
        load_role_map(write_map(tmp_path / "roles.yaml", roles))


def test_role_map_requires_owner_only_permissions(tmp_path):
    with pytest.raises(PermissionError, match="owner-only"):
        load_role_map(write_map(tmp_path / "roles.yaml", mode=0o640))


def test_role_map_rejects_symlinks(tmp_path):
    target = write_map(tmp_path / "target.yaml")
    link = tmp_path / "roles.yaml"
    link.symlink_to(target)

    with pytest.raises(ValueError, match="regular file"):
        load_role_map(link)


def test_role_map_resolves_canonicalized_runtime_address(tmp_path):
    role_map = load_role_map(
        write_map(
            tmp_path / "roles.yaml",
            {**IDS, "left_wrist": tracker_id(bytes.fromhex("230142b782d3"))},
        )
    )

    assert (
        role_map.role_for_address(bytes.fromhex("230642b782d3"))
        == "left_wrist"
    )


def test_cli_writes_private_atomic_role_map_without_printing_addresses(
    tmp_path, monkeypatch, capsys
):
    addresses = (
        bytes.fromhex("230142b782d3"),
        bytes.fromhex("310253c893e4"),
        bytes.fromhex("410364d9a4f5"),
    )
    live_module = ModuleType("pyvut.live_bootstrap_bundle")
    live_module.load_private_live_bundle = lambda path: SimpleNamespace(
        tracker_addresses=addresses
    )
    pyvut_module = ModuleType("pyvut")
    pyvut_module.live_bootstrap_bundle = live_module
    monkeypatch.setitem(sys.modules, "pyvut", pyvut_module)
    monkeypatch.setitem(
        sys.modules, "pyvut.live_bootstrap_bundle", live_module
    )
    destination = tmp_path / "roles.yaml"

    assert main(
        [
            "--bundle",
            str(tmp_path / "private-bundle.json"),
            "--output",
            str(destination),
            "--host",
            "torso",
            "--client0",
            "left_wrist",
            "--client1",
            "right_wrist",
        ]
    ) == 0

    assert stat.S_IMODE(destination.stat().st_mode) == 0o600
    role_map = load_role_map(destination)
    assert role_map.tracker_id_for_role("torso") == tracker_id(addresses[0])
    assert role_map.tracker_id_for_role("left_wrist") == tracker_id(
        addresses[1]
    )
    assert role_map.tracker_id_for_role("right_wrist") == tracker_id(
        addresses[2]
    )
    rendered = capsys.readouterr().out
    assert str(destination) in rendered
    assert all(address.hex() not in rendered for address in addresses)
    assert not tuple(destination.parent.glob(f".{destination.name}.*"))
