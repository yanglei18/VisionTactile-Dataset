"""Use filesystem fixtures only; identifying a port must never open hardware."""
from pathlib import Path

import pytest

from vt_capture_console.backend import serial_ports as ports


@pytest.fixture
def device_tree(tmp_path, monkeypatch):
    root = tmp_path / "dev"
    (root / "serial/by-id").mkdir(parents=True)
    (root / "serial/by-path").mkdir()
    device = root / "ttyACM0"
    device.touch()
    identity = root / "serial/by-id/usb-confirmed-right-glove"
    identity.symlink_to(device)
    topology = root / "serial/by-path/pci-selected-port"
    topology.symlink_to(device)
    monkeypatch.setattr(ports, "DEV_ROOT", root)
    return root, device, identity, topology


@pytest.mark.parametrize("selected", [1, 3])
def test_selected_tty_or_topology_becomes_its_unique_identity(device_tree, selected):
    root, _, identity, _ = device_tree
    assert ports.stable_serial_path(str(device_tree[selected])) == str(identity)
    assert ports.serial_ports(root)[0]["path"] == str(identity)


def test_saved_identity_follows_replug_without_switching_to_another_device(device_tree):
    root, old_device, identity, _ = device_tree
    saved = ports.stable_serial_path(str(old_device))
    identity.unlink()
    current_device = root / "ttyACM7"
    current_device.touch()
    identity.symlink_to(current_device)
    # The old numbered port now names another physical device.
    (root / "serial/by-id/usb-other-device").symlink_to(old_device)
    assert ports.stable_serial_path(saved) == str(identity)
    assert Path(saved).resolve(strict=True) == current_device


@pytest.mark.parametrize("selected", ["", "ttyACM9", "serial/by-id/usb-missing-glove",
                                     "serial/by-path/pci-missing-port"])
def test_only_available_device_does_not_replace_empty_or_missing_choice(device_tree, selected):
    root, _, _, _ = device_tree
    value = str(root / selected) if selected else ""
    assert ports.stable_serial_path(value) == value


@pytest.mark.parametrize("aliases", [0, 2])
def test_no_unique_identity_preserves_manual_choice(device_tree, aliases):
    root, device, identity, topology = device_tree
    if aliases == 0:
        identity.unlink()
    else:
        (root / "serial/by-id/usb-second-alias").symlink_to(device)
    for value in (device, topology):
        assert ports.stable_serial_path(str(value)) == str(value)
    # An explicit identity remains the operator's selection even if ambiguous.
    assert ports.stable_serial_path(str(identity)) == str(identity)


def test_custom_device_alias_is_not_replaced(device_tree):
    root, device, _, _ = device_tree
    custom = root / "my-confirmed-glove"
    custom.symlink_to(device)
    assert ports.stable_serial_path(str(custom)) == str(custom)


def test_generated_alias_must_remain_a_valid_single_line_launch_path(device_tree):
    root, device, identity, _ = device_tree
    identity.unlink()
    (root / "serial/by-id/usb-invalid\nidentity").symlink_to(device)
    assert ports.stable_serial_path(str(device)) == str(device)


def test_discovery_never_reads_or_opens_a_device(device_tree, monkeypatch):
    _, device, identity, _ = device_tree
    def forbidden(*args, **kwargs):
        pytest.fail("serial discovery attempted device IO")
    monkeypatch.setattr("builtins.open", forbidden)
    monkeypatch.setattr("os.open", forbidden)
    monkeypatch.setattr(Path, "open", forbidden)
    assert ports.stable_serial_path(str(device)) == str(identity)
    assert len(ports.serial_ports()) == 1
