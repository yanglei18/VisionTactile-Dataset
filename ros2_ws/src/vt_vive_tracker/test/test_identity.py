import hashlib

import pytest

from vt_vive_tracker.identity import canonical_address, tracker_id


def test_rf_slot_bits_do_not_change_physical_identity():
    left = bytes.fromhex("230142b782d3")
    same = bytes.fromhex("230642b782d3")
    assert canonical_address(left) == canonical_address(same)
    assert tracker_id(left) == tracker_id(same)


def test_identity_is_full_lowercase_sha256():
    raw = bytes.fromhex("230142b782d3")
    assert tracker_id(raw) == hashlib.sha256(
        bytes.fromhex("230042b782d3")
    ).hexdigest()


@pytest.mark.parametrize("raw", [b"", b"12345", b"1234567"])
def test_address_must_be_six_bytes(raw):
    with pytest.raises(ValueError, match="six bytes"):
        canonical_address(raw)


@pytest.mark.parametrize("raw", [bytearray(6), memoryview(bytes(6))])
def test_address_must_be_exact_bytes(raw):
    with pytest.raises(ValueError, match="six bytes"):
        canonical_address(raw)
