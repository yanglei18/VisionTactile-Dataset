import hashlib


def canonical_address(raw: bytes) -> bytes:
    """Remove RF slot bits that can change when a tracker is paired again."""
    if type(raw) is not bytes or len(raw) != 6:
        raise ValueError("Tracker address must contain six bytes")
    return bytes((raw[0], raw[1] & 0xF8, *raw[2:]))


def tracker_id(raw: bytes) -> str:
    """Return a non-reversible stable identifier for one physical tracker."""
    return hashlib.sha256(canonical_address(raw)).hexdigest()
