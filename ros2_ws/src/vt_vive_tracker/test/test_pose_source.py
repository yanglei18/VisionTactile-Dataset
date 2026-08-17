from __future__ import annotations

from dataclasses import dataclass
from pathlib import Path
from types import SimpleNamespace
import threading
import time

import pytest

from vt_vive_tracker.identity import tracker_id
from vt_vive_tracker.pose_source import (
    DongleCardinalityError,
    IdentityMismatch,
    ReadOnlyPoseSource,
)
from vt_vive_tracker.roles import RoleMap


ADDRESSES = (
    bytes.fromhex("230142b782d3"),
    bytes.fromhex("310253c893e4"),
    bytes.fromhex("410364d9a4f5"),
)
ROLES = ("left_wrist", "right_wrist", "torso")


def role_map(addresses=ADDRESSES):
    by_role = {
        role: tracker_id(address)
        for role, address in zip(ROLES, addresses)
    }
    return RoleMap(
        by_role=by_role,
        by_tracker_id={value: role for role, value in by_role.items()},
    )


def report(address, packet_index=1, marker=b"P", declared_length=37):
    payload = marker.ljust(declared_length, b"\0")
    canonical = (
        b"\x28"
        + packet_index.to_bytes(2, "little")
        + address
        + b"\x10\x01"
        + bytes((declared_length,))
        + payload
    )
    return canonical.ljust(64, b"\0")


def ack_report(address):
    payload = b"A"
    return (
        b"\x28"
        + b"\x01\x00"
        + address
        + b"\x01\x01"
        + bytes((len(payload),))
        + payload
    ).ljust(64, b"\0")


class FakeHandle:
    def __init__(self, reads, log):
        self.reads = list(reads)
        self.log = log
        self.close_count = 0

    def send_feature_report(self, value):
        raise AssertionError("Feature OUT is forbidden")

    def get_feature_report(self, report_id, length):
        raise AssertionError("Feature GET is forbidden")

    def read(self, length, timeout_ms):
        self.log.append(("read", length, timeout_ms))
        if not self.reads:
            return b""
        value = self.reads.pop(0)
        if isinstance(value, BaseException):
            raise value
        return value

    def close(self):
        self.close_count += 1
        self.log.append(("close",))


class FakeBackend:
    def __init__(self, handles, log, descriptors=None):
        self.handles = list(handles)
        self.log = log
        self.descriptors = (
            [
                {
                    "vendor_id": 0x0BB4,
                    "product_id": 0x0350,
                    "interface_number": 0,
                    "path": b"dongle-0",
                    "serial_number": "one",
                }
            ]
            if descriptors is None
            else descriptors
        )

    def enumerate(self, vid, pid):
        self.log.append(("enumerate", vid, pid))
        return tuple(self.descriptors)

    def open_path(self, path):
        self.log.append(("open", path))
        if not self.handles:
            raise RuntimeError("no replacement handle")
        return self.handles.pop(0)


@dataclass(frozen=True)
class FakeIdentity:
    path: bytes
    serial_number: str | None

    @property
    def path_sha256(self):
        return self.path.hex()


class ReadOnlyView:
    def __init__(self, handle):
        self.handle = handle

    def read(self, length, timeout_ms):
        return self.handle.read(length, timeout_ms)

    def close(self):
        self.handle.close()


def modules(log, addresses=ADDRESSES):
    def load_bundle(path):
        log.append(("bundle", Path(path)))
        return SimpleNamespace(tracker_addresses=addresses)

    def probe(backend):
        descriptors = backend.enumerate(0x0BB4, 0x0350)
        exact = [
            descriptor
            for descriptor in descriptors
            if descriptor["vendor_id"] == 0x0BB4
            and descriptor["product_id"] == 0x0350
            and descriptor["interface_number"] == 0
        ]
        identity = None
        if len(exact) == 1:
            identity = FakeIdentity(
                path=exact[0]["path"],
                serial_number=exact[0]["serial_number"],
            )
        return SimpleNamespace(
            exact_match_count=len(exact), identity=identity
        )

    def decode_envelope(value):
        log.append(("decode_envelope", len(value)))
        if value[0] != 0x28:
            raise ValueError("unsupported report id")
        if len(value) != 12 + value[11]:
            raise ValueError("report was not trimmed")
        return SimpleNamespace(
            address=value[3:9],
            packet_index=int.from_bytes(value[1:3], "little"),
            packet_kind=int.from_bytes(value[9:11], "little"),
            payload=value[12:],
        )

    def decode_pose(envelope):
        log.append(("decode_pose", envelope.packet_index))
        if envelope.packet_kind == 0x0101 or len(envelope.payload) == 2:
            return None
        if envelope.payload.startswith(b"N"):
            raise ValueError("pose components must be finite")
        return SimpleNamespace(
            address=envelope.address,
            packet_index=envelope.packet_index,
            tracker_index=1,
            buttons=3,
            position=(1.0, 2.0, 3.0),
            quaternion=(1.0, 0.0, 0.0, 0.0),
            acceleration=(4.0, 5.0, 6.0),
            angular_velocity=(7.0, 8.0, 9.0, 99.0),
            tracking_status=2,
        )

    return {
        "pyvut.live_bootstrap_bundle": SimpleNamespace(
            load_private_live_bundle=load_bundle
        ),
        "pyvut.live_hid": SimpleNamespace(
            RealHidBackend=lambda: None,
            ReadOnlyHandleView=ReadOnlyView,
        ),
        "pyvut.live_probe": SimpleNamespace(probe_dongle=probe),
        "pyvut.pose_decoder": SimpleNamespace(
            decode_dongle_envelope=decode_envelope,
            decode_native_pose=decode_pose,
        ),
    }


def make_source(backend, log, *, addresses=ADDRESSES, clocks=None):
    loaded_modules = modules(log, addresses)
    clocks = clocks or iter(range(100, 10_000))
    return ReadOnlyPoseSource(
        Path("/private/bootstrap.json"),
        role_map(),
        backend_factory=lambda: log.append(("backend",)) or backend,
        module_loader=lambda name: loaded_modules[name],
        monotonic_ns=lambda: log.append(("monotonic",)) or next(clocks),
        realtime_ns=lambda: log.append(("realtime",)) or next(clocks),
        read_timeout_ms=100,
    )


def collect_until(source, count, timeout=1.0):
    samples = []
    invalid = []
    errors = []
    ready = threading.Event()

    def on_sample(sample):
        samples.append(sample)
        if len(samples) >= count:
            ready.set()

    source.start(
        on_sample,
        lambda *value: invalid.append(value),
        lambda value: errors.append(value),
    )
    assert ready.wait(timeout)
    source.stop()
    return samples, invalid, errors


@pytest.mark.parametrize("descriptor_count", [0, 2])
def test_requires_exactly_one_interface_zero_dongle(descriptor_count):
    log = []
    descriptors = [
        {
            "vendor_id": 0x0BB4,
            "product_id": 0x0350,
            "interface_number": 0,
            "path": f"dongle-{index}".encode(),
            "serial_number": str(index),
        }
        for index in range(descriptor_count)
    ]
    backend = FakeBackend([], log, descriptors)
    source = make_source(backend, log)

    with pytest.raises(DongleCardinalityError):
        source.start(lambda value: None, lambda *value: None, lambda value: None)

    assert not any(call[0] == "open" for call in log)


def test_bundle_and_role_map_are_verified_before_backend_construction():
    log = []
    mismatched = tuple(bytes([index]) * 6 for index in (1, 2, 3))
    source = make_source(FakeBackend([], log), log, addresses=mismatched)

    with pytest.raises(IdentityMismatch):
        source.start(lambda value: None, lambda *value: None, lambda value: None)

    assert [call[0] for call in log] == ["bundle"]


def test_mapped_fixed_reports_are_trimmed_timestamped_and_normalized_in_order():
    log = []
    handle = FakeHandle(
        [
            report(ADDRESSES[0], packet_index=10),
            ack_report(ADDRESSES[0]),
            report(ADDRESSES[1], packet_index=11),
            report(ADDRESSES[2], packet_index=12),
        ],
        log,
    )
    source = make_source(FakeBackend([handle], log), log)

    samples, invalid, errors = collect_until(source, 3)

    assert [sample.role for sample in samples] == list(ROLES)
    assert [sample.packet_index for sample in samples] == [10, 11, 12]
    assert all(sample.pose_valid for sample in samples)
    assert invalid == []
    assert errors == []
    assert handle.close_count == 1
    assert all(
        call == ("read", 64, 100)
        for call in log
        if call[0] == "read"
    )
    nonempty_read_indices = [
        index
        for index, call in enumerate(log)
        if call[0] == "read"
    ][:4]
    for index in nonempty_read_indices:
        assert log[index + 1][0] == "monotonic"
        assert log[index + 2][0] == "realtime"
    assert ("decode_envelope", 49) in log


def test_nonfinite_pose_is_invalid_and_unknown_identity_is_an_error():
    log = []
    unknown = bytes.fromhex("510475eab506")
    handle = FakeHandle(
        [
            report(ADDRESSES[0], marker=b"N"),
            report(unknown),
            report(ADDRESSES[1]),
        ],
        log,
    )
    source = make_source(FakeBackend([handle], log), log)

    samples, invalid, errors = collect_until(source, 1)

    assert [sample.role for sample in samples] == ["right_wrist"]
    assert invalid[0][:2] == ("left_wrist", tracker_id(ADDRESSES[0]))
    assert len(invalid) == 1
    assert len(errors) == 1
    assert isinstance(errors[0], IdentityMismatch)
    assert unknown.hex() not in repr(errors[0])


def test_non_tracker_hid_background_report_is_ignored():
    log = []
    background = bytes((0x29,)).ljust(64, b"\0")
    handle = FakeHandle(
        [background, report(ADDRESSES[0], packet_index=9)],
        log,
    )
    source = make_source(FakeBackend([handle], log), log)

    samples, invalid, errors = collect_until(source, 1)

    assert [sample.packet_index for sample in samples] == [9]
    assert invalid == []
    assert errors == []


def test_read_error_closes_reenumerates_and_resumes_read_only():
    log = []
    failed = FakeHandle([OSError("disconnected")], log)
    replacement = FakeHandle([report(ADDRESSES[2], packet_index=77)], log)
    backend = FakeBackend([failed, replacement], log)
    source = make_source(backend, log)

    samples, invalid, errors = collect_until(source, 1)

    assert [sample.packet_index for sample in samples] == [77]
    assert failed.close_count == 1
    assert replacement.close_count == 1
    assert sum(call[0] == "enumerate" for call in log) == 2
    assert sum(call[0] == "open" for call in log) == 2
    assert invalid == []
    assert len(errors) == 1
    assert "disconnected" not in repr(errors[0])


def test_stop_is_bounded_idempotent_and_closes_once():
    log = []
    handle = FakeHandle([], log)
    source = make_source(FakeBackend([handle], log), log)
    source.start(lambda value: None, lambda *value: None, lambda value: None)

    started = time.monotonic()
    source.stop()
    source.stop()

    assert time.monotonic() - started < 1.0
    assert handle.close_count == 1
