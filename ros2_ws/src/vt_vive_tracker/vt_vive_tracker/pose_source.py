from __future__ import annotations

import importlib
import threading
import time
from pathlib import Path
from typing import Callable

from .identity import tracker_id
from .model import NativePose, normalize_pose
from .roles import RoleMap


class PoseSourceError(RuntimeError):
    """Base class for redacted read-only source failures."""


class DongleCardinalityError(PoseSourceError):
    def __init__(self, exact_match_count: int) -> None:
        super().__init__(
            f"expected one VIVE Dongle interface, found {exact_match_count}"
        )
        self.exact_match_count = exact_match_count


class IdentityMismatch(PoseSourceError):
    def __init__(self, context: str, value: str | None = None) -> None:
        message = f"tracker identity mismatch: {context}"
        if value is not None:
            message += f" tracker_id={value}"
        super().__init__(message)
        self.context = context
        self.tracker_id = value


class DongleIdentityChanged(PoseSourceError):
    def __init__(self) -> None:
        super().__init__("replacement Dongle identity does not match")


class ReadFailure(PoseSourceError):
    def __init__(self) -> None:
        super().__init__("read-only Dongle read failed")


class MalformedReport(PoseSourceError):
    def __init__(self) -> None:
        super().__init__("malformed Dongle input report")


class ReadOnlyPoseSource:
    def __init__(
        self,
        bundle_path: Path,
        role_map: RoleMap,
        *,
        backend_factory: Callable[[], object] | None = None,
        module_loader: Callable[[str], object] = importlib.import_module,
        monotonic_ns: Callable[[], int] = time.monotonic_ns,
        realtime_ns: Callable[[], int] = time.time_ns,
        read_timeout_ms: int = 100,
    ) -> None:
        if type(role_map) is not RoleMap:
            raise TypeError("role_map must be RoleMap")
        if backend_factory is not None and not callable(backend_factory):
            raise TypeError("backend_factory must be callable")
        if not callable(module_loader):
            raise TypeError("module_loader must be callable")
        if not callable(monotonic_ns) or not callable(realtime_ns):
            raise TypeError("clock functions must be callable")
        if (
            type(read_timeout_ms) is not int
            or not 1 <= read_timeout_ms <= 100
        ):
            raise ValueError("read_timeout_ms must be between 1 and 100")

        self._bundle_path = Path(bundle_path)
        self._role_map = role_map
        self._backend_factory = backend_factory
        self._module_loader = module_loader
        self._monotonic_ns = monotonic_ns
        self._realtime_ns = realtime_ns
        self._read_timeout_ms = read_timeout_ms
        self._stop_event = threading.Event()
        self._handle_lock = threading.Lock()
        self._handle = None
        self._thread: threading.Thread | None = None
        self._started = False
        self._backend = None
        self._identity_key: tuple[str, str] | None = None
        self._read_only_view_type = None
        self._probe_dongle = None
        self._decode_envelope = None
        self._decode_pose = None
        self._on_sample = None
        self._on_invalid = None
        self._on_error = None

    @staticmethod
    def _identity_key_for(identity: object) -> tuple[str, str]:
        serial_number = getattr(identity, "serial_number")
        if serial_number is not None:
            return ("serial", serial_number)
        return ("path", getattr(identity, "path_sha256"))

    def _load_and_verify_bundle(self) -> None:
        bundle_module = self._module_loader(
            "pyvut.live_bootstrap_bundle"
        )
        bundle = bundle_module.load_private_live_bundle(self._bundle_path)
        addresses = bundle.tracker_addresses
        if (
            type(addresses) is not tuple
            or len(addresses) != 3
            or any(type(address) is not bytes for address in addresses)
        ):
            raise IdentityMismatch("bundle address set is invalid")
        observed = {tracker_id(address) for address in addresses}
        expected = set(self._role_map.by_tracker_id)
        if len(observed) != 3 or observed != expected:
            raise IdentityMismatch("bundle and role map disagree")

    def _load_runtime(self) -> None:
        hid_module = self._module_loader("pyvut.live_hid")
        probe_module = self._module_loader("pyvut.live_probe")
        decoder_module = self._module_loader("pyvut.pose_decoder")
        self._read_only_view_type = hid_module.ReadOnlyHandleView
        self._probe_dongle = probe_module.probe_dongle
        self._decode_envelope = decoder_module.decode_dongle_envelope
        self._decode_pose = decoder_module.decode_native_pose
        if self._backend_factory is None:
            self._backend_factory = hid_module.RealHidBackend

    def _open_unique_handle(self, *, replacement: bool) -> None:
        result = self._probe_dongle(self._backend)
        if result.exact_match_count != 1 or result.identity is None:
            raise DongleCardinalityError(result.exact_match_count)
        identity_key = self._identity_key_for(result.identity)
        if replacement and identity_key != self._identity_key:
            raise DongleIdentityChanged()
        if not replacement:
            self._identity_key = identity_key
        raw_handle = self._backend.open_path(result.identity.path)
        read_only_handle = self._read_only_view_type(raw_handle)
        with self._handle_lock:
            self._handle = read_only_handle

    def start(
        self,
        on_sample: Callable[[object], None],
        on_invalid: Callable[[str, str, int], None],
        on_error: Callable[[Exception], None],
    ) -> None:
        if self._started:
            raise RuntimeError("pose source has already been started")
        if not all(callable(value) for value in (on_sample, on_invalid, on_error)):
            raise TypeError("source callbacks must be callable")

        self._load_and_verify_bundle()
        self._load_runtime()
        self._backend = self._backend_factory()
        self._open_unique_handle(replacement=False)
        self._on_sample = on_sample
        self._on_invalid = on_invalid
        self._on_error = on_error
        self._stop_event.clear()
        self._started = True
        self._thread = threading.Thread(
            target=self._run,
            name="vt-vive-tracker-reader",
            daemon=True,
        )
        try:
            self._thread.start()
        except BaseException:
            self._close_current_handle()
            self._started = False
            raise

    def _close_current_handle(self) -> None:
        with self._handle_lock:
            handle = self._handle
            self._handle = None
        if handle is not None:
            try:
                handle.close()
            except Exception:
                self._on_error(ReadFailure())

    @staticmethod
    def _canonical_report(report: bytes) -> bytes:
        if type(report) is not bytes or len(report) > 64:
            raise ValueError("invalid HID input report")
        if len(report) < 12:
            return report
        canonical_length = 12 + report[11]
        if (
            len(report) == 64
            and report[0] == 0x28
            and canonical_length <= 64
        ):
            return report[:canonical_length]
        return report

    def _mapped_identity(
        self, address: bytes
    ) -> tuple[str, str] | None:
        value = tracker_id(address)
        try:
            role = self._role_map.role_for_tracker_id(value)
        except KeyError:
            self._on_error(IdentityMismatch("unmapped report", value))
            return None
        return role, value

    def _process_report(
        self,
        report: bytes,
        host_monotonic_ns: int,
        host_realtime_ns: int,
    ) -> None:
        if type(report) is bytes and report and report[0] != 0x28:
            return
        try:
            envelope = self._decode_envelope(
                self._canonical_report(report)
            )
        except (TypeError, ValueError):
            self._on_error(MalformedReport())
            return

        identity = self._mapped_identity(envelope.address)
        if identity is None:
            return
        role, value = identity
        try:
            decoded = self._decode_pose(envelope)
        except (TypeError, ValueError):
            self._on_invalid(role, value, host_monotonic_ns)
            return
        if decoded is None:
            return
        try:
            native = NativePose(
                address=decoded.address,
                packet_index=decoded.packet_index,
                tracker_index=decoded.tracker_index,
                buttons=decoded.buttons,
                position=decoded.position,
                quaternion_wzyx=decoded.quaternion,
                acceleration=decoded.acceleration,
                angular_velocity_native=decoded.angular_velocity,
                tracking_status=decoded.tracking_status,
            )
            sample = normalize_pose(
                native,
                role=role,
                tracker_id=value,
                host_monotonic_ns=host_monotonic_ns,
                host_realtime_ns=host_realtime_ns,
            )
        except (TypeError, ValueError):
            self._on_invalid(role, value, host_monotonic_ns)
            return
        self._on_sample(sample)

    def _recover_read_only(self) -> None:
        while not self._stop_event.is_set():
            try:
                self._open_unique_handle(replacement=True)
                return
            except PoseSourceError as error:
                self._on_error(error)
            except Exception:
                self._on_error(ReadFailure())
            self._stop_event.wait(self._read_timeout_ms / 1000.0)

    def _run(self) -> None:
        try:
            while not self._stop_event.is_set():
                with self._handle_lock:
                    handle = self._handle
                if handle is None:
                    self._recover_read_only()
                    continue
                try:
                    report = handle.read(64, self._read_timeout_ms)
                except Exception:
                    self._close_current_handle()
                    self._on_error(ReadFailure())
                    self._recover_read_only()
                    continue
                if report == b"":
                    continue
                host_monotonic_ns = self._monotonic_ns()
                host_realtime_ns = self._realtime_ns()
                self._process_report(
                    report, host_monotonic_ns, host_realtime_ns
                )
        finally:
            self._close_current_handle()

    def stop(self) -> None:
        thread = self._thread
        if thread is None:
            return
        self._stop_event.set()
        thread.join(timeout=1.0)
        if thread.is_alive():
            raise TimeoutError("pose source did not stop within one second")
        self._thread = None
