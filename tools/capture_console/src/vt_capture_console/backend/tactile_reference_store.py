"""Device measurements and explicit layout defaults; no serial opens or filtering."""
from __future__ import annotations

from dataclasses import dataclass, replace
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path
import tempfile

from vt_tactile_capture.display_response import ReferenceProfile, ResponseSettings


MAX_PROFILE_BYTES = 256 * 1024
PROTOCOL = "vt_tactile_adc400_v1"
LAYOUT = "right_adc20x20_v1"


def _validate_components(reference, settings, captured_at_utc):
    if not isinstance(reference, ReferenceProfile) or not isinstance(settings, ResponseSettings):
        raise ValueError("Invalid tactile reference profile components")
    try:
        stamp = datetime.fromisoformat(captured_at_utc)
    except (TypeError, ValueError) as exc:
        raise ValueError("Reference capture time must be an ISO timestamp") from exc
    if stamp.tzinfo is None:
        raise ValueError("Reference capture time must include a timezone")


def _settings_document(settings):
    return dict(display_range_adc=settings.display_range_adc, direction=settings.direction)


def _read_settings(value):
    if not isinstance(value, dict) or set(value) != {"display_range_adc", "direction"}:
        raise ValueError("Invalid tactile display settings")
    return ResponseSettings(**value)


@dataclass(frozen=True)
class TactileDeviceIdentity:
    vendor_id: int
    product_id: int
    serial_number: str

    def __post_init__(self):
        for value in (self.vendor_id, self.product_id):
            if type(value) is not int or not 0 <= value <= 65535:
                raise ValueError("USB device IDs must be unsigned 16-bit integers")
        if (not isinstance(self.serial_number, str) or not self.serial_number.strip()
                or len(self.serial_number) > 128 or any(ord(c) < 32 for c in self.serial_number)):
            raise ValueError("A stable USB serial number is required")
        object.__setattr__(self, "serial_number", self.serial_number.strip().casefold())

    def to_dict(self):
        return dict(vendor_id=self.vendor_id, product_id=self.product_id, serial_number=self.serial_number)

    @property
    def key(self):
        return hashlib.sha256(json.dumps(self.to_dict(), sort_keys=True).encode()).hexdigest()


def device_identity_for_port(port):
    """Inspect USB enumeration metadata, never probe or write the device."""
    from serial.tools import list_ports

    selected = os.path.normcase(os.path.realpath(port))
    for entry in list_ports.comports():
        if os.path.normcase(os.path.realpath(entry.device)) != selected:
            continue
        try:
            return TactileDeviceIdentity(entry.vid, entry.pid, entry.serial_number)
        except ValueError:
            return None
    return None


@dataclass(frozen=True)
class TactileDeviceProfile:
    device: TactileDeviceIdentity
    reference: ReferenceProfile
    settings: ResponseSettings
    captured_at_utc: str

    def __post_init__(self):
        if not isinstance(self.device, TactileDeviceIdentity):
            raise ValueError("Invalid tactile USB identity")
        _validate_components(self.reference, self.settings, self.captured_at_utc)

    def to_dict(self):
        return dict(schema_version=1, protocol=PROTOCOL, hand="right", device=self.device.to_dict(),
                    reference=self.reference.to_dict(), captured_at_utc=self.captured_at_utc,
                    settings=_settings_document(self.settings))

    @classmethod
    def from_dict(cls, value):
        fields = {"schema_version", "protocol", "hand", "device", "reference", "settings", "captured_at_utc"}
        if (not isinstance(value, dict) or set(value) != fields
                or type(value["schema_version"]) is not int or value["schema_version"] != 1
                or value["protocol"] != PROTOCOL or value["hand"] != "right"):
            raise ValueError("Unsupported tactile device profile")
        try:
            if set(value["device"]) != {"vendor_id", "product_id", "serial_number"}:
                raise ValueError("Invalid tactile USB identity")
            return cls(TactileDeviceIdentity(**value["device"]),
                       ReferenceProfile.from_dict(value["reference"]),
                       _read_settings(value["settings"]), value["captured_at_utc"])
        except (TypeError, KeyError) as exc:
            raise ValueError("Invalid tactile profile fields") from exc


@dataclass(frozen=True)
class TactileDefaultProfile:
    """An explicit reference for this protocol/layout, never a device measurement.

    This scope is a deployment declaration. The wire protocol has no model or
    layout identifier, so it cannot establish another board's physical wiring.
    """

    reference: ReferenceProfile
    settings: ResponseSettings
    captured_at_utc: str

    def __post_init__(self):
        _validate_components(self.reference, self.settings, self.captured_at_utc)

    def to_dict(self):
        return dict(schema_version=2, scope="layout", protocol=PROTOCOL, hand="right", layout=LAYOUT,
                    reference=self.reference.to_dict(), settings=_settings_document(self.settings),
                    captured_at_utc=self.captured_at_utc)

    @classmethod
    def from_dict(cls, value):
        fields = {"schema_version", "scope", "protocol", "hand", "layout", "reference", "settings", "captured_at_utc"}
        if (not isinstance(value, dict) or set(value) != fields
                or type(value["schema_version"]) is not int or value["schema_version"] != 2
                or value["scope"] != "layout" or value["protocol"] != PROTOCOL
                or value["hand"] != "right" or value["layout"] != LAYOUT):
            raise ValueError("Unsupported tactile layout default")
        return cls(ReferenceProfile.from_dict(value["reference"]),
                   _read_settings(value["settings"]), value["captured_at_utc"])


def _read_document(path):
    with Path(path).open("rb") as stream:
        content = stream.read(MAX_PROFILE_BYTES + 1)
    if len(content) > MAX_PROFILE_BYTES:
        raise ValueError("Tactile reference file is too large")
    return json.loads(content.decode("utf-8"))


def read_device_profile(path):
    return TactileDeviceProfile.from_dict(_read_document(path))


def read_reference_profile(path):
    value = _read_document(path)
    if isinstance(value, dict) and value.get("schema_version") == 2:
        return TactileDefaultProfile.from_dict(value)
    return TactileDeviceProfile.from_dict(value)


def _write_document(path, value):
    content = (json.dumps(value, ensure_ascii=False, allow_nan=False) + "\n").encode()
    if len(content) > MAX_PROFILE_BYTES:
        raise ValueError("Tactile reference file is too large")
    path.parent.mkdir(parents=True, exist_ok=True)
    descriptor, temporary = tempfile.mkstemp(prefix=".reference-", suffix=".tmp", dir=path.parent)
    try:
        with os.fdopen(descriptor, "wb") as stream:
            stream.write(content)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        if os.path.exists(temporary):
            os.unlink(temporary)


class TactileReferenceStore:
    """Prefer device measurements, then an explicitly scoped deployment default.

    Layout display preferences contain no reference and cannot become a device
    calibration. They only override the shared default's display settings.
    """

    def __init__(self, directory, *, preset_path=None):
        self.directory = Path(directory)
        self.preset_path = Path(preset_path) if preset_path is not None else None
        self.diagnostics = []

    def load(self, device):
        self.diagnostics.clear()
        candidates = []
        if device is not None:
            candidates.append((self.directory / (device.key + ".json"), "saved"))
        if self.preset_path is not None:
            candidates.append((self.preset_path, "preset"))
        for path, source in candidates:
            try:
                # User calibration paths accept device documents only: a
                # misplaced generic file must never suppress identity checks.
                profile = (read_device_profile(path) if source == "saved"
                           else read_reference_profile(path))
            except FileNotFoundError:
                continue
            except (OSError, ValueError, UnicodeError) as exc:
                self.diagnostics.append(f"{source} reference: {exc}")
                continue
            if isinstance(profile, TactileDefaultProfile):
                settings = self._load_default_settings()
                return (replace(profile, settings=settings) if settings is not None else profile), "default"
            if profile.device == device:
                return profile, source
        return None

    def _load_default_settings(self):
        try:
            value = _read_document(self.directory / "default-settings.json")
            fields = {"schema_version", "scope", "protocol", "hand", "layout", "settings"}
            if (not isinstance(value, dict) or set(value) != fields
                    or type(value["schema_version"]) is not int or value["schema_version"] != 1
                    or value["scope"] != "layout" or value["protocol"] != PROTOCOL
                    or value["hand"] != "right" or value["layout"] != LAYOUT):
                raise ValueError("Unsupported tactile default display settings")
            return _read_settings(value["settings"])
        except FileNotFoundError:
            return None
        except (OSError, ValueError, UnicodeError) as exc:
            self.diagnostics.append(f"default display settings: {exc}")
            return None

    def save_default_settings(self, settings):
        if not isinstance(settings, ResponseSettings):
            raise ValueError("Expected validated tactile display settings")
        _write_document(self.directory / "default-settings.json",
                        dict(schema_version=1, scope="layout", protocol=PROTOCOL, hand="right", layout=LAYOUT,
                             settings=_settings_document(settings)))

    def save(self, profile):
        if not isinstance(profile, TactileDeviceProfile):
            raise ValueError("Expected a validated tactile device profile")
        _write_document(self.directory / (profile.device.key + ".json"), profile.to_dict())

    def capture(self, device, reference, settings):
        return TactileDeviceProfile(device, reference, settings, datetime.now(timezone.utc).isoformat())
