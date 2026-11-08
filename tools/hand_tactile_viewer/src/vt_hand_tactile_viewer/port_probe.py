"""Compatibility import for the shared hand implementation."""
from vt_capture_console.backend.hand_ports.port_probe import (
    PROBE_SECONDS,
    READ_TIMEOUT,
    MAX_READ_BYTES,
    MAX_BYTES,
    MAX_READS,
    MAX_BAUDRATES,
    ProbeResult,
    _imu_frame_key,
    _ConsecutiveFrames,
    _Evidence,
    _cancelled,
    _open,
    _close,
    _failure,
    _scan,
    probe_port
)
