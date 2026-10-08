"""Pure Python contracts shared by the desktop UI and local manager.

Importing this package does not import ROS, Qt, serial drivers or device SDKs.
"""

from .models import (
    Capabilities, CaptureSnapshot, DeviceSnapshot, EventSnapshot,
    RecordingSnapshot, Request, ResourceSnapshot, Response,
    ControllerSnapshot, OperationSnapshot, ManagementSnapshot,
    ControlRequest, ControlResponse, ControlSnapshot, DetailRequest, DetailResponse,
    ProcessGroupSnapshot, ProcessControlSnapshot, DeviceControlSnapshot, DeviceControlRequest, DeviceControlResponse,
)
from .protocol import (
    CONTROL_PROTOCOL_VERSION, DETAIL_PROTOCOL_VERSION, DEVICE_PROTOCOL_VERSION, MAX_MESSAGE_BYTES, PROTOCOL_VERSION, ProtocolError, decode_message, encode_message,
)

__all__ = [
    "Capabilities", "CaptureSnapshot", "DeviceSnapshot", "EventSnapshot",
    "RecordingSnapshot", "Request", "ResourceSnapshot", "Response",
    "MAX_MESSAGE_BYTES", "PROTOCOL_VERSION", "ProtocolError", "decode_message", "encode_message",
    "CONTROL_PROTOCOL_VERSION", "DETAIL_PROTOCOL_VERSION", "ControllerSnapshot", "OperationSnapshot", "ManagementSnapshot",
    "ControlRequest", "ControlResponse", "ControlSnapshot", "DetailRequest", "DetailResponse",
    "DEVICE_PROTOCOL_VERSION", "ProcessGroupSnapshot", "ProcessControlSnapshot", "DeviceControlSnapshot", "DeviceControlRequest", "DeviceControlResponse",
]
