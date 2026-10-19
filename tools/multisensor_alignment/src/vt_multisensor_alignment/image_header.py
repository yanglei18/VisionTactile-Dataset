"""Read only the standard ROS 2 Image CDR header; never deserialize pixels.

The storage reader still returns one complete serialized message. This view
avoids a second full Image allocation, pixel conversion, and retained histories;
it does not claim to skip storage I/O or validate the remaining image payload.
"""
import struct


def image_header_stamp_ns(payload: bytes) -> int:
    """Extract a bounded CDR1 Header stamp, rejecting unknown representations."""
    data = memoryview(payload)
    if data.nbytes < 17:
        raise ValueError("truncated Image CDR header")
    representation = int.from_bytes(data[:2], "big")
    if representation not in (0, 1):
        raise ValueError("unsupported Image CDR representation")
    seconds, nanos, length = struct.unpack_from("<iII" if representation else ">iII", data, 4)
    if seconds < 0 or nanos >= 1_000_000_000:
        raise ValueError("Image header stamp is outside its range")
    if not 1 <= length <= 4097 or 16 + length > data.nbytes or data[15 + length] != 0:
        raise ValueError("invalid or truncated Image header frame_id")
    return seconds * 1_000_000_000 + nanos
