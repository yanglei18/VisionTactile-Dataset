from __future__ import annotations

import numpy as np

from .bag_reader import stamp_to_nanoseconds
from .errors import DatasetFormatError, UnsupportedEncodingError
from .model import MessageRef
from .sdk_model import ImageData


_ENCODINGS: dict[str, tuple[np.dtype[object], int]] = {
    "rgb8": (np.dtype("u1"), 3),
    "bgr8": (np.dtype("u1"), 3),
    "mono8": (np.dtype("u1"), 1),
    "mono16": (np.dtype("u2"), 1),
    "16UC1": (np.dtype("u2"), 1),
    "32FC1": (np.dtype("f4"), 1),
}


def _integer_field(
    message: object,
    name: str,
    *,
    positive: bool = False,
) -> int:
    value = getattr(message, name, None)
    if type(value) is not int or (positive and value <= 0):
        condition = "a positive integer" if positive else "an integer"
        raise DatasetFormatError(f"Image.{name} must be {condition}")
    return value


def decode_image(message: object, reference: MessageRef) -> ImageData:
    """Decode a referenced ROS Image without changing its channel semantics."""
    if not isinstance(reference, MessageRef):
        raise TypeError("reference must be MessageRef")
    encoding = getattr(message, "encoding", None)
    if type(encoding) is not str or not encoding:
        raise DatasetFormatError("Image.encoding must be non-empty text")
    format_value = _ENCODINGS.get(encoding)
    if format_value is None:
        raise UnsupportedEncodingError(
            f"unsupported ROS image encoding: {encoding}"
        )
    base_dtype, channels = format_value
    width = _integer_field(message, "width", positive=True)
    height = _integer_field(message, "height", positive=True)
    step = _integer_field(message, "step", positive=True)
    is_bigendian = _integer_field(message, "is_bigendian")
    if is_bigendian not in {0, 1}:
        raise DatasetFormatError("Image.is_bigendian must be zero or one")
    row_bytes = width * channels * base_dtype.itemsize
    if step < row_bytes:
        raise DatasetFormatError(
            f"Image.step {step} is smaller than the {row_bytes}-byte pixel row"
        )
    try:
        payload = bytes(getattr(message, "data"))
    except (AttributeError, TypeError, ValueError) as error:
        raise DatasetFormatError("Image.data is not a byte sequence") from error
    expected_length = height * step
    if len(payload) != expected_length:
        raise DatasetFormatError(
            "Image.data length mismatch: "
            f"expected={expected_length} observed={len(payload)}"
        )
    try:
        header = getattr(message, "header")
        source_timestamp_ns = stamp_to_nanoseconds(getattr(header, "stamp"))
        frame_id = getattr(header, "frame_id")
    except (AttributeError, TypeError, ValueError) as error:
        raise DatasetFormatError(f"Image.header is malformed: {error}") from error
    if type(frame_id) is not str or not frame_id:
        raise DatasetFormatError("Image.header.frame_id must be non-empty text")
    if source_timestamp_ns != reference.source_timestamp_ns:
        raise DatasetFormatError(
            "Image source timestamp does not match message reference: "
            f"expected={reference.source_timestamp_ns} "
            f"observed={source_timestamp_ns}"
        )

    rows = np.frombuffer(payload, dtype=np.uint8).reshape(height, step)
    packed = rows[:, :row_bytes].copy().reshape(-1)
    byte_order = ">" if is_bigendian else "<"
    encoded_dtype = (
        base_dtype
        if base_dtype.itemsize == 1
        else base_dtype.newbyteorder(byte_order)
    )
    values = np.frombuffer(packed, dtype=encoded_dtype)
    native = np.array(values, dtype=base_dtype, copy=True)
    shape = (height, width, channels) if channels > 1 else (height, width)
    array = native.reshape(shape)
    array.setflags(write=False)
    return ImageData(
        array=array,
        encoding=encoding,
        frame_id=frame_id,
        source_timestamp_ns=source_timestamp_ns,
        reference=reference,
    )
