"""On-demand real Image decoding by exact alignment MessageRef.

ROS CDR deserialization is lazy and intended for an isolated system-Python
worker. The indexer continues to store references instead of full pixel arrays.
"""
from collections import OrderedDict
from dataclasses import dataclass
from pathlib import Path

import numpy as np

from .bag_reader import _open_reader, _message_types, _deserializer, stamp_to_nanoseconds
from .model import MessageRef

MAX_IMAGE_BYTES = 64 * 1024 * 1024
_ENCODINGS = {'16UC1': ('u2', 1), '32FC1': ('f4', 1), 'mono8': ('u1', 1),
              'rgb8': ('u1', 3), 'bgr8': ('u1', 3), 'rgba8': ('u1', 4), 'bgra8': ('u1', 4)}


@dataclass(frozen=True)
class ImageFrame:
    width: int
    height: int
    encoding: str
    step: int
    is_bigendian: bool
    data: bytes
    frame_id: str
    source_timestamp_ns: int
    reference: MessageRef | None = None

    def pixels(self) -> np.ndarray:
        """Read-only strided native-valued view; no unit or color conversion."""
        dtype, channels = _ENCODINGS[self.encoding]
        dtype = np.dtype(('>' if self.is_bigendian else '<')+dtype)
        shape = (self.height, self.width) if channels == 1 else (self.height, self.width, channels)
        strides = (self.step, dtype.itemsize) if channels == 1 else (self.step, channels*dtype.itemsize, dtype.itemsize)
        return np.ndarray(shape, dtype=dtype, buffer=self.data, strides=strides)


def decode_image(message, *, reference=None, expected_frame_id=None) -> ImageFrame:
    width, height, step = message.width, message.height, message.step
    if any(type(v) is not int or v <= 0 for v in (width, height, step)) or width > 16384 or height > 16384:
        raise ValueError('image dimensions/step are outside bounded limits')
    encoding = message.encoding
    if encoding not in _ENCODINGS:
        raise ValueError(f'unsupported image encoding: {encoding}')
    code, channels = _ENCODINGS[encoding]
    minimum = width*np.dtype(code).itemsize*channels
    if step < minimum or step*height > MAX_IMAGE_BYTES:
        raise ValueError('image step/byte count is invalid')
    endian = message.is_bigendian
    if type(endian) not in (int, bool) or endian not in (0, 1):
        raise ValueError('image is_bigendian must be 0 or 1')
    if len(message.data) != step*height:
        raise ValueError('image data does not match height times step')
    data = bytes(message.data)
    frame = message.header.frame_id
    if not isinstance(frame, str) or not frame or len(frame) > 256:
        raise ValueError('image frame is missing or invalid')
    if expected_frame_id is not None and frame != expected_frame_id:
        raise ValueError('image frame does not match requested stream')
    source = stamp_to_nanoseconds(message.header.stamp)
    if reference is not None and source != reference.source_timestamp_ns:
        raise ValueError('image source timestamp does not match reference')
    return ImageFrame(width, height, encoding, step, bool(endian), data, frame, source, reference)


class BagImageReader:
    def __init__(self, bag_path, *, reader_factory=None, deserialize=None,
                 cache_frames=12, cache_bytes=128*1024*1024, cancelled=None):
        if type(cache_frames) is not int or not 1 <= cache_frames <= 64:
            raise ValueError('cache_frames must be in [1, 64]')
        if type(cache_bytes) is not int or not 1 <= cache_bytes <= 256*1024*1024:
            raise ValueError('cache_bytes must be in [1, 256 MiB]')
        self.bag_path = Path(bag_path).resolve()
        if not self.bag_path.is_dir():
            raise FileNotFoundError(self.bag_path)
        self._factory = reader_factory or _open_reader
        self._deserialize = deserialize
        self._cancelled = cancelled or (lambda: False)
        self.cache_frames, self.cache_bytes = cache_frames, cache_bytes
        self._cache, self._bytes = OrderedDict(), 0
        self._closed = False
        self._reader = None
        self._open()

    def _open(self):
        if self._reader is not None and hasattr(self._reader, 'close'):
            self._reader.close()
        self._reader = self._factory(self.bag_path)
        self._types = _message_types(self._reader)
        self._counts = {}

    def read(self, reference, *, expected_frame_id=None):
        return self.read_many([reference], expected_frames={reference.topic: expected_frame_id})[reference]

    def read_many(self, references, *, expected_frames=None):
        if self._closed:
            raise ValueError('image reader is closed')
        references = tuple(references)
        if len(references) > 64 or any(not isinstance(value, MessageRef) for value in references):
            raise ValueError('at most 64 valid MessageRef objects are allowed')
        expected_frames = expected_frames or {}
        for ref in references:
            if self._types.get(ref.topic) != 'sensor_msgs/msg/Image':
                raise ValueError(f'reference topic is not Image: {ref.topic}')
        if self._cancelled():
            raise InterruptedError('image request cancelled')
        result = {ref: self._cache[ref] for ref in references if ref in self._cache}
        missing = set(references)-set(result)
        if any(ref.sequence < self._counts.get(ref.topic, 0) for ref in missing):
            self._open()
        targets = {}
        for ref in missing:
            key = (ref.topic, ref.sequence)
            if key in targets and targets[key] != ref:
                raise ValueError('conflicting references for the same message sequence')
            targets[key] = ref
        while targets and self._reader.has_next():
            if self._cancelled():
                raise InterruptedError('image request cancelled')
            topic, payload, bag_ns = self._reader.read_next()
            sequence = self._counts.get(topic, 0)
            self._counts[topic] = sequence+1
            ref = targets.get((topic, sequence))
            if ref is None:
                continue
            if bag_ns != ref.bag_timestamp_ns:
                raise ValueError('bag timestamp does not match image reference')
            if isinstance(payload, (bytes, bytearray, memoryview)) and len(payload) > MAX_IMAGE_BYTES+1024*1024:
                raise ValueError('serialized image exceeds bound')
            if self._deserialize is None:
                self._deserialize = _deserializer('sensor_msgs/msg/Image')
            image = decode_image(self._deserialize(payload), reference=ref,
                                 expected_frame_id=expected_frames.get(topic))
            result[ref] = image
            del targets[(topic, sequence)]
            self._cache[ref] = image
            self._bytes += len(image.data)
            while len(self._cache) > self.cache_frames or self._bytes > self.cache_bytes:
                _, expired = self._cache.popitem(last=False)
                self._bytes -= len(expired.data)
        if targets:
            raise ValueError('image reference is absent from bag')
        for ref, image in result.items():
            expected = expected_frames.get(ref.topic)
            if expected is not None and image.frame_id != expected:
                raise ValueError('cached image frame does not match requested stream')
            if ref in self._cache:
                self._cache.move_to_end(ref)
        return result

    def close(self):
        self._closed = True
        if self._reader is not None and hasattr(self._reader, 'close'):
            self._reader.close()
        self._reader = None
        self._cache.clear()
        self._bytes = 0

    def __enter__(self):
        return self

    def __exit__(self, *_):
        self.close()
