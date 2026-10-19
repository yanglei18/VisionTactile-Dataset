from types import SimpleNamespace as NS
import struct

import pytest

from vt_multisensor_alignment.image_reader import BagImageReader, decode_image
from vt_multisensor_alignment.model import MessageRef


def image(data, *, width=2, height=2, step=6, encoding='16UC1', bigendian=1, frame='depth', stamp=10):
    return NS(width=width, height=height, step=step, encoding=encoding, is_bigendian=bigendian,
              data=data, header=NS(frame_id=frame, stamp=NS(sec=0, nanosec=stamp)))


def test_endian_and_padded_rows_preserve_original_depth_values():
    source = struct.pack('>HHH', 1000, 2000, 999)+struct.pack('>HHH', 3000, 4000, 888)
    result = decode_image(image(source))
    assert result.pixels().tolist() == [[1000, 2000], [3000, 4000]]
    assert result.step == 6 and result.encoding == '16UC1'
    assert not result.pixels().flags.writeable


@pytest.mark.parametrize('changes', [{'step': 3}, {'data': b''}, {'width': 1000000},
                                    {'encoding': 'unknown'}, {'is_bigendian': 7}])
def test_invalid_image_layout_rejected(changes):
    message = image(bytes(12))
    for name, value in changes.items():
        setattr(message, name, value)
    with pytest.raises(ValueError):
        decode_image(message)


def test_precise_reference_checks_all_four_identity_fields_and_rewinds(tmp_path):
    events = [('/depth', image(bytes(12), stamp=10), 100),
              ('/other', object(), 101),
              ('/depth', image(bytes(12), stamp=20), 200)]
    calls = []
    def factory(path):
        queue = list(events)
        return NS(get_all_topics_and_types=lambda: [NS(name='/depth', type='sensor_msgs/msg/Image'), NS(name='/other', type='std_msgs/msg/String')],
                  has_next=lambda: bool(queue), read_next=lambda: queue.pop(0))
    def decode(payload):
        calls.append(payload)
        return payload
    reader = BagImageReader(tmp_path, reader_factory=factory, deserialize=decode, cache_frames=1)
    second = MessageRef('/depth', 1, 200, 20)
    first = MessageRef('/depth', 0, 100, 10)
    assert reader.read(second, expected_frame_id='depth').source_timestamp_ns == 20
    assert len(calls) == 1
    assert reader.read(first).source_timestamp_ns == 10
    assert len(calls) == 2
    for wrong in [MessageRef('/depth', 0, 101, 10), MessageRef('/depth', 0, 100, 11)]:
        with pytest.raises(ValueError, match='reference'):
            reader.read(wrong)
    with pytest.raises(ValueError, match='frame'):
        reader.read(first, expected_frame_id='color')
    reader.close()
    with pytest.raises(ValueError, match='closed'):
        reader.read(first)


def test_cancellation_prevents_unbounded_scan(tmp_path):
    factory = lambda path: NS(get_all_topics_and_types=lambda: [NS(name='/depth', type='sensor_msgs/msg/Image')],
                              has_next=lambda: True, read_next=lambda: pytest.fail('cancelled before read'))
    reader = BagImageReader(tmp_path, reader_factory=factory, deserialize=lambda x: x, cancelled=lambda: True)
    with pytest.raises(InterruptedError):
        reader.read(MessageRef('/depth', 5000, 100, 10))
