"""ROS conversion shared by live and hardware-free replay; no serial or Qt."""

from dataclasses import fields

from sensor_msgs.msg import Image
from vt_tactile_msgs.msg import TactileRow44, TactileRow44Array, TactileGridState

from .grid import CELL_COUNT, normalize_state


def set_stamp(header, realtime_ns: int, frame_id: str) -> None:
    header.stamp.sec, header.stamp.nanosec = divmod(int(realtime_ns), 1_000_000_000)
    header.frame_id = frame_id


def sample_message(batch, frame_id: str) -> TactileRow44Array:
    result = TactileRow44Array()
    for field in fields(batch):
        if field.name != "blocks":
            setattr(result, field.name, getattr(batch, field.name))
    result.blocks = [TactileRow44(row=block.row, adc=list(block.adc))
                     for block in batch.blocks]
    set_stamp(result.header, batch.host_realtime_ns, frame_id)
    return result


def state_message(snapshot, frame_id: str) -> TactileGridState:
    result = TactileGridState()
    for field in fields(snapshot):
        value = getattr(snapshot, field.name)
        setattr(result, field.name, list(value) if isinstance(value, tuple) else value)
    set_stamp(result.header, snapshot.host_realtime_ns, frame_id)
    return result


def _preview_lut():
    # Small fixed plasma-style LUT; exact plasma is used by the independent GUI.
    anchors = [(13, 8, 135), (75, 3, 161), (126, 3, 168), (168, 34, 150),
               (204, 71, 120), (229, 107, 93), (248, 148, 65), (253, 195, 40), (240, 249, 33)]
    table = []
    for index in range(256):
        position = index * (len(anchors) - 1) / 255
        lo = min(int(position), len(anchors) - 2)
        fraction = position - lo
        table.append(bytes(round(a + fraction * (b - a))
                           for a, b in zip(anchors[lo], anchors[lo + 1])))
    return table


_PREVIEW_LUT = _preview_lut()


def heatmap_message(state: TactileGridState) -> Image:
    """Native 20x20 ADC preview over the unsigned 16-bit scale; no force units."""
    state = normalize_state(state)
    if len(state.adc) != CELL_COUNT or len(state.valid_mask) != CELL_COUNT:
        raise ValueError("Heatmap requires 400 ADC values and validity flags")
    result = Image()
    result.header = state.header
    result.height, result.width = state.rows, state.cols
    result.encoding = "rgb8"
    result.is_bigendian = 0
    result.step = state.cols * 3
    result.data = b"".join(
        _PREVIEW_LUT[max(0, min(255, int(adc) * 255 // 65535))]
        if valid else b"\0\0\0"
        for adc, valid in zip(state.adc, state.valid_mask)
    )
    return result
