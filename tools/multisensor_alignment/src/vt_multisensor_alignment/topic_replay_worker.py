"""Read-only, topic-independent bag playback in an isolated ROS Python worker.

Only message timestamps are indexed. Payloads are fetched on demand, never
published, and never require another sensor, calibration, or a passing QC report.
"""
from __future__ import annotations

from array import array
from bisect import bisect_left, bisect_right
from collections import OrderedDict, deque
import copy
import json
import math
import os
from pathlib import Path
import sys

from .bag_input import absolute_path, inventory as _file_inventory, fingerprint as _fingerprint
from .bag_reader import _open_reader, _message_types, _deserializer, stamp_to_nanoseconds

MAX_MESSAGES = 10_000_000
MAX_REPLY_BYTES = 4 * 1024 * 1024
MAX_PAYLOAD_BYTES = 64 * 1024 * 1024
KINDS = {
    'sensor_msgs/msg/Image': 'camera', 'sensor_msgs/msg/CompressedImage': 'camera',
    'vt_tracker_msgs/msg/TrackerSample': 'tracker',
    'vt_glove_msgs/msg/GloveState': 'glove',
    'vt_tactile_msgs/msg/TactileState': 'tactile',
    'vt_tactile_msgs/msg/TactileSampleArray': 'tactile',
}


def _close(reader):
    if reader is not None and hasattr(reader, 'close'):
        reader.close()


def _bounded_fields(message):
    """Bounded recursive ROS fields; large buffers are described, not copied."""
    truncated = [False]
    budget = [1200]

    def visit(value, depth=0):
        budget[0] -= 1
        if budget[0] < 0 or depth > 6:
            truncated[0] = True
            return '<truncated>'
        if value is None or isinstance(value, (bool, int)):
            return value
        if isinstance(value, float):
            return value if math.isfinite(value) else str(value)
        if isinstance(value, str):
            if len(value) > 4096:
                truncated[0] = True
            return value[:4096]
        if isinstance(value, (bytes, bytearray, memoryview)):
            truncated[0] |= len(value) > 32
            return dict(bytes=len(value), prefix_hex=bytes(value[:32]).hex())
        if hasattr(value, 'get_fields_and_field_types'):
            fields = list(value.get_fields_and_field_types())
            truncated[0] |= len(fields) > 64
            return {key: visit(getattr(value, key), depth + 1) for key in fields[:64]}
        if isinstance(value, dict) or hasattr(value, '__dict__'):
            items = list((value if isinstance(value, dict) else vars(value)).items())
            truncated[0] |= len(items) > 64
            return {str(key)[:128]: visit(item, depth + 1) for key, item in items[:64] if not str(key).startswith('_')}
        if hasattr(value, '__len__') and hasattr(value, '__getitem__'):
            count = len(value)
            if count > 64:
                truncated[0] = True
                return dict(length=count, first=[visit(value[index], depth + 1) for index in range(64)])
            return [visit(value[index], depth + 1) for index in range(count)]
        return str(value)[:256]

    return dict(representation='decoded', fields=visit(message), truncated=truncated[0])


def _vector(values, count):
    values = list(values)
    if len(values) != count or any(not math.isfinite(float(value)) for value in values):
        raise ValueError('记录的坐标/数组无效')
    return [float(value) for value in values]


def _stamp(message):
    try:
        return stamp_to_nanoseconds(message.header.stamp)
    except (AttributeError, TypeError, ValueError):
        return None


def _device(topic, type_name, message, sample_ns, time_ns, warnings):
    kind = KINDS.get(type_name, 'message')
    preview = dict(mode='replay', replay=True)
    side = getattr(message, 'hand', None)
    frame_id = getattr(getattr(message, 'header', None), 'frame_id', '')
    source_ns = _stamp(message)
    if type_name == 'sensor_msgs/msg/Image':
        from vt_capture_manager.image_preview import jpeg_preview
        preview.update(jpeg_preview(message, max_width=640, max_height=480))
        if str(message.encoding).lower() in ('16uc1', '32fc1', 'mono16'):
            warnings.append('深度以相对灰度显示；未推断深度单位或空间标定')
    elif type_name == 'sensor_msgs/msg/CompressedImage':
        import cv2
        import numpy as np
        from types import SimpleNamespace
        from vt_capture_manager.image_preview import jpeg_preview
        if len(message.data) > 16 * 1024 * 1024:
            raise ValueError('压缩图像超过 16 MiB')
        pixels = cv2.imdecode(np.frombuffer(bytes(message.data), dtype=np.uint8), cv2.IMREAD_COLOR)
        if pixels is None or pixels.size > 32 * 1024 * 1024:
            raise ValueError('压缩图像无效或超出预览大小限制')
        height, width = pixels.shape[:2]
        preview.update(jpeg_preview(SimpleNamespace(width=width, height=height, step=width * 3,
                        encoding='bgr8', is_bigendian=False, data=pixels.tobytes()), max_width=640, max_height=480))
    elif kind == 'tracker':
        if not message.pose_valid:
            warnings.append('该条 Tracker 位姿标记为无效；保留原始字段供检查')
            return None
        p, q = message.pose.position, message.pose.orientation
        position, orientation = _vector((p.x, p.y, p.z), 3), _vector((q.x, q.y, q.z, q.w), 4)
        if not .9 <= math.hypot(*orientation) <= 1.1:
            raise ValueError('记录的 Tracker 四元数无效')
        preview.update(position=position, orientation=orientation, valid=True,
                       frame_id=frame_id, source_realtime_ns=source_ns)
    elif kind == 'glove':
        if not (message.pose_valid and message.time_valid):
            warnings.append('该条手套姿态或时间无效；保留原始字段供检查')
            return None
        joints = list(getattr(message, 'joint_positions_m', ()))
        if joints:
            flat = _vector(joints, 63)
            preview.update(joints=[flat[i:i+3] for i in range(0, 63, 3)], valid=True)
        pose, shape = list(getattr(message, 'mano_pose_rotvec', ())), list(getattr(message, 'shape_params', ()))
        if pose and shape:
            try:
                preview['mano'] = dict(pose=_vector(pose, 48), shape=_vector(shape, 10), hand=side,
                    source_mode='replay', source_realtime_ns=_stamp(message) or 0,
                    session_id=str(message.session_id), model_rest_sha256=str(message.model_rest_sha256),
                    config_sha256=str(message.config_sha256), calibration_version=int(message.calibration_version),
                    frame_index=int(message.frame_index))
                preview['valid'] = True
            except (AttributeError, TypeError, ValueError) as exc:
                warnings.append('MANO 参数不可用: ' + str(exc)[:160])
        if not preview.get('valid'):
            warnings.append('该条消息不含可显示的关节点或 MANO 参数')
            return None
    elif type_name == 'vt_tactile_msgs/msg/TactileState':
        rows, cols = int(message.rows), int(message.cols)
        if (int(message.schema_version), rows, cols) not in ((1, 80, 80), (2, 20, 20)):
            raise ValueError('触觉网格/版本不受支持')
        if len(message.adc) != rows * cols or len(message.valid_mask) != rows * cols:
            raise ValueError('触觉网格长度不匹配')
        values = _vector([message.adc[r*cols+c] for r in range(20) for c in range(20)], 400)
        mask = [bool(message.valid_mask[r*cols+c]) for r in range(20) for c in range(20)]
        # These are the masks actually recorded in this rolling snapshot, not
        # fresh live measurements. Never compare historical clocks with now.
        if not message.connected or not message.time_valid:
            mask = [False] * 400
        preview.update(values=values, valid_mask=mask, rows=20, cols=20)
        warnings.append('触觉显示记录时的滚动快照及有效掩码，不表示当前实时状态')
        if not any(mask):
            warnings.append('该触觉快照没有有效单元；保留原始字段供检查')
            return None
    else:
        if type_name == 'vt_tactile_msgs/msg/TactileSampleArray':
            warnings.append('触觉原始消息为稀疏批次；按记录展示其字段，不拼接为同时采样的整手图')
        return None
    return dict(id=topic, label=topic, kind=kind, side=side, status='ready', preview=preview,
                frame_id=frame_id, source_realtime_ns=source_ns, bag_timestamp_ns=sample_ns,
                host_realtime_ns=getattr(message, 'host_realtime_ns', None),
                tracker_id=getattr(message, 'tracker_id', None), role=getattr(message, 'role', None),
                detail='历史录制数据；以包内时间播放')


class TopicReplaySource:
    def __init__(self, path, *, reader_factory=None, deserializer_factory=None, cancelled=None):
        self.path = absolute_path(path)
        if self.path.is_file():
            self.path = self.path.parent
        if not (self.path / 'metadata.yaml').is_file() and (self.path / 'bag' / 'metadata.yaml').is_file():
            self.path = self.path / 'bag'
        self._factory = reader_factory or _open_reader
        self._decode_factory = deserializer_factory or _deserializer
        self._cancelled = cancelled or (lambda: False)
        self._reader = None
        self._counts = {}
        self._decoders = {}
        self._cache = OrderedDict()
        self._closed = False
        info, before = _file_inventory(self.path)
        self._inventory = before
        self._unordered = {}
        self._reader_topic = None
        self._seeking = False
        warnings = deque(maxlen=8192)
        declared = {}
        for entry in info.get('topics_with_message_count', ()):
            meta = entry.get('topic_metadata', {})
            if isinstance(meta.get('name'), str) and isinstance(meta.get('type'), str):
                declared[meta['name']] = (meta['type'], int(entry.get('message_count', 0)))
        reader = self._factory(self.path)
        try:
            self._types = {name: kind for name, (kind, _) in declared.items()}
            self._types.update(_message_types(reader))
            if not self._types or len(self._types) > 4096:
                raise ValueError('录制包没有话题或话题数超过 4096')
            self._times = {name: array('q') for name in self._types}
            count = 0
            sequences = {}
            while True:
                if self._cancelled():
                    raise InterruptedError('回放加载已取消')
                try:
                    if not reader.has_next():
                        break
                    topic, _payload, bag_ns = reader.read_next()
                except Exception as exc:
                    warnings.append('存储读取中断；可回放已读取的部分: ' + str(exc)[:600])
                    break
                count += 1
                if count > MAX_MESSAGES:
                    raise ValueError('录制包超过 1000 万条消息索引限制')
                if topic not in self._times:
                    warnings.append('忽略未声明的话题: ' + str(topic)[:256])
                    continue
                sequence = sequences.get(topic, 0)
                sequences[topic] = sequence + 1
                stamp = int(bag_ns)
                if not 0 <= stamp < 2**63:
                    warnings.append('忽略包时间无效的消息: ' + str(topic)[:256])
                    continue
                if sequence != len(self._times[topic]):
                    if topic not in self._unordered:
                        self._unordered[topic] = array('Q', range(len(self._times[topic])))
                    self._unordered[topic].append(sequence)
                self._times[topic].append(stamp)
        finally:
            _close(reader)
        _, after = _file_inventory(self.path)
        if before != after:
            raise ValueError('录制包在读取期间发生变化；请先结束录制')
        for name, times in self._times.items():
            if any(a > b for a, b in zip(times, times[1:])):
                order = sorted(range(len(times)), key=times.__getitem__)
                original = self._unordered.get(name)
                self._unordered[name] = array('Q', (original[index] if original is not None else index for index in order))
                self._times[name] = times = array('q', (times[index] for index in order))
                warnings.append(name + ': 消息时间乱序，按包时间排序回放')
            if not times:
                warnings.append(name + ': 没有录到消息')
            elif name in declared and declared[name][1] != len(times):
                warnings.append(name + ': 实际读取数量与 metadata.yaml 不一致')
        nonempty = [times for times in self._times.values() if times]
        if not any(self._types[name] == 'vt_tracker_msgs/msg/TrackerSample' and times for name, times in self._times.items()):
            warnings.append('缺少可用的 Tracker 消息；已有图像和其他话题仍可独立回放')
        for kind, label in (('glove', '手套姿态'), ('tactile', '触觉')):
            if not any(KINDS.get(self._types[name]) == kind and times for name, times in self._times.items()):
                warnings.append('未录入' + label + '消息；其他话题可独立回放')
        geometry_path = self.path.with_name(self.path.name + '.spatial') / 'manifest.json'
        if not geometry_path.is_file():
            warnings.append('未记录空间几何配置；原始图像和话题回放不依赖空间标定')
        else:
            try:
                if geometry_path.stat().st_size > 512 * 1024:
                    raise ValueError('空间几何清单过大')
                geometry = json.loads(geometry_path.read_text())
                if (not isinstance(geometry, dict) or geometry.get('geometry_status') == 'incomplete'
                        or not geometry.get('alignment_config') or not geometry.get('revisions')):
                    warnings.append('空间几何配置不完整；原始图像和话题仍可独立回放')
            except (OSError, ValueError, UnicodeError) as exc:
                warnings.append('空间几何清单无法读取，不影响话题回放: ' + str(exc)[:256])
        self.metadata = dict(path=str(self.path), start_ns=min((times[0] for times in nonempty), default=0),
            end_ns=max((times[-1] for times in nonempty), default=0),
            topics=[dict(name=name, type=kind, count=len(self._times[name]), kind=KINDS.get(kind, 'message'))
                    for name, kind in self._types.items()], warnings=list(warnings))

    def _payload(self, topic, index, stamp):
        if self._reader is None:
            self._reader = self._factory(self.path)
            self._counts = {}
        # A corrupt/out-of-order storage stream cannot safely seek by time,
        # including when selecting an otherwise ordered neighbouring topic.
        if hasattr(self._reader, 'seek') and not self._unordered:
            if self._reader_topic != topic and hasattr(self._reader, 'set_filter'):
                try:
                    from rosbag2_py import StorageFilter
                    self._reader.set_filter(StorageFilter(topics=[topic]))
                    self._reader_topic = topic
                except ImportError:
                    pass  # Injectable readers need not import the ROS runtime.
            self._reader.seek(stamp)
            self._seeking = True
            duplicate = index - bisect_left(self._times[topic], stamp)
            while self._reader.has_next():
                if self._cancelled():
                    raise InterruptedError('回放请求已取消')
                current, payload, current_ns = self._reader.read_next()
                if current_ns > stamp:
                    break
                if current == topic and current_ns == stamp:
                    if duplicate == 0:
                        return payload
                    duplicate -= 1
        else:
            index = int(self._unordered[topic][index]) if topic in self._unordered else index
            if self._seeking or index < self._counts.get(topic, 0):
                _close(self._reader)
                self._reader = self._factory(self.path)
                self._counts = {}
                self._reader_topic = None
                self._seeking = False
            while self._reader.has_next():
                if self._cancelled():
                    raise InterruptedError('回放请求已取消')
                current, payload, current_ns = self._reader.read_next()
                sequence = self._counts.get(current, 0)
                self._counts[current] = sequence + 1
                if current == topic and sequence == index:
                    if int(current_ns) != stamp:
                        raise ValueError('录制包索引与消息时间不一致')
                    return payload
        raise ValueError('录制包中未找到索引指定的消息')

    def frame(self, topic, time_ns):
        if self._closed:
            raise ValueError('回放源已关闭')
        if self._cancelled():
            raise InterruptedError('回放请求已取消')
        if _fingerprint(entry[0] for entry in self._inventory) != self._inventory:
            raise ValueError('录制包在加载后发生变化；请重新加载档案')
        if topic not in self._times or type(time_ns) is not int:
            raise ValueError('回放话题或时间无效')
        result = dict(topic=topic, time_ns=time_ns, sample_ns=None, device=None, details={}, warnings=[])
        times = self._times[topic]
        index = bisect_right(times, time_ns) - 1
        if index < 0:
            result['warnings'] = ['该话题没有录到消息' if not times else '当前回放时刻之前尚无该话题消息']
            return result
        sample_ns = int(times[index])
        key = (topic, index)
        if key in self._cache:
            result = copy.deepcopy(self._cache[key])
            self._cache.move_to_end(key)
        else:
            result['sample_ns'] = sample_ns
            try:
                payload = self._payload(topic, index, sample_ns)
                if len(payload) > MAX_PAYLOAD_BYTES:
                    raise ValueError('单条消息超过 64 MiB 解码限制')
                kind = self._types[topic]
                if kind not in self._decoders:
                    try:
                        self._decoders[kind] = self._decode_factory(kind)
                    except Exception as exc:
                        self._decoders[kind] = str(exc)[:600]
                decoder = self._decoders[kind]
                if not callable(decoder):
                    result['details'] = dict(representation='serialized', type=kind,
                                              serialized_bytes=len(payload), prefix_hex=bytes(payload[:64]).hex())
                    result['warnings'].append('当前环境缺少该消息类型的解码支持；显示原始消息摘要: ' + str(decoder))
                else:
                    message = decoder(payload)
                    result['details'] = _bounded_fields(message)
                    result['frame_id'] = getattr(getattr(message, 'header', None), 'frame_id', '')
                    result['source_realtime_ns'] = _stamp(message)
                    result['host_realtime_ns'] = getattr(message, 'host_realtime_ns', None)
                    result['tracker_id'] = getattr(message, 'tracker_id', None)
                    result['role'] = getattr(message, 'role', None)
                    try:
                        result['device'] = _device(topic, kind, message, sample_ns, time_ns, result['warnings'])
                    except Exception as exc:
                        result['warnings'].append('本条消息无法生成图形预览，仍可查看字段: ' + str(exc)[:600])
            except InterruptedError:
                raise
            except Exception as exc:
                result['warnings'].append('本条消息读取或解码失败，可切换时间或其他话题: ' + str(exc)[:600])
            self._cache[key] = copy.deepcopy(result)
            while len(self._cache) > 8:
                self._cache.popitem(last=False)
        result['time_ns'] = time_ns
        result['sample_age_ns'] = max(0, time_ns - sample_ns)
        if time_ns - sample_ns > 1_000_000_000:
            result['warnings'].append(f'保留最近一条历史消息，距当前回放时刻 {(time_ns-sample_ns)/1e9:.2f} 秒')
        return result

    def close(self):
        self._closed = True
        _close(self._reader)
        self._reader = None
        self._cache.clear()


# Public alias kept descriptive for callers that use the bag terminology.
TopicBagSource = TopicReplaySource


def main(argv=None):
    args = sys.argv[1:] if argv is None else argv
    # ROS storage libraries may print to stdout at the native file-descriptor
    # level. Reserve a duplicate for IPC and redirect all ordinary output.
    protocol = os.fdopen(os.dup(sys.stdout.fileno()), 'w', buffering=1)
    os.dup2(sys.stderr.fileno(), sys.stdout.fileno())
    source = None

    def reply(value):
        encoded = json.dumps(value, ensure_ascii=False, allow_nan=False)
        if len(encoded.encode('utf-8')) > MAX_REPLY_BYTES:
            encoded = json.dumps(dict(ok=False, request_id=value.get('request_id'), error='回放响应超过 4 MiB'))
        protocol.write(encoded + '\n')
        protocol.flush()

    try:
        if len(args) != 1:
            raise ValueError('需要一个录制包路径')
        source = TopicReplaySource(args[0])
        reply(dict(ok=True, metadata=source.metadata))
        for line in sys.stdin:
            request_id = None
            try:
                if len(line) > 65536:
                    raise ValueError('回放请求过大')
                request = json.loads(line)
                request_id = request.get('request_id')
                if type(request_id) is not int or request_id <= 0 or request.get('command') != 'frame':
                    raise ValueError('回放请求无效')
                frame = source.frame(request.get('topic'), request.get('time_ns'))
                reply(dict(ok=True, frame=frame, request_id=request_id))
            except Exception as exc:
                reply(dict(ok=False, request_id=request_id, error=str(exc)[:1800]))
    except Exception as exc:
        reply(dict(ok=False, error=str(exc)[:1800]))
        return 1
    finally:
        if source is not None:
            source.close()
        protocol.close()
    return 0


if __name__ == '__main__':
    raise SystemExit(main())
