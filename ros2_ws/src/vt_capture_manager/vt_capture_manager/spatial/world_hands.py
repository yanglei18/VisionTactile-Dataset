"""Project calibrated optical hand frames at their own colour callback host time."""
from collections import deque
from dataclasses import dataclass
import re

from vt_capture_core.spatial.transforms import RigidTransform, finite_vector
from .camera_geometry import OpticalRegistration


SIDES = ('left', 'right')
MAX_AGE_NS = 250_000_000
REASONS = {
    'cube_lost': '方块暂时不可见', 'cube_unbound': '请绑定该手的方块',
    'cube_ambiguous': '方块身份不确定，请重新绑定', 'mount_missing': '请先完成手背标定',
    'mount_changed': '正在加载新标定', 'mount_model_mismatch': '手模已更换，请重新标定该手',
    'layout_mismatch': '方块布局与标定不符', 'camera_timing_unavailable': '等待相机帧时序',
    'identity_mismatch': '相机设备身份不符', 'identity_unverified': '等待确认相机设备身份',
    'missing_calibration': '缺少相机与 Tracker 的外参', 'tracker_mismatch': 'Tracker 身份与相机标定不符',
    'tracker_unavailable': '等待有效的相机 Tracker 位姿',
}


@dataclass(frozen=True)
class HandFrame:
    stamp: int
    color_number: int
    source_ns: int
    received_ns: int
    registration: OpticalRegistration
    wrist: RigidTransform
    joints: tuple
    mount: str
    layout: str
    shape_identity: tuple


class OpticalHandTrack:
    def __init__(self, hand):
        self.hand = hand
        self.instance = ''
        self.index = 0
        self.retired = deque(maxlen=16)
        self.frames = deque(maxlen=24)
        self.camera = None
        self.reason = ''
        self.epoch_start = 0

    def invalidate(self, reason):
        self.frames.clear()
        self.reason = reason

    def reset(self, source_ns):
        self.epoch_start = source_ns
        self.invalidate('场景坐标或时间已变化，等待新观测')

    def observe(self, msg, registry, now_ns, received_ns):
        """Keep ordering across visibility loss, stop/restart and duplicate delivery."""
        if msg.schema_version != 1 or msg.hand != self.hand:
            return
        instance, index = msg.cube_instance_id, msg.cube_source_image_index
        if not instance or len(instance) > 128 or index < 1 or instance in self.retired:
            return
        if self.instance != instance:
            if self.instance:
                self.retired.append(self.instance)
            self.instance, self.index = instance, 0
            self.frames.clear()
        if index <= self.index:
            return
        self.index = index
        self.camera = msg.camera_name if isinstance(msg.camera_name, str) and 0 < len(msg.camera_name) <= 128 else None
        if not msg.wrist_valid:
            self.invalidate(REASONS.get(msg.status, '等待有效的手腕观测'))
            return
        registration = registry.optical_registration(self.camera) if registry is not None else None
        if registration is None:
            raise ValueError('场景中缺少该相机的内参或 Tracker 外参')
        if (msg.header.frame_id != registration.frame or msg.camera_serial != registration.serial
                or msg.intrinsics_sha256 != registration.intrinsics):
            raise ValueError('手部观测的相机、光学坐标系或内参与场景不符')
        stamp = msg.header.stamp.sec * 1_000_000_000 + msg.header.stamp.nanosec
        source_ns = msg.color_host_realtime_ns
        if (stamp <= 0 or source_ns <= 0 or source_ns < self.epoch_start
                or not 0 <= now_ns-source_ns <= MAX_AGE_NS):
            raise ValueError('手部观测时标不可用或已经过期')
        if not all(re.fullmatch('[0-9a-f]{64}', v) for v in (msg.mount_sha256, msg.layout_sha256)):
            raise ValueError('缺少手背标定或方块布局')
        p, q = msg.wrist.position, msg.wrist.orientation
        wrist = RigidTransform((p.x, p.y, p.z), (q.x, q.y, q.z, q.w))
        joints = ()
        if msg.joints_valid:
            if len(msg.joint_positions_m) != 63:
                raise ValueError('手部关节点数量不符')
            flat = finite_vector(msg.joint_positions_m, 63, '手部关节')
            joints = tuple(flat[i:i+3] for i in range(0, 63, 3))
            if (msg.status != 'tracked' or any(point[2] <= 0 for point in joints)
                    or any(abs(a-b) > .0002 for a,b in zip(joints[0], wrist.translation_m))):
                raise ValueError('手部关节与腕部位姿不一致')
        if wrist.translation_m[2] <= 0:
            raise ValueError('手腕在相机视域后方')
        identity = (msg.glove_session_id, msg.glove_model_rest_sha256,
                    msg.glove_config_sha256, msg.glove_calibration_version) if joints else ()
        frame = HandFrame(stamp, msg.color_frame_number, source_ns, received_ns, registration, wrist, joints,
                          msg.mount_sha256, msg.layout_sha256, identity)
        if self.frames:
            previous = self.frames[-1]
            if (previous.registration != registration or previous.mount != frame.mount or previous.layout != frame.layout
                    or previous.shape_identity != identity or source_ns <= previous.source_ns or stamp <= previous.stamp):
                self.frames.clear()
        self.frames.append(frame)
        self.reason = ''

    def entity(self, cameras, registry, buffers, now, mono, max_gap, aligned):
        result = dict(id=self.hand, hand=self.hand, label='左手' if self.hand == 'left' else '右手',
                      camera_id=self.camera, source_realtime_ns=None, state='waiting', valid=False,
                      reason='等待手背跟踪', position_m=None, quaternion_xyzw=None, joint_positions_m=[])
        if not self.frames:
            if not cameras:
                result['reason'] = '请在联合空间配置头部相机及其 Tracker 外参'
            elif self.reason:
                result.update(reason=self.reason, state='invalid')
            return result
        camera = cameras.get(self.camera)
        row = self.frames[-1]
        if aligned:
            key = (camera or {}).get('frame_key') or {}
            row = next((frame for frame in reversed(self.frames)
                        if frame.stamp == key.get('shared_ros_timestamp_ns')
                        and frame.color_number == key.get('color_frame_number')), None)
            if row is None:
                result['reason'] = '等待与本路点云同帧的手部观测'
                return result
        result['source_realtime_ns'] = row.source_ns
        if not (0 <= now-row.source_ns <= MAX_AGE_NS and 0 <= mono-row.received_ns <= MAX_AGE_NS):
            result.update(state='offline', reason='手部观测已过期')
            return result
        if camera is None or registry is None or row.registration != registry.optical_registration(self.camera):
            result.update(state='invalid', reason='相机几何已变化，等待新观测')
            return result
        if camera.get('registration_state') != 'ready':
            result.update(reason=REASONS.get(camera.get('registration_state'), '等待有效的相机空间标定'))
            return result
        role = row.registration.tracker_role
        buffer = buffers.get(role)
        if buffer is None or buffer.latest is None or buffer.latest.tracker_id != row.registration.tracker_id:
            result['reason'] = '等待标定所用的 Tracker'
            return result
        selection = buffer.select(row.source_ns, max_gap_ns=max_gap)
        if selection.pose is None:
            result['reason'] = '图像时刻缺少可用的 Tracker 位姿'
            return result
        world_from_color = selection.pose.compose(row.registration.tracker_from_color)
        wrist = world_from_color.compose(row.wrist)
        result.update(valid=True, state='fresh', reason='' if row.joints else '仅腕部有效，等待手套关节数据',
                      position_m=list(wrist.translation_m), quaternion_xyzw=list(wrist.quaternion_xyzw),
                      joint_positions_m=[list(world_from_color.apply(point)) for point in row.joints])
        return result
