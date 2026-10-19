"""Immutable source samples and explicit selection outcomes."""

from dataclasses import dataclass

from .transforms import RigidTransform


def timestamp_ns(value, name='timestamp_ns'):
    if type(value) is not int or not 0 <= value <= 2**63-1:
        raise ValueError(f'{name} must be an unsigned signed-64-bit-range integer')
    return value


@dataclass(frozen=True)
class PoseSample:
    role: str
    tracker_id: str
    parent_frame: str
    source_realtime_ns: int
    received_monotonic_ns: int
    time_epoch: str
    coordinate_epoch: str
    pose: RigidTransform | None
    pose_valid: bool

    def __post_init__(self):
        for name in ('role', 'tracker_id', 'parent_frame', 'time_epoch', 'coordinate_epoch'):
            value = getattr(self, name)
            if not isinstance(value, str) or not value.strip() or len(value) > 256:
                raise ValueError(f'{name} must be a nonempty bounded string')
        timestamp_ns(self.source_realtime_ns, 'source_realtime_ns')
        timestamp_ns(self.received_monotonic_ns, 'received_monotonic_ns')
        if type(self.pose_valid) is not bool or (self.pose is not None and not isinstance(self.pose, RigidTransform)):
            raise ValueError('invalid pose or validity flag')
        if self.pose_valid and self.pose is None:
            raise ValueError('valid pose requires a transform')

    @property
    def identity(self):
        return (self.role, self.tracker_id, self.parent_frame, self.time_epoch, self.coordinate_epoch)


@dataclass(frozen=True)
class InsertResult:
    accepted: bool
    reason: str = ''
    reset: bool = False


@dataclass(frozen=True)
class PoseSelection:
    pose: RigidTransform | None
    status: str
    reason: str = ''
    before_ns: int | None = None
    after_ns: int | None = None
    gap_ns: int | None = None
