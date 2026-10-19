"""Bounded, source-time ordered poses; no extrapolation or boundary crossing.

The owner serializes access and owns publication order and clock discontinuity
audits. A selection is a pure query so different camera timestamps may query
the same history without changing its freshness or publication state.
"""

from bisect import bisect_left
from collections import deque

from .types import InsertResult, PoseSample, PoseSelection, timestamp_ns


class PoseBuffer:
    def __init__(self, *, max_samples=512, max_age_ns=2_000_000_000):
        if type(max_samples) is not int or not 1 <= max_samples <= 65536:
            raise ValueError('max_samples must be between 1 and 65536')
        timestamp_ns(max_age_ns, 'max_age_ns')
        if max_age_ns == 0:
            raise ValueError('max_age_ns must be positive')
        self.max_samples, self.max_age_ns = max_samples, max_age_ns
        self._samples = []
        self._times = []
        self._identity = None
        self._retired = deque(maxlen=32)
        self._expected_epochs = None

    def __len__(self):
        return len(self._samples)

    @property
    def samples(self):
        return tuple(self._samples)

    @property
    def latest(self):
        return self._samples[-1] if self._samples else None

    def reset(self, *, time_epoch: str, coordinate_epoch: str):
        if not time_epoch or not coordinate_epoch:
            raise ValueError('reset requires explicit epochs')
        if self._identity is not None:
            self._retired.append(self._identity)
        self._identity = None
        self._samples.clear()
        self._times.clear()
        self._expected_epochs = (time_epoch, coordinate_epoch)

    def append(self, sample: PoseSample) -> InsertResult:
        if not isinstance(sample, PoseSample):
            raise TypeError('sample must be PoseSample')
        if self._expected_epochs is not None and sample.identity[-2:] != self._expected_epochs:
            return InsertResult(False, 'epoch_mismatch')
        identity = sample.identity
        if identity in self._retired:
            return InsertResult(False, 'retired_identity')
        changed = self._identity is not None and self._identity != identity
        if changed:
            self._retired.append(self._identity)
            self._samples.clear()
            self._times.clear()
        self._identity = identity
        t = sample.source_realtime_ns
        if self._times and t < self._times[-1] - self.max_age_ns:
            return InsertResult(False, 'too_old')
        index = bisect_left(self._times, t)
        if index < len(self._times) and self._times[index] == t:
            return InsertResult(False, 'duplicate')
        self._times.insert(index, t)
        self._samples.insert(index, sample)
        cut = max(bisect_left(self._times, self._times[-1]-self.max_age_ns), len(self._times)-self.max_samples)
        if cut > 0:
            del self._times[:cut]
            del self._samples[:cut]
        return InsertResult(True, 'identity_changed' if changed else '', changed)

    def select(self, timestamp_ns: int, *, max_gap_ns: int = 50_000_000) -> PoseSelection:
        # Keep the public parameter's name while sharing strict timestamp rules.
        from .types import timestamp_ns as validate_ns
        validate_ns(timestamp_ns)
        validate_ns(max_gap_ns, 'max_gap_ns')
        if not self._times:
            return PoseSelection(None, 'unavailable', 'empty')
        index = bisect_left(self._times, timestamp_ns)
        if index < len(self._times) and self._times[index] == timestamp_ns:
            sample = self._samples[index]
            if not sample.pose_valid:
                return PoseSelection(None, 'unavailable', 'invalid_pose', timestamp_ns, timestamp_ns, 0)
            return PoseSelection(sample.pose, 'exact', '', timestamp_ns, timestamp_ns, 0)
        if index == 0:
            return PoseSelection(None, 'unavailable', 'no_before', after_ns=self._times[0])
        if index == len(self._times):
            return PoseSelection(None, 'unavailable', 'no_after', before_ns=self._times[-1])
        left, right = self._samples[index-1:index+1]
        before, after = left.source_realtime_ns, right.source_realtime_ns
        gap = after-before
        if not left.pose_valid or not right.pose_valid:
            return PoseSelection(None, 'unavailable', 'invalid_pose', before, after, gap)
        if gap > max_gap_ns:
            return PoseSelection(None, 'unavailable', 'gap_exceeded', before, after, gap)
        pose = left.pose.interpolate(right.pose, (timestamp_ns-before)/gap)
        return PoseSelection(pose, 'interpolated', '', before, after, gap)
