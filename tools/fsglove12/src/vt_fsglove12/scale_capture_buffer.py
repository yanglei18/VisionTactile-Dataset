"""Bounded, trial-long raw evidence for interactive input-scale calibration.

The reader owns this container under the existing session lock. Samples are
immutable RawSample references, not converted/filter outputs. Unlike a moving
time window, a slow user advancing through the three holds does not erase the
first completed hold or the rotations between holds. The hard sample budget is
still enforced; exceeding it invalidates the evidence explicitly, never moves
or replaces a requested hold. No device, worker, file or timer is created here.
"""
from __future__ import annotations

from collections import deque


DEFAULT_MAX_SAMPLES = 1_200_000


class ScaleHistoryEvictedError(ValueError):
    """The bounded trial no longer contains all evidence it originally saw."""


class ScaleCaptureBuffer:
    """Deque-compatible raw history with explicit, sticky evidence-loss status.

    At 120 Hz across 12 nodes the default budget holds about 13.9 minutes.
    ``freeze`` must be called under the session lock before starting a job.
    Call ``clear`` (or construct a new buffer) only for a new calibration trial;
    loss cannot be repaired by waiting for newer samples in the same trial.
    """

    def __init__(self, samples=(), *, maxlen=DEFAULT_MAX_SAMPLES):
        if type(maxlen) is not int or not 1 <= maxlen <= DEFAULT_MAX_SAMPLES:
            raise ValueError(f"maxlen must be an integer between 1 and {DEFAULT_MAX_SAMPLES}")
        self._samples = deque(maxlen=maxlen)
        self.evicted_samples = 0
        self.evicted_through_s = None
        self.extend(samples)

    @property
    def maxlen(self):
        return self._samples.maxlen

    @property
    def history_evicted(self):
        return self.evicted_samples > 0

    def __len__(self):
        return len(self._samples)

    def __iter__(self):
        return iter(self._samples)

    def __getitem__(self, index):
        return self._samples[index]

    def _record_eviction(self, sample):
        self.evicted_samples += 1
        self.evicted_through_s = sample.host_monotonic_s

    def append(self, sample):
        if len(self._samples) == self.maxlen:
            self._record_eviction(self._samples[0])
        self._samples.append(sample)

    def extend(self, samples):
        for sample in samples:
            self.append(sample)

    def popleft(self):
        # Compatibility must not let a legacy time-pruning caller silently
        # erase captured evidence: any removal marks this trial incomplete.
        sample = self._samples.popleft()
        self._record_eviction(sample)
        return sample

    def clear(self):
        self._samples.clear()
        self.evicted_samples = 0
        self.evicted_through_s = None

    def freeze(self):
        """Return a stable prefix, refusing an incompletely retained trial."""
        if self.history_evicted:
            raise ScaleHistoryEvictedError(
                "history_evicted: input-scale raw evidence exceeded the bounded capture "
                "budget; start a new three-hold trial, without substituting later samples")
        return tuple(self._samples)

    def snapshot(self):
        """Small diagnostic payload only; raw samples never belong in GUI IPC."""
        return dict(status="history_evicted" if self.history_evicted else "retained",
                    sample_count=len(self), max_samples=self.maxlen,
                    history_evicted=self.history_evicted,
                    evicted_samples=self.evicted_samples,
                    evicted_through_s=self.evicted_through_s)
