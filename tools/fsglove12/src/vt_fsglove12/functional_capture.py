"""Bounded selection of an actual stable functional-reference window.

The caller supplies unadapted official local rotations and source identities.
This class never integrates sensors, manufactures samples or changes the
reference orientation. It keeps a moving two-second window until it is usable.
"""
from collections import deque
from copy import deepcopy
import math

import numpy as np
from scipy.spatial.transform import Rotation

from .functional_quality import FUNCTIONAL_REFERENCE_P95_MAX_DEG


REFERENCE_WAIT_SECONDS = 12.


class FunctionalReferenceCapture:
    def __init__(self, slots, start_s, *, duration_s=2., wait_s=REFERENCE_WAIT_SECONDS,
                 sample_rate_hz=60., max_gap_s=.25):
        self.slots = tuple(str(slot) for slot in slots)
        self.start_s, self.duration_s = float(start_s), float(duration_s)
        self.deadline_s = self.start_s + wait_s
        self.period_s, self.max_gap_s = 1. / sample_rate_hz, max_gap_s
        self._rows = deque(maxlen=math.ceil(duration_s * sample_rate_hz) + 3)
        self._seen = {}
        self._last_sample_s = None
        self._last_assessment_s = None
        self.selected = None
        self.status = "waiting_for_samples"
        self.diagnostics = {}
        self.stable_slots = []

    def needs_sample(self, now):
        return now >= self.start_s and (self._last_sample_s is None
            or now - self._last_sample_s >= self.period_s - 1e-9)

    def observe(self, frame, local_rotations, now):
        """Return pending/accepted/partial/failed; selected frames stay original."""
        if self.selected is not None or self.status == "failed":
            return self.status
        if now < self.start_s:
            return "pending"
        sampled = False
        if self.needs_sample(now):
            values = {}
            for slot in self.slots:
                item = local_rotations.get(slot)
                if item is None:
                    continue
                previous = self._seen.get(slot)
                current = item["identity"]
                if previous is not None:
                    if any(a[0] != b[0] or a[1] < b[1] for a, b in zip(current, previous)):
                        self._rows.clear()
                        self._seen.clear()
                        self.stable_slots.clear()
                        self._last_assessment_s = None
                        values.clear()
                    elif not all(a[1] > b[1] for a, b in zip(current, previous)):
                        continue
                self._seen[slot] = item["identity"]
                values[slot] = list(item["quat_xyzw"])
            if values:
                self._rows.append((float(now), deepcopy(frame), values))
                self._last_sample_s = float(now)
                sampled = True
        # Keep the preceding sample at the beginning of the requested span.
        # No selected sample predates the reported actual start time.
        while len(self._rows) > 1 and self._rows[1][0] <= now - self.duration_s:
            self._rows.popleft()
        # Sample acquisition remains bounded at 60 Hz; a 10 Hz assessment
        # does not run a window-sized SVD/percentile on every solver tick.
        if (now >= self.deadline_s or sampled and
                (self._last_assessment_s is None or now - self._last_assessment_s >= .1 - 1e-9)):
            self._assess(now)
            self._last_assessment_s = float(now)
        if len(self.stable_slots) == len(self.slots):
            self.status = "accepted"
        elif now >= self.deadline_s:
            self.status = "partial" if self.stable_slots else "failed"
        else:
            self.status = "waiting_for_stability"
        if self.status in {"accepted", "partial"}:
            self.selected = [row[1] for row in self._rows]
        return self.status

    def _assess(self, now):
        self.stable_slots, self.diagnostics = [], {}
        for slot in self.slots:
            samples = [(stamp, values[slot]) for stamp, _, values in self._rows if slot in values]
            stamps = [item[0] for item in samples]
            complete = (len(samples) >= 3 and stamps[-1] >= now - self.max_gap_s
                        and stamps[-1] - stamps[0] >= self.duration_s - 1e-9
                        and max(np.diff(stamps), default=0.) <= self.max_gap_s)
            item = dict(samples=len(samples), stable=False, reason="incomplete_window",
                        p95_deg=None)
            if complete:
                rotations = Rotation.from_quat([item[1] for item in samples])
                p95 = float(np.percentile(np.rad2deg((rotations.mean().inv() * rotations).magnitude()), 95))
                item.update(p95_deg=p95, stable=p95 <= FUNCTIONAL_REFERENCE_P95_MAX_DEG,
                            reason="stable" if p95 <= FUNCTIONAL_REFERENCE_P95_MAX_DEG else "moving")
            if item["stable"]:
                self.stable_slots.append(slot)
            self.diagnostics[slot] = item

    def summary(self, now):
        return dict(status=self.status, wait_elapsed_s=max(0., now - self.start_s),
                    wait_limit_s=self.deadline_s - self.start_s,
                    window_start_s=self._rows[0][0] if self._rows else None,
                    window_end_s=self._rows[-1][0] if self._rows else None,
                    stable_slots=list(self.stable_slots),
                    unstable_slots=[slot for slot in self.slots if slot not in self.stable_slots],
                    diagnostics=deepcopy(self.diagnostics),
                    criterion="official_local_rotation_p95",
                    threshold_deg=FUNCTIONAL_REFERENCE_P95_MAX_DEG)
