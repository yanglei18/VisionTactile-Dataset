"""Causal host-arrival continuity decisions shared by live fusion and CSV replay.

Without a device sequence number an interval alone does not prove lost samples.
A node-specific sampling deficit is different from a delay affecting the whole
stream: quarantine the returned orientation while buffered samples can catch up.
Continue integrating every received row exactly once at the nominal period.
"""
from bisect import bisect_left, bisect_right
from collections import deque
from dataclasses import dataclass
import math


REFERENCE_LOSS_EVENTS = frozenset({"gap_filter_reinitialized", "peer_sample_deficit_filter_reinitialized"})


@dataclass
class _Backlog:
    returned_at: float
    peer_samples: int
    received: int = 0


class ArrivalGapMonitor:
    def __init__(self, settings):
        self.rate = settings["sample_rate_hz"]
        self.stale = settings["stale_after_s"]
        self.reset_gap = settings["filter_reset_gap_s"]
        # Bounded positive evidence; if an atypical rate exceeds the recorded
        # history we cannot prove that a peer spanned the gap.
        self._history = {str(n): deque(maxlen=512) for n in range(1, 13)}
        self._backlogs = {}
        self.diagnostics = {}

    def clear_reference(self):
        self._backlogs.clear()
        for history in self._history.values():
            history.clear()

    def is_pending(self, key):
        return key in self._backlogs

    def _peer_samples(self, key, start, end):
        # Exclude both edge reads: one large USB transfer at either timestamp
        # is not evidence that another node kept arriving during the silence.
        best = 0
        half_interval = (end - start) / 2
        for peer, history in self._history.items():
            if peer == key or not history:
                continue
            first = bisect_right(history, start)
            stop = bisect_left(history, end)
            if stop - first < 3 or history[stop - 1] - history[first] < half_interval:
                continue
            # Require an interior read as well as both ends. Two isolated,
            # coalesced peer transfers can otherwise look like a steady stream.
            interior = bisect_right(history, history[first], first, stop)
            if history[interior] < history[stop - 1]:
                best = max(best, stop - first)
        return best if best >= self.rate * half_interval else 0

    def observe(self, key, stamp, *, paused=False):
        """Return a one-shot sample-arrival event. Members of REFERENCE_LOSS_EVENTS
        require a fresh filter and epoch. No event is produced by a UI snapshot.
        """
        history = self._history[key]
        previous = history[-1] if history else None
        history.append(stamp)
        if paused:
            self._backlogs.pop(key, None)
        gap = 0. if previous is None else stamp - previous
        if gap > self.stale:
            item = self.diagnostics.setdefault(key, dict(short_gaps=0, filter_resets=0))
            item.update(last_gap_s=gap, last_gap_timestamp_s=stamp)
            if gap > self.reset_gap and not paused:
                self._backlogs.pop(key, None)
                item["filter_resets"] += 1
                item["last_action"] = "filter_reset"
                return "gap_filter_reinitialized"
            item["short_gaps"] += 1
            item["last_action"] = "filter_preserved"
            if paused:
                return "gap_input_health_paused"
            if key not in self._backlogs:
                n = self._peer_samples(key, previous, stamp)
                if not n:
                    return "short_gap_filter_preserved"
                item["last_action"] = "awaiting_backlog"
                self._backlogs[key] = _Backlog(stamp, min(n, math.ceil(gap * self.rate)))

        backlog = self._backlogs.get(key)
        if backlog is None:
            return None
        backlog.received += 1
        elapsed = stamp - backlog.returned_at
        # New arrivals also have to account for time since the first return:
        # ordinary, on-time samples after a real loss cannot pay off old debt.
        # One row of endpoint skew is normal for independently arriving nodes.
        required = backlog.peer_samples + math.floor(elapsed * self.rate) - 1
        if backlog.received >= required:
            del self._backlogs[key]
            self.diagnostics[key]["last_action"] = "backlog_recovered"
            return "short_gap_backlog_recovered"
        if elapsed >= self.stale:
            del self._backlogs[key]
            item = self.diagnostics[key]
            item["filter_resets"] += 1
            item["short_gaps"] -= 1
            item["last_action"] = "peer_sample_deficit"
            return "peer_sample_deficit_filter_reinitialized"
        if backlog.received == 1:
            return "short_gap_awaiting_backlog"
        return None
