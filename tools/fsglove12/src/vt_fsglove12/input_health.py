"""Causal receive-health interlock, separate from the transient input guard.

An absent policy preserves historical replay behavior. With the policy enabled,
two seconds of aggregate per-node arrivals establish a rate baseline; a gross
rate mismatch must then persist for one second before integration is stopped.
Equal host timestamps are counted, not interpreted as zero device periods.

A frozen *six-node bank* needs at least twelve identical complete payloads over
one second, six changes in the other bank, and >=5 dps on at least two frozen
nodes. Exact quiet readings alone therefore are not called frozen. This is
software evidence of an unusable stream, not a hardware/saturation diagnosis.

Faults latch until one second of healthy cadence and twelve real payload
changes per node. Owners invalidate their filter epoch once on fault, pause
integration, then create a fresh filter on recovery. Calibration reset must not
reset this monitor. Snapshots are detached and never advance health state.
"""
from collections import deque
from dataclasses import dataclass, field
import math
from numbers import Integral, Real


DEFAULTS = dict(enabled=True, rate_window_s=2.0, rate_min_ratio=0.5,
                rate_max_ratio=1.5, rate_confirm_s=1.0, frozen_bank_s=1.0,
                frozen_min_samples=12, frozen_min_gyro_dps=5.0,
                peer_min_changes=6, recovery_s=1.0, recovery_min_changes=12)
_BANKS = (tuple(str(n) for n in range(1, 7)), tuple(str(n) for n in range(7, 13)))
_MAX_RATE_GROUPS = 8192


def health_settings(config):
    options = config.get("input_health", {})
    if not isinstance(options, dict) or set(options) - set(DEFAULTS):
        raise ValueError("input_health requires supported option fields")
    settings = {**DEFAULTS, **options}
    if "input_health" not in config:
        settings["enabled"] = False
    if type(settings["enabled"]) is not bool:
        raise ValueError("input_health enabled must be boolean")
    integers = {"frozen_min_samples", "peer_min_changes", "recovery_min_changes"}
    for key, value in settings.items():
        if key == "enabled":
            continue
        if (isinstance(value, bool) or not isinstance(value, Real)
                or not math.isfinite(value) or value <= 0):
            raise ValueError(f"input_health {key} must be finite and positive")
        if key in integers:
            if not isinstance(value, Integral) or value < 2:
                raise ValueError(f"input_health {key} must be an integer >= 2")
            settings[key] = int(value)
        else:
            settings[key] = float(value)
    if not settings["rate_min_ratio"] < 1. < settings["rate_max_ratio"]:
        raise ValueError("input_health rate ratios must straddle 1")
    return settings


@dataclass
class _Node:
    groups: deque = field(default_factory=deque)
    window_count: int = 0
    samples: int = 0
    first_stamp: float | None = None
    last_stamp: float | None = None
    payload: tuple | None = None
    changes: int = 0
    last_change: float | None = None
    gyro_dps: float = 0.
    rate: float | None = None
    rate_ready: bool = False
    bad_since: float | None = None
    faulted: bool = False
    reasons: tuple = ()
    recovery_start: float | None = None
    recovery_changes: int = 0
    faults: int = 0


@dataclass
class _Bank:
    seen: set = field(default_factory=set)
    signature: tuple | None = None
    start: float | None = None
    repeats: int = 0
    changes: int = 0
    peer_baseline: int = 0
    active: bool = False
    last_stamp: float | None = None


class InputHealthMonitor:
    def __init__(self, config):
        self.settings = health_settings(config)
        self.nominal = float(config.get("sample_rate_hz", 120.))
        if not math.isfinite(self.nominal) or self.nominal <= 0:
            raise ValueError("input_health sample_rate_hz must be positive")
        self.stale_after = float(config.get("stale_after_s", .15))
        self.nodes = {node: _Node() for bank in _BANKS for node in bank}
        self.banks = [_Bank(), _Bank()]
        self.clock_resets = 0

    def reset_clock(self):
        """Forget invalid time-domain evidence, not already confirmed faults."""
        self.clock_resets += 1
        for key, old in list(self.nodes.items()):
            self.nodes[key] = _Node(samples=old.samples, changes=old.changes,
                faulted=old.faulted, reasons=old.reasons, faults=old.faults)
        self.banks = [_Bank(), _Bank()]

    def _rate(self, node, stamp):
        if node.first_stamp is None:
            node.first_stamp = stamp
        groups = node.groups
        if groups and groups[-1][0] == stamp:
            groups[-1][1] += 1
        else:
            groups.append([stamp, 1])
        node.window_count += 1
        cutoff = stamp - self.settings["rate_window_s"]
        while len(groups) > 1 and (groups[1][0] <= cutoff or len(groups) > _MAX_RATE_GROUPS):
            node.window_count -= groups.popleft()[1]
        span = stamp - groups[0][0]
        node.rate = ((node.window_count - groups[0][1]) / span if span > 0 else None)
        node.rate_ready = (stamp - node.first_stamp >= self.settings["rate_window_s"] - 1e-9
                           and (span >= min(.25, self.settings["rate_window_s"])
                                or len(groups) >= _MAX_RATE_GROUPS and span > 0))
        good = (node.rate_ready and node.rate is not None
                and self.settings["rate_min_ratio"] <= node.rate / self.nominal
                <= self.settings["rate_max_ratio"])
        if node.rate_ready and not good:
            if node.bad_since is None:
                node.bad_since = stamp
        else:
            node.bad_since = None
        return good

    def _bank(self, key, stamp):
        number = 0 if int(key) <= 6 else 1
        bank, peer = self.banks[number], self.banks[1 - number]
        bank.seen.add(key)
        if len(bank.seen) != 6:
            return ()
        bank.seen.clear()
        bank.last_stamp = stamp
        signature = tuple(self.nodes[node].payload for node in _BANKS[number])
        if signature != bank.signature:
            bank.signature, bank.start, bank.repeats = signature, stamp, 1
            bank.changes += 1
            bank.peer_baseline, bank.active = peer.changes, False
        else:
            bank.repeats += 1
            moving = sum(self.nodes[node].gyro_dps >= self.settings["frozen_min_gyro_dps"]
                         for node in _BANKS[number]) >= 2
            if (stamp - bank.start >= self.settings["frozen_bank_s"] - 1e-9
                    and bank.repeats >= self.settings["frozen_min_samples"]
                    and peer.changes - bank.peer_baseline >= self.settings["peer_min_changes"]
                    and moving):
                bank.active = True
        return _BANKS[number]

    def _evaluate(self, key, stamp, events, *, allow_recovery):
        node = self.nodes[key]
        bank = self.banks[0 if int(key) <= 6 else 1]
        reasons = []
        if (node.bad_since is not None
                and stamp - node.bad_since >= self.settings["rate_confirm_s"] - 1e-9):
            reasons.append("receive_rate_mismatch")
        if bank.active:
            reasons.append("frozen_bank_payload")
        if reasons:
            if not node.faulted:
                node.faulted = True
                node.faults += 1
                events.append(("fault", key))
            node.reasons = tuple(sorted(set(node.reasons).union(reasons)))
            node.recovery_start = None
        elif node.faulted and allow_recovery:
            cadence = (node.rate_ready and node.rate is not None
                       and self.settings["rate_min_ratio"] <= node.rate / self.nominal
                       <= self.settings["rate_max_ratio"])
            if not cadence or node.last_stamp is None or stamp - node.last_stamp > self.stale_after:
                node.recovery_start = None
            elif node.recovery_start is None:
                node.recovery_start, node.recovery_changes = stamp, node.changes
            elif (stamp - node.recovery_start >= self.settings["recovery_s"] - 1e-9
                  and node.changes - node.recovery_changes >= self.settings["recovery_min_changes"]
                  and node.last_change is not None and stamp - node.last_change <= self.stale_after):
                node.faulted, node.reasons, node.recovery_start = False, (), None
                events.append(("recovered", key))

    def observe(self, node_id, values, status, stamp, gyro_dps):
        """Return one-shot (fault|recovered, node) transitions for the owner."""
        if not self.settings["enabled"]:
            return ()
        key = str(node_id)
        node = self.nodes[key]
        payload = (*values, status)
        node.samples += 1
        if node.payload != payload:
            node.changes += 1
            node.last_change = stamp
        node.payload, node.last_stamp = payload, stamp
        node.gyro_dps = max(abs(float(value)) for value in gyro_dps)
        self._rate(node, stamp)
        peers = self._bank(key, stamp)
        events = []
        for affected in dict.fromkeys((key, *peers)):
            self._evaluate(affected, stamp, events, allow_recovery=affected == key)
        return tuple(events)

    def integration_allowed(self, node_id):
        return not self.settings["enabled"] or not self.nodes[str(node_id)].faulted

    def node_status(self, node_id, now):
        node = self.nodes[str(node_id)]
        number = 0 if int(node_id) <= 6 else 1
        bank, peer = self.banks[number], self.banks[1 - number]
        age = None if node.last_stamp is None else now - node.last_stamp
        cadence = (node.rate_ready and node.rate is not None
                   and self.settings["rate_min_ratio"] <= node.rate / self.nominal
                   <= self.settings["rate_max_ratio"])
        state = ("recovering" if node.recovery_start is not None else "faulted") if node.faulted else (
            "healthy" if cadence else "warming_up")
        if not node.faulted and age is not None and (age < 0 or age > self.stale_after):
            state = "stale"
        if not self.settings["enabled"]:
            state = "disabled"
        return dict(enabled=self.settings["enabled"], state=state,
            healthy=state in ("healthy", "disabled"), faulted=node.faulted,
            reasons=list(node.reasons), observed_receive_rate_hz=node.rate,
            nominal_sample_rate_hz=self.nominal, samples=node.samples,
            payload_changes=node.changes, fault_count=node.faults, age_s=age,
            bank="1-6" if number == 0 else "7-12", bank_repeated_samples=bank.repeats,
            bank_repeated_duration_s=(None if bank.start is None or bank.last_stamp is None
                                      else max(0., bank.last_stamp - bank.start)),
            peer_bank_payload_changes=max(0, peer.changes - bank.peer_baseline),
            integration_allowed=self.integration_allowed(node_id))

    def snapshot(self, now):
        nodes = {key: self.node_status(key, now) for key in self.nodes}
        return dict(enabled=self.settings["enabled"], settings=dict(self.settings), nodes=nodes,
            faulted_nodes=[key for key, node in nodes.items() if node["faulted"]],
            recovering_nodes=[key for key, node in nodes.items() if node["state"] == "recovering"],
            clock_resets=self.clock_resets, rate_basis="aggregate_host_arrivals_not_device_odr",
            raw_values_modified=False, device_commands_sent=False,
            physical_saturation="unknown")
