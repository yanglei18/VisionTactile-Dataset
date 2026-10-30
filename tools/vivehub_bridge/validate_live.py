"""Read-only bounded ROS verification. Saves statistics, not raw sensor data."""
import argparse
import json
import math
from pathlib import Path
import statistics
import time

import rclpy
from rclpy.qos import QoSProfile, ReliabilityPolicy, DurabilityPolicy
from std_msgs.msg import String
from vt_tracker_msgs.msg import TrackerSample, TrackerStatus
from vt_vive_tracker.hub_source import CLOCK_POLICY, FRAME_ID, load_bindings


def main():
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--bindings", required=True)
    parser.add_argument("--seconds", type=float, default=30.)
    parser.add_argument("--output", required=True)
    args = parser.parse_args()
    _, roles = load_bindings(args.bindings)
    rclpy.init()
    node = rclpy.create_node("vt_hub_readonly_validation")
    qos = QoSProfile(depth=100, reliability=ReliabilityPolicy.BEST_EFFORT)
    latched = QoSProfile(depth=1, reliability=ReliabilityPolicy.RELIABLE,
                         durability=DurabilityPolicy.TRANSIENT_LOCAL)
    def empty():
        return dict(messages=0, valid=0, errors=0, nonmonotonic=0,
                    realtime_nonmonotonic=0, clock_interval_errors=0,
                    last=0, last_real=0, offset_min_ns=None, offset_max_ns=None,
                    ages_ms=[], states={})

    values = {r: empty() for r in roles.by_role}
    statuses, source = {}, {}

    def sample(role, msg):
        v = values[role]
        v["messages"] += 1
        v["valid"] += int(msg.pose_valid)
        v["states"][str(msg.tracking_status)] = v["states"].get(str(msg.tracking_status), 0) + 1
        q = msg.pose.orientation
        norm = math.sqrt(q.x*q.x + q.y*q.y + q.z*q.z + q.w*q.w)
        stamp = msg.header.stamp.sec * 10**9 + msg.header.stamp.nanosec
        if (msg.role != role or msg.tracker_id != roles.by_role[role] or msg.header.frame_id != FRAME_ID
                or stamp != msg.host_realtime_ns or abs(norm-1.) > 1e-5):
            v["errors"] += 1
        if v["last"] and msg.host_monotonic_ns <= v["last"]:
            v["nonmonotonic"] += 1
        if v["last_real"] and msg.host_realtime_ns <= v["last_real"]:
            v["realtime_nonmonotonic"] += 1
        if v["last"] and (msg.host_realtime_ns - v["last_real"] != msg.host_monotonic_ns - v["last"]):
            v["clock_interval_errors"] += 1
        offset = msg.host_realtime_ns - msg.host_monotonic_ns
        v["offset_min_ns"] = offset if v["offset_min_ns"] is None else min(v["offset_min_ns"], offset)
        v["offset_max_ns"] = offset if v["offset_max_ns"] is None else max(v["offset_max_ns"], offset)
        v["last"] = msg.host_monotonic_ns
        v["last_real"] = msg.host_realtime_ns
        v["ages_ms"].append((time.time_ns()-stamp)/1e6)

    for role in roles.by_role:
        node.create_subscription(TrackerSample, f"/vive/{role}/sample", lambda m, r=role: sample(r, m), qos)
        node.create_subscription(TrackerStatus, f"/vive/{role}/status",
            lambda m, r=role: statuses.update({r: dict(state=m.state, invalid=m.invalid_report_count,
                                                     queue_drops=m.dropped_queue_count)}), latched)
    node.create_subscription(String, "/vive/source_info", lambda m: source.update(json.loads(m.data)), latched)
    warm = time.monotonic() + 2
    while time.monotonic() < warm:
        rclpy.spin_once(node, timeout_sec=.05)
    for v in values.values():
        v.update(empty())
    start = time.monotonic()
    while time.monotonic()-start < args.seconds:
        rclpy.spin_once(node, timeout_sec=.05)
    elapsed = time.monotonic()-start
    for role, v in values.items():
        ages = sorted(v.pop("ages_ms"))
        v.pop("last")
        v.pop("last_real")
        v["receive_hz"] = v["messages"]/elapsed
        v["valid_hz"] = v["valid"]/elapsed
        v["source_age_median_ms"] = statistics.median(ages) if ages else None
        v["source_age_p95_ms"] = ages[min(len(ages)-1, int(len(ages)*.95))] if ages else None
        v["publishers"] = node.count_publishers(f"/vive/{role}/sample")
        v["status"] = statuses.get(role)
    report = dict(elapsed_s=elapsed, frame_id=FRAME_ID, roles=values, source_info=source)
    offsets = {v[k] for v in values.values() for k in ("offset_min_ns", "offset_max_ns")}
    report["shared_constant_clock_offset"] = len(offsets) == 1 and None not in offsets
    report["pass"] = (source.get("clock_policy") == CLOCK_POLICY
        and report["shared_constant_clock_offset"]
        and all(v["valid"] >= 10 and v["errors"] == v["nonmonotonic"]
                == v["realtime_nonmonotonic"] == v["clock_interval_errors"] == 0
                and v["publishers"] == 1 and (v["status"] or {}).get("queue_drops") == 0
                for v in values.values()))
    Path(args.output).write_text(json.dumps(report, indent=2, ensure_ascii=False)+"\n")
    print(json.dumps(report, indent=2, ensure_ascii=False))
    node.destroy_node()
    rclpy.shutdown()
    return 0 if report["pass"] else 1


if __name__ == "__main__":
    raise SystemExit(main())
