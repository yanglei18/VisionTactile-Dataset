"""Bounded, local-only evidence recording for a guided 31-second thumb trial.

The caller owns acquisition, time and threading. This recorder never resets a
filter, display reference or bias. Finish runs after the caller stops feeding;
it can be dispatched to a background worker without owning the serial port.
"""
from __future__ import annotations

from copy import deepcopy
from datetime import datetime, timezone
import hashlib
import json
import math
from pathlib import Path
import uuid

from .config import validate_config, write_json_exclusive
from .records import Sample
from .stream import RawRecorder


MAX_RAW_SAMPLES = 100000
MAX_PAIR_SNAPSHOTS = 10000
TOTAL_SECONDS = 31.
PHASES = (
    dict(phase="prepare", start_s=0., end_s=3., label="准备开始",
         instruction="摆好舒适的起始姿态。只记录现有状态，不会重新归零或修改偏置。"),
    dict(phase="initial_hold", start_s=3., end_s=8., label="保持起始姿态",
         instruction="手掌和拇指保持当前姿态 5 秒，记住这个姿势，之后回到这里。"),
    dict(phase="motion", start_s=8., end_s=20., label="做 3 次慢弯伸",
         instruction="拇指慢慢弯曲、再伸直，共 3 次；近节可以自然活动，不必刻意锁死。"),
    dict(phase="return", start_s=20., end_s=23., label="回到起始姿态",
         instruction="将手掌朝向和拇指形状恢复到刚才起始姿态，不点击显示归零。"),
    dict(phase="final_hold", start_s=23., end_s=31., label="保持回位姿态",
         instruction="保持回位姿态 8 秒。程序自动记录回位差异和静止变化，不需要手抄数字。"),
)
_SNAPSHOT_KEYS = ("host_monotonic_s", "pair_count", "node_ids", "paired_fresh", "reference_set",
                  "reference_ready", "reference_source", "reference_host_s", "valid", "reasons")
_NODE_KEYS = ("sensor_quat_wxyz", "relative_quat_wxyz", "gyro_dps", "accel_g", "fresh", "age_s",
              "rotation_magnitude_deg", "gravity_direction_change_deg", "accel_norm_g", "gyro_dps_per_lsb")


def _time(value):
    if isinstance(value, bool) or not isinstance(value, (int, float)) or not math.isfinite(value):
        raise ValueError("trial time must be a finite host monotonic observation")
    return float(value)


def _file_info(path):
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(65536), b""):
            digest.update(block)
    return dict(bytes=path.stat().st_size, sha256=digest.hexdigest())


class ThumbTrial:
    """One explicit experiment; no shared files or automatic hardware actions.

    ``feed_snapshot`` expects actual completed-pair processor outputs, not GUI
    redraws. Equal host timestamps are retained separately. Once a bound or I/O
    error occurs, due() is true and status is aborted; all evidence remains for
    finish(). No failed attempt is deleted or silently retried.
    """

    def __init__(self, config, output_root, start_s, reference_snapshot, *, angle_candidate=None):
        self.config = validate_config(config)
        self.start_s = _time(start_s)
        self.reference_snapshot = json.loads(json.dumps(reference_snapshot, allow_nan=False))
        self.angle_candidate_profile = (None if angle_candidate is None else
                                        json.loads(json.dumps(angle_candidate, allow_nan=False)))
        ids = (reference_snapshot or {}).get("node_ids")
        if ids is None:
            finger = self.config["fingers"]["thumb"]
            ids = [finger["proximal"], finger["distal"]]
        if (not isinstance(ids, (list, tuple)) or len(ids) != 2 or ids[0] == ids[1]
                or any(type(node) is not int or not 1 <= node <= 12 for node in ids)):
            raise ValueError("trial needs two distinct sensor node IDs")
        self.node_ids = list(ids)
        self.raw_sample_count = self.pair_snapshot_count = 0
        self._raw_by_node = {}
        self._rows = []
        self._stop_reason = None
        self._errors = []
        self._result = None
        self._closed = False
        self._raw = self._online = None
        root = Path(output_root)
        root.mkdir(parents=True, exist_ok=True)
        now = datetime.now(timezone.utc)
        self.directory = root/f"thumb_trial_{now.strftime('%Y%m%dT%H%M%S%fZ')}_{uuid.uuid4().hex[:8]}"
        self.directory.mkdir(exist_ok=False)
        metadata = dict(kind="vt_glove_pose.thumb_trial", schema_version=1,
            created_utc=now.isoformat(), start_s=self.start_s, planned_duration_s=TOTAL_SECONDS,
            time_basis="host_monotonic_observations_not_device_sampling_time",
            config=deepcopy(self.config), node_ids=list(self.node_ids),
            reference_snapshot=deepcopy(self.reference_snapshot), planned_phases=deepcopy(PHASES),
            raw_scope="all_received_nodes_without_synthesizing_missing_samples",
            online_scope="actual_completed_pair_outputs_not_display_rate_snapshots",
            limits=dict(raw_samples=MAX_RAW_SAMPLES, pair_snapshots=MAX_PAIR_SNAPSHOTS),
            modifies_reference=False, modifies_gyro_bias=False, anatomical_joint_angle=False,
            independent_angle_truth_available=False, production_acceptance="not_evaluated")
        if self.angle_candidate_profile is not None:
            metadata.update(angle_candidate_profile=deepcopy(self.angle_candidate_profile),
                angle_candidate_online_scope="actual_observer_output_from_same_raw_pair_not_replay_or_ui_poll",
                angle_candidate_pairing="same_process_call_completion_and_matching_host_stamp_and_node_ids",
                angle_candidate_missing_policy="explicit_null_when_no_output_or_pair_mismatch_never_fill_from_history",
                angle_candidate_accuracy_verified=False)
        try:
            write_json_exclusive(self.directory/"metadata.json", metadata)
            self._raw = RawRecorder(self.directory/"raw.csv").open()
            self._online = (self.directory/"online.jsonl").open("x", encoding="utf-8")
        except Exception:
            self._close_files()
            raise

    def _abort(self, reason, error=None):
        if self._stop_reason is None:
            self._stop_reason = str(reason)
        if error is not None:
            self._errors.append(str(error))

    def feed_raw(self, samples):
        """Write only received validated integer samples, including nodes 1..12."""
        if self._closed or self._stop_reason is not None:
            return False
        pending = []
        for sample in samples:
            if not isinstance(sample, Sample):
                self._abort("invalid_raw_sample", "raw input must contain Sample instances")
                break
            if self.raw_sample_count+len(pending) >= MAX_RAW_SAMPLES:
                self._abort("raw_sample_limit")
                break
            pending.append(sample)
        try:
            if pending:
                self._raw.write(pending)
                self.raw_sample_count += len(pending)
                for sample in pending:
                    key = str(sample.node_id)
                    self._raw_by_node[key] = self._raw_by_node.get(key, 0)+1
        except Exception as exc:
            self._abort("raw_write_error", exc)
        return self._stop_reason is None

    def feed_snapshot(self, snapshot, *, angle_snapshot=None):
        """Keep each real completed-pair output; never deduplicate timestamps."""
        if snapshot is None:
            return False
        if self._closed or self._stop_reason is not None:
            return False
        if self.pair_snapshot_count >= MAX_PAIR_SNAPSHOTS:
            self._abort("pair_snapshot_limit")
            return False
        try:
            stamp = _time(snapshot["host_monotonic_s"])
            row = {key: snapshot[key] for key in _SNAPSHOT_KEYS if key in snapshot}
            row.update(kind="vt_glove_pose.thumb_trial_snapshot", schema_version=1,
                       host_monotonic_s=stamp, trial_elapsed_s=stamp-self.start_s)
            row["nodes"] = {str(node): {key: value for key, value in values.items() if key in _NODE_KEYS}
                            for node, values in snapshot["nodes"].items()}
            if self.angle_candidate_profile is not None:
                # The caller supplies only the return of the candidate process
                # call for this exact raw sample. Timestamps are consistency
                # checks, never lookup keys: several pairs can share one stamp.
                row["angle_candidate"] = None
                row["angle_candidate_record_status"] = "no_output_for_this_pair"
                if angle_snapshot is not None:
                    if (isinstance(angle_snapshot, dict)
                            and angle_snapshot.get("host_monotonic_s") == stamp
                            and angle_snapshot.get("node_ids") == row.get("node_ids", self.node_ids)):
                        row["angle_candidate"] = angle_snapshot
                        row["angle_candidate_record_status"] = "actual_online_pair_output"
                    else:
                        row["angle_candidate_record_status"] = "timestamp_or_node_ids_mismatch"
            encoded = json.dumps(row, ensure_ascii=False, allow_nan=False, separators=(",", ":"))
            self._online.write(encoded+"\n")
            # Line buffering is explicit so an interrupted experiment retains
            # the latest pair rather than only a Python in-memory summary.
            self._online.flush()
            self._rows.append(json.loads(encoded))
            self.pair_snapshot_count += 1
        except Exception as exc:
            self._abort("online_write_or_snapshot_error", exc)
        return self._stop_reason is None

    def due(self, now_s):
        return self._result is not None or self._stop_reason is not None or _time(now_s) >= self.start_s+TOTAL_SECONDS

    def status(self, now_s):
        elapsed = max(0., _time(now_s)-self.start_s)
        base = dict(directory=str(self.directory), raw_samples=self.raw_sample_count,
                    pair_snapshots=self.pair_snapshot_count, total_remaining_s=max(0., TOTAL_SECONDS-elapsed))
        if self._result is not None:
            completed = self._result["status"] == "completed"
            return dict(base, active=False, phase=self._result["status"], remaining_s=0.,
                        label="试跑已保存" if completed else "试跑已中止并保留记录",
                        instruction="数据和分析结果已保存，不需要手抄数字。", reason=self._result["reason"])
        if self._stop_reason is not None:
            return dict(base, active=False, phase="aborted", remaining_s=0., label="记录已停止，正在保留数据",
                        instruction="已收到的原始数据会保留，不会自动重新采集。", reason=self._stop_reason)
        if elapsed >= TOTAL_SECONDS:
            return dict(base, active=False, phase="finishing", remaining_s=0., label="采集结束，正在保存分析",
                        instruction="动作完成，无需继续重复。")
        stage = next(stage for stage in PHASES if elapsed < stage["end_s"])
        label = stage["label"]
        if stage["phase"] == "motion":
            label = f"第 {min(3, int((elapsed-8.)//4.)+1)} / 3 次慢弯伸"
        return dict(base, active=True, phase=stage["phase"], label=label,
                    instruction=stage["instruction"], remaining_s=stage["end_s"]-elapsed)

    def _close_files(self):
        if self._closed:
            return
        self._closed = True
        for stream in (self._raw, self._online):
            if stream is not None:
                try:
                    stream.close()
                except Exception as exc:
                    self._abort("file_close_error", exc)

    def finish(self, now_s, reason="completed"):
        """Close and analyze saved evidence; completed means capture, not accuracy."""
        if self._result is not None:
            return deepcopy(self._result)
        end_s = _time(now_s)
        if not isinstance(reason, str) or not reason:
            raise ValueError("finish reason must be a nonempty string")
        self._close_files()
        reason = self._stop_reason or reason
        if reason == "completed" and end_s < self.start_s+TOTAL_SECONDS:
            reason = "ended_before_plan"
        if reason == "completed" and (not self.raw_sample_count or not self.pair_snapshot_count):
            reason = "no_raw_data" if not self.raw_sample_count else "no_pair_data"
        status = "completed" if reason == "completed" else "aborted"
        capture_status = status
        report = None
        report_saved = False
        try:
            from .thumb_trial_analysis import analyze_trial
            report = analyze_trial(self._rows, self.start_s, self.node_ids, completion_reason=reason)
            write_json_exclusive(self.directory/"report.json", report)
            report_saved = True
        except Exception as exc:
            self._errors.append(f"analysis_or_report_error: {exc}")
        files = {}
        for name in ("metadata.json", "raw.csv", "online.jsonl", "report.json"):
            path = self.directory/name
            if path.is_file():
                try:
                    files[name] = _file_info(path)
                except OSError as exc:
                    self._errors.append(f"hash_error {name}: {exc}")
        if self._errors and status == "completed":
            status, reason = "aborted", "save_or_analysis_error"
        manifest = dict(kind="vt_glove_pose.thumb_trial_manifest", schema_version=1,
                        status=status, capture_status=capture_status, reason=reason, start_s=self.start_s, end_s=end_s,
                        planned_duration_s=TOTAL_SECONDS, elapsed_s=max(0., end_s-self.start_s),
                        raw_sample_count=self.raw_sample_count, raw_samples_by_node=dict(self._raw_by_node),
                        pair_snapshot_count=self.pair_snapshot_count, files=files, errors=list(self._errors),
                        export_complete=not self._errors, analysis_complete=report_saved,
                        product_acceptance="not_evaluated", source_data_preserved=True)
        manifest_saved = False
        try:
            write_json_exclusive(self.directory/"manifest.json", manifest)
            manifest_saved = True
        except Exception as exc:
            self._errors.append(f"manifest_write_error: {exc}")
            if status == "completed":
                status, reason = "aborted", "manifest_write_error"
        self._result = dict(directory=str(self.directory), status=status, reason=reason,
                            report=report, report_path=str(self.directory/"report.json") if report_saved else None,
                            manifest_path=str(self.directory/"manifest.json") if manifest_saved else None,
                            export_complete=manifest_saved and not self._errors,
                            raw_sample_count=self.raw_sample_count, pair_snapshot_count=self.pair_snapshot_count,
                            errors=list(self._errors))
        return deepcopy(self._result)
