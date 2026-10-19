"""Read-only, asynchronous playback of the topics actually stored in a bag.

ROS stays in an owned system-Python decoder. Quality verdicts and spatial
calibration do not gate this browser; the acquisition backend is never used.
"""
from __future__ import annotations

import copy
import threading
import time

from .replay_transport import ReplayDecoder


MAX_VISUAL_TOPICS = 24
_KINDS = {"camera", "tracker", "glove", "tactile", "message"}
_VISUAL_KINDS = _KINDS - {"message"}


def validate_metadata(value):
    if not isinstance(value, dict):
        raise ValueError("Invalid playback inventory")
    start, end = value.get("start_ns"), value.get("end_ns")
    if any(type(v) is not int or not 0 <= v < 2**63 for v in (start, end)) or end < start:
        raise ValueError("Invalid playback time range")
    topics = value.get("topics")
    if not isinstance(topics, list) or not 1 <= len(topics) <= 4096:
        raise ValueError("Invalid or excessive playback topic count")
    names = set()
    for topic in topics:
        if not isinstance(topic, dict):
            raise ValueError("Invalid playback topic")
        name = topic.get("name")
        if not isinstance(name, str) or not 1 <= len(name) <= 1024 or name in names:
            raise ValueError("Invalid or duplicate playback topic name")
        names.add(name)
        if (not isinstance(topic.get("type"), str) or len(topic["type"]) > 1024
                or type(topic.get("count")) is not int or not 0 <= topic["count"] <= 10_000_000
                or topic.get("kind") not in _KINDS):
            raise ValueError("Invalid playback topic description")
    warnings = value.get("warnings", [])
    if not isinstance(warnings, list) or len(warnings) > 8192 or any(
            not isinstance(item, str) or len(item) > 4096 for item in warnings):
        raise ValueError("Invalid playback warnings")


class TopicBagSource:
    """Topic payload validation on the shared owned-decoder transport."""

    def __init__(self, path, cancelled=lambda: False):
        self._decoder = ReplayDecoder("topic_replay_worker", path, cancelled)
        try:
            self.metadata = self._decoder.metadata()
            validate_metadata(self.metadata)
        except BaseException:
            self.close()
            raise

    def set_cancelled(self, cancelled):
        self._decoder.set_cancelled(cancelled)

    def frame(self, topic, time_ns):
        frame = self._decoder.request(dict(command="frame", topic=topic, time_ns=time_ns)).get("frame")
        if not isinstance(frame, dict) or frame.get("topic") != topic or frame.get("time_ns") != time_ns:
            raise ValueError("Playback response does not match the selected topic and time")
        return frame

    def close(self):
        self._decoder.close()


class TopicReplayBackend:
    """A bag-time playhead shared by all topics, independent of live clocks."""
    mode = "replay"

    def __init__(self, *, loader=TopicBagSource, max_visual_topics=MAX_VISUAL_TOPICS):
        if type(max_visual_topics) is not int or not 1 <= max_visual_topics <= 64:
            raise ValueError("Visual playback topic limit must be between 1 and 64")
        self.loader = loader
        self.max_visual_topics = max_visual_topics
        self._condition = threading.Condition()
        self._stop = False
        self._source = None
        self._metadata = {}
        self._latest = None
        self._snapshot = None
        self._visual_topics = ()
        self._snapshot_warnings = []
        self._sequence = 0
        self._seek_revision = 0
        self._path = ""
        self._topic = ""
        self._time_ns = 0
        self._generation = 0
        self._revision = 0
        self._load = None
        self._request = False
        self._playing = False
        self._rate = 1.
        self._anchor_wall = time.monotonic()
        self._anchor_ns = 0
        self._deadline = 0.
        self._status = dict(state="waiting", message="请选择档案中的录制包")
        self._thread = threading.Thread(target=self._run, name="capture-topic-replay", daemon=True)
        self._thread.start()

    def load(self, path):
        with self._condition:
            if self._stop:
                return
            self._generation += 1
            self._revision += 1
            self._seek_revision += 1
            self._path = str(path)
            self._latest = None
            self._snapshot = None
            self._visual_topics = ()
            self._snapshot_warnings = []
            self._metadata = {}
            self._topic = ""
            self._time_ns = 0
            self._playing = False
            self._request = False
            self._load = (str(path), self._generation)
            self._status = dict(state="loading", message="正在读取录制包中的话题")
            self._condition.notify_all()

    def clear(self):
        with self._condition:
            self.load("")
            self._load = (None, self._generation)
            self._status = dict(state="waiting", message="请选择档案中的录制包")

    def _clock(self):
        return min(self._metadata.get("end_ns", self._time_ns),
                   self._anchor_ns + int((time.monotonic() - self._anchor_wall) * 1e9 * self._rate))

    def _reset_anchor(self):
        self._anchor_wall, self._anchor_ns = time.monotonic(), self._time_ns

    def select_topic(self, name):
        with self._condition:
            if name not in {entry["name"] for entry in self._metadata.get("topics", ())}:
                return
            if self._playing:
                self._time_ns = self._clock()
                self._reset_anchor()
            self._topic = name
            self._queue_frame()

    def _queue_frame(self):
        self._revision += 1
        self._latest = None
        self._request = True
        self._status = dict(state="loading", message="正在读取所选话题")
        self._condition.notify_all()

    def seek(self, time_ns):
        with self._condition:
            if not self._metadata:
                return
            self._time_ns = min(self._metadata["end_ns"], max(self._metadata["start_ns"], int(time_ns)))
            self._reset_anchor()
            self._snapshot = None
            self._seek_revision += 1
            self._queue_frame()

    def play(self):
        with self._condition:
            if not self._metadata or self._source is None:
                return
            if self._time_ns >= self._metadata["end_ns"]:
                self._time_ns = self._metadata["start_ns"]
                self._snapshot = None
                self._seek_revision += 1
            self._revision += 1
            self._request = True
            self._playing = True
            self._reset_anchor()
            self._deadline = 0.
            self._condition.notify_all()

    def pause(self):
        with self._condition:
            if self._playing:
                self._time_ns = self._clock()
                self._revision += 1
                self._request = True
            self._playing = False
            self._condition.notify_all()

    def set_rate(self, rate):
        if rate not in (.25, .5, 1., 2., 4.):
            raise ValueError("Unsupported playback rate")
        with self._condition:
            if self._playing:
                self._time_ns = self._clock()
            self._rate = rate
            self._reset_anchor()

    def metadata(self):
        with self._condition:
            return copy.deepcopy(self._metadata)

    def latest(self):
        with self._condition:
            return copy.deepcopy(self._latest)

    def snapshot(self):
        """An atomic set of visual topics sampled at one requested bag time.

        Individual samples may be older than that time; each frame retains its
        own sample_ns. This aligns the playhead, not sensor acquisition clocks.
        Empty frames mean loading/cleared. A topic selection keeps the previous
        visual snapshot visible until its replacement is complete.
        """
        with self._condition:
            return copy.deepcopy(self._snapshot) if self._snapshot is not None else dict(
                time_ns=self._time_ns, frames={}, generation=self._generation,
                revision=self._revision, sequence=self._sequence, path=self._path,
                seek_revision=self._seek_revision,
                warnings=list(self._snapshot_warnings))

    def status(self):
        with self._condition:
            return dict(self._status)

    def timeline(self):
        with self._condition:
            return dict(start_ns=self._metadata.get("start_ns", 0), end_ns=self._metadata.get("end_ns", 0),
                        time_ns=self._time_ns, playing=self._playing, rate=self._rate,
                        path=self._path, topic=self._topic)

    def _cancelled(self, generation):
        with self._condition:
            return self._stop or generation != self._generation

    def _obsolete(self, generation, revision):
        with self._condition:
            return self._cancelled(generation) or revision != self._revision

    @staticmethod
    def _read_frame(source, topic, stamp):
        try:
            frame = source.frame(topic, stamp)
            if not isinstance(frame, dict) or frame.get("topic") != topic or frame.get("time_ns") != stamp:
                raise ValueError("Playback response does not match the selected topic and time")
            return frame
        except InterruptedError:
            raise
        except Exception as exc:
            message = str(exc)[:1800]
            return dict(topic=topic, time_ns=stamp, sample_ns=None, device=None,
                        details={}, warnings=[message], error=message)

    def _run(self):
        source = None
        try:
            while True:
                with self._condition:
                    if self._stop:
                        return
                    load, self._load = self._load, None
                    if load is None and not self._request and not (
                            self._playing and time.monotonic() >= self._deadline):
                        self._condition.wait(.03)
                        continue
                    generation = self._generation
                    revision = self._revision
                try:
                    if load:
                        if source is not None:
                            source.close()
                        source = None
                        with self._condition:
                            self._source = None
                        path, generation = load
                        if path is None:
                            continue
                        source = self.loader(path, lambda g=generation: self._cancelled(g))
                        try:
                            validate_metadata(source.metadata)
                        except Exception:
                            source.close()
                            source = None
                            raise
                        with self._condition:
                            if self._cancelled(generation):
                                source.close()
                                source = None
                                continue
                            self._source = source
                            self._metadata = copy.deepcopy(source.metadata)
                            topics = self._metadata["topics"]
                            visual_entries = [t for t in topics if t["kind"] in _VISUAL_KINDS]
                            # Empty declarations remain visible when space is
                            # available, but never crowd actual recordings out.
                            visual = [t["name"] for t in sorted(visual_entries, key=lambda t: not t["count"])]
                            self._visual_topics = tuple(visual[:self.max_visual_topics])
                            self._snapshot_warnings = []
                            omitted = visual[self.max_visual_topics:]
                            if omitted:
                                warning = (f"多设备预览最多显示 {self.max_visual_topics} 个话题；"
                                           f"其余 {len(omitted)} 个仍可在话题详情中单独浏览")
                                self._snapshot_warnings.append(warning)
                                self._metadata["warnings"] = self._metadata.get("warnings", [])[:8191] + [warning]
                            self._metadata["visual_topics"] = list(self._visual_topics)
                            self._metadata["visual_topics_omitted"] = omitted
                            self._topic = next((t["name"] for t in topics if t["count"] and t["kind"] != "message"),
                                               next((t["name"] for t in topics if t["count"]), topics[0]["name"]))
                            self._time_ns = self._metadata["start_ns"]
                            self._reset_anchor()
                    with self._condition:
                        if self._cancelled(generation) or source is None:
                            continue
                        self._request = False
                        if self._playing:
                            self._time_ns = self._clock()
                            if self._time_ns >= self._metadata["end_ns"]:
                                self._playing = False
                        topic, stamp, revision = self._topic, self._time_ns, self._revision
                        seek_revision = self._seek_revision
                        previous = self._snapshot
                        reuse = (previous is not None and previous["generation"] == generation
                                 and previous["time_ns"] == stamp)
                        frames = dict(previous["frames"]) if reuse else {}
                        visual_topics = self._visual_topics
                        warnings = list(self._snapshot_warnings)
                    for name in visual_topics:
                        if self._obsolete(generation, revision):
                            break
                        if name not in frames:
                            frames[name] = self._read_frame(source, name, stamp)
                        if frames[name].get("error"):
                            warnings.append(f"{name}: {frames[name]['error']}")
                    if self._obsolete(generation, revision):
                        continue
                    frame = frames.get(topic)
                    if frame is None:
                        frame = self._read_frame(source, topic, stamp)
                    with self._condition:
                        if self._cancelled(generation) or revision != self._revision:
                            continue
                        self._latest = frame
                        self._sequence += 1
                        self._snapshot = dict(time_ns=stamp, frames=frames, generation=generation,
                                              revision=revision, sequence=self._sequence, path=self._path,
                                              seek_revision=seek_revision,
                                              warnings=warnings)
                        self._deadline = time.monotonic() + .033
                        self._status = dict(state="ready", message="回放已就绪；缺失话题不影响已有数据")
                except Exception as exc:
                    with self._condition:
                        if not self._cancelled(generation) and revision == self._revision:
                            self._latest = None
                            self._snapshot = None
                            self._playing = False
                            self._request = False
                            self._status = dict(state="error", message=str(exc)[:1800])
        finally:
            if source is not None:
                source.close()

    def close(self):
        with self._condition:
            self._stop = True
            self._playing = False
            source = self._source
            self._condition.notify_all()
        if source is not None:
            source.close()
        self._thread.join(timeout=1.)
