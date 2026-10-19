"""Cancellable offline spatial playback without ROS or current calibration.

Supports bounded scene archives and integrity-checked alignment JSONL. The
latter replays saved world poses only: its image references are not pixels.
"""
from __future__ import annotations

from array import array
import base64
import copy
import hashlib
import json
from pathlib import Path
import threading
import time
import uuid

from vt_capture_core.spatial.scene_protocol import MAX_SCENE_BYTES, validate_scene
from vt_capture_core.spatial.cloud_protocol import unpack_cloud_frame

from .replay_transport import ReplayDecoder, decode_json as _json


class ArchiveSource:
    note = "场景快照回放；未加载原始图像，点云不可用"

    def __init__(self, path):
        if path.stat().st_size > 32*1024*1024:
            raise ValueError("场景回放文件超过 32 MiB")
        document = _json(path.read_bytes())
        if not isinstance(document, dict) or document.get("schema_version") != 1 or document.get("kind") != "vt.spatial.archive":
            raise ValueError("不支持的场景回放格式")
        frames = document.get("frames")
        if not isinstance(frames, list) or not 1 <= len(frames) <= 4096:
            raise ValueError("场景快照数量须为 1 至 4096")
        for frame in frames:
            validate_scene(frame)
            if len(json.dumps(frame).encode()) > MAX_SCENE_BYTES:
                raise ValueError("单个场景快照超过大小上限")
        self.frames = frames
        self.frame_count = len(frames)

    def frame(self, index):
        return copy.deepcopy(self.frames[index])


class AlignmentSource:
    note = "对齐位姿回放；未加载 RGB / 深度像素与完整几何，点云不可用"

    def __init__(self, path, cancelled=lambda: False):
        self.path = path
        self._stat = (path.stat().st_size, path.stat().st_mtime_ns)
        if self._stat[0] > 1024**3:
            raise ValueError("对齐回放索引超过 1 GiB")
        manifest_path = path.parent / "manifest.json"
        if not manifest_path.is_file() or manifest_path.stat().st_size > 512*1024:
            raise ValueError("对齐回放缺少完整性 manifest.json")
        self.manifest = _json(manifest_path.read_bytes())
        if not isinstance(self.manifest, dict) or self.manifest.get("schema_version") != 1 or self.manifest.get("tool") != "vt-multisensor-alignment":
            raise ValueError("不支持的对齐结果版本")
        inventory = self.manifest.get("files", {}).get(path.name, {})
        if inventory.get("size_bytes") != self._stat[0]:
            raise ValueError("对齐文件完整性大小不匹配")
        self.offsets = array("Q")
        digest = hashlib.sha256()
        with path.open("rb") as stream:
            while True:
                if cancelled():
                    raise InterruptedError("已取消回放索引加载")
                offset = stream.tell()
                line = stream.readline(MAX_SCENE_BYTES+1)
                if not line:
                    break
                if len(line) > MAX_SCENE_BYTES or len(self.offsets) >= 1_000_000:
                    raise ValueError("对齐回放行或帧数超过上限")
                if line.strip():
                    self.offsets.append(offset)
                digest.update(line)
        if not self.offsets or digest.hexdigest() != inventory.get("sha256"):
            raise ValueError("对齐文件完整性 SHA-256 不匹配")
        self.frame_count = len(self.offsets)
        self.identity = digest.hexdigest()

    def frame(self, index):
        info = self.path.stat()
        if (info.st_size, info.st_mtime_ns) != self._stat:
            raise ValueError("回放文件已改变，请重新加载")
        with self.path.open("rb") as stream:
            stream.seek(self.offsets[index])
            record = _json(stream.readline(MAX_SCENE_BYTES+1))
        source = record["reference_time_ns"]
        if any(not isinstance(record.get(kind, {}), dict) for kind in ("trackers", "cameras")):
            raise ValueError("对齐帧设备集合必须为映射")
        trackers, cameras = [], []
        for role, value in record.get("trackers", {}).items():
            if value is not None and not isinstance(value, dict):
                raise ValueError("对齐 Tracker 条目无效")
            pose = value.get("world_from_tracker") if value else None
            trackers.append(dict(id=role, role=role, label=role, tracker_id=value.get("tracker_id") if value else None,
                                 source_realtime_ns=value.get("timestamp_ns", source) if value else source,
                                 position_m=pose.get("translation_m") if pose else None,
                                 quaternion_xyzw=pose.get("quaternion_xyzw") if pose else None,
                                 state="fresh" if pose else "waiting", valid=bool(pose), age_s=None, trail_segments=[]))
        for name, value in record.get("cameras", {}).items():
            if value is not None and not isinstance(value, dict):
                raise ValueError("对齐相机条目无效")
            pose = value.get("world_from_camera") if value else None
            cameras.append(dict(id=name, label=name, source_realtime_ns=value.get("host_realtime_ns", source) if value else source,
                                position_m=pose.get("translation_m") if pose else None,
                                quaternion_xyzw=pose.get("quaternion_xyzw") if pose else None,
                                state="fresh" if pose else "waiting", registration_state="ready" if pose else "missing",
                                reason="保存的世界位姿；内参/像素未加载，不能重建点云", age_s=None))
        result = dict(manager_instance_id="alignment-"+self.identity[:24], scene_epoch="alignment",
                      sequence=index+1, mode="replay", world_frame=self.manifest.get("configuration", {}).get("world_frame", "unknown"),
                      time_epoch="alignment", coordinate_epoch="alignment", sync_epoch="alignment",
                      config_revision="alignment", geometry_revision="unavailable", time_mode="aligned",
                      scene_time_ns=source, generated_realtime_ns=source, active_reference_camera=record.get("reference_camera"),
                      map_identity=None, display_from_world={"translation_m": [0., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]},
                      trackers=trackers, cameras=cameras,
                      diagnostics=[self.note]+list(record.get("quality_flags", ()))[:32])
        validate_scene(result)
        return result


class BagSubprocessSource:
    """Spatial payload validation on the shared owned-decoder transport."""
    def __init__(self, path, cancelled=lambda: False):
        self._decoder = ReplayDecoder("spatial_replay_worker", path, cancelled)
        self.last_clouds = []
        try:
            metadata = self._decoder.metadata()
            if not isinstance(metadata, dict):
                raise ValueError("bag 回放响应无效")
            self.frame_count = metadata.get("frame_count", 0)
            if type(self.frame_count) is not int or not 1 <= self.frame_count <= 1_000_000:
                raise ValueError("bag 回放帧数无效或超过上限")
            self.note = str(metadata.get("note", "已关闭 bag 的离线回放"))[:600]
        except BaseException:
            self.close()
            raise

    def set_cancelled(self, cancelled):
        self._decoder.set_cancelled(cancelled)

    def frame(self, index):
        response = self._decoder.request(dict(command="frame", index=index))
        if type(response.get("index")) is not int or response["index"] != index:
            raise ValueError("Playback response does not match the selected frame index")
        clouds = response.get("clouds", [])
        if not isinstance(clouds, list) or len(clouds) > 3:
            raise ValueError("bag 回放点云数量超限")
        scene = response["scene"]
        validate_scene(scene)
        self.last_clouds = clouds
        return scene

    def close(self):
        self._decoder.close()


def load_replay_source(path, cancelled=lambda: False):
    path = Path(path).expanduser().absolute()
    if path.is_dir():
        candidate = path / "aligned_frames.jsonl"
        if not candidate.is_file():
            if (path / "metadata.yaml").is_file():
                return BagSubprocessSource(path, cancelled)
            raise ValueError("该目录缺少 bag metadata.yaml 或 aligned_frames.jsonl；不会借用实时标定重建点云。")
        path = candidate
    return AlignmentSource(path, cancelled) if path.suffix == ".jsonl" else ArchiveSource(path)


class ReplayCloudCache:
    """Only clouds returned with the exact saved scene can enter playback."""
    def __init__(self):
        self._lock = threading.Lock()
        self._key = None
        self._frames = {}

    @staticmethod
    def prepare(encoded_clouds, original_scene):
        import numpy as np
        frames = {}
        cameras = {camera.get("id", camera.get("camera_id")): camera for camera in original_scene.get("cameras", ())}
        for encoded in encoded_clouds:
            if not isinstance(encoded, str) or len(encoded) > 1_400_000:
                raise ValueError("回放点云编码超过大小上限")
            packet = unpack_cloud_frame(base64.b64decode(encoded, validate=True))
            header = packet.header
            camera = cameras.get(header["camera_id"])
            if camera is None or any(field not in original_scene or header.get(field) != original_scene[field]
                                     for field in ("manager_instance_id", "scene_epoch", "time_epoch", "coordinate_epoch", "sync_epoch")):
                raise ValueError("回放点云与场景身份或时钟段不匹配")
            if any(field not in camera or camera[field] != header.get(field) for field in ("frame_key", "geometry_revision", "geometry_sha256")):
                raise ValueError("回放点云与图像帧或几何材料不匹配")
            positions = np.frombuffer(packet.xyz_bytes, dtype="<f4").reshape(-1, 3)
            colors = np.frombuffer(packet.rgba_bytes, dtype="u1").reshape(-1, 4).astype("f4")/255.
            positions.setflags(write=False)
            colors.setflags(write=False)
            frames[header["camera_id"]] = dict(header=header, positions=positions, colors=colors)
        return frames

    def publish(self, frames, scene):
        with self._lock:
            self._frames = frames
            self._key = (scene["manager_instance_id"], scene["scene_epoch"], scene["sequence"])

    def latest(self, scene):
        if scene is None:
            return {}
        with self._lock:
            key = (scene.get("manager_instance_id"), scene.get("scene_epoch"), scene.get("sequence"))
            return dict(self._frames) if key == self._key else {}

    def status(self):
        with self._lock:
            return "保存的同帧点云" if self._frames else "当前历史帧缺少可用几何材料或深度像素，点云不可用"


class SpatialReplayBackend:
    mode = "replay"

    def __init__(self, *, loader=load_replay_source):
        self.loader = loader
        self._condition = threading.Condition()
        self._stop = False
        self._request = None
        self._source = self._scene = None
        self._path = ""
        self._index = 0
        self._generation = 0
        self._session = uuid.uuid4().hex
        self.cloud_backend = ReplayCloudCache()
        self._playing = False
        self._rate = 1.
        self._deadline = 0.
        self._status = {"state": "waiting", "message": "请选择场景快照或离线对齐结果"}
        self._thread = threading.Thread(target=self._run, name="capture-console-spatial-replay", daemon=True)
        self._thread.start()

    def load(self, path):
        with self._condition:
            if self._source is not None and hasattr(self._source, "close"):
                self._source.close()
            self._generation += 1
            self._playing = False
            self._scene = self._source = None
            self._path = str(path)
            self._status = {"state": "loading", "message": "正在后台加载回放索引"}
            self._request = ("load", path, self._generation)
            self._condition.notify_all()

    def seek(self, index):
        with self._condition:
            if self._source is None:
                return
            self._generation += 1
            index = min(max(0, int(index)), self._source.frame_count-1)
            self._request = ("seek", index, self._generation)
            self._status = {"state": "loading", "message": "正在定位回放帧"}
            self._condition.notify_all()

    def step(self, delta):
        self.pause()
        self.seek(self._index+delta)

    def play(self):
        with self._condition:
            self._playing = self._source is not None
            self._deadline = time.monotonic()
            self._condition.notify_all()

    def pause(self):
        with self._condition:
            self._playing = False

    def set_rate(self, rate):
        if rate not in (.25, .5, 1., 2., 4.):
            raise ValueError("unsupported playback rate")
        with self._condition:
            self._rate = rate

    def latest(self):
        with self._condition:
            return copy.deepcopy(self._scene)

    def status(self):
        with self._condition:
            return dict(self._status)

    def timeline(self):
        with self._condition:
            return {"index": self._index, "frame_count": self._source.frame_count if self._source else 0,
                    "playing": self._playing, "rate": self._rate, "path": self._path}

    def set_manager_instance(self, identifier):
        pass

    def set_interest(self, viewer_id, layers):
        pass

    def _cancelled(self, generation):
        with self._condition:
            return self._stop or generation != self._generation

    def _run(self):
        while True:
            with self._condition:
                if self._stop:
                    return
                if self._request:
                    request, self._request = self._request, None
                elif self._playing and self._source and time.monotonic() >= self._deadline:
                    if self._index+1 >= self._source.frame_count:
                        self._playing = False
                        continue
                    request = ("seek", self._index+1, self._generation)
                else:
                    self._condition.wait(.03)
                    continue
                source = self._source
            operation, value, generation = request
            temporary_source = None
            try:
                if operation == "load":
                    source = self.loader(value, lambda g=generation: self._cancelled(g))
                    temporary_source = source
                    index = 0
                else:
                    index = value
                if self._cancelled(generation):
                    continue
                if hasattr(source, "set_cancelled"):
                    source.set_cancelled(lambda g=generation: self._cancelled(g))
                frame = source.frame(index)
                validate_scene(frame)
                clouds = self.cloud_backend.prepare(getattr(source, "last_clouds", ()), frame)
                frame["mode"] = "replay"
                frame["scene_epoch"] = f"replay-{self._session}-{generation}"
                frame["sequence"] = index+1
                with self._condition:
                    if self._cancelled(generation):
                        continue
                    previous = self._scene
                    self._scene, self._source, self._index = frame, source, index
                    temporary_source = None  # Backend owns it after a valid first frame.
                    self.cloud_backend.publish(clouds, frame)
                    interval = .033
                    if previous:
                        elapsed = frame.get("generated_realtime_ns", 0)-previous.get("generated_realtime_ns", 0)
                        interval = min(1., max(.001, elapsed/1e9)) if elapsed > 0 else .033
                    self._deadline = time.monotonic()+interval/self._rate
                    self._status = {"state": "ready", "message": "REPLAY · "+source.note}
            except (OSError, ValueError, TypeError, KeyError, IndexError, RuntimeError, AttributeError) as exc:
                with self._condition:
                    if not self._cancelled(generation):
                        self._scene = None
                        self._playing = False
                        self._status = {"state": "error", "message": "回放不可用："+str(exc)[:220]}
            finally:
                if temporary_source is not None and hasattr(temporary_source, "close"):
                    temporary_source.close()

    def close(self):
        with self._condition:
            self._stop = True
            if self._source is not None and hasattr(self._source, "close"):
                self._source.close()
            self._condition.notify_all()
        self._thread.join(timeout=1.)
