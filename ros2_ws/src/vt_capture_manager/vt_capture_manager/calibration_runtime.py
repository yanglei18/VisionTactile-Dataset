"""Wire configured device bindings to the optional calibration service.

All numerical work stays in the separately installed calibration worker. This
module imports neither OpenCV nor Qt and never starts a sensor.
"""
from copy import deepcopy
from pathlib import Path
import re

from .calibration.config import build_config
from .calibration.storage import read_document


class CalibrationBindings:
    def __init__(self, config, *, default_camera_config=None):
        self.world_frame = "vive_map"
        scene_cameras = {}
        scene_trackers = {}
        if config.scene_config_path:
            from .spatial.config import load_scene_config
            scene = load_scene_config(config.scene_config_path)
            self.world_frame = scene.world_frame
            scene_cameras = {c["id"]: deepcopy(c) for c in scene.cameras}
            scene_trackers = {t["role"]: t for t in (scene.alignment_document or {}).get("trackers", [])}
        profiles = [p for p in config.process_groups if p["preset"] == "cameras"]
        if len(profiles) > 1:
            raise ValueError("calibration requires an unambiguous camera launch configuration")
        path = profiles[0]["parameters"].get("config_path") if profiles else None
        if not path:
            path = default_camera_config
        if not path and not scene_cameras:
            from ament_index_python.packages import get_package_share_directory
            path = Path(get_package_share_directory("vt_realsense_capture")) / "config/cameras.yaml"
        cameras = []
        if path:
            raw = read_document(path)
            entries = raw.get("cameras")
            if not isinstance(entries, list) or not 1 <= len(entries) <= 16:
                raise ValueError("camera configuration requires a bounded camera list")
            for entry in entries:
                name = entry.get("name", "")
                if not re.fullmatch(r"[A-Za-z_][A-Za-z0-9_]{0,127}", name):
                    raise ValueError("invalid configured camera name")
                camera = dict(id=name, name=name, model=entry["model_token"], serial=entry["serial"],
                    color_frame_id=f"{name}_color_optical_frame", depth_frame_id=f"{name}_depth_optical_frame",
                    color_topic=f"/{name}/color/image_raw", depth_topic=f"/{name}/depth/image_rect_raw",
                    camera_info_topic=f"/{name}/color/camera_info", depth_camera_info_topic=f"/{name}/depth/camera_info",
                    timing_topic=f"/{name}/frame_timing", depth_to_color_topic=f"/{name}/extrinsics/depth_to_color",
                    device_info_service=f"/{name}/device_info", tracker_role="torso")
                if name in scene_cameras:
                    scene_camera = scene_cameras[name]
                    if scene_camera["serial"] != camera["serial"]:
                        raise ValueError("scene and capture camera identity disagree")
                    camera.update(scene_camera)
                    if not camera.get("device_info_service"):
                        camera["device_info_service"] = f"/{name}/device_info"
                cameras.append(camera)
        else:
            cameras = list(scene_cameras.values())
        monitored = {d.topic for d in config.devices if d.kind == "camera"}
        self.cameras = [c for c in cameras if c["color_topic"] in monitored]
        if len({c["id"] for c in self.cameras}) != len(self.cameras):
            raise ValueError("duplicate configured camera identity")
        self.trackers = []
        for device in config.devices:
            if device.kind != "tracker" or device.id not in {"torso", "left_wrist", "right_wrist"}:
                continue
            explicit = scene_trackers.get(device.id, {})
            if explicit.get("sample_topic", device.topic) != device.topic:
                raise ValueError("scene and monitored Tracker topic bindings disagree")
            frame = explicit.get("frame_id", f"vive_tracker_{device.id}")
            if not isinstance(frame, str) or not frame or len(frame) > 128:
                raise ValueError("configured Tracker frame must be a bounded name")
            self.trackers.append(dict(role=device.id, frame_id=frame, sample_topic=device.topic))

    def catalog(self):
        return dict(cameras=[dict(name=c["id"], model=c["model"], serial=c["serial"]) for c in self.cameras],
                    trackers=[dict(role=t["role"]) for t in self.trackers])

    def prepare(self, params, *, expected_tracker_id=""):
        camera = next((c for c in self.cameras if c["id"] == params["camera_name"]), None)
        tracker = next((t for t in self.trackers if t["role"] == params["tracker_role"]), None)
        if camera is None or tracker is None:
            raise ValueError("selected camera or Tracker is not configured in this workstation")
        config = build_config(camera, tracker["role"], params["board"],
                              tracker_frame_id=tracker["frame_id"], world_frame=self.world_frame)
        config["tracker"]["sample_topic"] = tracker["sample_topic"]
        if params.get("config_path"):
            imported = read_document(params["config_path"])
            for branch in ("camera", "tracker"):
                if imported.get(branch) != config[branch]:
                    raise ValueError(f"imported {branch} identity/topic binding differs from configured device")
            if imported.get("frames", {}).get("world") != self.world_frame:
                raise ValueError("imported world frame binding differs from configured world")
            config = imported
        return dict(config=config, expected_tracker_id=expected_tracker_id)


def prepare_session(bindings, adapter, params):
    """Bind the live publishers before a guided worker can start its Recorder."""
    tracker_id = adapter.expected_tracker_id(params["tracker_role"]) if params["mode"] == "guided" else ""
    if params["mode"] == "guided" and not tracker_id:
        raise ValueError("Tracker has no current valid identity; start the selected device before calibration")
    prepared = bindings.prepare(params, expected_tracker_id=tracker_id)
    if params["mode"] == "guided":
        config = prepared["config"]
        proof = dict(camera=config["camera"], tracker=config["tracker"] | dict(tracker_id=tracker_id), frames=config["frames"])
        valid, reason = adapter.identity(proof)
        if not valid:
            raise ValueError(f"Live calibration identity not verified: {reason}")
    return prepared


def attach_calibration(node, management, *, root_dir, worker_python):
    """Create the optional service; its adapter is independent of the space page."""
    from .calibration.service import CalibrationService
    from .calibration.ros_adapter import CalibrationRosAdapter
    bindings = CalibrationBindings(node.model.config)
    adapter = CalibrationRosAdapter(node, bindings.cameras, bindings.trackers,
                                    world_frame=bindings.world_frame, instance_id=node.model.instance_id)

    def prepare(params):
        return prepare_session(bindings, adapter, params)

    def apply(document, manifest):
        scene = node.scene_store
        if scene is not None and scene.camera_registry is not None:
            camera = document["camera"]["name"]
            registered = {c["id"] for c in scene.config.cameras}
            if camera in registered:
                tracker = next(t for t in bindings.trackers if t["role"] == document["tracker"]["role"])
                scene.apply_camera_calibration(camera, document, tracker_sample_topic=tracker["sample_topic"])

    try:
        service = CalibrationService(root_dir, node.model.instance_id, worker_python,
            can_start=management.calibration_allowed, prepare_config=prepare,
            identity=adapter.identity, apply=apply, publish=adapter.publish)
    except BaseException:
        adapter.close()
        raise
    management.calibration = service
    management.calibration_catalog = bindings.catalog()
    return adapter
