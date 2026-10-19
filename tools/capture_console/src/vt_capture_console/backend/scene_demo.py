"""Explicit synthetic Tracker/camera scene; never used as a live fallback."""
from __future__ import annotations

import math
import time
from vt_capture_core.spatial.transforms import RigidTransform


def demo_scene(seconds, *, sequence=1, scenario="moving"):
    now = time.time_ns()
    trackers, cameras = [], []
    roles = ("left_wrist", "right_wrist", "torso")
    labels = ("左腕 Tracker", "右腕 Tracker", "躯干 Tracker")
    camera_ids = ("d405_1", "d405_2", "d436")
    for index, role in enumerate(roles):
        offline = scenario == "offline" and index == 1
        phase = (0. if offline else seconds) + index * 1.3
        x = (-.4, .4, 0.)[index] + (.06 * math.sin(phase) if scenario != "static" else 0)
        y = (.0, .0, .3)[index] + (.05 * math.cos(phase) if scenario != "static" else 0)
        z = (1.1, 1.1, 1.35)[index]
        angle = .15 * math.sin(phase)
        rotation = [0., 0., math.sin(angle/2), math.cos(angle/2)]
        trail = [[(-.4, .4, 0.)[index]+.06*math.sin(phase-age/30),
                  (.0, .0, .3)[index]+.05*math.cos(phase-age/30), z] for age in range(60, -1, -1)]
        if scenario == "static":
            trail = [[x, y, z]]
        trackers.append(dict(id=role, role=role, label=labels[index], tracker_id=f"DEMO-{index+1}",
                             frame_id=f"{role}_tracker", source_realtime_ns=now,
                             position_m=[x, y, z], quaternion_xyzw=rotation,
                             state="offline" if offline else "fresh", age_s=2. if offline else .015,
                             valid=not offline, trail_segments=[trail]))
        intrinsics = dict(width=640, height=480, fx=500., fy=500., cx=320., cy=240.)
        vertices = [[(u-320)/500*d, (v-240)/500*d, d] for d in (.02, .35)
                    for u, v in ((0, 0), (640, 0), (640, 480), (0, 480))]
        camera_pose = RigidTransform((x, y, z), tuple(rotation)).compose(
            RigidTransform((0., -.05, -.04), (math.sqrt(.5), 0., 0., math.sqrt(.5))))
        cameras.append(dict(id=camera_ids[index], label=("D405 1", "D405 2", "D436")[index],
                            serial=f"SYNTHETIC-{index+1}", tracker_role=role,
                            frame_id=f"{camera_ids[index]}_color_optical_frame",
                            source_realtime_ns=now, age_s=.015,
                            position_m=list(camera_pose.translation_m), quaternion_xyzw=list(camera_pose.quaternion_xyzw),
                            state="offline" if offline else "fresh", registration_state="ready", reason="合成刚性装配 · DEMO",
                            geometry_revision=1, intrinsics=intrinsics, frustum_vertices=vertices))
    return dict(manager_instance_id="demo-manager", scene_epoch="demo-scene", sequence=sequence,
                mode="demo", world_frame="vive_map", time_epoch="demo-time", coordinate_epoch="demo-map",
                sync_epoch="demo-sync", time_mode="monitor", scene_time_ns=None,
                generated_realtime_ns=now, config_revision=1, geometry_revision=1,
                active_reference_camera=None, map_identity="SYNTHETIC", display_from_world={
                    "translation_m": [0., 0., 0.], "quaternion_xyzw": [0., 0., 0., 1.]},
                trackers=trackers, cameras=cameras, diagnostics=["DEMO · 合成数据，不代表实际设备位置"])


class DemoSceneBackend:
    mode = "demo"

    def __init__(self):
        self._started = time.monotonic()
        self._sequence = 0
        self._interests = {}
        self.scenario = "moving"
        self.closed = False

    def set_manager_instance(self, _identifier):
        pass

    def set_interest(self, viewer_id, layers):
        if layers:
            self._interests[viewer_id] = frozenset(layers)
        else:
            self._interests.pop(viewer_id, None)

    def latest(self):
        if self.closed:
            return None
        self._sequence += 1
        return demo_scene(time.monotonic()-self._started, sequence=self._sequence, scenario=self.scenario)

    def status(self):
        return {"state": "ready", "message": "DEMO · 合成空间数据"}

    def description(self):
        return {}

    def close(self):
        self.closed = True
        self._interests.clear()
