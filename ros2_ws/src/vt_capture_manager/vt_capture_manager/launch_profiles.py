"""Local, explicit launch recipes. IPC clients may select IDs, never supply argv.

Reuse existing launch files and their validation. This is not a general shell
runner, a pairing workflow, or an automatic hardware discovery mechanism.
"""
from __future__ import annotations

import os
from pathlib import Path
from vt_capture_core.workstation import PATH_KEYS, PRESETS, parse_profiles


def build_specs(profiles):
    from .device_processes import ProcessSpec, ResourceArgument

    specs = []
    for profile in profiles:
        preset, p = profile["preset"], dict(profile["parameters"])
        python_paths = ()
        kind, package, launch, _, _ = PRESETS[preset]
        if preset == "cameras":
            topics = tuple(f"/{camera}/{stream}" for camera in ("d405_1", "d405_2", "d436")
                           for stream in ("color/image_raw", "depth/image_rect_raw")) + ("/capture/status",)
            resources = ("realsense-cameras", "capture-controller")
            if p.get("config_path"):
                resources += ("camera-config:" + p["config_path"],)
        elif preset == "trackers":
            topics = tuple(f"/vive/{role}/sample" for role in ("left_wrist", "right_wrist", "torso"))
            resources = ("vive-dongle",)
            # These two local recipe fields are not ROS launch arguments.
            # Optional values preserve old profiles' inherited environment.
            python_paths = tuple(p.pop(key, "") for key in ("pyvut_source", "pyvut_site_packages"))
            if p.get("backend") == "vivehub":
                python_paths = ()  # Hub clients never load the legacy HID driver.
                p.pop("bundle_path", None)
                p.pop("role_map_path", None)
        else:
            hand = p["hand"]
            topics = (f"/gloves/{hand}/" + ("state" if preset == "glove" else "tactile/state"),)
            resources = (f"{preset}-{hand}",)
            if p.get("port"):
                resources += ("serial:" + p["port"],)
            if preset == "tactile":
                # The workbench is the viewer; do not launch an unrelated Qt interpreter.
                p.update(namespace=f"/gloves/{hand}/tactile", gui="false")
            else:
                p.setdefault("launch_gui", "true")
                if p.get("socket_path"):
                    resources += ("socket:" + str(Path(p["socket_path"]).resolve()),)
        argv = ("ros2", "launch", package, launch) + tuple(f"{key}:={value}" for key, value in sorted(p.items()) if value)
        if any(python_paths):
            python_path = os.pathsep.join(path for path in (*python_paths, os.environ.get("PYTHONPATH", "")) if path)
            argv = ("/usr/bin/env", "PYTHONPATH=" + python_path, *argv)
        bindings = ()
        if preset in ("glove", "tactile") and p.get("port"):
            bindings = (ResourceArgument(argv.index("port:=" + p["port"]), "serial:" + p["port"], "port:="),)
        specs.append(ProcessSpec(id=profile["id"], label=profile["label"], kind=kind, argv=argv,
                                 topics=topics, resources=resources, enabled=profile["enabled"],
                                 startup_timeout_s=profile["startup_timeout_s"], resource_arguments=bindings))
    return tuple(specs)
