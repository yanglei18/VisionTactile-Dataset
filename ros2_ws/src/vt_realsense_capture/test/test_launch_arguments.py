import inspect
from pathlib import Path

from vt_realsense_capture.config import load_config
from vt_realsense_capture.launch_arguments import camera_launch_arguments

import yaml


CONFIG = Path(__file__).parents[1] / "config" / "cameras.yaml"
REALSENSE_OPTIONS = CONFIG.with_name("realsense_options.yaml")
INSTALLED_OPTIONS = (
    "/install/vt_realsense_capture/share/vt_realsense_capture/"
    "config/realsense_options.yaml"
)
def _fully_qualified_name(*parts: str) -> str:
    return "/" + "/".join(part.strip("/") for part in parts if part.strip("/"))


def test_d405_and_d436_use_their_real_color_modules():
    cfg = load_config(CONFIG)
    args = {
        camera.name: camera_launch_arguments(camera, cfg)
        for camera in cfg.cameras
    }

    assert args["d405_1"]["depth_module.color_profile"] == "1280x720x30"
    assert "rgb_camera.color_profile" not in args["d405_1"]
    assert args["d405_2"]["depth_module.color_profile"] == "1280x720x30"
    assert "rgb_camera.color_profile" not in args["d405_2"]
    assert args["d436"]["rgb_camera.color_profile"] == "1280x720x30"
    assert "depth_module.color_profile" not in args["d436"]


def test_launch_arguments_are_exact_and_disable_processing():
    cfg = load_config(CONFIG)
    common = {
        "camera_namespace": "",
        "enable_color": "true",
        "enable_depth": "true",
        "depth_module.depth_profile": "1280x720x30",
        "depth_module.depth_format": "Z16",
        "enable_infra": "false",
        "enable_infra1": "false",
        "enable_infra2": "false",
        "enable_sync": "true",
        "enable_rgbd": "false",
        "align_depth.enable": "false",
        "pointcloud.enable": "false",
        "colorizer.enable": "false",
        "publish_tf": "true",
        "initial_reset": "false",
    }

    for camera in cfg.cameras:
        expected = {
            **common,
            "camera_name": camera.name,
            "serial_no": f"_{camera.serial}",
            f"{camera.color_module}.color_profile": "1280x720x30",
            f"{camera.color_module}.color_format": "RGB8",
        }
        assert camera_launch_arguments(camera, cfg) == expected


def test_each_camera_has_exactly_one_model_specific_color_profile_key():
    cfg = load_config(CONFIG)

    for camera in cfg.cameras:
        args = camera_launch_arguments(camera, cfg)
        assert len([key for key in args if key.endswith(".color_profile")]) == 1
        assert len([key for key in args if key.endswith(".color_format")]) == 1


def test_every_camera_is_serial_bound_without_usb_port_binding():
    cfg = load_config(CONFIG)

    for camera in cfg.cameras:
        args = camera_launch_arguments(camera, cfg)
        assert args["serial_no"] == f"_{camera.serial}"
        assert args["serial_no"].startswith("_")
        assert "usb_port_id" not in args


def test_wrapper_syncs_sensor_frames_without_resampling_depth():
    cfg = load_config(CONFIG)

    for camera in cfg.cameras:
        args = camera_launch_arguments(camera, cfg)
        assert args["enable_sync"] == "true"
        assert args["enable_rgbd"] == "false"
        assert args["align_depth.enable"] == "false"
        assert "align_depth" not in {
            key for key, value in args.items() if value == "true"
        }


def test_device_info_service_paths_have_one_camera_prefix():
    cfg = load_config(CONFIG)

    for camera in cfg.cameras:
        args = camera_launch_arguments(camera, cfg)
        service = _fully_qualified_name(
            args["camera_namespace"], args["camera_name"], "device_info"
        )
        assert service == f"/{camera.name}/device_info"


def test_config_file_is_optional_and_explicit_without_changing_two_arg_callers():
    cfg = load_config(CONFIG)
    camera = cfg.cameras[0]
    two_argument_result = camera_launch_arguments(camera, cfg)
    assert "config_file" not in two_argument_result

    parameters = inspect.signature(camera_launch_arguments).parameters
    assert "config_file" in parameters

    configured_result = camera_launch_arguments(
        camera, cfg, config_file=INSTALLED_OPTIONS
    )
    assert configured_result == {
        **two_argument_result,
        "config_file": INSTALLED_OPTIONS,
    }


def test_options_are_flat_for_the_installed_official_wrapper():
    # rs_launch.py passes this top-level mapping directly to its Node action.
    assert yaml.safe_load(REALSENSE_OPTIONS.read_text()) == {
        "depth_module.global_time_enabled": False,
        "rgb_camera.global_time_enabled": False,
    }
