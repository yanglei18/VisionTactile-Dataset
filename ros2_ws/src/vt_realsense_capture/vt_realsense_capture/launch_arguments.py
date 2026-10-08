from .config import CameraConfig, CaptureConfig


def camera_launch_arguments(
    camera: CameraConfig,
    cfg: CaptureConfig,
    *,
    config_file: str | None = None,
) -> dict[str, str]:
    profile = f"{cfg.width}x{cfg.height}x{cfg.fps}"
    result = {
        "camera_namespace": "",
        "camera_name": camera.name,
        "serial_no": f"_{camera.serial}",
        "enable_color": "true",
        "enable_depth": "true",
        "depth_module.depth_profile": profile,
        "depth_module.depth_format": cfg.depth_format,
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
    result[f"{camera.color_module}.color_profile"] = profile
    result[f"{camera.color_module}.color_format"] = cfg.color_format
    if config_file is not None:
        result["config_file"] = config_file
    return result
