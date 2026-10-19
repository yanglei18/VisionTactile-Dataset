"""One capture transport policy, shared by manager and direct ROS launch.

Only the two process-tree entry points apply this environment, before creating
any ROS participant. Camera publishers, the controller and Recorder inherit it.
No middleware is selected and no endpoint QoS or OS setting is changed here.
"""
from __future__ import annotations

from collections.abc import Mapping
from pathlib import Path


_FAST_DDS_IMPLEMENTATIONS = {"", "rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp"}
_EXPLICIT_TRANSPORT_SETTINGS = (
    "FASTRTPS_DEFAULT_PROFILES_FILE",
    "FASTDDS_DEFAULT_PROFILES_FILE",
    "FASTDDS_BUILTIN_TRANSPORTS",
    "SKIP_DEFAULT_XML",
    "FASTDDS_ENVIRONMENT_FILE",
)
_DEFAULT_PROFILE_NAMES = ("DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml")


def capture_transport_environment(
    environment: Mapping[str, str], *, cwd: str | Path | None = None,
) -> dict[str, str]:
    """Copy the environment, adding a bounded Fast DDS default when unconfigured.

    Explicit transport settings, including empty or invalid values, belong to
    the operator and are left for the middleware to validate. A cwd profile is
    likewise preserved without opening its contents. With no explicit RMW the
    platform still selects its middleware; other middleware ignores this XML.
    The packaged path is permanent for descendants, not an extracted temp file.
    """
    result = dict(environment)
    if result.get("RMW_IMPLEMENTATION", "") not in _FAST_DDS_IMPLEMENTATIONS:
        return result
    if any(key in result for key in _EXPLICIT_TRANSPORT_SETTINGS):
        return result
    directory = Path.cwd() if cwd is None else Path(cwd)
    if any((directory / name).exists() or (directory / name).is_symlink()
           for name in _DEFAULT_PROFILE_NAMES):
        return result
    profile = Path(__file__).resolve().parent / "resources" / "fastdds_capture.xml"
    if not profile.is_file():
        raise FileNotFoundError("The installed capture Fast DDS transport profile is missing")
    # Fast DDS 2.x (ROS Jazzy) recognizes the historical FASTRTPS spelling.
    result["FASTRTPS_DEFAULT_PROFILES_FILE"] = str(profile)
    return result
