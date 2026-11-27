"""One capture transport policy, shared by manager and direct ROS launch.

Only the process-tree entry points (manager, direct camera launch and the
standalone hand-cube viewer) apply this environment, before creating any ROS
participant. Camera publishers, the controller and Recorder inherit it.
No middleware is selected and no endpoint QoS or OS setting is changed here.
Discovery is limited to this host unless the operator chooses otherwise.
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


DISCOVERY_CHOICE = "VT_CAPTURE_DISCOVERY_RANGE"
_DISCOVERY_RANGES = ("LOCALHOST", "SUBNET", "SYSTEM_DEFAULT", "OFF")


def capture_discovery_environment(environment: Mapping[str, str]) -> dict[str, str]:
    """Keep capture participants discoverable only on this host unless chosen.

    Recording commands, calibration and hand-binding services and camera
    streams carry no authentication, so other computers on the network must
    not discover them automatically. ROS setup scripts export
    ROS_AUTOMATIC_DISCOVERY_RANGE=SUBNET whenever it is unset, so its presence
    is not an operator decision; multi-host capture is chosen explicitly with
    VT_CAPTURE_DISCOVERY_RANGE (optionally with ROS_STATIC_PEERS). Participants
    on this host with any range other than OFF still discover these processes.
    """
    result = dict(environment)
    choice = result.get(DISCOVERY_CHOICE, "LOCALHOST")
    if choice not in _DISCOVERY_RANGES:
        raise ValueError(f"{DISCOVERY_CHOICE} must be one of {', '.join(_DISCOVERY_RANGES)}")
    result["ROS_AUTOMATIC_DISCOVERY_RANGE"] = choice
    return result


def capture_process_environment(
    environment: Mapping[str, str], *, cwd: str | Path | None = None,
) -> dict[str, str]:
    """The complete environment for a capture process tree entry point."""
    return capture_discovery_environment(capture_transport_environment(environment, cwd=cwd))
