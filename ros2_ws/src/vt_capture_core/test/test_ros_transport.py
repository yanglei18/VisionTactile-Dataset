"""Transport defaults are bounded, inherited and subordinate to user choices."""
import os
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET

import pytest


def environment(base, tmp_path):
    from vt_capture_core.ros_transport import capture_transport_environment
    return capture_transport_environment(base, cwd=tmp_path)


@pytest.mark.parametrize("middleware", [None, "", "rmw_fastrtps_cpp", "rmw_fastrtps_dynamic_cpp"])
def test_default_profile_is_installed_without_selecting_middleware(tmp_path, middleware):
    base = {"PATH": "/usr/bin", "ROS_DOMAIN_ID": "217"}
    if middleware is not None:
        base["RMW_IMPLEMENTATION"] = middleware
    original = dict(base)
    result = environment(base, tmp_path)
    assert base == original
    assert result == {**base, "FASTRTPS_DEFAULT_PROFILES_FILE": result["FASTRTPS_DEFAULT_PROFILES_FILE"]}
    assert Path(result["FASTRTPS_DEFAULT_PROFILES_FILE"]).is_absolute()
    assert Path(result["FASTRTPS_DEFAULT_PROFILES_FILE"]).is_file()
    assert environment(result, tmp_path) == result


@pytest.mark.parametrize("middleware", ["rmw_cyclonedds_cpp", "rmw_connextdds", "rmw_zenoh_cpp", "custom_rmw"])
def test_other_middleware_keeps_exact_environment(tmp_path, middleware):
    base = {"RMW_IMPLEMENTATION": middleware, "OTHER": "untouched"}
    assert environment(base, tmp_path) == base


@pytest.mark.parametrize("key", [
    "FASTRTPS_DEFAULT_PROFILES_FILE", "FASTDDS_DEFAULT_PROFILES_FILE",
    "FASTDDS_BUILTIN_TRANSPORTS", "SKIP_DEFAULT_XML", "FASTDDS_ENVIRONMENT_FILE",
])
@pytest.mark.parametrize("value", ["", "operator configuration"])
def test_explicit_transport_configuration_is_never_replaced(tmp_path, key, value):
    base = {"RMW_IMPLEMENTATION": "rmw_fastrtps_cpp", key: value}
    assert environment(base, tmp_path) == base


@pytest.mark.parametrize("name", ["DEFAULT_FASTRTPS_PROFILES.xml", "DEFAULT_FASTDDS_PROFILES.xml"])
def test_working_directory_profiles_are_preserved_without_reading_them(tmp_path, name):
    (tmp_path / name).symlink_to(tmp_path / "operator-profile-not-mounted")
    assert environment({}, tmp_path) == {}


def test_middleware_qos_options_do_not_get_overridden(tmp_path):
    base = {"RMW_FASTRTPS_USE_QOS_FROM_XML": "1", "RMW_FASTRTPS_PUBLICATION_MODE": "ASYNCHRONOUS"}
    result = environment(base, tmp_path)
    assert all(result[key] == value for key, value in base.items())


def test_profile_contains_only_bounded_participant_transports(tmp_path):
    path = environment({}, tmp_path)["FASTRTPS_DEFAULT_PROFILES_FILE"]
    root = ET.parse(path).getroot()
    ns = {"d": "http://www.eprosima.com/XMLSchemas/fastRTPS_Profiles"}
    descriptors = root.findall("d:transport_descriptors/d:transport_descriptor", ns)
    by_type = {entry.findtext("d:type", namespaces=ns): entry for entry in descriptors}
    assert set(by_type) == {"SHM", "UDPv4"}
    assert int(by_type["SHM"].findtext("d:segment_size", namespaces=ns)) == 64 * 1024 * 1024
    assert int(by_type["SHM"].findtext("d:port_queue_capacity", namespaces=ns)) == 4096
    assert int(by_type["SHM"].findtext("d:maxMessageSize", namespaces=ns)) == 65500
    participant = root.find("d:participant", ns)
    assert participant.attrib["is_default_profile"] == "true"
    assert participant.findtext("d:rtps/d:useBuiltinTransports", namespaces=ns) == "false"
    assert {item.text for item in participant.findall("d:rtps/d:userTransports/d:transport_id", ns)} == {
        item.findtext("d:transport_id", namespaces=ns) for item in descriptors}
    # XML must not silently alter ROS QoS, discovery, socket buffers or logs.
    assert {item.tag.split("}")[-1] for item in root} == {"transport_descriptors", "participant"}
    assert {item.tag.split("}")[-1] for item in participant.find("d:rtps", ns)} == {
        "userTransports", "useBuiltinTransports"}


def test_profile_path_survives_two_generations_of_child_inheritance(tmp_path):
    # Use a clean transport environment even when a developer selected another DDS.
    result = {**os.environ, **environment({}, tmp_path)}
    child = "import os,pathlib; p=os.environ['FASTRTPS_DEFAULT_PROFILES_FILE']; assert pathlib.Path(p).is_file(); print(p)"
    parent = "import subprocess,sys; subprocess.run([sys.executable,'-c',sys.argv[1]],check=True)"
    run = subprocess.run([sys.executable, "-c", parent, child], env=result, cwd=tmp_path,
                         capture_output=True, text=True, timeout=10, check=True)
    assert run.stdout.strip() == result["FASTRTPS_DEFAULT_PROFILES_FILE"]
