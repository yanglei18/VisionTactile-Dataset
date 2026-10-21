"""Versioned inputs and literal-argument commands for the Ubuntu installer."""

from dataclasses import dataclass
from pathlib import Path

from .contract import DEPENDENCIES_RELATIVE, SDK_VERSION, WRAPPER_VERSION


@dataclass(frozen=True)
class Source:
    name: str
    url: str
    tag: str
    commit: str


# Verified against the official release tags. A moved tag fails verification.
SDK = Source("librealsense", "https://github.com/realsenseai/librealsense.git",
             "v" + SDK_VERSION, "bf2778061d5dd29776e9aca8765f75852671760b")
WRAPPER = Source("realsense-ros", "https://github.com/realsenseai/realsense-ros.git",
                 WRAPPER_VERSION, "53aceeac29e2e848e5e1ede5430124f07b2fe924")
ROS_SETUP = Path("/opt/ros/jazzy/setup.bash")


@dataclass(frozen=True)
class Paths:
    home: Path
    source_root: Path

    @property
    def dependencies(self):
        return self.home / DEPENDENCIES_RELATIVE

    @property
    def sdk(self):
        return self.dependencies / ("librealsense-" + SDK_VERSION)

    @property
    def driver(self):
        return self.dependencies / ("drivers-" + WRAPPER_VERSION)

    @property
    def wrapper(self):
        return self.driver / "src/realsense-ros"

    @property
    def workspace(self):
        return self.source_root / "ros2_ws"

    @property
    def python(self):
        return self.source_root / ".venv-capture-console/bin/python"


def sdk_configure(paths):
    return ["/usr/bin/cmake", "-S", str(paths.sdk), "-B", str(paths.sdk / "build"),
            "-DCMAKE_BUILD_TYPE=Release", "-DCMAKE_INSTALL_PREFIX=/usr/local",
            "-DFORCE_RSUSB_BACKEND=ON", "-DBUILD_EXAMPLES=OFF", "-DBUILD_TOOLS=ON",
            "-DBUILD_GRAPHICAL_EXAMPLES=OFF", "-DBUILD_GLSL_EXTENSIONS=OFF",
            "-DBUILD_PYTHON_BINDINGS=OFF", "-DBUILD_WITH_DDS=OFF", "-DBUILD_ROSBAG2=OFF",
            "-DBUILD_UNIT_TESTS=OFF", "-DCHECK_FOR_UPDATES=OFF"]


def ros_command(argv, *, underlays=()):
    """Source known underlays without interpolating paths or commands into shell."""
    script = 'set -eo pipefail\nsource "$1"\nshift\n'
    for _ in underlays:
        script += 'source "$1"\nshift\n'
    script += 'exec "$@"\n'
    return ["/bin/bash", "--noprofile", "--norc", "-c", script, "vt-installer",
            str(ROS_SETUP), *(str(item) for item in underlays), *map(str, argv)]


def driver_build():
    return ros_command(["/usr/bin/colcon", "build", "--parallel-workers", "2",
                        "--packages-select", "realsense2_camera_msgs", "realsense2_description",
                        "realsense2_camera", "--event-handlers", "console_direct+", "--cmake-args",
                        "-DCMAKE_BUILD_TYPE=Release", "-DBUILD_TESTING=OFF",
                        "-DUSE_LIFECYCLE_NODE=OFF", "-DBUILD_ACCELERATE_GPU_WITH_GLSL=OFF"])


def main_build(paths):
    return ros_command(["/usr/bin/colcon", "build", "--parallel-workers", "2",
                        "--event-handlers", "console_direct+"],
                       underlays=(paths.driver / "install/setup.bash",))


RUNTIME_CHECK = '''import json, pathlib
import rclpy, rosbag2_py, cv2, numpy, yaml
from vt_camera_msgs.msg import CameraFrameTiming
from ament_index_python.packages import get_package_prefix, get_package_share_directory
import xml.etree.ElementTree as ET
version = ET.parse(pathlib.Path(get_package_share_directory('realsense2_camera')) / 'package.xml').getroot().findtext('version')
assert version == '4.58.1', 'Unexpected RealSense wrapper version'
for package in ('vt_capture_manager', 'rosbag2_storage_mcap', 'vt_vive_tracker', 'vt_tactile_capture'):
    get_package_prefix(package)
print(json.dumps({'wrapper': version, 'runtime': 'ready'}))
'''


def runtime_check(paths):
    return ros_command(["/usr/bin/python3", "-c", RUNTIME_CHECK],
                       underlays=(paths.driver / "install/setup.bash",
                                  paths.workspace / "install/setup.bash"))
