#!/usr/bin/env bash
set -eo pipefail
root="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
if [[ ! -r /opt/ros/jazzy/setup.bash || ! -r "$root/ros2_ws/install/setup.bash" ]]; then
    echo '请先构建 ROS 工作区，并安装 ROS 2 Jazzy。' >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$root/ros2_ws/install/setup.bash"
export PYTHONPATH="$root/ros2_ws/src/vt_capture_manager${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m vt_capture_manager.optical.viewer "$@"
