#!/usr/bin/env bash
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
# sg forks and would invalidate the desktop's authenticated manager PID.  Only
# the desktop entry refreshes groups; an old desktop must be reopened first.
for arg in "$@"; do
    if [[ "$arg" == --enable-device-control ]]; then
        /usr/bin/python3 -I "$ROOT/tools/capture_deploy/serial_session.py" require
        break
    fi
done
if [[ ! -r /opt/ros/jazzy/setup.bash || ! -r "$ROOT/ros2_ws/install/setup.bash" ]]; then
    printf '%s\n' '缺少 ROS 2 Jazzy 或工作区构建结果，请先按 README 构建。' >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export PYTHONPATH="$ROOT/ros2_ws/src/vt_capture_core:$ROOT/ros2_ws/src/vt_capture_manager:$ROOT/tools/multisensor_alignment/src${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -m vt_capture_manager.node "$@"
