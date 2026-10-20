#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON="${VT_CONSOLE_PYTHON:-$ROOT/.venv-capture-console/bin/python}"
if [[ -n "${VT_CONSOLE_PYTHON:-}" ]]; then
    if [[ "$PYTHON" != /* || ! -f "$PYTHON" || ! -x "$PYTHON" ]]; then
        printf '%s\n' 'VT_CONSOLE_PYTHON 必须指向已存在、可执行的 Python 绝对路径。' >&2
        exit 2
    fi
elif [[ ! -x "$PYTHON" ]]; then
    python3 -m venv "$ROOT/.venv-capture-console"
fi
env -u PYTHONPATH -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH -u LD_LIBRARY_PATH \
    "$PYTHON" -I -m pip install -r "$ROOT/tools/capture_console/requirements-tested.txt" \
    -e "$ROOT/ros2_ws/src/vt_capture_core" -e "$ROOT/ros2_ws/src/vt_tactile_capture" \
    -e "$ROOT/tools/fsglove12" \
    -e "$ROOT/tools/capture_console"
"$PYTHON" -I -m vt_capture_console.desktop_entry --repo-root "$ROOT" --desktop
printf '%s\n' 'Qt 工作台与桌面入口已安装。未启动硬件、未注册开机服务。' \
    '在应用菜单打开 Vision Tactile 采集工作台，会自动准备本机路径并启动后台。' \
    '首次部署仍须安装 ROS 和传感器依赖；运行中无需另开终端启动后台。'
