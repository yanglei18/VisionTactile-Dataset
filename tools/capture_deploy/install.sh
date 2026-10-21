#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON="${VT_CONSOLE_PYTHON:-$ROOT/.venv-capture-console/bin/python}"
TRACKER_PACKAGE=""
REGISTER_DESKTOP=true
while [[ $# -gt 0 ]]; do
    case "$1" in
        --no-desktop)
            REGISTER_DESKTOP=false
            shift
            ;;
        --tracker-package)
            if [[ $# -lt 2 || -z "$2" || ! -f "$2" ]]; then
                printf '%s\n' '--tracker-package 需要一个已存在的 .vttracker 私有部署包。' >&2
                exit 2
            fi
            TRACKER_PACKAGE="$2"
            shift 2
            ;;
        --help|-h)
            printf '%s\n' '用法：bash tools/capture_deploy/install.sh [--tracker-package FILE.vttracker] [--no-desktop]'
            exit 0
            ;;
        *)
            printf '%s\n' '未知选项；使用 --help 查看安装方式。' >&2
            exit 2
            ;;
    esac
done
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
    -e "$ROOT/tools/tracker_deploy" \
    -e "$ROOT/tools/capture_console"
if [[ -n "$TRACKER_PACKAGE" ]]; then
    "$PYTHON" -I "$ROOT/tools/tracker_deploy/manage.py" install --package "$TRACKER_PACKAGE"
fi
if [[ "$REGISTER_DESKTOP" == false ]]; then
    printf '%s\n' '工作台 Python 与所选 Tracker 依赖已安装；桌面入口等待完整环境验证后注册。'
    exit 0
fi
"$PYTHON" -I -m vt_capture_console.desktop_entry --repo-root "$ROOT" --desktop
printf '%s\n' 'Qt 工作台与桌面入口已安装。未启动硬件、未注册开机服务。' \
    '在应用菜单打开 Vision Tactile 采集工作台，会自动准备本机路径并启动后台。' \
    '首次部署仍须安装 ROS 和传感器依赖；运行中无需另开终端启动后台。'
