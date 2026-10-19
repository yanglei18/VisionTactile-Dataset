#!/usr/bin/env bash
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
PYTHON="${VT_CONSOLE_PYTHON:-$ROOT/.venv-capture-console/bin/python}"
if [[ ! -x "$PYTHON" ]]; then
    printf '%s\n' '请先运行 bash tools/capture_deploy/install.sh' >&2
    exit 1
fi
exec env -u PYTHONPATH -u PYTHONHOME -u AMENT_PREFIX_PATH -u COLCON_PREFIX_PATH \
    -u LD_LIBRARY_PATH -u QT_PLUGIN_PATH -u QT_QPA_PLATFORM_PLUGIN_PATH \
    "$PYTHON" -I -m vt_capture_console "$@"
