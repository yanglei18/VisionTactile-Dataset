#!/usr/bin/env bash
# Desktop entry: the GUI owns backend setup; hardware starts only by explicit UI action.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
# Use the desktop's X11/Xwayland connection when available. The workstation's
# native Wayland/EGL path can keep software-rendering without a usable window.
# An explicit platform choice and Wayland-only sessions retain their selection.
if [[ -z "${QT_QPA_PLATFORM:-}" && -n "${DISPLAY:-}" ]]; then
    export QT_QPA_PLATFORM=xcb
fi
exec /usr/bin/python3 -I "$ROOT/tools/capture_deploy/serial_session.py" launch -- \
    /bin/bash "$ROOT/tools/capture_deploy/start_console.sh" --desktop "$@"
