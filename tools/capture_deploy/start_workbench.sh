#!/usr/bin/env bash
# Desktop entry: the GUI owns backend setup; hardware starts only by explicit UI action.
set -euo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
exec /usr/bin/python3 -I "$ROOT/tools/capture_deploy/serial_session.py" launch -- \
    /bin/bash "$ROOT/tools/capture_deploy/start_console.sh" --desktop "$@"
