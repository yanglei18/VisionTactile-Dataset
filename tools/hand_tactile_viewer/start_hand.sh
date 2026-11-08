#!/usr/bin/env bash
# Shared source and environment checks for the standalone hand displays.
set -euo pipefail
script=$(readlink -f -- "$0")
checkout=$(cd -- "$(dirname -- "$script")/../.." && pwd -P)
hand=${1:?expected left or right}
shift
case "$hand" in
    left) module=vt_hand_tactile_viewer.left; runtime=left-hand ;;
    right) module=vt_hand_tactile_viewer; runtime=oct29-right ;;
    *) echo "Unknown hand: $hand" >&2; exit 2 ;;
esac
export VIRTUAL_ENV="$checkout/.venv-$hand-hand"
python="$VIRTUAL_ENV/bin/python"
if [[ ! -x "$python" ]]; then
    printf 'Prepare the hand display environment at %s (see tools/hand_tactile_viewer/README.md).\n' "$VIRTUAL_ENV" >&2
    exit 1
fi
export PATH="$VIRTUAL_ENV/bin:$PATH"
unset PYTHONHOME PYTHONPATH QT_PLUGIN_PATH QT_QPA_PLATFORM_PLUGIN_PATH
export PYQTGRAPH_QT_LIB=PySide6
"$python" -I - "$checkout" "$VIRTUAL_ENV" <<'PY'
from importlib.util import find_spec
from pathlib import Path
import sys

root = Path(sys.argv[1])
if Path(sys.prefix) != Path(sys.argv[2]):
    sys.exit(f"Unexpected hand display Python environment: {sys.prefix}")
for name, source in (
    ("vt_hand_tactile_viewer", "tools/hand_tactile_viewer/src"),
    ("vt_fsglove12", "tools/fsglove12/src"),
    ("vt_capture_console", "tools/capture_console/src"),
    ("vt_capture_core", "ros2_ws/src/vt_capture_core"),
    ("vt_tactile_capture", "ros2_ws/src/vt_tactile_capture"),
):
    spec = find_spec(name)
    if not spec or not spec.origin or Path(spec.origin).resolve().parent != root / source / name:
        sys.exit(f"{name} must be installed from {root / source}. Found: {getattr(spec, 'origin', None)}")
PY
exec "$python" -I -m "$module" --resources "$checkout/.runtime/$runtime" "$@"
