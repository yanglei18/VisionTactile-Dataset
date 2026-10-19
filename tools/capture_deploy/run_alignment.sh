#!/usr/bin/env bash
# Fixed offline entry point. No sensors, pairing or recording commands.
set -eo pipefail
ROOT="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")/../.." && pwd)"
case "${1:-}" in
    inspect|align|validate|--version|--help|-h) ;;
    *) printf '%s\n' '仅支持离线检查、对齐与结果验证。' >&2; exit 2 ;;
esac
if [[ ! -r /opt/ros/jazzy/setup.bash || ! -r "$ROOT/ros2_ws/install/setup.bash" ]]; then
    printf '%s\n' '离线工具所需运行环境尚未安装完整，请联系工作站维护人员。' >&2
    exit 1
fi
source /opt/ros/jazzy/setup.bash
source "$ROOT/ros2_ws/install/setup.bash"
export PYTHONPATH="$ROOT/tools/multisensor_alignment/src${PYTHONPATH:+:$PYTHONPATH}"
exec /usr/bin/python3 -u -m vt_multisensor_alignment "$@"
