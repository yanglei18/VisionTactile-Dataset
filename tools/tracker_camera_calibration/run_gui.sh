#!/usr/bin/env bash
# Explicit GUI entry point for this checkout; never opens or configures devices.
set -euo pipefail

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
TOOL_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd -P)"
REPO_ROOT="$(cd -- "${TOOL_DIR}/../.." && pwd -P)"
VENV="${REPO_ROOT}/.venv-calibration"
PYTHON="${VENV}/bin/python"
GUI_ENTRY="${VENV}/bin/vt-tracker-camera-calibrate-gui"

if [[ ! -x "${PYTHON}" || ! -x "${GUI_ENTRY}" ]]; then
    printf '无法启动标定 GUI：本仓库虚拟环境或 GUI 入口尚未安装。\n' >&2
    printf '请先按 %s 的「一次性安装」安装/更新依赖。\n' "${TOOL_DIR}/README.md" >&2
    printf '预期 Python：%s\n预期 GUI 入口：%s\n' "${PYTHON}" "${GUI_ENTRY}" >&2
    exit 1
fi

# ROS setup scripts may reference unset shell variables internally.
set +u
ROS_SETUP="/opt/ros/${ROS_DISTRO:-jazzy}/setup.bash"
if [[ -f "${ROS_SETUP}" ]]; then
    source "${ROS_SETUP}"
else
    printf '提示：未找到 %s；在线预览/录制需要相应 ROS 2 环境。\n' "${ROS_SETUP}" >&2
fi
WORKSPACE_SETUP="${REPO_ROOT}/ros2_ws/install/setup.bash"
if [[ -f "${WORKSPACE_SETUP}" ]]; then
    source "${WORKSPACE_SETUP}"
else
    printf '提示：未找到 %s；自定义消息请按 README 构建后再使用在线功能。\n' "${WORKSPACE_SETUP}" >&2
fi
set -u

# The installed console script is checked above, but module execution gives
# this checkout's src precedence over another worktree's installed package.
export PYTHONPATH="${TOOL_DIR}/src${PYTHONPATH:+:${PYTHONPATH}}"
cd -- "${REPO_ROOT}"
exec "${PYTHON}" -m vt_tracker_camera_calib.gui "$@"
