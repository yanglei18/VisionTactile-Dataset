import ast
from pathlib import Path


ROOT = Path(__file__).parents[1]
LAUNCH_PATH = ROOT / "launch" / "triple_tracker.launch.py"
EXPECTED_ARGUMENTS = {
    "bundle_path",
    "role_map_path",
    "frame_id",
    "status_rate_hz",
    "read_timeout_ms",
    "disconnect_timeout_ms",
    "queue_capacity",
}


def string_argument(call):
    if (
        isinstance(call.func, ast.Name)
        and call.func.id == "DeclareLaunchArgument"
        and call.args
        and isinstance(call.args[0], ast.Constant)
    ):
        return call.args[0].value
    return None


def test_launch_declares_exactly_the_read_only_runtime_arguments():
    tree = ast.parse(LAUNCH_PATH.read_text(encoding="utf-8"))
    arguments = {
        value
        for node in ast.walk(tree)
        if isinstance(node, ast.Call)
        if (value := string_argument(node)) is not None
    }

    assert arguments == EXPECTED_ARGUMENTS
    assert all(
        token not in LAUNCH_PATH.read_text(encoding="utf-8").lower()
        for token in ("bootstrap", "feature_write", "pair_device")
    )


def test_launch_targets_the_installed_tracker_node():
    source = LAUNCH_PATH.read_text(encoding="utf-8")
    assert "package='vt_vive_tracker'" in source
    assert "executable='vt_vive_tracker_node'" in source


def test_public_runbook_uses_ros2_run_and_exports_hid_environment():
    runbook = (ROOT.parents[2] / "docs/tracker-ros2-publisher.md").read_text(
        encoding="utf-8"
    )
    assert (
        "ros2 run vt_vive_tracker vt-vive-write-role-map" in runbook
    )
    assert (
        "ros2 run vt_vive_tracker vt-vive-validate-topics" in runbook
    )
    assert 'export VT_PYVUT_SITE="$(' in runbook
    assert "site.getsitepackages()[0]" in runbook
    assert (
        'export PYTHONPATH="${VT_PYVUT_ROOT}:'
        '${VT_PYVUT_SITE}:${PYTHONPATH:-}"' in runbook
    )
