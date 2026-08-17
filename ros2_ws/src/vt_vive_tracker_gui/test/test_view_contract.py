import ast
from pathlib import Path


PACKAGE = Path(__file__).parents[1] / "vt_vive_tracker_gui"
VIEW = PACKAGE / "view.py"


def test_dashboard_source_has_fixed_roles_and_only_read_only_controls():
    source = VIEW.read_text(encoding="utf-8")

    for role in ("left_wrist", "right_wrist", "torso"):
        assert role in source
    assert "ROS 2 read-only" in source
    for control in ("俯视", "前视", "侧视", "适应全部", "重置视角"):
        assert control in source
    for forbidden in (
        "配对",
        "建图",
        "录制",
        "重置硬件",
        "启动 Publisher",
        "停止 Publisher",
    ):
        assert forbidden not in source


def test_gui_package_imports_no_hardware_or_modal_dialog_modules():
    forbidden_roots = {
        "bleak",
        "hid",
        "openvr",
        "pyopenvr",
        "serial",
        "triad_openvr",
        "usb",
    }
    violations = []

    for path in sorted(PACKAGE.glob("*.py")):
        tree = ast.parse(path.read_text(encoding="utf-8"), filename=str(path))
        for node in ast.walk(tree):
            if isinstance(node, ast.Import):
                names = tuple(alias.name for alias in node.names)
            elif isinstance(node, ast.ImportFrom):
                names = (() if node.module is None else (node.module,))
                if node.module == "tkinter":
                    names += tuple(
                        f"tkinter.{alias.name}" for alias in node.names
                    )
            else:
                continue
            for name in names:
                if (
                    name == "tkinter.messagebox"
                    or name.split(".")[0] in forbidden_roots
                ):
                    violations.append(f"{path.name}: {name}")

    assert violations == []
