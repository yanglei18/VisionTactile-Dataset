import ast
from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_core_imports_without_ros_qt_or_array_runtime():
    result = subprocess.run([
        sys.executable, "-c",
        "import sys; import vt_capture_core; "
        "from vt_capture_core import recipes, timebase, glove_sources; "
        "assert not any(name.startswith(('rclpy', 'PyQt', 'PySide', 'numpy', 'serial')) "
        "for name in sys.modules); "
        "assert vt_capture_core.decode_message(vt_capture_core.encode_message("
        "vt_capture_core.Request('smoke')))['command'] == 'snapshot'",
    ], cwd=PACKAGE_ROOT, text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr


def test_ament_manifest_matches_python_distribution():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    assert root.findtext("name") == "vt_capture_core"
    tree = ast.parse((PACKAGE_ROOT / "setup.py").read_text())
    setup = next(node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name) and node.func.id == "setup")
    python_version = next(ast.literal_eval(item.value) for item in setup.keywords if item.arg == "version")
    assert root.findtext("version") == python_version
    assert root.findtext("export/build_type") == "ament_python"
    assert root.findall("exec_depend") == []
    assert (PACKAGE_ROOT / "resource/vt_capture_core").exists()
    assert (PACKAGE_ROOT / "pyproject.toml").exists()


def test_python_build_type_is_not_declared_as_a_package_dependency():
    # Like the ROS demo_nodes_py manifest, ament_python is an exported build
    # type, not an installable rosdep key. Check the whole local workspace.
    for path in sorted(PACKAGE_ROOT.parent.glob("*/package.xml")):
        root = ET.parse(path).getroot()
        if root.findtext("export/build_type") != "ament_python":
            continue
        assert all(
            (element.text or "").strip() != "ament_python"
            for element in root if element.tag.endswith("depend")
        ), f"{path.parent.name}: ament_python belongs only in export/build_type"
