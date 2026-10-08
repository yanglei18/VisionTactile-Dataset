from pathlib import Path
import subprocess
import sys
import xml.etree.ElementTree as ET


PACKAGE_ROOT = Path(__file__).resolve().parents[1]


def test_core_imports_without_ros_qt_or_array_runtime():
    result = subprocess.run([
        sys.executable, "-c",
        "import sys; import vt_capture_core; "
        "from vt_capture_core import recipes, timebase; "
        "assert not any(name.startswith(('rclpy', 'PyQt', 'PySide', 'numpy', 'serial')) "
        "for name in sys.modules); "
        "assert vt_capture_core.decode_message(vt_capture_core.encode_message("
        "vt_capture_core.Request('smoke')))['command'] == 'snapshot'",
    ], cwd=PACKAGE_ROOT, text=True, capture_output=True, check=False)
    assert result.returncode == 0, result.stderr


def test_ament_manifest_matches_python_distribution():
    root = ET.parse(PACKAGE_ROOT / "package.xml").getroot()
    assert root.findtext("name") == "vt_capture_core"
    assert root.findtext("version") == "0.5.0"
    assert root.findtext("export/build_type") == "ament_python"
    assert root.findall("exec_depend") == []
    assert (PACKAGE_ROOT / "resource/vt_capture_core").exists()
    assert (PACKAGE_ROOT / "pyproject.toml").exists()
