from pathlib import Path
import json
import subprocess
import sys

import pytest

from vt_capture_console.desktop_entry import install, launcher_text, main


def test_repository_desktop_icon_exists():
    root = Path(__file__).resolve().parents[3]
    content = launcher_text(root)
    icon = next(line.removeprefix("Icon=") for line in content.splitlines() if line.startswith("Icon="))
    assert Path(icon).is_file()


def repo(tmp_path):
    root = tmp_path / "带空格 repo"
    script = root / "tools/capture_deploy/start_workbench.sh"
    script.parent.mkdir(parents=True)
    script.write_text("# fixture, never executed")
    return root


def test_desktop_launcher_uses_fixed_entry_without_terminal_or_autostart(tmp_path):
    root = repo(tmp_path)
    content = launcher_text(root)
    assert 'Exec=/bin/bash "' in content
    assert "start_workbench.sh" in content
    assert "Terminal=false" in content
    assert "--execute-feature-writes" not in content and "ros2 launch" not in content
    assert "Autostart" not in content
    desktop = tmp_path / "Desktop"
    desktop.mkdir()
    paths = install(root, applications_dir=tmp_path / "applications", desktop_dir=desktop)
    assert len(paths) == 2
    assert all(path.read_text() == content for path in paths)
    assert all(path.stat().st_mode & 0o111 for path in paths)


def test_launcher_does_not_follow_user_file_symlink(tmp_path):
    root = repo(tmp_path)
    directory = tmp_path / "apps"
    directory.mkdir()
    other = tmp_path / "existing"
    other.write_text("keep")
    (directory / "visiontactile-capture.desktop").symlink_to(other)
    with pytest.raises(ValueError, match="symbolic"):
        install(root, applications_dir=directory)
    assert other.read_text() == "keep"


def test_launcher_remembers_installed_python_without_resolving_venv_symlink(tmp_path):
    root = repo(tmp_path)
    python = tmp_path / "中文 venv/bin/python"
    python.parent.mkdir(parents=True)
    python.symlink_to(sys.executable)
    content = launcher_text(root, python=python)
    assert f'Exec=/usr/bin/env "VT_CONSOLE_PYTHON={python}" /bin/bash "' in content
    assert str(python.resolve()) not in content
    assert "Terminal=false" in content


def test_installer_cli_registers_the_interpreter_used_to_install(tmp_path, monkeypatch):
    monkeypatch.setattr(Path, "home", lambda: tmp_path)
    root = repo(tmp_path)
    assert main(["--repo-root", str(root)]) == 0
    content = (tmp_path / ".local/share/applications/visiontactile-capture.desktop").read_text()
    assert f'"VT_CONSOLE_PYTHON={sys.executable}"' in content


@pytest.mark.parametrize("python", ["relative/python", "/missing/python", "/tmp/\npython"])
def test_launcher_rejects_unusable_interpreter(tmp_path, python):
    with pytest.raises(ValueError):
        launcher_text(repo(tmp_path), python=python)


@pytest.mark.parametrize("special", ["中文 空格", "a$b", "a`b", 'a"b', "a\\b", "a%b"])
def test_desktop_parser_preserves_literal_interpreter_paths(tmp_path, special):
    """Use the desktop's parser without launching an application or shell."""
    system_python = Path("/usr/bin/python3")
    if not system_python.is_file():
        pytest.skip("Linux desktop parser not available")
    root = repo(tmp_path / special)
    python = tmp_path / special / "venv/bin/python"
    python.parent.mkdir(parents=True)
    python.symlink_to(sys.executable)
    content = launcher_text(root, python=python)
    script = """
import json, sys
try:
    from gi.repository import GLib, Gio
except ImportError:
    sys.exit(77)
text = sys.stdin.read()
keyfile = GLib.KeyFile.new()
keyfile.load_from_data(text, len(text.encode()), GLib.KeyFileFlags.NONE)
command = keyfile.get_string('Desktop Entry', 'Exec')
valid, argv = GLib.shell_parse_argv(command)
assert valid
assert Gio.DesktopAppInfo.new_from_keyfile(keyfile).get_commandline() == command
print(json.dumps({'argv': argv, 'icon': keyfile.get_string('Desktop Entry', 'Icon')}))
"""
    parsed = subprocess.run([str(system_python), "-I", "-c", script], input=content,
                            capture_output=True, text=True, timeout=5)
    if parsed.returncode == 77:
        pytest.skip("GLib Python bindings not available")
    assert parsed.returncode == 0, parsed.stderr
    result = json.loads(parsed.stdout)
    # shell_parse_argv precedes the launcher's percent-field expansion.
    assert result["argv"] == [
        "/usr/bin/env", "VT_CONSOLE_PYTHON=" + str(python).replace("%", "%%"),
        "/bin/bash", str(root / "tools/capture_deploy/start_workbench.sh").replace("%", "%%"),
    ]
    assert result["icon"] == str(root / "tools/capture_console/src/vt_capture_console/theme/icons/orbit.svg")
