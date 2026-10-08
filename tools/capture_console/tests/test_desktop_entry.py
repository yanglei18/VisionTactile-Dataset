from pathlib import Path

import pytest

from vt_capture_console.desktop_entry import install, launcher_text


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
