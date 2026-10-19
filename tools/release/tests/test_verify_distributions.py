"""Regression checks for artifact omissions and isolation; no pip/network/Qt."""
from __future__ import annotations

import importlib.util
import io
import os
from pathlib import Path
import stat
import tarfile
import tempfile
import unittest
import warnings
from unittest.mock import patch
import zipfile


SCRIPT = Path(__file__).resolve().parents[1] / "verify_distributions.py"
SPEC = importlib.util.spec_from_file_location("distribution_check", SCRIPT)
distribution = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(distribution)


class DistributionChecks(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="vt-distribution-unit-")
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.source = self.base / "source"
        package = self.source / "src" / "example"
        package.mkdir(parents=True)
        (self.source / "LICENSE").write_bytes(b"Synthetic license fixture\n")
        (self.source / "THIRD_PARTY_NOTICES.md").write_bytes(b"Required attribution fixture\n")
        (package / "__init__.py").write_text("")
        (package / "theme.qss").write_bytes(b"QWidget { color: white; }\n")
        self.contents = {"example.dist-info/licenses/LICENSE": (self.source / "LICENSE").read_bytes(),
                         "example.dist-info/licenses/THIRD_PARTY_NOTICES.md":
                             (self.source / "THIRD_PARTY_NOTICES.md").read_bytes(),
                         "example/theme.qss": (package / "theme.qss").read_bytes()}

    def wheel(self, contents):
        path = self.base / "example.whl"
        with zipfile.ZipFile(path, "w") as archive:
            for name, data in contents.items():
                archive.writestr(name, data)
        return path

    def test_valid_wheel_contains_exact_license_notice_and_assets(self):
        result = distribution.verify_artifact(self.wheel(self.contents), self.source, "example")
        self.assertEqual(result["verified_assets"], 1)
        self.assertEqual(len(result["sha256"]), 64)

    def test_omitted_or_stale_license_notice_or_resource_fails(self):
        for member in self.contents:
            for replacement in (None, b"stale content"):
                with self.subTest(member=member, replacement=replacement):
                    contents = dict(self.contents)
                    if replacement is None:
                        del contents[member]
                    else:
                        contents[member] = replacement
                    with self.assertRaisesRegex(ValueError, "omits or changes"):
                        distribution.verify_artifact(self.wheel(contents), self.source, "example")

    def test_existing_notices_required_but_not_invented(self):
        (self.source / "THIRD_PARTY_NOTICES.md").unlink()
        del self.contents["example.dist-info/licenses/THIRD_PARTY_NOTICES.md"]
        distribution.verify_artifact(self.wheel(self.contents), self.source, "example")

    def test_transport_xml_must_be_present_and_match_source(self):
        resource = self.source / "src/example/resources/transport.xml"
        resource.parent.mkdir()
        resource.write_bytes(b"<profiles />\n")
        for replacement in (None, b"<stale />"):
            contents = dict(self.contents)
            if replacement is not None:
                contents["example/resources/transport.xml"] = replacement
            with self.subTest(replacement=replacement):
                with self.assertRaisesRegex(ValueError, "package asset example/resources/transport.xml"):
                    distribution.verify_artifact(self.wheel(contents), self.source, "example")
        contents["example/resources/transport.xml"] = resource.read_bytes()
        result = distribution.verify_artifact(self.wheel(contents), self.source, "example")
        self.assertEqual(result["verified_assets"], 2)

    def test_sdist_matches_nested_package_assets_without_extraction(self):
        path = self.base / "example.tar.gz"
        with tarfile.open(path, "w:gz") as archive:
            for relative, content in self.contents.items():
                name = ("example-1/src/" + relative if relative.startswith("example/")
                        else "example-1/" + Path(relative).name)
                member = tarfile.TarInfo(name)
                member.size = len(content)
                archive.addfile(member, io.BytesIO(content))
        result = distribution.verify_artifact(path, self.source, "example")
        self.assertEqual(result["verified_assets"], 1)
        self.assertFalse((self.base / "example-1").exists())

    def test_unsafe_archive_names_are_rejected(self):
        for name in ("../escape", "/absolute", "example/../../escape"):
            with self.subTest(name=name):
                with self.assertRaisesRegex(ValueError, "Unsafe archive"):
                    distribution.verify_artifact(self.wheel({**self.contents, name: b"bad"}), self.source, "example")

    def test_duplicate_and_linked_archive_members_are_rejected(self):
        path = self.base / "duplicate.whl"
        with warnings.catch_warnings():
            warnings.simplefilter("ignore", UserWarning)
            with zipfile.ZipFile(path, "w") as archive:
                archive.writestr("repeated", "first")
                archive.writestr("repeated", "second")
        with self.assertRaisesRegex(ValueError, "Unsafe archive"):
            distribution.archive_files(path)
        linked = self.base / "linked.whl"
        with zipfile.ZipFile(linked, "w") as archive:
            info = zipfile.ZipInfo("link")
            info.external_attr = (stat.S_IFLNK | 0o777) << 16
            archive.writestr(info, "../outside")
        with self.assertRaisesRegex(ValueError, "Linked or special"):
            distribution.archive_files(linked)
        for kind in (tarfile.SYMTYPE, tarfile.LNKTYPE, tarfile.FIFOTYPE):
            path = self.base / "linked.tar.gz"
            with tarfile.open(path, "w:gz") as archive:
                info = tarfile.TarInfo("link")
                info.type, info.linkname = kind, "../outside"
                archive.addfile(info)
            with self.assertRaisesRegex(ValueError, "Linked or special"):
                distribution.archive_files(path)

    def test_ros_share_files_are_checked_separately_from_python_package(self):
        (self.source / "package.xml").write_text("<package />")
        with self.assertRaisesRegex(ValueError, "ROS share asset package.xml"):
            distribution.verify_artifact(self.wheel(self.contents), self.source, "example")
        for name in ("LICENSE", "package.xml"):
            self.contents["example.data/data/share/example/" + name] = (self.source / name).read_bytes()
        result = distribution.verify_artifact(self.wheel(self.contents), self.source, "example")
        self.assertEqual(result["verified_ros_share_assets"], 2)

    def test_inherited_python_ros_qt_and_app_settings_are_isolated(self):
        poisoned = {"PATH": "/private/venv/bin:/opt/ros/jazzy/bin", "PYTHONPATH": "/source",
                    "PYTHONHOME": "/old", "VIRTUAL_ENV": "/private/venv", "AMENT_PREFIX_PATH": "/overlay",
                    "LD_LIBRARY_PATH": "/drivers", "QT_PLUGIN_PATH": "/wrong/qt",
                    "PIP_TARGET": "/user/worktree", "QT_API": "pyqt5", "VT_WORKSTATION": "/live",
                    "GIT_INDEX_FILE": "/other/index", "GIT_DIR": "/other/git",
                    "GIT_WORK_TREE": "/other/worktree", "GIT_OBJECT_DIRECTORY": "/other/objects",
                    "DISPLAY": ":0", "DBUS_SESSION_BUS_ADDRESS": "live", "LANG": "C.UTF-8"}
        with patch.dict(os.environ, poisoned, clear=True):
            env = distribution.clean_environment(self.base / "state")
        for key in poisoned.keys() - {"PATH", "QT_API", "LANG"}:
            self.assertNotIn(key, env)
        self.assertEqual(env["PATH"], os.defpath)
        self.assertEqual(env["QT_API"], "pyside6")
        self.assertEqual(env["QT_QPA_PLATFORM"], "offscreen")
        self.assertEqual(Path(env["XDG_RUNTIME_DIR"]).stat().st_mode & 0o777, 0o700)

    def test_source_snapshot_ignores_stale_build_data_and_records_content(self):
        for name in ("build", "dist", "example.egg-info", "__pycache__", ".venv"):
            directory = self.source / name
            directory.mkdir()
            (directory / "stale").write_text("must not help build")
        target = self.base / "copied"
        distribution.snapshot_package(self.source, target)
        inventory = distribution.source_inventory(target)
        self.assertEqual([entry["path"] for entry in inventory],
                         ["LICENSE", "THIRD_PARTY_NOTICES.md", "src/example/__init__.py", "src/example/theme.qss"])
        before = inventory[-1]["sha256"]
        (target / "src/example/theme.qss").write_text("changed")
        self.assertNotEqual(distribution.source_inventory(target)[-1]["sha256"], before)
        self.assertEqual((self.source / "src/example/theme.qss").read_bytes(), self.contents["example/theme.qss"])

    def test_source_symlinks_and_special_files_are_rejected(self):
        outside = self.base / "outside"
        outside.write_text("external bytes")
        link = self.source / "link"
        link.symlink_to(outside)
        with self.assertRaisesRegex(ValueError, "Linked or special source"):
            distribution.snapshot_package(self.source, self.base / "linked-copy")
        link.unlink()
        os.mkfifo(link)
        with self.assertRaisesRegex(ValueError, "Linked or special source"):
            distribution.snapshot_package(self.source, self.base / "fifo-copy")

    def test_package_root_symlink_is_rejected_before_copying(self):
        linked_source = self.base / "linked-source"
        linked_source.symlink_to(self.source, target_is_directory=True)
        target = self.base / "root-link-copy"
        with self.assertRaisesRegex(ValueError, "real directory"):
            distribution.snapshot_package(linked_source, target)
        self.assertFalse(target.exists())

    def test_source_allowlist_excludes_git_ignored_files(self):
        (self.source / "private.txt").write_text("local only")
        target = self.base / "selected-copy"
        distribution.snapshot_package(self.source, target, allowed={"LICENSE"})
        self.assertEqual([entry["path"] for entry in distribution.source_inventory(target)], ["LICENSE"])


if __name__ == "__main__":
    unittest.main()
