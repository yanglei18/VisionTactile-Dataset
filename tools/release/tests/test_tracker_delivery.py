"""Combined-delivery contracts using only synthetic sources and package bytes."""
from __future__ import annotations

from contextlib import redirect_stderr
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import tarfile
import tempfile
from types import SimpleNamespace
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "build_tracker_delivery.py"
SPEC = importlib.util.spec_from_file_location("delivery_under_test", SCRIPT)
delivery = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(delivery)

CHECKER = '''from pathlib import Path
ROOT = Path(__file__).resolve().parents[1]
MAX_TRACKED_BYTES = 5 * 1024 * 1024
REQUIRED_FILES = {"README.md", "tools/check_public_tree.py"}
EXPECTED_SUBMODULES = set()

def forbidden_path(name):
    return "private" in Path(name).parts or name.endswith((".pcapng", ".vttracker"))

def validation_errors():
    return [] if (ROOT / "README.md").is_file() else ["missing README"]
'''


@unittest.skipUnless(shutil.which("git"), "Git is required for source export")
class TrackerDeliveryTests(unittest.TestCase):
    def setUp(self):
        temporary = tempfile.TemporaryDirectory(prefix="vt-delivery-test-")
        self.addCleanup(temporary.cleanup)
        self.base = Path(temporary.name)
        self.root = self.base / "source"
        self.root.mkdir()
        self.git("init", "-q")
        self.git("config", "user.name", "Delivery Test")
        self.git("config", "user.email", "delivery-test@example.invalid")
        self.git("config", "commit.gpgsign", "false")
        self.write("README.md", b"Public source\n")
        self.write("tools/check_public_tree.py", CHECKER.encode())
        self.write(delivery.INSTALLER, b"#!/usr/bin/env bash\n# --tracker-package\nprintf '%s\\n' \"$@\"\n")
        self.git("add", ".")
        self.git("commit", "-q", "-m", "Synthetic baseline")
        self.package = self.base / "fixture.vttracker"
        self.payload = b"synthetic private package fixture"
        self.package.write_bytes(self.payload)
        self.output = self.base / "delivery.tar.gz"

    def git(self, *arguments):
        environment = {key: value for key, value in os.environ.items() if not key.startswith("GIT_")}
        environment.update(GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull)
        return subprocess.run(["git", "-C", str(self.root), *arguments], check=True,
                              stdout=subprocess.PIPE, stderr=subprocess.PIPE,
                              env=environment).stdout

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content)
        return path

    def build(self):
        verified = SimpleNamespace(package_id=hashlib.sha256(self.payload).hexdigest())
        with patch.object(delivery, "read_package", return_value=verified):
            return delivery.build_delivery(self.root, self.package, self.output)

    def test_archive_preserves_public_source_boundary_and_hashes(self):
        self.write("tools/private/identity.json", b"synthetic device identity")
        self.write("tools/private/startup.pcapng", b"synthetic capture")
        self.write("tools/new.py", b"print('new source')\n")
        before_index = (self.root / ".git/index").read_bytes()
        report = self.build()
        self.assertEqual(report["status"], "PASS")
        self.assertEqual(self.output.stat().st_mode & 0o777, 0o600)
        self.assertEqual(report["sha256"], hashlib.sha256(self.output.read_bytes()).hexdigest())
        self.assertEqual(before_index, (self.root / ".git/index").read_bytes())
        prefix = delivery.PREFIX
        with tarfile.open(self.output, "r:gz") as archive:
            names = archive.getnames()
            self.assertFalse(any(".git" in Path(name).parts or "private" in Path(name).parts for name in names))
            manifest = json.load(archive.extractfile(prefix + "DELIVERY_MANIFEST.json"))
            self.assertTrue(manifest["source"]["working_tree_dirty"])
            self.assertEqual(manifest["source"]["baseline_commit"], self.git("rev-parse", "HEAD").decode().strip())
            source_manifest = archive.extractfile(prefix + manifest["source"]["manifest_path"]).read()
            self.assertEqual(manifest["source"]["manifest_sha256"], hashlib.sha256(source_manifest).hexdigest())
            public = json.loads(source_manifest)
            public_names = {item["path"] for item in public["files"]}
            self.assertIn("tools/new.py", public_names)
            self.assertFalse(any("deployment" in name or "vttracker" in name or "private" in name for name in public_names))
            for item in public["files"]:
                content = archive.extractfile(prefix + delivery.SOURCE_PREFIX + item["path"]).read()
                self.assertEqual(hashlib.sha256(content).hexdigest(), item["sha256"])
            package = archive.getmember(prefix + manifest["tracker_package"]["path"])
            content = archive.extractfile(package).read()
            self.assertEqual(content, self.payload)
            self.assertEqual(package.mode, 0o600)
            self.assertEqual(hashlib.sha256(content).hexdigest(), manifest["tracker_package"]["sha256"])
            self.assertNotIn(str(self.base), json.dumps(manifest))

    def test_clean_worktree_is_recorded_as_clean(self):
        self.build()
        with tarfile.open(self.output, "r:gz") as archive:
            manifest = json.load(archive.extractfile(delivery.PREFIX + "DELIVERY_MANIFEST.json"))
        self.assertFalse(manifest["source"]["working_tree_dirty"])

    def test_long_and_unicode_source_paths_are_relocated_with_the_manifest(self):
        long_path = ("ros2_ws/src/vt_capture_core/vt_capture_core/"
                     "recording_and_sensor_geometry_contracts/geometry_recording.py")
        unicode_path = "tools/标定工具/示例.py"
        self.assertGreater(len(delivery.SOURCE_PREFIX + long_path), 100)
        for name in (long_path, unicode_path):
            self.write(name, b"# synthetic source\n")
        self.build()
        with tarfile.open(self.output, "r:gz") as archive:
            names = set(archive.getnames())
            self.assertTrue(all(name.startswith(delivery.PREFIX) for name in names))
            source_prefix = delivery.PREFIX + delivery.SOURCE_PREFIX
            manifest = json.load(archive.extractfile(source_prefix + "PUBLIC_SOURCE_MANIFEST.json"))
            entries = {item["path"]: item for item in manifest["files"]}
            self.assertIn(long_path, entries)
            self.assertIn(unicode_path, entries)
            for name, item in entries.items():
                expected_path = source_prefix + name
                self.assertIn(expected_path, names)
                content = archive.extractfile(expected_path).read()
                self.assertEqual(hashlib.sha256(content).hexdigest(), item["sha256"])

    def test_public_source_failure_never_reads_package_or_creates_output(self):
        (self.root / "README.md").unlink()
        with patch.object(delivery, "read_package", side_effect=AssertionError("must not read private inputs")):
            with self.assertRaisesRegex(delivery.DeliveryError, "Public source validation failed"):
                delivery.build_delivery(self.root, self.package, self.output)
        self.assertFalse(self.output.exists())

    def test_snapshot_validation_failure_does_not_create_output(self):
        with patch.object(delivery.release, "verify_snapshot", return_value=["synthetic rejection"]):
            with self.assertRaisesRegex(delivery.DeliveryError, "Public source validation failed"):
                self.build()
        self.assertFalse(self.output.exists())

    def test_corrupt_package_is_rejected_by_real_validator(self):
        with self.assertRaisesRegex(delivery.DeliveryError, "Tracker package validation failed"):
            delivery.build_delivery(self.root, self.package, self.output)
        self.assertFalse(self.output.exists())

    def test_package_change_after_validation_is_rejected(self):
        def change_after_read(_path):
            self.package.write_bytes(b"changed synthetic package")
            return SimpleNamespace(package_id=hashlib.sha256(self.payload).hexdigest())
        with patch.object(delivery, "read_package", side_effect=change_after_read):
            with self.assertRaisesRegex(delivery.DeliveryError, "package changed"):
                delivery.build_delivery(self.root, self.package, self.output)
        self.assertFalse(self.output.exists())

    def test_existing_output_is_never_overwritten(self):
        self.output.write_bytes(b"earlier delivery")
        with patch.object(delivery.release, "collect", side_effect=AssertionError("must reject before collection")):
            with self.assertRaisesRegex(delivery.DeliveryError, "already exists"):
                self.build()
        self.assertEqual(self.output.read_bytes(), b"earlier delivery")

    def test_output_must_be_outside_source_and_have_expected_extension(self):
        for output in (self.root / "delivery.tar.gz", self.base / "delivery.zip"):
            with self.subTest(output=output.name):
                self.output = output
                with self.assertRaisesRegex(delivery.DeliveryError, "outside the source repository"):
                    self.build()
                self.assertFalse(output.exists())

    def test_failed_archive_write_removes_partial_output(self):
        with patch.object(delivery, "_add", side_effect=OSError("synthetic disk failure")):
            with self.assertRaises(OSError):
                self.build()
        self.assertFalse(self.output.exists())

    def test_failed_write_does_not_delete_concurrently_replaced_output(self):
        def replace_then_fail(*_arguments):
            self.output.unlink()
            self.output.write_bytes(b"concurrent replacement")
            raise OSError("synthetic write failure")
        with patch.object(delivery, "_add", side_effect=replace_then_fail):
            with self.assertRaises(OSError):
                self.build()
        self.assertEqual(self.output.read_bytes(), b"concurrent replacement")

    def test_cli_does_not_print_private_validator_exception(self):
        stderr = io.StringIO()
        with patch.object(delivery, "read_package", side_effect=ValueError("synthetic-private-secret")):
            with redirect_stderr(stderr):
                code = delivery.main(["--root", str(self.root), "--tracker-package", str(self.package),
                                      "--output", str(self.output)])
        self.assertEqual(code, 1)
        self.assertNotIn("synthetic-private-secret", stderr.getvalue())
        self.assertNotIn(str(self.package), stderr.getvalue())

    @unittest.skipUnless(shutil.which("bash"), "Bash is required for wrapper testing")
    def test_wrapper_resolves_package_from_space_and_chinese_path(self):
        self.build()
        extracted = self.base / "含空格 delivery directory"
        extracted.mkdir()
        # Extract only the two inert scripts from this synthetic fixture.
        with tarfile.open(self.output, "r:gz") as archive:
            for name in ("install.sh", delivery.SOURCE_PREFIX + delivery.INSTALLER):
                path = extracted / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(archive.extractfile(delivery.PREFIX + name).read())
        completed = subprocess.run(["bash", str(extracted / "install.sh"), "--help"], cwd=self.base,
                                   check=True, text=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE)
        self.assertEqual(completed.stdout.splitlines(), [
            "--tracker-package", str(extracted / "deployment/tracker.vttracker"), "--help"])


if __name__ == "__main__":
    unittest.main()
