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

    def build(self, bootstrap_directory=None):
        verified = SimpleNamespace(package_id=hashlib.sha256(self.payload).hexdigest())
        with patch.object(delivery, "read_package", return_value=verified):
            return delivery.build_delivery(self.root, self.package, self.output, bootstrap_directory)

    def bootstrap(self):
        sources = {}
        for name in ("tools/capture_deploy/install_full.py", "tools/capture_deploy/oneclick/engine.py",
                     "tools/capture_deploy/oneclick/delivery.py", "tools/capture_deploy/oneclick/gui.py",
                     "tools/tracker_deploy/src/vt_tracker_deploy/archive.py"):
            content = b"import sys\nprint('\\n'.join(sys.argv[1:]))\n"
            self.write(name, content)
            sources[name] = content
        directory = self.base / "bootstrap"
        directory.mkdir()
        binary = bytearray(20)
        binary[:6] = b"\x7fELF\x02\x01"
        binary[18] = 62
        payload = {delivery.BOOTSTRAP_NAME: bytes(binary), "LICENSES/runtime.txt": b"Synthetic license\n"}
        for name, content in payload.items():
            path = directory / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
        entry = lambda name, content: {"path": name, "bytes": len(content),
                                       "sha256": hashlib.sha256(content).hexdigest()}
        manifest = {
            "schema_version": 1, "kind": "visiontactile-linux-bootstrap",
            "target": dict(delivery.BOOTSTRAP_TARGET),
            "sources": [entry(name, content) for name, content in sources.items()],
            "files": [entry(name, content) for name, content in payload.items()],
        }
        self.save_bootstrap_manifest(directory, manifest)
        return directory, manifest

    def save_bootstrap_manifest(self, directory, manifest):
        (directory / "BUILD_MANIFEST.json").write_text(json.dumps(manifest), encoding="utf-8")

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

    def test_bootstrap_export_binds_binary_licenses_and_manifest_to_public_sources(self):
        directory, _ = self.bootstrap()
        self.build(directory)
        self.assertEqual(self.output.stat().st_mode & 0o777, 0o600)
        with tarfile.open(self.output, "r:gz") as archive:
            prefix = delivery.PREFIX
            manifest = json.load(archive.extractfile(prefix + "DELIVERY_MANIFEST.json"))
            bootstrap = manifest["bootstrap"]
            binary = archive.extractfile(prefix + bootstrap["path"]).read()
            self.assertEqual(bootstrap["bytes"], len(binary))
            self.assertEqual(bootstrap["sha256"], hashlib.sha256(binary).hexdigest())
            self.assertEqual(archive.getmember(prefix + bootstrap["path"]).mode, 0o755)
            self.assertEqual(archive.getmember(prefix + "LICENSES/runtime.txt").mode, 0o644)
            build_manifest = archive.extractfile(prefix + bootstrap["manifest_path"]).read()
            self.assertEqual(build_manifest, (directory / "BUILD_MANIFEST.json").read_bytes())
            self.assertEqual(bootstrap["manifest_sha256"], hashlib.sha256(build_manifest).hexdigest())
            public = json.load(archive.extractfile(prefix + delivery.SOURCE_PREFIX + "PUBLIC_SOURCE_MANIFEST.json"))
            self.assertNotIn(delivery.BOOTSTRAP_NAME, {item["path"] for item in public["files"]})
            wrapper = archive.extractfile(prefix + "install.sh").read()
            self.assertEqual(wrapper, delivery.FULL_WRAPPER)
            self.assertIn(b"--headless", wrapper)
            self.assertEqual(archive.extractfile(prefix + "README.zh-CN-en.txt").read(), delivery.FULL_README)

    def test_bootstrap_rejects_changed_source_snapshot_before_reading_private_inputs(self):
        directory, _ = self.bootstrap()
        self.write("tools/capture_deploy/oneclick/engine.py", b"# changed source\n")
        with patch.object(delivery, "read_package", side_effect=AssertionError("no private input read")):
            with self.assertRaisesRegex(delivery.DeliveryError, "sources differ"):
                delivery.build_delivery(self.root, self.package, self.output, directory)
        self.assertFalse(self.output.exists())

    def test_bootstrap_rejects_new_source_file_missing_from_build_manifest(self):
        directory, _ = self.bootstrap()
        self.write("tools/capture_deploy/oneclick/new_module.py", b"# new module\n")
        with self.assertRaisesRegex(delivery.DeliveryError, "sources differ"):
            self.build(directory)
        self.assertFalse(self.output.exists())

    def test_bootstrap_rejects_wrong_kind_schema_and_target(self):
        directory, original = self.bootstrap()
        for key, value in (("kind", "other"), ("schema_version", True),
                           ("schema_version", 2), ("target", {"system": "Windows"})):
            with self.subTest(key=key, value=value):
                manifest = dict(original)
                manifest[key] = value
                self.save_bootstrap_manifest(directory, manifest)
                with self.assertRaisesRegex(delivery.DeliveryError, "unsupported"):
                    self.build(directory)
                self.assertFalse(self.output.exists())

    def test_bootstrap_rejects_tampered_file_and_incorrect_declared_size(self):
        directory, manifest = self.bootstrap()
        original = (directory / delivery.BOOTSTRAP_NAME).read_bytes()
        (directory / delivery.BOOTSTRAP_NAME).write_bytes(original + b"modified")
        with self.assertRaisesRegex(delivery.DeliveryError, "digest or byte count"):
            self.build(directory)
        (directory / delivery.BOOTSTRAP_NAME).write_bytes(original)
        manifest["files"][0]["bytes"] += 1
        self.save_bootstrap_manifest(directory, manifest)
        with self.assertRaisesRegex(delivery.DeliveryError, "digest or byte count"):
            self.build(directory)

    def test_bootstrap_rejects_other_architecture_even_with_valid_digest(self):
        directory, manifest = self.bootstrap()
        content = bytearray((directory / delivery.BOOTSTRAP_NAME).read_bytes())
        content[18] = 183  # AArch64
        (directory / delivery.BOOTSTRAP_NAME).write_bytes(content)
        manifest["files"][0]["sha256"] = hashlib.sha256(content).hexdigest()
        self.save_bootstrap_manifest(directory, manifest)
        with self.assertRaisesRegex(delivery.DeliveryError, "ELF64"):
            self.build(directory)

    def test_bootstrap_rejects_links_extra_files_and_duplicate_entries(self):
        directory, manifest = self.bootstrap()
        license_path = directory / "LICENSES/runtime.txt"
        payload = license_path.read_bytes()
        license_path.unlink()
        license_path.symlink_to(self.package)
        with self.assertRaises(delivery.DeliveryError):
            self.build(directory)
        license_path.unlink()
        license_path.write_bytes(payload)
        extra = directory / "unlisted.txt"
        extra.write_text("unexpected")
        with self.assertRaisesRegex(delivery.DeliveryError, "file sets differ"):
            self.build(directory)
        extra.unlink()
        manifest["files"].append(manifest["files"][0])
        self.save_bootstrap_manifest(directory, manifest)
        with self.assertRaisesRegex(delivery.DeliveryError, "duplicate"):
            self.build(directory)

    def test_bootstrap_rejects_unsafe_manifest_paths(self):
        directory, manifest = self.bootstrap()
        for name in ("LICENSES/../../outside", "LICENSES//runtime.txt", "LICENSES/../runtime.txt",
                     "LICENSES/\\outside", "/outside", "unapproved.txt"):
            with self.subTest(name=name):
                manifest["files"][1]["path"] = name
                self.save_bootstrap_manifest(directory, manifest)
                with self.assertRaises(delivery.DeliveryError):
                    self.build(directory)
                self.assertFalse(self.output.exists())

    def test_bootstrap_rejects_hard_linked_payload(self):
        directory, _ = self.bootstrap()
        os.link(directory / "LICENSES/runtime.txt", self.base / "outside-license")
        with self.assertRaisesRegex(delivery.DeliveryError, "invalid type"):
            self.build(directory)
        self.assertFalse(self.output.exists())

    def test_cli_accepts_optional_bootstrap_directory(self):
        with patch.object(delivery, "build_delivery", return_value={"status": "PASS"}) as build:
            with patch("sys.stdout", new=io.StringIO()):
                result = delivery.main(["--root", str(self.root), "--tracker-package", str(self.package),
                                        "--output", str(self.output), "--bootstrap-directory", str(self.base)])
        self.assertEqual(result, 0)
        build.assert_called_once_with(self.root, self.package, self.output, self.base)

    @unittest.skipUnless(shutil.which("bash"), "Bash is required for wrapper testing")
    def test_full_wrapper_keeps_arguments_and_uses_delivery_path_with_spaces(self):
        directory, _ = self.bootstrap()
        self.build(directory)
        extracted = self.base / "含空格 full delivery"
        extracted.mkdir()
        entry = "tools/capture_deploy/install_full.py"
        with tarfile.open(self.output, "r:gz") as archive:
            for name in ("install.sh", delivery.SOURCE_PREFIX + entry):
                path = extracted / name
                path.parent.mkdir(parents=True, exist_ok=True)
                path.write_bytes(archive.extractfile(delivery.PREFIX + name).read())
        arguments = ["--help", "literal;$(not-executed)"]
        result = subprocess.run(["bash", str(extracted / "install.sh"), *arguments], cwd=self.base,
                                check=True, text=True, capture_output=True)
        self.assertEqual(result.stdout.splitlines(), ["--delivery", str(extracted), "--headless", *arguments])

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
