"""Release boundaries against disposable repositories, never the real index.

Run with ``python3 -m unittest discover -s tools/release/tests -v``.
The fixture checker is intentionally small: these tests exercise export and
isolation, while the repository's real public-tree contract is tested separately.
"""
from __future__ import annotations

from contextlib import redirect_stderr, redirect_stdout
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path
import shutil
import subprocess
import sys
import tarfile
import tempfile
import unittest
from unittest.mock import patch


SCRIPT = Path(__file__).resolve().parents[1] / "prepare_release.py"
SPEC = importlib.util.spec_from_file_location("release_under_test", SCRIPT)
release = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(release)

CHECKER = '''from pathlib import Path

ROOT = Path(__file__).resolve().parents[1]
MAX_TRACKED_BYTES = 5 * 1024 * 1024
REQUIRED_FILES = {"README.md", "tools/check_public_tree.py"}
EXPECTED_SUBMODULES = {"pyvut"}


def forbidden_path(name):
    parts = name.split("/")
    return (any(p in {".local", ".runtime", "private", "Window_reference", "__pycache__",
                     "artifacts", "bags"} or p.startswith(".venv") for p in parts)
            or name.startswith("docs/superpowers/")
            or name.endswith((".pcap", ".pcapng", ".bag", ".mcap", ".pkl", ".pyc")))


def git(*args):
    raise RuntimeError("checker Git must be rebound to the temporary snapshot")


def validation_errors():
    errors, names = [], set()
    for record in git("ls-files", "--stage", "-z").split("\\0"):
        if not record:
            continue
        metadata, name = record.split("\\t", 1)
        mode, _, stage = metadata.split()
        names.add(name)
        if stage != "0" or forbidden_path(name):
            errors.append("invalid snapshot entry: " + name)
        if mode != "160000":
            if git("show", ":" + name).encode() != (ROOT / name).read_bytes():
                errors.append("snapshot index differs from bytes: " + name)
    errors.extend("missing snapshot entry: " + n for n in REQUIRED_FILES - names)
    return errors
'''


@unittest.skipUnless(shutil.which("git"), "Git is required for source export")
class PrepareReleaseTests(unittest.TestCase):
    def setUp(self):
        self.temporary = tempfile.TemporaryDirectory(prefix="vt-release-test-")
        self.addCleanup(self.temporary.cleanup)
        self.base = Path(self.temporary.name)
        self.root = self.base / "source"
        self.root.mkdir()
        self.git("init", "-q")
        self.git("config", "user.name", "Release Test")
        self.git("config", "user.email", "release-test@example.invalid")
        self.git("config", "commit.gpgsign", "false")
        self.git("config", "core.autocrlf", "false")
        self.git("config", "core.whitespace", "trailing-space,space-before-tab,blank-at-eof")
        self.write("README.md", "Initial source\n")
        self.write("tools/check_public_tree.py", CHECKER)
        self.write(".gitignore", "tools/ignored.py\n")
        self.git("add", "--", "README.md", "tools/check_public_tree.py", ".gitignore")
        self.git("commit", "-q", "-m", "Synthetic baseline")

    def git(self, *args):
        # Even a caller with real-workspace GIT_* overrides can run this suite.
        env = {key: value for key, value in os.environ.items() if not key.startswith("GIT_")}
        env.update(LC_ALL="C", GIT_CONFIG_NOSYSTEM="1", GIT_CONFIG_GLOBAL=os.devnull)
        return subprocess.run(
            ["git", "-C", str(self.root), *args], check=True,
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env,
        ).stdout

    def write(self, name, content):
        path = self.root / name
        path.parent.mkdir(parents=True, exist_ok=True)
        path.write_bytes(content.encode() if isinstance(content, str) else content)
        return path

    def collect(self):
        return release.collect(self.root)

    def report(self, command="check", *args):
        stream = io.StringIO()
        with redirect_stdout(stream):
            status = release.main([command, "--root", str(self.root), *args])
        return status, json.loads(stream.getvalue()), stream.getvalue()

    def index_and_objects(self):
        index = (self.root / ".git/index").read_bytes()
        objects = {
            str(path.relative_to(self.root / ".git/objects")): path.read_bytes()
            for path in (self.root / ".git/objects").rglob("*") if path.is_file()
        }
        return index, objects

    def test_collect_uses_current_tracked_and_untracked_content(self):
        self.write("README.md", "Staged source\n")
        self.git("add", "--", "README.md")
        self.write("README.md", "Current unstaged source\n").chmod(0o755)
        self.write("tools/new.py", "print('new source')\n")
        self.write("tools/ignored.py", "ignored source\n")
        self.write("tools/entry.py", "#!/usr/bin/env python3\nprint('entry')\n")
        files, links, excluded, errors = self.collect()
        self.assertEqual(errors, [])
        self.assertEqual(links, {})
        self.assertEqual(files["README.md"], (b"Current unstaged source\n", 0o644))
        self.assertIn("tools/new.py", files)
        self.assertEqual(files["tools/entry.py"][1], 0o755)
        self.assertNotIn("tools/ignored.py", files)
        self.assertNotIn("tools/ignored.py", excluded)

    def test_deleted_tracked_source_is_not_resurrected(self):
        path = self.write("tools/deleted.py", "old source\n")
        self.git("add", "--", "tools/deleted.py")
        path.unlink()
        files, _, excluded, errors = self.collect()
        self.assertEqual(errors, [])
        self.assertNotIn("tools/deleted.py", files)
        self.assertIn("tools/deleted.py", excluded)

    def test_private_and_out_of_scope_paths_are_excluded(self):
        names = (
            ".local/credentials.json", ".runtime/frame.json", "tools/private/config.json",
            "tools/.venv-test/config.py", "docs/superpowers/plan.md",
            "Window_reference/application.py", "scratch.py", "photo.jpg",
            "tools/model/MANO_RIGHT.pkl", "bags/session.mcap",
        )
        for name in names:
            self.write(name, "private fixture\n")
        files, _, excluded, errors = self.collect()
        self.assertEqual(errors, [])
        for name in names:
            self.assertNotIn(name, files)
            self.assertIn(name, excluded)

    def test_secret_report_contains_path_but_not_source(self):
        # Deliberately synthetic pattern; never uses any real credential.
        token = "ghp_" + "X" * 40
        self.write("tools/leak.py", "fixture_token = '" + token + "'\n")
        status, report, output = self.report()
        self.assertEqual(status, 1)
        self.assertEqual(report["public_source_check"], "FAIL")
        self.assertIn("possible credential; review privately: tools/leak.py", report["errors"])
        self.assertNotIn(token, output)
        self.assertNotIn("fixture_token", output)

    def test_file_and_broken_symlinks_are_rejected(self):
        outside = self.base / "outside.txt"
        outside.write_text("not public\n")
        for name, target in (("tools/linked.py", outside),
                             ("tools/missing.py", self.base / "missing-target")):
            (self.root / name).symlink_to(target)
        files, _, _, errors = self.collect()
        for name in ("tools/linked.py", "tools/missing.py"):
            self.assertNotIn(name, files)
            self.assertIn("symlink not exportable: " + name, errors)

    def test_tracked_file_beneath_replaced_directory_symlink_is_rejected(self):
        path = self.write("tools/linked_dir/child.py", "tracked source\n")
        self.git("add", "--", "tools/linked_dir/child.py")
        path.unlink()
        path.parent.rmdir()
        outside = self.base / "outside-dir"
        outside.mkdir()
        (outside / "child.py").write_text("not public\n")
        path.parent.symlink_to(outside, target_is_directory=True)
        files, _, _, errors = self.collect()
        self.assertNotIn("tools/linked_dir/child.py", files)
        self.assertTrue(any(error.startswith("symlink not exportable: tools/linked_dir")
                            for error in errors))

    @unittest.skipUnless(hasattr(os, "mkfifo"), "POSIX FIFO test")
    def test_fifo_is_rejected_without_blocking_on_a_read(self):
        path = self.write("tools/input.py", "ordinary source\n")
        self.git("add", "--", "tools/input.py")
        path.unlink()
        os.mkfifo(path)
        env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
        completed = subprocess.run(
            [sys.executable, str(SCRIPT), "check", "--root", str(self.root)],
            stdout=subprocess.PIPE, stderr=subprocess.PIPE, text=True, timeout=10, env=env,
        )
        self.assertEqual(completed.returncode, 1)
        report = json.loads(completed.stdout)
        self.assertIn("non-regular source file: tools/input.py", report["errors"])

    def test_oversized_source_is_rejected_before_reading(self):
        path = self.root / "tools/large.py"
        with path.open("wb") as stream:
            stream.truncate(5 * 1024 * 1024 + 1)
        files, _, _, errors = self.collect()
        self.assertNotIn("tools/large.py", files)
        self.assertIn("source file exceeds 5 MiB: tools/large.py", errors)

    def test_snapshot_checks_captured_bytes_without_changing_original_repository(self):
        self.write("tools/new.py", "untracked current source\n")
        files, links, _, errors = self.collect()
        self.assertEqual(errors, [])
        before = self.index_and_objects()
        self.write("README.md", "Changed after capture; not the snapshot\n")
        self.assertEqual(release.verify_snapshot(files, links), [])
        self.assertTrue(self.index_and_objects() == before,
                        "snapshot validation must not change original index or object store")
        self.assertEqual((self.root / "README.md").read_text(),
                         "Changed after capture; not the snapshot\n")
        self.assertNotIn(b"tools/new.py", self.git("ls-files", "-z"))

    def test_snapshot_ignores_inherited_source_git_index_and_object_store(self):
        self.write("README.md", "Staged bad whitespace   \n")
        self.git("add", "--", "README.md")
        self.write("README.md", "Current source without whitespace defects\n")
        files, links, _, errors = self.collect()
        self.assertEqual(errors, [])
        before = self.index_and_objects()
        overrides = {
            "GIT_INDEX_FILE": str(self.root / ".git/index"),
            "GIT_OBJECT_DIRECTORY": str(self.root / ".git/objects"),
            "GIT_DIR": str(self.root / ".git"),
            "GIT_WORK_TREE": str(self.root),
        }
        with patch.dict(os.environ, overrides):
            self.assertEqual(release.verify_snapshot(files, links), [])
        self.assertTrue(self.index_and_objects() == before,
                        "inherited Git variables must not mutate the source repository")

    def test_archive_has_source_manifest_pinned_gitlink_but_no_git_or_submodule_source(self):
        revision = self.git("rev-parse", "HEAD").decode().strip()
        self.write(".gitmodules", '[submodule "pyvut"]\n\tpath = pyvut\n'
                   '\turl = https://example.invalid/fixture.git\n')
        self.git("update-index", "--add", "--cacheinfo", f"160000,{revision},pyvut")
        self.write("pyvut/local_only.py", "private submodule source\n")
        files, links, _, errors = self.collect()
        self.assertEqual(errors, [])
        self.assertEqual(links, {"pyvut": revision})
        self.assertEqual(release.verify_snapshot(files, links), [])
        archive_path = self.base / "public.tar.gz"
        release.export_archive(archive_path, files, links)
        prefix = "Vision-Tactile-Dataset/"
        with tarfile.open(archive_path, "r:gz") as archive:
            names = archive.getnames()
            self.assertFalse(any(".git" in Path(name).parts for name in names))
            self.assertFalse(any(name.startswith(prefix + "pyvut/") for name in names))
            self.assertIn(prefix + ".gitmodules", names)
            manifest = json.load(archive.extractfile(prefix + "PUBLIC_SOURCE_MANIFEST.json"))
            self.assertEqual(manifest["schema_version"], 1)
            self.assertFalse(manifest["history_included"])
            self.assertFalse(manifest["submodule_sources_included"])
            self.assertEqual(manifest["submodules"], {"pyvut": revision})
            manifest_files = {item["path"]: item for item in manifest["files"]}
            self.assertEqual(set(manifest_files), set(files))
            for name, (content, mode) in files.items():
                member = archive.getmember(prefix + name)
                self.assertTrue(member.isfile())
                self.assertEqual(member.mode, mode)
                self.assertEqual(archive.extractfile(member).read(), content)
                self.assertEqual(manifest_files[name]["bytes"], len(content))
                self.assertEqual(manifest_files[name]["sha256"], hashlib.sha256(content).hexdigest())

    def test_export_refuses_to_overwrite_an_existing_delivery(self):
        path = self.base / "already-exists.tar.gz"
        original = b"previous delivery must remain unchanged"
        path.write_bytes(original)
        files, links, _, errors = self.collect()
        self.assertEqual(errors, [])
        with self.assertRaises(FileExistsError):
            release.export_archive(path, files, links)
        self.assertEqual(path.read_bytes(), original)

    def test_report_file_matches_the_printed_report(self):
        path = self.base / "source-check.json"
        status, report, output = self.report("check", "--report", str(path))
        self.assertEqual(status, 0)
        self.assertEqual(report["public_source_check"], "PASS")
        self.assertEqual(json.loads(path.read_text()), report)
        self.assertEqual(path.read_text(), output)

    def test_cli_rejects_existing_output_or_report_before_inspecting_source(self):
        for command, option, name in (("check", "--report", "old-report.json"),
                                      ("export", "--output", "old-delivery.tar.gz")):
            with self.subTest(option=option):
                path = self.base / name
                original = b"existing local artifact"
                path.write_bytes(original)
                stderr = io.StringIO()
                with patch.object(release, "collect", side_effect=AssertionError("should not inspect")):
                    with redirect_stderr(stderr), self.assertRaises(SystemExit) as raised:
                        self.report(command, option, str(path))
                self.assertEqual(raised.exception.code, 2)
                self.assertIn("already exists; use a new path", stderr.getvalue())
                self.assertEqual(path.read_bytes(), original)

    def test_historical_capture_blocks_history_but_clean_snapshot_can_pass_and_export(self):
        path = self.write("tools/old-session.pcapng", b"synthetic capture fixture")
        self.git("add", "--", "tools/old-session.pcapng")
        self.git("commit", "-q", "-m", "Synthetic old capture")
        path.unlink()
        self.git("add", "-u", "--", "tools/old-session.pcapng")
        self.git("commit", "-q", "-m", "Remove capture from current tree")
        before = self.index_and_objects()
        archive_path = self.base / "history-free.tar.gz"
        status, report, _ = self.report("export", "--output", str(archive_path))
        self.assertEqual(status, 0)
        self.assertEqual(report["public_source_check"], "PASS")
        self.assertEqual(report["history_path_check"], "BLOCKED")
        self.assertEqual(report["history_risk_paths"], ["tools/old-session.pcapng"])
        with tarfile.open(archive_path, "r:gz") as archive:
            self.assertFalse(any(name.endswith(".pcapng") for name in archive.getnames()))
        self.assertTrue(self.index_and_objects() == before,
                        "export must not rewrite original index or captured history")


if __name__ == "__main__":
    unittest.main()
