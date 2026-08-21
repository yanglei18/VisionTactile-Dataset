#!/usr/bin/env python3
from __future__ import annotations

import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "tools/check_public_tree.py"


class AlternateIndexTests(unittest.TestCase):
    def setUp(self) -> None:
        self.temp_dir = tempfile.TemporaryDirectory()
        self.addCleanup(self.temp_dir.cleanup)
        self.env = os.environ.copy()
        self.env["GIT_INDEX_FILE"] = str(
            Path(self.temp_dir.name) / "alternate-index"
        )
        self.git("read-tree", "HEAD")
        required_worktree_files = (
            "README.md",
            "README.zh-CN.md",
            "THIRD_PARTY_NOTICES.md",
            "docs/capture-guide.md",
            "docs/hardware-reference.md",
            "docs/interface-reference.md",
            "docs/release-checklist.md",
            "docs/tracker-linux-validation.md",
            "docs/tracker-camera-calibration.md",
            "docs/tracker-ros2-publisher.md",
            "docs/troubleshooting.md",
            "docs/user-manual.md",
            "ros2_ws/src/vt_vive_tracker_gui/README.md",
            "tools/tracker_camera_calibration/README.md",
            "tools/tracker_camera_calibration/config/calibration.example.yaml",
            "tools/tracker_camera_calibration/pyproject.toml",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/__init__.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/bag_reader.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/charuco.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/cli.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/config.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/config_writer.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/export.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/handeye.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/model.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/pairing.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/repeatability.py",
            "tools/tracker_camera_calibration/src/vt_tracker_camera_calib/transforms.py",
            "tools/tracker_camera_calibration/tests/test_charuco.py",
            "tools/tracker_camera_calibration/tests/test_config_and_bag_helpers.py",
            "tools/tracker_camera_calibration/tests/test_config_writer.py",
            "tools/tracker_camera_calibration/tests/test_export.py",
            "tools/tracker_camera_calibration/tests/test_handeye.py",
            "tools/tracker_camera_calibration/tests/test_pairing.py",
            "tools/tracker_camera_calibration/tests/test_repeatability.py",
            "tools/tracker_camera_calibration/tests/test_transforms.py",
        ) + tuple(
            str(path.relative_to(ROOT))
            for path in sorted((ROOT / "tools/multisensor_alignment").rglob("*"))
            if path.is_file() and "__pycache__" not in path.parts
        )
        for path in required_worktree_files:
            worktree_file = ROOT / path
            if worktree_file.is_file():
                self.add_index_blob(
                    path,
                    worktree_file.read_text(encoding="utf-8"),
                )

    def git(
        self,
        *args: str,
        input_text: str | None = None,
    ) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            ["git", "-C", str(ROOT), *args],
            check=True,
            env=self.env,
            input=input_text,
            text=True,
            stdout=subprocess.PIPE,
        )

    def add_index_blob(self, path: str, text: str = "ghost\n") -> None:
        object_id = self.git(
            "hash-object",
            "-w",
            "--stdin",
            input_text=text,
        ).stdout.strip()
        self.git(
            "update-index",
            "--add",
            "--cacheinfo",
            "100644",
            object_id,
            path,
        )

    def run_checker(self) -> subprocess.CompletedProcess[str]:
        return subprocess.run(
            [sys.executable, str(CHECKER)],
            cwd=ROOT,
            env=self.env,
            text=True,
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
        )

    def test_required_worktree_file_must_be_in_index(self) -> None:
        self.assertTrue((ROOT / "LICENSE").is_file())
        self.git("update-index", "--force-remove", "--", "LICENSE")

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn("missing tracked file: LICENSE", result.stderr)

    def test_tracker_windows_runbook_is_required(self) -> None:
        self.git(
            "update-index",
            "--force-remove",
            "--",
            "docs/tracker-windows-map.md",
        )

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            "missing tracked file: docs/tracker-windows-map.md",
            result.stderr,
        )

    def test_tracker_ros2_runbook_is_required(self) -> None:
        self.git(
            "update-index",
            "--force-remove",
            "--",
            "docs/tracker-ros2-publisher.md",
        )

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            "missing tracked file: docs/tracker-ros2-publisher.md",
            result.stderr,
        )

    def test_tracker_camera_calibration_runbook_is_required(self) -> None:
        self.git(
            "update-index",
            "--force-remove",
            "--",
            "docs/tracker-camera-calibration.md",
        )

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            "missing tracked file: docs/tracker-camera-calibration.md",
            result.stderr,
        )

    def test_multisensor_alignment_manual_is_required(self) -> None:
        self.git(
            "update-index",
            "--force-remove",
            "--",
            "tools/multisensor_alignment/README.md",
        )

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            "missing tracked file: tools/multisensor_alignment/README.md",
            result.stderr,
        )

    def test_aligned_dataset_sdk_source_is_required(self) -> None:
        path = (
            "tools/multisensor_alignment/src/"
            "vt_multisensor_alignment/dataset.py"
        )
        self.git("update-index", "--force-remove", "--", path)

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(f"missing tracked file: {path}", result.stderr)

    def test_alignment_manual_keeps_validation_command(self) -> None:
        path = "tools/multisensor_alignment/README.md"
        manual = (ROOT / path).read_text(encoding="utf-8")
        token = "vt-multisensor-align validate"
        self.assertIn(token, manual)
        self.add_index_blob(path, manual.replace(token, "removed-command"))

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(f"{path} is missing required text: {token}", result.stderr)

    def test_product_calibration_manual_keeps_repeatability_command(self) -> None:
        path = "tools/tracker_camera_calibration/README.md"
        runbook = (ROOT / path).read_text(encoding="utf-8")
        token = "vt-tracker-camera-calibrate compare"
        self.assertIn(token, runbook)
        self.add_index_blob(path, runbook.replace(token, "removed-command"))

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(f"{path} is missing required text: {token}", result.stderr)

    def test_product_calibration_manual_excludes_private_startup(self) -> None:
        path = "tools/tracker_camera_calibration/README.md"
        runbook = (ROOT / path).read_text(encoding="utf-8")
        token = "live_windows_bootstrap.py"
        self.assertNotIn(token, runbook)
        self.add_index_blob(path, runbook + f"\n{token}\n")

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(f"{path} contains forbidden text: {token}", result.stderr)

    def test_tracker_ros2_runbook_keeps_acceptance_command(self) -> None:
        path = "docs/tracker-ros2-publisher.md"
        runbook = (ROOT / path).read_text(encoding="utf-8")
        token = "vt-vive-validate-topics"
        self.assertIn(token, runbook)
        self.add_index_blob(path, runbook.replace(token, "removed-command"))

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(f"{path} is missing required text: {token}", result.stderr)

    def test_forbidden_index_entry_is_rejected_when_absent_from_worktree(
        self,
    ) -> None:
        ghost_path = "artifacts/ghost.txt"
        self.assertFalse((ROOT / ghost_path).exists())
        license_blob = self.git("rev-parse", "HEAD:LICENSE").stdout.strip()
        self.git(
            "update-index",
            "--add",
            "--cacheinfo",
            "100644",
            license_blob,
            ghost_path,
        )

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(f"forbidden tracked path: {ghost_path}", result.stderr)

    def test_every_forbidden_directory_component_is_rejected(self) -> None:
        ghost_paths = (
            "build/ghost.txt",
            "install/ghost.txt",
            "log/ghost.txt",
            "pkg/artifacts/ghost.txt",
            "pkg/bags/ghost.txt",
            "pkg/__pycache__/x.pyc",
            ".pytest_cache/v/cache/nodeids",
            "pkg/.cache/state.json",
            "pkg/.worktrees/branch/file.txt",
            ".superpowers/internal.md",
            ".idea/workspace.xml",
            ".vscode/settings.json",
            ".venv-calibration/bin/python",
            "docs/superpowers/internal.md",
        )

        for ghost_path in ghost_paths:
            with self.subTest(path=ghost_path):
                self.git("read-tree", "HEAD")
                self.add_index_blob(ghost_path)

                result = self.run_checker()

                self.assertEqual(
                    result.returncode,
                    1,
                    result.stdout + result.stderr,
                )
                self.assertIn(
                    f"forbidden tracked path: {ghost_path}",
                    result.stderr,
                )

    def test_forbidden_artifact_names_and_suffixes_are_rejected(self) -> None:
        ghost_paths = (
            ".DS_Store",
            "pkg/.DS_Store",
            "rs-save-to-disk-output-Color.png",
            "ghost.bag",
            "ghost.db3",
            "ghost.mcap",
            "ghost.pcap",
            "ghost.pcapng",
            "ghost.webm",
            "ghost.zip",
            "ghost.pyc",
            "ghost.pyo",
            "ghost.pyd",
            "captures.bag/frame.txt",
            "cache.pyc/value",
            "pkg/.DS_Store/inside.txt",
            "pkg/rs-save-to-disk-output-Color/metadata.txt",
        )

        for ghost_path in ghost_paths:
            with self.subTest(path=ghost_path):
                self.git("read-tree", "HEAD")
                self.add_index_blob(ghost_path)

                result = self.run_checker()

                self.assertIn(
                    f"forbidden tracked artifact: {ghost_path}",
                    result.stderr,
                )
                self.assertEqual(
                    result.returncode,
                    1,
                    result.stdout + result.stderr,
                )

    def test_broken_relative_markdown_link_is_rejected(self) -> None:
        path = "docs/user-manual.md"
        manual = (ROOT / path).read_text(encoding="utf-8")
        self.add_index_blob(path, f"{manual}\n[missing](does-not-exist.md)\n")

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            f"broken relative Markdown link: {path}: does-not-exist.md",
            result.stderr,
        )

    def test_document_tokens_are_read_from_index_blob(self) -> None:
        readme = self.git("show", "HEAD:README.md").stdout
        self.assertIn("Ubuntu 24.04", readme)
        invalid_readme = readme.replace("Ubuntu 24.04", "Ubuntu 22.04")
        self.assertNotIn("Ubuntu 24.04", invalid_readme)
        self.add_index_blob("README.md", invalid_readme)

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            "README.md is missing required text: Ubuntu 24.04",
            result.stderr,
        )

    def test_qos_depth_is_required_from_architecture_index_blob(self) -> None:
        architecture = self.git("show", "HEAD:docs/architecture.md").stdout
        qos_token = "keep-last depth 30"
        if qos_token not in architecture:
            architecture = architecture.replace(
                "keep-last QoS overrides",
                f"{qos_token} QoS overrides",
            )
        self.assertIn(qos_token, architecture)
        invalid_documents = (
            ("removed", architecture.replace(qos_token, "keep-last")),
            ("changed", architecture.replace("depth 30", "depth 10")),
        )

        for case, invalid_document in invalid_documents:
            with self.subTest(case=case):
                self.git("read-tree", "HEAD")
                self.add_index_blob("docs/architecture.md", invalid_document)

                result = self.run_checker()

                self.assertEqual(
                    result.returncode,
                    1,
                    result.stdout + result.stderr,
                )
                self.assertIn(
                    "docs/architecture.md is missing required text: "
                    f"{qos_token}",
                    result.stderr,
                )

    def test_obsolete_document_text_is_rejected_from_index_blob(self) -> None:
        readme = self.git("show", "HEAD:README.md").stdout
        obsolete_tokens = (
            "29-topic",
            "29 个 topic",
            "bag_validate",
            "message-Zstd",
            "EOF validator",
        )
        clean_readme = readme
        for token in obsolete_tokens:
            clean_readme = clean_readme.replace(token, "current contract")

        for obsolete_token in obsolete_tokens:
            with self.subTest(token=obsolete_token):
                self.git("read-tree", "HEAD")
                self.add_index_blob(
                    "README.md",
                    f"{clean_readme}\n{obsolete_token}\n",
                )

                result = self.run_checker()

                self.assertEqual(
                    result.returncode,
                    1,
                    result.stdout + result.stderr,
                )
                self.assertIn(
                    "obsolete public documentation text: "
                    f"README.md: {obsolete_token}",
                    result.stderr,
                )

    def test_obsolete_worktree_text_is_ignored_when_index_is_clean(self) -> None:
        readme_path = ROOT / "README.md"
        original_readme = readme_path.read_text()
        index_readme = self.git("show", ":README.md").stdout
        obsolete_token = "bag_validate"
        self.assertNotIn(obsolete_token, index_readme)
        try:
            readme_path.write_text(f"{original_readme}\n{obsolete_token}\n")
            result = self.run_checker()
        finally:
            readme_path.write_text(original_readme)

        self.assertNotIn(
            "obsolete public documentation text: "
            f"README.md: {obsolete_token}",
            result.stderr,
        )

    def test_non_regular_required_document_is_rejected(self) -> None:
        commit_id = self.git("rev-parse", "HEAD").stdout.strip()
        self.git(
            "update-index",
            "--add",
            "--cacheinfo",
            "160000",
            commit_id,
            "README.md",
        )

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            "non-regular tracked entry is not allowed: README.md",
            result.stderr,
        )

    def test_pyvut_submodule_revision_is_pinned(self) -> None:
        wrong_commit = self.git("rev-parse", "HEAD").stdout.strip()
        self.git(
            "update-index",
            "--add",
            "--cacheinfo",
            "160000",
            wrong_commit,
            "third_party/pyvut",
        )

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            "submodule revision mismatch: third_party/pyvut",
            result.stderr,
        )

    def test_unexpected_submodule_is_rejected(self) -> None:
        commit_id = self.git("rev-parse", "HEAD").stdout.strip()
        self.git(
            "update-index",
            "--add",
            "--cacheinfo",
            "160000",
            commit_id,
            "third_party/unexpected",
        )

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(
            "unexpected tracked submodule: third_party/unexpected",
            result.stderr,
        )

class WorkflowContractTests(unittest.TestCase):
    def setUp(self) -> None:
        self.workflow = (ROOT / ".github/workflows/ci.yml").read_text()

    def test_repository_contract_runs_checker_unit_tests(self) -> None:
        self.assertIn(
            "python3 tools/test_check_public_tree.py",
            self.workflow,
        )

    def test_repository_contract_runs_tracker_validator_tests(self) -> None:
        self.assertIn(
            "PYTHONPATH=tools/vut_validation/src "
            "python3 -m unittest discover "
            "-s tools/vut_validation/tests -v",
            self.workflow,
        )

    def test_repository_contract_runs_calibration_tests(self) -> None:
        self.assertIn(
            "PYTHONPATH=tools/tracker_camera_calibration/src "
            "python3 -m unittest discover "
            "-s tools/tracker_camera_calibration/tests -v",
            self.workflow,
        )

    def test_repository_contract_checks_out_and_validates_pyvut(self) -> None:
        self.assertIn("submodules: recursive", self.workflow)
        self.assertIn(
            "python3 third_party/pyvut/tools/check_public_tree.py",
            self.workflow,
        )

    def test_repository_contract_runs_multisensor_alignment_tests(self) -> None:
        self.assertIn(
            "PYTHONPATH=tools/multisensor_alignment/src "
            "python3 -m unittest discover "
            "-s tools/multisensor_alignment/tests -v",
            self.workflow,
        )

    def test_whitespace_check_compares_full_tip_to_empty_tree(self) -> None:
        self.assertIn(
            'EMPTY_TREE="$(git hash-object -w -t tree /dev/null)"',
            self.workflow,
        )
        self.assertIn(
            'git diff --check "$EMPTY_TREE" HEAD',
            self.workflow,
        )

    def test_private_repository_ros_ci_uses_default_import_token(self) -> None:
        action_block = self.workflow.split(
            "uses: ros-tooling/action-ros-ci@v0.4",
            maxsplit=1,
        )[1]
        self.assertIn(
            "import-token: ${{ secrets.GITHUB_TOKEN }}",
            action_block,
        )

    def test_ros_ci_builds_tracker_message_package(self) -> None:
        self.assertIn(
            "package-name: "
            "vt_camera_msgs vt_realsense_capture vt_tracker_msgs "
            "vt_vive_tracker",
            self.workflow,
        )


if __name__ == "__main__":
    unittest.main()
