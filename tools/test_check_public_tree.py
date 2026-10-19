#!/usr/bin/env python3
from __future__ import annotations

import importlib.util
import json
import os
import subprocess
import sys
import tempfile
import unittest
from pathlib import Path


ROOT = Path(__file__).resolve().parents[1]
CHECKER = ROOT / "tools/check_public_tree.py"
SPEC = importlib.util.spec_from_file_location("public_tree_policy", CHECKER)
policy = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(policy)


class ArtifactBoundaryTests(unittest.TestCase):
    def test_static_geometry_is_detected_even_when_renamed(self) -> None:
        model = {"schema_version": 1, "neutral_geometry": {
            "vertices_m": [[0, 0, 0]], "faces": [[0, 0, 0]], "joints_m": [[0, 0, 0]],
        }, "model_axes": {"hand": "right"}}
        for path in ("tools/assets/reference.json", "pkg/custom.JSON"):
            for content in (json.dumps(model), json.dumps(model).encode()):
                with self.subTest(path=path, binary=isinstance(content, bytes)):
                    self.assertTrue(policy.is_static_hand_model(path, content))

    def test_non_model_json_and_source_code_are_allowed(self) -> None:
        cases = (
            ("assets/config.json", '{"schema_version": 1, "hand": "right"}'),
            ("i18n/catalog.json", '{"neutral_geometry": "Neutral geometry"}'),
            ("configs/right_hand.json", '{"model_axes": {"hand": "right"}}'),
            ("tests/test_geometry.py", '{"schema_version": 1, "neutral_geometry": '
             '{"vertices_m": [], "faces": [], "joints_m": []}, "model_axes": {}}'),
            ("assets/data.json", '[]'),
            ("assets/data.json", 'invalid json'),
            ("assets/data.json", b'\xff'),
        )
        for path, content in cases:
            with self.subTest(path=path, content=content):
                self.assertFalse(policy.is_static_hand_model(path, content))

    def test_generated_artifact_boundary_is_shared_with_source_export(self) -> None:
        for path in ("dist/source.txt", "pkg/dist/manifest.json", "tool.exe", "Qt.DLL",
                     "setup.msi", "runtime.whl", "native.so", "native.so.6.1", "native.dylib"):
            with self.subTest(path=path):
                self.assertTrue(policy.forbidden_path(path))
        for path in ("tools/tactile_viewer_windows/build.py", "assets/icon.png",
                     "tools/capture_console/i18n/catalog.json", "config/right_hand.json"):
            with self.subTest(path=path):
                self.assertFalse(policy.forbidden_path(path))


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
            "docs/architecture.md",
            "docs/capture-guide.md",
            "docs/hardware-reference.md",
            "docs/interface-reference.md",
            "docs/release-checklist.md",
            "docs/tracker-linux-validation.md",
            "docs/tracker-camera-calibration.md",
            "docs/tracker-ros2-publisher.md",
            "docs/troubleshooting.md",
            "docs/user-manual.md",
            "ros2_ws/src/vt_camera_msgs/msg/CameraTrackerCalibration.msg",
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
            path for path in self.git(
                "ls-files", "-z", "--cached", "--others", "--exclude-standard",
                "--", "tools/multisensor_alignment", "tools/tactile_viewer_windows",
            ).stdout.split("\0") if path
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

    def test_independently_distributed_package_licenses_must_be_in_index(self) -> None:
        paths = (
            "tools/vut_validation/LICENSE",
            "tools/tracker_camera_calibration/LICENSE",
            "tools/multisensor_alignment/LICENSE",
            "ros2_ws/src/vt_capture_core/LICENSE",
            "ros2_ws/src/vt_camera_msgs/LICENSE",
        )
        self.git("update-index", "--force-remove", "--", *paths)

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        for path in paths:
            self.assertIn(f"missing tracked file: {path}", result.stderr)

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
            "dist/ghost.txt",
            "pkg/dist/manifest.json",
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
            "ghost.exe",
            "ghost.DLL",
            "ghost.msi",
            "ghost.whl",
            "ghost.so",
            "ghost.so.6.1",
            "ghost.dylib",
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

    def test_renamed_static_hand_model_is_rejected_from_index(self) -> None:
        path = "tools/assets/display_geometry.json"
        self.assertFalse((ROOT / path).exists())
        self.add_index_blob(path, json.dumps({
            "schema_version": 1,
            "neutral_geometry": {"vertices_m": [], "faces": [], "joints_m": []},
            "model_axes": {"hand": "right"},
        }))

        result = self.run_checker()

        self.assertEqual(result.returncode, 1, result.stdout + result.stderr)
        self.assertIn(f"static hand model must remain outside public sources: {path}", result.stderr)

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

    def test_factory_calibration_document_contract_is_checked_from_index(self) -> None:
        baseline = self.run_checker()
        self.assertEqual(baseline.returncode, 0, baseline.stdout + baseline.stderr)
        tokens = {
            "docs/architecture.md": ("21 topics", "unified-dataset-v2", "extrinsics/depth_to_color"),
            "tools/multisensor_alignment/README.md": ("21 个候选 Topic", "unified-dataset-v2"),
        }
        for path, required in tokens.items():
            original = (ROOT / path).read_text(encoding="utf-8")
            for token in required:
                with self.subTest(path=path, token=token):
                    self.assertIn(token, original)
                    self.add_index_blob(path, original.replace(token, "removed contract text"))
                    result = self.run_checker()
                    self.assertIn(f"{path} is missing required text: {token}", result.stderr)
                    self.add_index_blob(path, original)

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
        index_readme = self.git("show", ":README.md").stdout
        obsolete_token = "bag_validate"
        self.assertNotIn(obsolete_token, index_readme)
        worktree = Path(self.temp_dir.name) / "worktree"
        worktree.mkdir()
        (worktree / "README.md").write_text(
            f"{index_readme}\n{obsolete_token}\n", encoding="utf-8",
        )
        self.env["GIT_WORK_TREE"] = str(worktree)
        result = self.run_checker()

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

    def test_expected_pyvut_submodule_is_allowed(self) -> None:
        result = self.run_checker()

        self.assertNotIn(
            "non-regular tracked entry is not allowed: pyvut",
            result.stderr,
        )
        self.assertNotIn("unexpected tracked submodule: pyvut", result.stderr)


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

    def test_repository_contract_runs_multisensor_alignment_tests(self) -> None:
        self.assertIn(
            "PYTHONPATH=tools/multisensor_alignment/src "
            "python3 -m pytest tools/multisensor_alignment/tests -q",
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

    def test_ci_runs_pytest_calibration_gui_and_source_release_checks(self) -> None:
        self.assertIn("xvfb-run -a python -m pytest tools/tracker_camera_calibration/tests -q", self.workflow)
        self.assertIn("python3 -m unittest discover -s tools/release/tests -v", self.workflow)

    def test_ci_covers_host_clock_and_static_exporter_without_model_assets(self) -> None:
        self.assertIn("ros2_ws/src/vt_tactile_capture/test/test_host_clock.py", self.workflow)
        console = (ROOT / ".github/workflows/capture-console.yml").read_text()
        self.assertIn("tools/fsglove12/tests/test_tactile_preview_export.py", console)

    def test_windows_ci_is_read_only_and_uses_synthetic_tests(self) -> None:
        workflow = (ROOT / ".github/workflows/tactile-windows.yml").read_text()
        self.assertIn("permissions:\n  contents: read", workflow)
        self.assertIn("test_host_clock.py", workflow)
        self.assertIn("test_tactile_i18n.py", workflow)
        self.assertIn("test_tactile_diagnostics_i18n.py", workflow)
        self.assertNotIn("actions/upload-artifact", workflow)
        self.assertNotIn("contents: write", workflow)
        self.assertNotIn("secrets.", workflow)


if __name__ == "__main__":
    unittest.main()
