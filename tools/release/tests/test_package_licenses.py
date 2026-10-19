"""Keep independently distributable packages licensed without importing their code.

The distribution smoke check additionally inspects real sdist and wheel contents.
These checks cover ROS share installs, including CMake-only interface packages.
"""
from __future__ import annotations

from contextlib import chdir
from pathlib import Path
import re
import runpy
import sys
import tomllib
from types import ModuleType
import unittest
from unittest.mock import patch
from xml.etree import ElementTree


ROOT = Path(__file__).resolve().parents[3]
TOOL_PACKAGES = sorted(path.parent for path in (ROOT / "tools").glob("*/pyproject.toml"))
ROS_PACKAGES = sorted(path.parent for path in (ROOT / "ros2_ws/src").glob("*/package.xml"))


class PackageLicenseTests(unittest.TestCase):
    def test_each_package_carries_the_root_license_text(self):
        expected = (ROOT / "LICENSE").read_bytes()
        for package in TOOL_PACKAGES + ROS_PACKAGES:
            with self.subTest(package=package.relative_to(ROOT).as_posix()):
                self.assertEqual((package / "LICENSE").read_bytes(), expected)

    def test_tool_metadata_declares_license_and_includes_the_file(self):
        for package in TOOL_PACKAGES:
            with self.subTest(package=package.name):
                config = tomllib.loads((package / "pyproject.toml").read_text())
                license_value = config["project"]["license"]
                if isinstance(license_value, dict):
                    license_value = license_value["text"]
                self.assertEqual(license_value, "Apache-2.0")
                self.assertIn("LICENSE", config["tool"]["setuptools"]["license-files"])

    def test_ros_manifests_declare_the_matching_license(self):
        for package in ROS_PACKAGES:
            with self.subTest(package=package.name):
                manifest = ElementTree.parse(package / "package.xml").getroot()
                self.assertEqual(manifest.findtext("license"), "Apache-2.0")

    def test_python_ros_packages_install_license_to_share_and_distribution(self):
        for package in ROS_PACKAGES:
            setup_path = package / "setup.py"
            if not setup_path.is_file():
                continue
            with self.subTest(package=package.name):
                arguments = {}
                setuptools = ModuleType("setuptools")
                setuptools.find_packages = lambda **kwargs: []
                setuptools.setup = lambda **kwargs: arguments.update(kwargs)
                with patch.dict(sys.modules, {"setuptools": setuptools}), chdir(package):
                    runpy.run_path(str(setup_path))
                self.assertEqual(arguments["license"], "Apache-2.0")
                self.assertIn("LICENSE", arguments["license_files"])
                share_files = dict(arguments["data_files"])[f"share/{package.name}"]
                self.assertIn("LICENSE", share_files)

    def test_cmake_ros_packages_install_license_to_share(self):
        for package in ROS_PACKAGES:
            cmake_path = package / "CMakeLists.txt"
            if not cmake_path.is_file():
                continue
            with self.subTest(package=package.name):
                source = re.sub(r"#[^\n]*", "", cmake_path.read_text())
                self.assertRegex(
                    source,
                    r"install\(\s*FILES\s+LICENSE\s+DESTINATION\s+share/\$\{PROJECT_NAME\}\s*\)",
                )


if __name__ == "__main__":
    unittest.main()
