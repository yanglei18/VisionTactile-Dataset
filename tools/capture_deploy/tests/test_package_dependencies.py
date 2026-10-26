"""Partial upgrades must reject the last core release without glove_sources.

Check Python and ROS dependency resolution without downloading or installing
packages. Wheel contents are also checked when preparing a release.
"""
import ast
from pathlib import Path
import tomllib
import xml.etree.ElementTree as ET

from packaging.requirements import Requirement
from packaging.specifiers import SpecifierSet
from packaging.utils import canonicalize_name
import pytest


ROOT = Path(__file__).resolve().parents[3]
CORE = ROOT / "ros2_ws/src/vt_capture_core"
MANAGER = ROOT / "ros2_ws/src/vt_capture_manager"


def setup_value(directory, key):
    tree = ast.parse((directory / "setup.py").read_text())
    setup = next(node for node in ast.walk(tree) if isinstance(node, ast.Call)
                 and isinstance(node.func, ast.Name) and node.func.id == "setup")
    return next(ast.literal_eval(item.value) for item in setup.keywords if item.arg == key)


def project(directory):
    return tomllib.loads((ROOT / directory / "pyproject.toml").read_text())["project"]


@pytest.mark.parametrize("consumer", ["manager", "console", "replay"])
def test_consumers_reject_pre_provenance_core_and_accept_current_release(consumer):
    if consumer == "manager":
        dependencies = setup_value(MANAGER, "install_requires")
    elif consumer == "console":
        dependencies = project("tools/capture_console")["dependencies"]
    else:
        dependencies = project("tools/multisensor_alignment")["optional-dependencies"]["replay"]
    requirements = [Requirement(value) for value in dependencies]
    matches = [item for item in requirements if canonicalize_name(item.name) == "vt-capture-core"]
    assert len(matches) == 1, f"{consumer} must declare its shared core dependency"
    requirement = matches[0]
    assert requirement.marker is None, "The core API cannot be optional on some platforms"
    assert "0.5.0" not in requirement.specifier, "Old core lacks the imported glove_sources API"
    assert setup_value(CORE, "version") in requirement.specifier


def test_ros_and_python_release_contracts_agree():
    for package in (CORE, MANAGER, ROOT / "ros2_ws/src/vt_glove_bridge"):
        manifest = ET.parse(package / "package.xml").getroot()
        assert manifest.findtext("version") == setup_value(package, "version")
    manifest = ET.parse(MANAGER / "package.xml").getroot()
    dependencies = [item for item in manifest.findall("exec_depend") if item.text == "vt_capture_core"]
    assert len(dependencies) == 1
    floor = dependencies[0].get("version_gte")
    assert floor, "ROS installations also need a minimum supported core version"
    ros_requirement = SpecifierSet(">=" + floor)
    python_requirement = next(Requirement(value) for value in setup_value(MANAGER, "install_requires")
                              if canonicalize_name(Requirement(value).name) == "vt-capture-core")
    assert ros_requirement == python_requirement.specifier
    assert "0.5.0" not in ros_requirement
    assert setup_value(CORE, "version") in ros_requirement


@pytest.mark.parametrize("directory,package", [
    ("tools/capture_console", "vt_capture_console"),
    ("tools/multisensor_alignment", "vt_multisensor_alignment"),
])
def test_runtime_version_matches_distribution(directory, package):
    tree = ast.parse((ROOT / directory / "src" / package / "__init__.py").read_text())
    version = next(ast.literal_eval(node.value) for node in tree.body if isinstance(node, ast.Assign)
                   and any(isinstance(target, ast.Name) and target.id == "__version__" for target in node.targets))
    assert version == project(directory)["version"]
