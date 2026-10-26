"""The CI guard cannot mistake collection skips for successful regressions."""
import importlib.util
from pathlib import Path
import xml.etree.ElementTree as ET

import pytest


SCRIPT = Path(__file__).resolve().parents[1] / "scripts" / "verify_ci_results.py"
SPEC = importlib.util.spec_from_file_location("verify_ci_results", SCRIPT)
guard = importlib.util.module_from_spec(SPEC)
SPEC.loader.exec_module(guard)


def report(tmp_path, cases):
    root = ET.Element("testsuites")
    suite = ET.SubElement(root, "testsuite")
    for attributes, outcome in cases:
        case = ET.SubElement(suite, "testcase", attributes)
        if outcome:
            tag, message = outcome
            ET.SubElement(case, tag, message=message)
    path = tmp_path / "results.xml"
    ET.ElementTree(root).write(path, encoding="utf-8")
    return path


@pytest.mark.parametrize("identity", [
    {"classname": "tests.test_live_ui_workspace"},
    {"classname": "tools.fsglove12.tests.test_live_ui_workspace.TestWorkspace"},
    {"file": "tools/fsglove12/tests/test_live_ui_workspace.py"},
    {"file": r"tools\fsglove12\tests\test_live_ui_workspace.py"},
])
def test_success_supports_pytest_junit_module_and_file_forms(tmp_path, identity):
    path = report(tmp_path, [({**identity, "name": "test_status"}, None)])
    assert guard.check_report(path, {"test_live_ui_workspace"}) == []


def test_missing_required_module_is_not_masked_by_other_passes(tmp_path):
    path = report(tmp_path, [({"classname": "tests.test_config", "name": "test_valid"}, None)])
    assert guard.check_report(path, {"test_live_ui_workspace"}) == [
        "Required test module not collected: test_live_ui_workspace"]


@pytest.mark.parametrize("case", [
    {"classname": "tests.test_live_ui_workspace", "name": "test_status"},
    {"name": "tests.test_live_ui_workspace"},
])
def test_missing_gui_dependency_fails_for_test_and_module_skip(tmp_path, case):
    path = report(tmp_path, [(case, ("skipped", "could not import PyQt5"))])
    errors = guard.check_report(path, {"test_live_ui_workspace"})
    assert any("Required test skipped" in error for error in errors)
    assert any("did not execute any tests" in error for error in errors)


def test_optional_model_skips_do_not_hide_required_execution(tmp_path):
    path = report(tmp_path, [
        ({"classname": "tests.test_live_ui_workspace", "name": "test_status"}, None),
        ({"classname": "tests.test_mano_solver_integration", "name": "test_model"},
         ("skipped", "No explicit licensed MANO assets")),
    ])
    assert guard.check_report(path, {"test_live_ui_workspace"}) == []


def test_only_explicit_legacy_unsupported_cases_are_allowed_to_skip(tmp_path):
    module, name, reason = next(iter(guard.ALLOWED_SKIPS))
    cases = [({"classname": f"tests.{module}", "name": "test_official"}, None),
             ({"classname": f"tests.{module}", "name": name}, ("skipped", reason))]
    assert guard.check_report(report(tmp_path, cases), {module}) == []
    cases[-1] = (cases[-1][0], ("skipped", "missing dependency"))
    assert any("Required test skipped" in error
               for error in guard.check_report(report(tmp_path, cases), {module}))


def test_an_allowlisted_skip_alone_does_not_satisfy_module_execution(tmp_path):
    module, name, reason = next(iter(guard.ALLOWED_SKIPS))
    path = report(tmp_path, [({"classname": f"tests.{module}", "name": name}, ("skipped", reason))])
    assert guard.check_report(path, {module}) == [
        f"Required test module did not execute any tests: {module}"]


@pytest.mark.parametrize("outcome", ["failure", "error"])
def test_failures_in_any_collected_module_fail_the_report(tmp_path, outcome):
    path = report(tmp_path, [
        ({"classname": "tests.test_live_ui_workspace", "name": "test_status"}, None),
        ({"classname": "tests.test_optional", "name": "test_other"}, (outcome, "broken")),
    ])
    assert guard.check_report(path, {"test_live_ui_workspace"}) == [
        "Test failed: tests.test_optional::test_other"]


def test_empty_report_and_missing_required_sources_fail(tmp_path):
    errors = guard.check_report(report(tmp_path, []), {"test_live_ui_workspace"})
    assert "JUnit report contains no test cases" in errors
    with pytest.raises(ValueError, match="Required test source is missing"):
        guard.required_modules(tmp_path)


def test_missing_or_invalid_report_has_nonzero_exit(tmp_path, capsys):
    for pattern in guard.REQUIRED_PATTERNS:
        (tmp_path / pattern.replace("*", "_workspace")).write_text("")
    path = tmp_path / "results.xml"
    assert guard.main([str(path), "--tests-dir", str(tmp_path)]) == 1
    path.write_text("not xml")
    assert guard.main([str(path), "--tests-dir", str(tmp_path)]) == 1
    assert "Calibration CI contract failed" in capsys.readouterr().err
