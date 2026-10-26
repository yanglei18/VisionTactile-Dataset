#!/usr/bin/env python3
"""Reject missing or silently skipped calibration regressions in a JUnit report.

Optional model, hardware and alternate-Qt integration tests may remain skipped;
the snapshot-driven calibration UI and presentation contracts must execute.
This checker uses only the standard library and never imports the application.
"""
from __future__ import annotations

import argparse
from pathlib import Path
import sys
import xml.etree.ElementTree as ET


REQUIRED_PATTERNS = (
    "test_live_ui*.py",
    "test_calibration_workspace_regressions.py",
    "test_calibration_presentation.py",
    "test_functional_capture.py",
    "test_functional_quality.py",
    "test_live_display_continuity_ui.py",
    "test_ui_theme.py",
)
# These two parameter combinations test a deliberately unsupported legacy
# workflow. Their official-mode counterparts must still run. Match both the
# precise test identity and reason, so a missing dependency cannot pass here.
ALLOWED_SKIPS = {
    ("test_live_ui_clear",
     "test_clear_interrupts_connected_capture_and_drops_stale_ui_until_generation_ack[legacy-active5]",
     "whole-hand capture exists only in the official UI"),
    ("test_live_ui_clear",
     "test_clear_interrupts_connected_capture_and_drops_stale_ui_until_generation_ack[legacy-active6]",
     "whole-hand capture exists only in the official UI"),
}


def required_modules(tests_dir: Path) -> set[str]:
    result = set()
    for pattern in REQUIRED_PATTERNS:
        files = sorted(tests_dir.glob(pattern))
        if not files:
            raise ValueError(f"Required test source is missing: {pattern}")
        result.update(path.stem for path in files if path.is_file())
    return result


def testcase_module(case: ET.Element, required: set[str]) -> str | None:
    # pytest's default xunit2 omits the file attribute. Its classname contains
    # the dotted module and possibly a class name; module-level import skips
    # can instead carry the module in name. Support all three representations.
    filename = Path(case.get("file", "").replace("\\", "/")).stem
    if filename in required:
        return filename
    for attribute in ("classname", "name"):
        value = case.get(attribute, "").replace("\\", "/").replace("/", ".")
        for part in value.split("."):
            if part in required:
                return part
    return None


def check_report(report: Path, required: set[str]) -> list[str]:
    tree = ET.parse(report)
    cases = list(tree.iter("testcase"))
    errors = []
    seen, executed = set(), set()
    if not cases:
        errors.append("JUnit report contains no test cases")
    for case in cases:
        module = testcase_module(case, required)
        name = case.get("name", "<unnamed>")
        label = f"{case.get('classname', module or '')}::{name}"
        if case.find("failure") is not None or case.find("error") is not None:
            errors.append(f"Test failed: {label}")
        if module is None:
            continue
        seen.add(module)
        skipped = case.find("skipped")
        if skipped is None:
            executed.add(module)
        elif (module, name, skipped.get("message", "")) not in ALLOWED_SKIPS:
            errors.append(f"Required test skipped: {label}: {skipped.get('message', '')}")
    for module in sorted(required - seen):
        errors.append(f"Required test module not collected: {module}")
    for module in sorted(seen - executed):
        errors.append(f"Required test module did not execute any tests: {module}")
    return errors


def main(argv=None) -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("report", type=Path)
    parser.add_argument("--tests-dir", type=Path,
                        default=Path(__file__).resolve().parents[1] / "tests")
    args = parser.parse_args(argv)
    try:
        required = required_modules(args.tests_dir)
        errors = check_report(args.report, required)
    except (OSError, ValueError, ET.ParseError) as exc:
        print(f"Calibration CI contract failed: {exc}", file=sys.stderr)
        return 1
    if errors:
        for error in errors:
            print(error, file=sys.stderr)
        return 1
    print(f"Calibration CI contract passed: {len(required)} required test modules executed.")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
