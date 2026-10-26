#!/usr/bin/env python3
"""Compile actual firmware modules against bounded deterministic HAL mocks."""
import json
from pathlib import Path
import subprocess
import tempfile
ROOT = Path(__file__).resolve().parent.parent
with tempfile.TemporaryDirectory(prefix="vt-firmware-test-") as temp:
    for name, project in (("node", "IMU_SLV_MCU"), ("transport", "IMU_MST_2")):
        exe = Path(temp) / name
        subprocess.run(["cc", "-std=c11", "-Wall", "-Wextra", "-Werror", "-fsanitize=address,undefined", "-fno-omit-frame-pointer", "-g", "-I" + str(ROOT / "tests/stubs"), "-I" + str(ROOT / project / "Core/Inc"), str(ROOT / "tests" / ("test_" + name + ".c")), "-o", str(exe)], check=True)
        result = subprocess.run([str(exe)], check=True, capture_output=True, text=True)
        for line in result.stdout.splitlines():
            if line.startswith("VTIMU STATUS 1 "):
                status = json.loads(line[len("VTIMU STATUS 1 "):])
                assert status["schema_version"] == 1 and status["firmware_version"] == 2
                assert status["request_id"] in {"0011223344556677", "fedcba9876543210"}
                assert len(status["nodes"]) == 12
                assert all(n["configuration_verified"] and n["registers"]["ctrl3_c"] == 0x44 for n in status["nodes"])
            else:
                print(line)
