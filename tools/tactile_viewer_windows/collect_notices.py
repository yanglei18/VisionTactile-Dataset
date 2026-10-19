"""Record installed build inputs and preserve dependency license files."""
from __future__ import annotations

import importlib.metadata
import json
from pathlib import Path
import platform
import re
import shutil
import sys


def collect(destination: Path) -> list[dict]:
    destination.mkdir(parents=True, exist_ok=True)
    # CPython is bundled by the freezer but is not an installed distribution.
    python_license = Path(sys.base_prefix) / "LICENSE.txt"
    if not python_license.is_file():
        raise FileNotFoundError(f"CPython 安装目录缺少 LICENSE.txt：{sys.base_prefix}")
    python_notices = destination / "CPython"
    python_notices.mkdir(exist_ok=True)
    shutil.copyfile(python_license, python_notices / "LICENSE.txt")
    packages = [{"name": "CPython", "version": platform.python_version(),
                 "license": "PSF-2.0 and bundled notices", "notice_files": ["CPython/LICENSE.txt"]}]
    for distribution in sorted(importlib.metadata.distributions(),
                               key=lambda item: item.metadata["Name"].lower()):
        name = distribution.metadata["Name"]
        directory = destination / re.sub(r"[^A-Za-z0-9_.-]", "_", name)
        notices = []
        for item in distribution.files or ():
            # Includes dist-info/licenses and Qt's bundled third-party notices.
            if not any(part.lower().startswith(("license", "copying", "notice", "third_party"))
                       for part in item.parts):
                continue
            source = Path(distribution.locate_file(item))
            if not source.is_file():
                continue
            safe_parts = [part for part in item.parts if part not in (".", "..")]
            target = directory.joinpath(*safe_parts)
            target.parent.mkdir(parents=True, exist_ok=True)
            shutil.copyfile(source, target)
            notices.append(target.relative_to(destination).as_posix())
        packages.append({"name": name, "version": distribution.version,
                         "license": distribution.metadata.get("License-Expression")
                                    or distribution.metadata.get("License", ""),
                         "notice_files": notices})
    return packages


if __name__ == "__main__":
    target = Path(sys.argv[1])
    Path(sys.argv[2]).write_text(json.dumps(collect(target), indent=2, ensure_ascii=False),
                                 encoding="utf-8")
