"""Launch an already prepared, isolated candidate; never re-import global settings."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path

from vt_capture_core.workstation import validate_configuration
from .backend.candidate_binding import load_candidate_binding
from .backend.candidate_migration import default_candidate_settings, verify_candidate_runtimes
from .backend.workstation_settings import read_private_json, repository_root


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", type=Path, help="Deployment script's checkout; checked against installed code")
    parser.add_argument("--settings-dir", type=Path, help="Prepared candidate directory; defaults to this checkout's private candidate")
    parser.add_argument("--check-only", action="store_true", help="Validate configuration and numerical environments without opening Qt or devices")
    parser.add_argument("--smoke-seconds", type=float, default=0.)
    parser.add_argument("--screenshot", type=Path)
    parser.add_argument("--size", default="1440x900")
    parser.add_argument("--language", choices=("zh_CN", "en"))
    args = parser.parse_args(argv)
    try:
        root = repository_root().resolve(strict=True)
        if args.repo_root is not None and args.repo_root.resolve(strict=True) != root:
            raise ValueError("候选界面 Python 安装来自另一工作树；请重新安装当前工作树的界面环境。")
        directory, identity = load_candidate_binding(args.settings_dir or default_candidate_settings(root), repo_root=root)
        configuration = validate_configuration(read_private_json(directory / "workstation.json"))
        verify_candidate_runtimes(configuration, root)
        if args.check_only:
            print(json.dumps(dict(status="verified", settings_dir=str(directory), software=identity,
                                  hardware_accessed=False), ensure_ascii=False))
            return 0
        command = ["/bin/bash", str(root / "tools/capture_deploy/start_workbench.sh"),
                   "--candidate-settings", str(directory), "--size", args.size]
        if args.smoke_seconds:
            command += ["--smoke-seconds", str(args.smoke_seconds)]
        if args.screenshot is not None:
            command += ["--screenshot", str(args.screenshot)]
        if args.language:
            command += ["--language", args.language]
        os.execv(command[0], command)
    except (OSError, ValueError, RuntimeError) as exc:
        parser.exit(2, f"候选工作台无法启动：{exc}\n")
    return 0


if __name__ == "__main__":
    raise SystemExit(main())
