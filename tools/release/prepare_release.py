#!/usr/bin/env python3
"""Check/export current public sources without staging or publishing this repo.

Reads tracked plus non-ignored untracked working files, never exports .git or
submodule sources. Existing index/history/remotes and running devices are not
modified. The source contract runs in a disposable repository of the snapshot.
"""
from __future__ import annotations

import argparse
import hashlib
import importlib.util
import io
import json
import os
from pathlib import Path, PurePosixPath
import re
import stat
import subprocess
import tarfile
import tempfile

ROOT = Path(__file__).resolve().parents[2]
ROOT_FILES = {".gitignore", ".gitmodules", ".gitattributes", ".editorconfig",
              "README.md", "README.zh-CN.md", "LICENSE", "THIRD_PARTY_NOTICES.md",
              "CONTRIBUTING.md", "CONTRIBUTING.zh-CN.md", "SECURITY.md", "CHANGELOG.md", "Glove.py", "Ploy_IMU.py",
              "WINDOWS_USBPcap_CAPTURE_GUIDE.md"}
PREFIXES = (".github/", "docs/", "ros2_ws/src/", "tools/")
SECRET_PATTERNS = (
    rb"gh[pousr]_[A-Za-z0-9]{36,255}",
    rb"github_pat_[A-Za-z0-9_]{70,255}",
    rb"AKIA[0-9A-Z]{16}",
    rb"-----BEGIN (?:RSA |EC |OPENSSH )?PRIVATE KEY-----",
)


def git(root, *args, input=None):
    # Ignore inherited alternate-index/object-store variables. The checks below
    # must not write into another repository via an ambient Git environment.
    env = {k: v for k, v in os.environ.items() if not k.startswith("GIT_")}
    return subprocess.run(["git", "-C", str(root), *args], input=input,
                          check=True, stdout=subprocess.PIPE, stderr=subprocess.PIPE, env=env).stdout


def contract(root):
    spec = importlib.util.spec_from_file_location("public_tree_contract", root / "tools/check_public_tree.py")
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


def candidates(root):
    paths = git(root, "ls-files", "--cached", "--others", "--exclude-standard", "-z")
    return sorted(set(p.decode("utf-8") for p in paths.split(b"\0") if p))


def collect(root):
    policy = contract(root)
    selected, excluded, errors, links = {}, [], [], {}
    stages = git(root, "ls-files", "--stage", "-z").split(b"\0")
    for entry in stages:
        if not entry:
            continue
        metadata, path = entry.decode().split("\t", 1)
        mode, oid, stage = metadata.split()
        if stage != "0":
            errors.append("unmerged index entry: " + path)
        if mode == "160000":
            if path in policy.EXPECTED_SUBMODULES:
                links[path] = oid
            else:
                errors.append("unexpected submodule: " + path)
    for name in candidates(root):
        if name in links:
            continue
        parts = PurePosixPath(name).parts
        if PurePosixPath(name).is_absolute() or ".." in parts:
            errors.append("unsafe repository path")
            continue
        path = root / name
        if policy.forbidden_path(name) or not (name in ROOT_FILES or name.startswith(PREFIXES)):
            excluded.append(name)
            continue
        # Do not follow directory symlinks into local sensor data or secrets.
        if any((root.joinpath(*parts[:i])).is_symlink() for i in range(1, len(parts) + 1)):
            errors.append("symlink not exportable: " + name)
            continue
        if not path.exists():
            excluded.append(name)  # A working-tree deletion is not resurrected.
            continue
        info = path.stat()
        if not stat.S_ISREG(info.st_mode):
            errors.append("non-regular source file: " + name)
            continue
        if info.st_size > policy.MAX_TRACKED_BYTES:
            errors.append("source file exceeds 5 MiB: " + name)
            continue
        content = path.read_bytes()
        if len(content) > policy.MAX_TRACKED_BYTES:
            errors.append("source file grew beyond 5 MiB: " + name)
            continue
        if any(re.search(pattern, content) for pattern in SECRET_PATTERNS):
            errors.append("possible credential; review privately: " + name)
            continue
        # Windows migration made many Markdown/Python files executable. Export
        # ordinary text as 0644; executable scripts retain an entrypoint mode.
        mode = 0o755 if content.startswith(b"#!") or path.suffix == ".sh" else 0o644
        selected[name] = (content, mode)
    missing = policy.REQUIRED_FILES - selected.keys() - links.keys()
    errors.extend("missing public source: " + name for name in sorted(missing))
    return selected, links, excluded, errors


def verify_snapshot(files, links):
    """Validate exactly the captured bytes, in an isolated Git object store."""
    with tempfile.TemporaryDirectory(prefix="vt-public-check-") as directory:
        root = Path(directory)
        git(root, "init", "-q")
        for name, (content, mode) in files.items():
            path = root / name
            path.parent.mkdir(parents=True, exist_ok=True)
            path.write_bytes(content)
            path.chmod(mode)
        # Ignore rules cannot hide a mistakenly selected file from this check.
        for name, (content, mode) in files.items():
            oid = git(root, "hash-object", "-w", "--stdin", input=content).decode().strip()
            git(root, "update-index", "--add", "--cacheinfo", f"{0o100000 | mode:o},{oid},{name}")
        for name, oid in links.items():
            git(root, "update-index", "--add", "--cacheinfo", f"160000,{oid},{name}")
        policy = contract(root)
        # Policy Git invocations also stay inside this temporary repository.
        policy.git = lambda *args: git(root, *args).decode("utf-8")
        errors = policy.validation_errors()
        try:
            git(root, "diff", "--cached", "--check")
        except subprocess.CalledProcessError:
            # Do not include offending source lines, which can contain secrets.
            errors.append("source whitespace check failed (run git diff --check on reviewed files)")
        return errors


def history_risks(root):
    """Path-only advisory, not a comprehensive secret/history certification."""
    names = git(root, "log", "--all", "--format=", "--name-only", "--diff-filter=A").decode("utf-8")
    policy = contract(root)
    return sorted({name for name in names.splitlines() if name and policy.forbidden_path(name)})


def export_archive(output, files, links):
    manifest = dict(schema_version=1, history_included=False, submodule_sources_included=False,
                    submodules=links, files=[dict(path=n, bytes=len(c), sha256=hashlib.sha256(c).hexdigest())
                                            for n, (c, _) in sorted(files.items())])
    manifest_bytes = (json.dumps(manifest, ensure_ascii=False, indent=2) + "\n").encode()
    # Exclusive creation: never overwrite an earlier source delivery.
    with output.open("xb") as stream:
        try:
            with tarfile.open(fileobj=stream, mode="w:gz") as archive:
                for name, (content, mode) in sorted(files.items()):
                    info = tarfile.TarInfo("Vision-Tactile-Dataset/" + name)
                    info.size, info.mode, info.mtime = len(content), mode, 0
                    archive.addfile(info, io.BytesIO(content))
                info = tarfile.TarInfo("Vision-Tactile-Dataset/PUBLIC_SOURCE_MANIFEST.json")
                info.size, info.mode = len(manifest_bytes), 0o644
                archive.addfile(info, io.BytesIO(manifest_bytes))
        except BaseException:
            output.unlink(missing_ok=True)
            raise


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("command", choices=("check", "export"))
    parser.add_argument("--root", type=Path, default=ROOT)
    parser.add_argument("--output", type=Path, help="New .tar.gz file, parent must exist; export only")
    parser.add_argument("--report", type=Path, help="Write JSON report to a new local file (never overwrite)")
    args = parser.parse_args(argv)
    if args.command == "export" and args.output is None:
        parser.error("export requires --output")
    if args.command == "check" and args.output is not None:
        parser.error("--output is only for export")
    if args.report is not None and args.report.exists():
        parser.error("report already exists; use a new path")
    if args.output is not None and args.output.exists():
        parser.error("output already exists; use a new path")
    root = args.root.resolve()
    files, links, excluded, errors = collect(root)
    if not errors:
        errors.extend(verify_snapshot(files, links))
    risks = history_risks(root)
    report = dict(public_source_check="FAIL" if errors else "PASS", source_files=len(files),
                  source_bytes=sum(len(value[0]) for value in files.values()),
                  submodules=sorted(links), excluded_paths=excluded, errors=errors,
                  history_path_check="BLOCKED" if risks else "NO_KNOWN_FINDINGS",
                  history_risk_paths=risks,
                  scope="Current source snapshot only; not a full secret scan or hardware acceptance")
    if args.command == "export" and not errors:
        export_archive(args.output, files, links)
        report["archive"] = str(args.output)
        report["archive_bytes"] = args.output.stat().st_size
    rendered = json.dumps(report, ensure_ascii=False, indent=2)
    if args.report is not None:
        with args.report.open("x", encoding="utf-8") as stream:
            stream.write(rendered + "\n")
    print(rendered)
    return 1 if errors else 0


if __name__ == "__main__":
    raise SystemExit(main())
