"""Small deployment CLI; reports status without printing private input contents."""
from __future__ import annotations

import argparse
import json
import os
from pathlib import Path
import stat
import sys

from .archive import build_package, read_package
from .manifest import for_bundle


def _private_json(path):
    fd = os.open(path, os.O_RDONLY | os.O_NOFOLLOW | os.O_NONBLOCK)
    with os.fdopen(fd, "rb") as stream:
        info = os.fstat(stream.fileno())
        if (not stat.S_ISREG(info.st_mode) or info.st_uid != os.getuid()
                or info.st_mode & 0o077 or info.st_size > 1024 * 1024):
            raise ValueError("Private workstation settings are unsafe or too large")
        raw = stream.read(1024 * 1024 + 1)
        if len(raw) > 1024 * 1024:
            raise ValueError("Private workstation settings are too large")
        return json.loads(raw)


def workstation_inputs(home=None):
    home = Path.home() if home is None else Path(home)
    directory = home / ".config/visiontactile/workbench"
    workstation = _private_json(directory / "workstation.json")
    groups = [group for group in workstation["process_groups"] if group["preset"] == "trackers"]
    if len(groups) != 1:
        raise ValueError("Exactly one Tracker profile is required")
    parameters = groups[0]["parameters"]
    bundle = Path(parameters["bundle_path"]).expanduser()
    managed = for_bundle(bundle, home=home)
    if managed is not None:
        capture, digest = managed.capture, managed.capture_sha256
    else:
        settings = _private_json(directory / "tracker-bootstrap.json")
        if settings["schema_version"] != 1:
            raise ValueError("Unsupported bootstrap settings schema")
        values = settings["values"]
        if Path(values["bundle"]).expanduser().absolute() != bundle.absolute():
            raise ValueError("Publisher and initializer must use the same bundle")
        capture, digest = Path(values["capture"]).expanduser(), values["expected_sha256"]
    return dict(source=Path(parameters["pyvut_source"]).expanduser(),
                bundle=bundle, roles=Path(parameters["role_map_path"]).expanduser(),
                capture=capture, capture_sha256=digest)


def parser():
    result = argparse.ArgumentParser(description="Build or import a private offline Tracker deployment package.")
    commands = result.add_subparsers(dest="command", required=True)
    build = commands.add_parser("build", help="Package the verified runtime and this device set's private inputs")
    build.add_argument("--from-workstation", action="store_true")
    for name in ("source", "bundle", "roles", "capture"):
        build.add_argument("--" + name, type=Path)
    build.add_argument("--capture-sha256")
    build.add_argument("--wheelhouse", type=Path, required=True)
    build.add_argument("--output", type=Path, required=True)
    install = commands.add_parser("install", help="Import offline and activate after validation; no hardware access")
    install.add_argument("--package", type=Path, required=True)
    install.add_argument("--home", type=Path, help="Alternative user home for deployment validation")
    install.add_argument("--python", default="/usr/bin/python3", help="Local Python 3.12 interpreter")
    inspect = commands.add_parser("inspect", help="Verify archive integrity and show a non-private summary")
    inspect.add_argument("--package", type=Path, required=True)
    return result


def main(argv=None):
    arguments = parser()
    args = arguments.parse_args(argv)
    if sys.platform != "linux":
        print("Tracker deployment failed (unsupported_platform). See docs/tracker-deployment.md.",
              file=sys.stderr)
        return 1
    try:
        if args.command == "build":
            values = {key: getattr(args, key) for key in ("source", "bundle", "roles", "capture", "capture_sha256")}
            if args.from_workstation:
                if any(value is not None for value in values.values()):
                    arguments.error("--from-workstation cannot be combined with explicit source/input fields")
                values = workstation_inputs()
            elif any(value is None for value in values.values()):
                arguments.error("build requires --from-workstation or all five source/input fields")
            build_package(**values, wheelhouse=args.wheelhouse, output=args.output)
            verified = read_package(args.output)
            print(json.dumps({"ok": True, "package": str(args.output.absolute()),
                              "sha256": verified.package_id}))
        elif args.command == "install":
            # Import lazily so CLI help and optional desktop discovery do not
            # require the Linux-only process/locking implementation.
            from .installer import install_package
            installation = install_package(args.package, home=args.home, python=args.python)
            print(json.dumps({"ok": True, "installation_id": installation.installation_id,
                              "source": str(installation.source), "hardware_started": False}))
        else:
            verified = read_package(args.package)
            print(json.dumps({"ok": True, "sha256": verified.package_id,
                              "target": verified.manifest["target"],
                              "file_count": len(verified.files)}))
    except KeyboardInterrupt:
        print("Tracker deployment cancelled.", file=sys.stderr)
        return 130
    except (OSError, ValueError, RuntimeError, KeyError, TypeError, ImportError) as exc:
        # Do not format arbitrary exception strings: input parsers may expose
        # credentials, addresses, or raw packet contents in those messages.
        public_codes = {
            "unsupported_platform", "unsupported_python", "python_unavailable", "unsafe_directory",
            "process_timeout", "runtime_dependency", "runtime_validation", "existing_installation_changed",
            "incomplete_installation", "venv_failed", "dependencies_failed", "cancelled",
        }
        code = getattr(exc, "code", "")
        if code not in public_codes:
            code = "invalid_package_or_settings"
        print(f"Tracker deployment failed ({code}). See docs/tracker-deployment.md.", file=sys.stderr)
        return 1
    return 0
