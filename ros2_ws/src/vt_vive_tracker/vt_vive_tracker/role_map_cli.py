from __future__ import annotations

import argparse
import os
import tempfile
from pathlib import Path
from typing import Sequence

import yaml

from .identity import tracker_id
from .roles import EXPECTED_ROLES


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(
        description="Create a private stable role map from a PyVUT bundle."
    )
    parser.add_argument("--bundle", required=True, type=Path)
    parser.add_argument("--output", required=True, type=Path)
    parser.add_argument("--host", required=True, choices=sorted(EXPECTED_ROLES))
    parser.add_argument(
        "--client0", required=True, choices=sorted(EXPECTED_ROLES)
    )
    parser.add_argument(
        "--client1", required=True, choices=sorted(EXPECTED_ROLES)
    )
    return parser


def _write_private_yaml(path: Path, document: dict[str, object]) -> None:
    path = path.expanduser()
    path.parent.mkdir(parents=True, exist_ok=True, mode=0o700)
    descriptor, temporary_name = tempfile.mkstemp(
        dir=path.parent, prefix=f".{path.name}.", suffix=".tmp"
    )
    temporary_path = Path(temporary_name)
    try:
        os.fchmod(descriptor, 0o600)
        with os.fdopen(descriptor, "w", encoding="utf-8") as stream:
            yaml.safe_dump(
                document,
                stream,
                default_flow_style=False,
                sort_keys=True,
            )
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary_path, path)
        directory_descriptor = os.open(path.parent, os.O_RDONLY)
        try:
            os.fsync(directory_descriptor)
        finally:
            os.close(directory_descriptor)
    except BaseException:
        try:
            os.close(descriptor)
        except OSError:
            pass
        temporary_path.unlink(missing_ok=True)
        raise


def main(argv: Sequence[str] | None = None) -> int:
    arguments = _parser().parse_args(argv)
    assignments = {
        "host": arguments.host,
        "client0": arguments.client0,
        "client1": arguments.client1,
    }
    if set(assignments.values()) != EXPECTED_ROLES:
        raise ValueError("host, client0, and client1 must map to unique roles")

    from pyvut.live_bootstrap_bundle import load_private_live_bundle

    bundle = load_private_live_bundle(arguments.bundle)
    roles = {
        assignments[protocol_role]: tracker_id(address)
        for protocol_role, address in zip(
            ("host", "client0", "client1"),
            bundle.tracker_addresses,
        )
    }
    _write_private_yaml(
        arguments.output,
        {"schema_version": 1, "roles": roles},
    )
    print(f"role_map_written={arguments.output}")
    return 0
