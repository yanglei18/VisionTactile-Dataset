from __future__ import annotations

import os
import stat
from dataclasses import dataclass
from pathlib import Path
from types import MappingProxyType
from typing import Mapping

import yaml

from .identity import tracker_id


EXPECTED_ROLES = frozenset({"left_wrist", "right_wrist", "torso"})
_LOWERCASE_HEX = frozenset("0123456789abcdef")


def _is_tracker_id(value: object) -> bool:
    return (
        type(value) is str
        and len(value) == 64
        and all(character in _LOWERCASE_HEX for character in value)
    )


@dataclass(frozen=True)
class RoleMap:
    by_role: Mapping[str, str]
    by_tracker_id: Mapping[str, str]

    def __post_init__(self) -> None:
        role_values = dict(self.by_role)
        tracker_values = dict(self.by_tracker_id)
        if set(role_values) != EXPECTED_ROLES:
            raise ValueError("role map must contain exactly the supported roles")
        if (
            len(role_values) != len(tracker_values)
            or any(tracker_values.get(value) != role for role, value in role_values.items())
        ):
            raise ValueError("role map reverse index is inconsistent")
        object.__setattr__(self, "by_role", MappingProxyType(role_values))
        object.__setattr__(
            self, "by_tracker_id", MappingProxyType(tracker_values)
        )

    def tracker_id_for_role(self, role: str) -> str:
        return self.by_role[role]

    def role_for_tracker_id(self, value: str) -> str:
        return self.by_tracker_id[value]

    def role_for_address(self, raw: bytes) -> str:
        return self.role_for_tracker_id(tracker_id(raw))


def load_role_map(path: Path) -> RoleMap:
    path = Path(path)
    metadata = path.lstat()
    if not stat.S_ISREG(metadata.st_mode) or stat.S_ISLNK(metadata.st_mode):
        raise ValueError("role map must be a regular file")
    if metadata.st_uid != os.getuid():
        raise PermissionError("role map must be owned by the current user")
    if stat.S_IMODE(metadata.st_mode) & 0o077:
        raise PermissionError("role map must have owner-only permissions")

    document = yaml.safe_load(path.read_text(encoding="utf-8"))
    if type(document) is not dict or set(document) != {
        "schema_version",
        "roles",
    }:
        raise ValueError("role map document is invalid")
    if document["schema_version"] != 1:
        raise ValueError("unsupported role map schema version")
    roles = document["roles"]
    if type(roles) is not dict or set(roles) != EXPECTED_ROLES:
        raise ValueError("role map must contain exactly the supported roles")
    if not all(_is_tracker_id(value) for value in roles.values()):
        raise ValueError("tracker IDs must be lowercase SHA-256 values")
    if len(set(roles.values())) != len(EXPECTED_ROLES):
        raise ValueError("tracker IDs must be unique")

    by_role = {role: roles[role] for role in sorted(EXPECTED_ROLES)}
    by_tracker_id = {value: role for role, value in by_role.items()}
    return RoleMap(by_role=by_role, by_tracker_id=by_tracker_id)
