"""Validated operator selections, independent of ROS launch and YAML libraries.

Recording accepts any nonempty subset of the known device inventory.
Preview selections may be any subset of the known inventory, including empty.
This module validates selections only: it does not launch or configure hardware.
"""

from __future__ import annotations

from dataclasses import dataclass

from .camera_streams import CAMERA_TOPIC_SUFFIXES, RECIPE_VERSION, camera_streams
from .validation import ProtocolError, choice, mapping, number, text


MAX_RECORDING_DURATION_S = 300
CAMERA_IDS = ("d405_1", "d405_2", "d436")
TRACKER_IDS = ("left_wrist", "right_wrist", "torso")
HAND_IDS = ("left", "right")


class RecipeError(ValueError):
    """A selection is unsupported or would violate the recording contract."""


def _selection(value, inventory, field):
    if type(value) is not list:
        raise RecipeError(f"recipe.{field} must be an array")
    for item in value:
        if type(item) is not str or item not in inventory:
            raise RecipeError(f"recipe.{field} contains an unknown selection")
    if len(value) != len(set(value)):
        raise RecipeError(f"recipe.{field} contains duplicate selections")
    return tuple(item for item in inventory if item in value)


@dataclass(frozen=True)
class Recipe:
    name: str
    mode: str = "record"
    duration_s: float = 300.0
    cameras: tuple[str, ...] = CAMERA_IDS
    trackers: tuple[str, ...] = TRACKER_IDS
    pose_hands: tuple[str, ...] = ()
    tactile_hands: tuple[str, ...] = ()
    version: int = RECIPE_VERSION

    def _document(self) -> dict:
        return {
            "version": self.version, "name": self.name, "mode": self.mode,
            "duration_s": self.duration_s, "cameras": list(self.cameras),
            "trackers": list(self.trackers), "pose_hands": list(self.pose_hands),
            "tactile_hands": list(self.tactile_hands),
        }

    def to_dict(self) -> dict:
        """Return a validated, deterministically ordered JSON-compatible recipe."""
        return validate_recipe(self._document())._document()

    @property
    def device_ids(self) -> tuple[str, ...]:
        """Stable selection IDs; these are not hardware serial numbers."""
        recipe = validate_recipe(self._document())
        return (
            *(f"camera:{name}" for name in recipe.cameras),
            *(f"tracker:{name}" for name in recipe.trackers),
            *(f"glove:{hand}" for hand in recipe.pose_hands),
            *(f"tactile:{hand}" for hand in recipe.tactile_hands),
        )

    def record_topics(self) -> tuple[str, ...]:
        """Return the strict bag allowlist; preview recipes cannot become records."""
        recipe = validate_recipe(self._document())
        if recipe.mode != "record":
            raise RecipeError("preview-only recipes do not have a recording allowlist")
        additional = sorted([
            *(f"/gloves/{hand}/state" for hand in recipe.pose_hands),
            *(f"/gloves/{hand}/tactile/samples" for hand in recipe.tactile_hands),
        ])
        return (
            *(f"/{name}/{suffix}" for name in recipe.cameras
              for suffix, _type_name in camera_streams(recipe.version)),
            *(f"/vive/{role}/sample" for role in recipe.trackers),
            *additional,
        )


def validate_recipe(document: dict) -> Recipe:
    """Validate v1/v2 input without mutation and preserve its camera contract.

    All fields are explicit so missing selections cannot silently enable devices.
    Record duration is in seconds, inclusive 1..300; preview duration must be 0.
    Cameras, Tracker roles, hand pose and tactile are independent selections.
    """
    try:
        mapping(document, {"version", "name", "mode", "duration_s", "cameras", "trackers",
                           "pose_hands", "tactile_hands"}, "recipe")
        if type(document["version"]) is not int or document["version"] not in (1, RECIPE_VERSION):
            raise RecipeError("unsupported recipe version; expected integer 1 or 2")
        text(document["name"], "recipe.name", nonempty=True)
        choice(document["mode"], {"record", "preview"}, "recipe.mode")
        duration = number(document["duration_s"], "recipe.duration_s")
        cameras = _selection(document["cameras"], CAMERA_IDS, "cameras")
        trackers = _selection(document["trackers"], TRACKER_IDS, "trackers")
        pose_hands = _selection(document["pose_hands"], HAND_IDS, "pose_hands")
        tactile_hands = _selection(document["tactile_hands"], HAND_IDS, "tactile_hands")
        if document["mode"] == "record":
            if not (cameras or trackers or pose_hands or tactile_hands):
                raise RecipeError("record recipes require at least one selected device")
            if not 1 <= duration <= MAX_RECORDING_DURATION_S:
                raise RecipeError("record duration_s must be between 1 and 300 seconds")
        elif duration != 0:
            raise RecipeError("preview-only duration_s must be 0")
        return Recipe(name=document["name"], mode=document["mode"], duration_s=float(duration),
                      cameras=cameras, trackers=trackers, pose_hands=pose_hands,
                      tactile_hands=tactile_hands, version=document["version"])
    except ProtocolError as error:
        raise RecipeError(str(error)) from error
