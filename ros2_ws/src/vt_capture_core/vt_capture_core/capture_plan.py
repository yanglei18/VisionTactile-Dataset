"""Resolved, immutable recording intent derived from the existing Recipe.

The v2 plan separates the configured Recorder allowlist from streams required
for this recording. Admission freezes available streams as required; the other
configured streams can still arrive later. Historical v1 plans retain their
strict complete-inventory contract. Neither version proves bag completeness.
No ROS, Qt or storage access belongs here.
"""

from __future__ import annotations

from collections import Counter
from collections.abc import Sequence
from dataclasses import dataclass, replace
import hashlib
import json
import re

from .camera_streams import CAMERA_STREAMS, RECIPE_VERSION, camera_streams, capture_schema_version
from .recipes import CAMERA_IDS, HAND_IDS, TRACKER_IDS, Recipe, RecipeError, validate_recipe
from .validation import ProtocolError, boolean, mapping, text


CAPTURE_PLAN_VERSION = 2
CAPTURE_SCHEMA_VERSION = capture_schema_version()
MAX_CONTROLLER_TOPICS = 200
MAX_TOPIC_NAME_BYTES = 512
_ABSOLUTE_TOPIC_PATTERN = re.compile(r"(?:/[A-Za-z_][A-Za-z0-9_]*)+")
_MESSAGE_TYPE_PATTERN = re.compile(r"[A-Za-z_][A-Za-z0-9_]*/msg/[A-Za-z_][A-Za-z0-9_]*")


class CapturePlanError(ValueError):
    """A contract violation with a stable code and localizable evidence."""

    def __init__(self, message: str, *, code: str = "invalid_capture_plan", details: dict | None = None):
        super().__init__(message)
        self.code = code
        self.details = dict(details or {})


def _topic(value: str) -> str:
    if type(value) is not str or _ABSOLUTE_TOPIC_PATTERN.fullmatch(value) is None:
        raise CapturePlanError(
            "controller topic must be an absolute ROS topic name", code="invalid_controller_topic",
        )
    if len(value.encode("utf-8")) > MAX_TOPIC_NAME_BYTES:
        raise CapturePlanError(
            f"controller topic exceeds {MAX_TOPIC_NAME_BYTES} bytes", code="controller_topic_too_long",
        )
    return value


def _controller_topics(
    topics: Sequence[str], *, allow_duplicates: bool = False, allow_empty: bool = False,
) -> tuple[str, ...]:
    if isinstance(topics, (str, bytes)) or not isinstance(topics, Sequence):
        raise CapturePlanError(
            "controller topics must be an array of topic names", code="invalid_controller_topics",
        )
    minimum = 0 if allow_empty else 1
    if not minimum <= len(topics) <= MAX_CONTROLLER_TOPICS:
        raise CapturePlanError(
            f"controller topics must contain between {minimum} and {MAX_CONTROLLER_TOPICS} names",
            code="controller_topics_size",
        )
    result = tuple(_topic(topic) for topic in topics)
    if not allow_duplicates and len(set(result)) != len(result):
        raise CapturePlanError(
            "controller topics contains duplicate names", code="duplicate_controller_topics",
            details={"duplicate_topics": sorted(topic for topic, count in Counter(result).items() if count > 1)},
        )
    return result


def topics_fingerprint(topics: Sequence[str]) -> str:
    """Identify an allowlist by sorted names, independently of publication order.

    Message types, device availability and a bag's contents are not included.
    """
    canonical = json.dumps(sorted(_controller_topics(topics)), ensure_ascii=False, separators=(",", ":"))
    return hashlib.sha256(canonical.encode("utf-8")).hexdigest()


def recipe_for_topics(
    topics: Sequence[str], *, name: str = "controller-config", duration_s: float = 30,
) -> Recipe:
    """Describe an existing supported allowlist without changing the Recorder.

    Each selected camera keeps its complete configured stream bundle. Tracker
    roles and hand streams are independent selections. An incomplete camera
    bundle or unknown extra is rejected instead of being silently omitted.
    Complete legacy bundles resolve to recipe v1. With no cameras selected the
    current recipe version applies, since the remaining stream contracts agree.
    """
    configured = set(_controller_topics(topics))
    cameras = tuple(camera for camera in CAMERA_IDS if any(
        f"/{camera}/{suffix}" in configured for suffix, _type_name in CAMERA_STREAMS))
    trackers = tuple(role for role in TRACKER_IDS if f"/vive/{role}/sample" in configured)
    pose_hands = tuple(hand for hand in HAND_IDS if f"/gloves/{hand}/state" in configured)
    tactile_hands = tuple(hand for hand in HAND_IDS if f"/gloves/{hand}/tactile/samples" in configured)
    legacy_suffixes = {suffix for suffix, _type_name in camera_streams(1)}
    has_calibration = any(
        f"/{camera}/{suffix}" in configured
        for camera in cameras for suffix, _type_name in CAMERA_STREAMS
        if suffix not in legacy_suffixes
    )
    version = RECIPE_VERSION if not cameras or has_calibration else 1
    recipe = Recipe(name, duration_s=duration_s, cameras=cameras, trackers=trackers,
                    pose_hands=pose_hands, tactile_hands=tactile_hands, version=version)
    recognized = {stream.topic for stream in _streams_for_recipe(recipe)}
    missing = sorted(recognized - configured)
    unexpected = sorted(configured - recognized)
    if missing or unexpected:
        raise CapturePlanError(
            "controller topics do not match a supported recipe: "
            f"missing={missing}, unexpected={unexpected}", code="unsupported_controller_topics",
            details={"missing_topics": missing, "unexpected_topics": unexpected},
        )
    try:
        return validate_recipe(recipe.to_dict())
    except RecipeError as error:
        raise CapturePlanError(str(error)) from error


@dataclass(frozen=True)
class PlannedStream:
    topic: str
    type_name: str
    device_id: str
    required: bool = True

    def __post_init__(self):
        try:
            _topic(self.topic)
            if type(self.type_name) is not str or _MESSAGE_TYPE_PATTERN.fullmatch(self.type_name) is None:
                raise CapturePlanError("planned stream type_name must be a ROS message type")
            text(self.device_id, "planned stream device_id", nonempty=True)
            boolean(self.required, "planned stream required")
        except ProtocolError as error:
            raise CapturePlanError(str(error)) from error

    def to_dict(self) -> dict:
        return {
            "topic": self.topic, "type_name": self.type_name,
            "device_id": self.device_id, "required": self.required,
        }


def _record_recipe(recipe: Recipe) -> Recipe:
    if not isinstance(recipe, Recipe):
        raise CapturePlanError("capture plan requires a Recipe")
    try:
        normalized = validate_recipe(recipe.to_dict())
        normalized.record_topics()
        return normalized
    except RecipeError as error:
        raise CapturePlanError(str(error)) from error


def _streams_for_recipe(recipe: Recipe, *, required: bool = False) -> tuple[PlannedStream, ...]:
    streams = [
        PlannedStream(f"/{camera}/{suffix}", type_name, f"camera:{camera}", required)
        for camera in recipe.cameras
        for suffix, type_name in camera_streams(recipe.version)
    ]
    streams.extend(
        PlannedStream(f"/vive/{role}/sample", "vt_tracker_msgs/msg/TrackerSample", f"tracker:{role}", required)
        for role in recipe.trackers
    )
    additional = [
        *(PlannedStream(f"/gloves/{hand}/state", "vt_glove_msgs/msg/GloveState", f"glove:{hand}", required)
          for hand in recipe.pose_hands),
        *(PlannedStream(f"/gloves/{hand}/tactile/samples", "vt_tactile_msgs/msg/TactileSampleArray",
                        f"tactile:{hand}", required) for hand in recipe.tactile_hands),
    ]
    return (*streams, *sorted(additional, key=lambda item: item.topic))


@dataclass(frozen=True)
class ResolvedCapturePlan:
    recipe: Recipe
    streams: tuple[PlannedStream, ...]
    version: int = CAPTURE_PLAN_VERSION
    schema_version: str = CAPTURE_SCHEMA_VERSION

    def __post_init__(self):
        if type(self.version) is not int or self.version not in (1, CAPTURE_PLAN_VERSION):
            raise CapturePlanError("unsupported capture plan version; expected integer 1 or 2")
        recipe = _record_recipe(self.recipe)
        if self.schema_version != capture_schema_version(recipe.version):
            raise CapturePlanError("capture plan schema_version does not match its recipe version")
        if self.version == 1 and recipe.version != 1:
            raise CapturePlanError("v1 capture plans require the historical v1 recipe")
        if self.version == 1 and (recipe.cameras != CAMERA_IDS or recipe.trackers != TRACKER_IDS):
            raise CapturePlanError("v1 capture plans require all three cameras and all three trackers")
        if type(self.streams) is not tuple or any(not isinstance(stream, PlannedStream) for stream in self.streams):
            raise CapturePlanError("capture plan streams must be an immutable tuple of PlannedStream values")
        expected = _streams_for_recipe(recipe, required=self.version == 1)
        normalized = (self.streams if self.version == 1 else
                      tuple(replace(stream, required=False) for stream in self.streams))
        if normalized != expected:
            raise CapturePlanError("capture plan streams do not match the resolved recipe")
        # Recipe accepts selections supplied as lists. Detach and normalize them
        # before exposing an immutable snapshot to the manager or serializer.
        object.__setattr__(self, "recipe", recipe)

    def _document(self) -> dict:
        return {
            "version": self.version, "schema_version": self.schema_version,
            "recipe": self.recipe.to_dict(),
            "streams": [stream.to_dict() for stream in self.streams],
        }

    @property
    def fingerprint(self) -> str:
        """SHA-256 of canonical UTF-8 JSON excluding the fingerprint itself.

        This detects changed intent; it does not authenticate the author or
        establish integrity of a recorded bag's contents.
        """
        canonical = json.dumps(self._document(), ensure_ascii=False, sort_keys=True, separators=(",", ":"))
        return hashlib.sha256(canonical.encode("utf-8")).hexdigest()

    def to_dict(self) -> dict:
        return {**self._document(), "fingerprint": self.fingerprint}

    def record_topics(self) -> tuple[str, ...]:
        return tuple(stream.topic for stream in self.streams)

    def required_topics(self) -> tuple[str, ...]:
        """Streams frozen as present at admission, or all streams for v1."""
        return tuple(stream.topic for stream in self.streams if stream.required)

    def optional_topics(self) -> tuple[str, ...]:
        """Configured subscriptions whose absence alone does not fail quality."""
        return tuple(stream.topic for stream in self.streams if not stream.required)

    @property
    def topic_types(self) -> dict[str, str]:
        return {stream.topic: stream.type_name for stream in self.streams}

    @property
    def device_ids(self) -> tuple[str, ...]:
        return self.recipe.device_ids


def resolve_capture_plan(recipe: Recipe) -> ResolvedCapturePlan:
    """Resolve v2 candidates without claiming any stream was observed."""
    recipe = _record_recipe(recipe)
    return ResolvedCapturePlan(recipe, _streams_for_recipe(recipe),
                               schema_version=capture_schema_version(recipe.version))


def require_available_streams(plan: ResolvedCapturePlan, topics: Sequence[str]) -> ResolvedCapturePlan:
    """Freeze an explicit available subset while preserving every subscription.

    This pure operation does not observe devices. The admission layer supplies
    observations and owns minimum-data and failure policies. Old v1 plans are
    immutable historical contracts and cannot be weakened through this API.
    """
    if not isinstance(plan, ResolvedCapturePlan) or plan.version != CAPTURE_PLAN_VERSION:
        raise CapturePlanError("available streams require a v2 capture plan")
    available = set(_controller_topics(topics, allow_empty=True))
    unexpected = sorted(available - set(plan.record_topics()))
    if unexpected:
        raise CapturePlanError("available topics are outside the capture plan",
                               details={"unexpected_topics": unexpected})
    return replace(plan, streams=tuple(
        replace(stream, required=stream.topic in available) for stream in plan.streams))


def validate_capture_plan(document: dict) -> ResolvedCapturePlan:
    """Read a frozen v1 or v2 plan; reject inconsistent or altered fields."""
    try:
        mapping(document, {"version", "schema_version", "recipe", "streams", "fingerprint"}, "capture_plan")
        recipe = validate_recipe(document["recipe"])
        if type(document["streams"]) is not list:
            raise CapturePlanError("capture_plan.streams must be an array")
        streams = []
        for value in document["streams"]:
            mapping(value, {"topic", "type_name", "device_id", "required"}, "capture_plan.stream")
            streams.append(PlannedStream(**value))
        plan = ResolvedCapturePlan(recipe, tuple(streams), document["version"], document["schema_version"])
        if type(document["fingerprint"]) is not str or document["fingerprint"] != plan.fingerprint:
            raise CapturePlanError("capture_plan.fingerprint does not match the resolved intent")
        return plan
    except (ProtocolError, RecipeError) as error:
        raise CapturePlanError(str(error)) from error


@dataclass(frozen=True)
class ControllerTopicCheck:
    missing_topics: tuple[str, ...] = ()
    unexpected_topics: tuple[str, ...] = ()
    duplicate_topics: tuple[str, ...] = ()

    @property
    def matches(self) -> bool:
        return not (self.missing_topics or self.unexpected_topics or self.duplicate_topics)

    def to_dict(self) -> dict:
        return {
            "matches": self.matches, "missing_topics": list(self.missing_topics),
            "unexpected_topics": list(self.unexpected_topics),
            "duplicate_topics": list(self.duplicate_topics),
        }


def compare_controller_topics(
    plan: ResolvedCapturePlan, configured_topics: Sequence[str],
) -> ControllerTopicCheck:
    """Compare a recorder allowlist with intent, independently of topic order.

    Controller configuration is evidence of an allowlist only. A matching result
    makes no claim about publishers, message types observed on ROS or bag quality.
    """
    if not isinstance(plan, ResolvedCapturePlan):
        raise CapturePlanError("controller comparison requires a ResolvedCapturePlan")
    counts = Counter(_controller_topics(configured_topics, allow_duplicates=True, allow_empty=True))
    expected = plan.record_topics()
    expected_set = set(expected)
    return ControllerTopicCheck(
        missing_topics=tuple(topic for topic in expected if topic not in counts),
        unexpected_topics=tuple(sorted(set(counts) - expected_set)),
        duplicate_topics=tuple(sorted(topic for topic, count in counts.items() if count > 1)),
    )
