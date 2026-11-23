"""Compare the runtime evidence surrounding a formal trial with its recorded messages."""

from vt_capture_core.validation import UINT32_MAX


REQUIRED_IDENTITY = {"source_mode", "session_id", "config_sha256", "profile_version"}


def frozen_hand_expectations(hands, start, finish):
    def boundary(metadata, hand):
        item = (metadata or {}).get("hand_runtime_evidence", {}).get("hands", {}).get(hand, {})
        if item.get("status") != "verified_snapshot":
            return None
        identity = {key: item.get(key) for key in REQUIRED_IDENTITY}
        if "rest_sha256" in item:
            identity["rest_sha256"] = item["rest_sha256"]
        return identity

    return {hand: dict(start=boundary(start, hand), finish=boundary(finish, hand)) for hand in hands}


def validate_expectations(value):
    if type(value) is not dict or not set(value) <= {"left", "right"}:
        raise ValueError("Invalid formal hand selection")
    for phases in value.values():
        if type(phases) is not dict or set(phases) != {"start", "finish"}:
            raise ValueError("Invalid formal hand boundaries")
        for edge in phases.values():
            if edge is None:
                continue
            if not isinstance(edge, dict) or set(edge) not in (REQUIRED_IDENTITY, REQUIRED_IDENTITY | {"rest_sha256"}):
                raise ValueError("Invalid formal hand identity")
            if (not all(type(edge[k]) is str and 0 < len(edge[k]) <= 256 for k in ("source_mode", "session_id", "config_sha256"))
                    or ("rest_sha256" in edge and not (isinstance(edge["rest_sha256"], str) and 0 < len(edge["rest_sha256"]) <= 128))
                    or type(edge["profile_version"]) is not int or not 0 <= edge["profile_version"] <= UINT32_MAX):
                raise ValueError("Incomplete formal hand identity")
    return value


def hand_source_warnings(topics, expected):
    warnings = []
    for hand, edges in expected.items():
        start, finish = edges["start"], edges["finish"]
        if start is None or finish is None:
            warnings.append(dict(code="hand_runtime_boundary_unverified", detail=f"{hand} runtime evidence is missing at a trial boundary"))
        if start is not None and finish is not None and start != finish:
            warnings.append(dict(code="hand_runtime_boundary_changed", detail=f"{hand} runtime identity differs around the recording"))
        for suffix in ("imu/raw", "state"):
            topic = f"/gloves/{hand}/{suffix}"
            observation = topics.get(topic, {}).get("hand_source", {})
            first = observation.get("first")
            if not first or observation.get("unknown_messages") or observation.get("identity_changes"):
                warnings.append(dict(code="recorded_hand_identity_unverified", topic=topic,
                                     detail="Recorded messages do not establish one stable hand identity"))
                continue
            if start is not None:
                fields = dict(hand=hand, source_mode="live", session_id=start["session_id"], config_sha256=start["config_sha256"])
                if suffix == "state":
                    fields["calibration_version"] = start["profile_version"]
                    if "rest_sha256" in start:
                        fields["model_rest_sha256"] = start["rest_sha256"]
                if start["source_mode"] != "live" or any(first.get(k) != v for k, v in fields.items()):
                    warnings.append(dict(code="recorded_hand_identity_mismatch", topic=topic,
                                         detail="Recorded hand identity differs from the runtime snapshot preceding START"))
    return warnings
