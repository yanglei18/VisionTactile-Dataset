from vt_capture_manager.hand_quality import frozen_hand_expectations, hand_source_warnings, validate_expectations


def test_quality_compares_stored_hand_payloads_with_both_trial_boundaries():
    identity = dict(status="verified_snapshot", source_mode="live", session_id="s", config_sha256="a"*64, profile_version=7, rest_sha256="b"*64)
    metadata = {"hand_runtime_evidence": {"hands": {"left": identity}}}
    expected = validate_expectations(frozen_hand_expectations(["left"], metadata, metadata))
    topics = {}
    for suffix in ("imu/raw", "state"):
        first = dict(hand="left", source_mode="live", session_id="s", config_sha256="a"*64)
        if suffix == "state":
            first.update(calibration_version=7, model_rest_sha256="b"*64)
        topics["/gloves/left/"+suffix] = dict(hand_source=dict(first=first, unknown_messages=0, identity_changes=0))
    assert hand_source_warnings(topics, expected) == []
    pose = topics["/gloves/left/state"]["hand_source"]["first"]
    pose["model_rest_sha256"] = "c"*64
    assert [w["code"] for w in hand_source_warnings(topics, expected)] == ["recorded_hand_identity_mismatch"]
    pose.update(calibration_version=8, model_rest_sha256="b"*64)
    expected["left"]["finish"] = None
    assert [w["code"] for w in hand_source_warnings(topics, expected)] == [
        "hand_runtime_boundary_unverified", "recorded_hand_identity_mismatch"]
