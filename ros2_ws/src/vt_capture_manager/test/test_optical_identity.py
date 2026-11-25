from dataclasses import dataclass

import pytest

from vt_capture_manager.optical.identity import BilateralTracker


@dataclass(frozen=True)
class Observation:
    translation_m: tuple
    # Both physical cubes can present exactly the same tag to the camera.
    tag_ids: tuple = (2,)


FRAME_NS = 33_000_000


def cube(x, z=0.5):
    return Observation((x, 0, z))


def setup_pair():
    tracker = BilateralTracker()
    left, right = cube(.18), cube(-.18)
    tracker.update(FRAME_NS, [left])
    assert tracker.bind("left")["left"].observation == left
    tracker.update(2 * FRAME_NS, [right, left])
    assert tracker.bind("right")["right"].observation == right
    return tracker


def test_binding_needs_a_single_unassigned_cube_and_is_idempotent():
    tracker = BilateralTracker()
    with pytest.raises(ValueError):
        tracker.bind("left")
    with pytest.raises(ValueError):
        tracker.bind("other")
    a, b = cube(-.16), cube(.16)
    states = tracker.update(FRAME_NS, [a, b])
    assert all(s.status == "unbound" and s.observation is None for s in states.values())
    with pytest.raises(ValueError):
        tracker.bind("right")
    tracker.update(2 * FRAME_NS, [a])
    assert tracker.bind("right") == tracker.bind("right")
    with pytest.raises(ValueError):
        tracker.bind("left")
    tracker.update(3 * FRAME_NS, [a, b])
    assert tracker.bind("left")["left"].observation == b


def test_duplicate_tag_ids_and_reversed_image_order_still_follow_operator_labels():
    tracker = setup_pair()
    a, b = cube(.20), cube(-.20)
    for i, visible in enumerate(([a, b], [b, a]), start=3):
        rows = tracker.update(i * FRAME_NS, visible)
        assert rows["left"].observation == a
        assert rows["right"].observation == b


def test_separated_depth_allows_projected_centres_to_overlap():
    tracker = BilateralTracker()
    near, far = cube(0, .4), cube(0, .7)
    tracker.update(FRAME_NS, [near])
    tracker.bind("right")
    tracker.update(2 * FRAME_NS, [near, far])
    tracker.bind("left")
    rows = tracker.update(3 * FRAME_NS, [far, near])
    assert rows["right"].observation == near
    assert rows["left"].observation == far


def test_one_frame_miss_requires_rebind_only_for_the_missing_hand():
    tracker = setup_pair()
    a, b = cube(.18), cube(-.18)
    result = tracker.update(3 * FRAME_NS, [b])
    assert result["left"].status == "lost" and result["left"].observation is None
    assert result["right"].observation == b
    result = tracker.update(4 * FRAME_NS, [a, b])
    assert result["left"].observation is None
    assert result["right"].observation == b
    assert tracker.bind("left")["left"].observation == a


def test_same_position_after_empty_frame_never_auto_rebinds():
    tracker = setup_pair()
    tracker.update(3 * FRAME_NS, [])
    result = tracker.update(4 * FRAME_NS, [cube(-.18)])
    assert all(s.status == "lost" and s.observation is None for s in result.values())


def test_crossing_with_multiple_admissible_matches_drops_both_assignments():
    tracker = BilateralTracker()
    tracker.update(FRAME_NS, [cube(-.055)])
    tracker.bind("left")
    tracker.update(2 * FRAME_NS, [cube(-.055), cube(.055)])
    tracker.bind("right")
    # 55 ms makes both 40-mm-apart new centres reachable from both old centres.
    t = 2 * FRAME_NS + 55_000_000
    result = tracker.update(t, [cube(.02), cube(-.02)])
    assert all(s.status == "ambiguous" and s.observation is None for s in result.values())
    with pytest.raises(ValueError):
        tracker.bind("left")
    result = tracker.update(t + FRAME_NS, [cube(.09), cube(-.09)])
    assert all(s.observation is None for s in result.values())


def test_one_detection_with_two_possible_origins_including_anonymous_is_ambiguous():
    tracker = BilateralTracker()
    tracker.update(FRAME_NS, [cube(-.06)])
    tracker.bind("left")
    tracker.update(2 * FRAME_NS, [cube(-.06), cube(.06)])
    result = tracker.update(3 * FRAME_NS, [cube(.0)])
    assert result["left"].status == "ambiguous" and result["left"].observation is None


@pytest.mark.parametrize("visible", [
    [cube(-.01), cube(.01)], [cube(.18), cube(-.18), cube(.5)],
    [cube(float("nan"))], [cube(.1, 0)], [Observation((1.,))],
])
def test_invalid_or_unresolvable_detections_clear_the_frame_for_binding(visible):
    tracker = setup_pair()
    result = tracker.update(3 * FRAME_NS, visible)
    assert all(s.status == "ambiguous" and s.observation is None for s in result.values())
    with pytest.raises(ValueError):
        tracker.bind("left")


@pytest.mark.parametrize("t", [2 * FRAME_NS, FRAME_NS, 0, 2 * FRAME_NS + 150_000_000])
def test_time_discontinuity_and_camera_reset_cannot_transfer_identity(t):
    tracker = setup_pair()
    result = tracker.update(t, [cube(-.18), cube(.18)])
    assert all(s.status == "lost" and s.observation is None for s in result.values())
    status = tracker.reset()
    assert all(s.status == "unbound" for s in status.values())
    with pytest.raises(ValueError):
        tracker.bind("right")


def test_motion_out_of_reach_is_not_silently_attached_to_previous_hand():
    tracker = BilateralTracker()
    tracker.update(FRAME_NS, [cube(0)])
    tracker.bind("left")
    result = tracker.update(2*FRAME_NS, [cube(.20)])
    assert result["left"].status == "lost" and result["left"].observation is None


@dataclass(frozen=True)
class ClickObservation:
    center_px: tuple
    translation_m: tuple


def at(px, x):
    return ClickObservation((px, 80), (x, 0, .5))


def test_explicit_observation_must_be_current_and_not_owned_by_other_hand():
    tracker = BilateralTracker()
    a, b = cube(.15), cube(-.13)
    tracker.update(FRAME_NS, [a, b])
    assert tracker.bind("left", b)["left"].observation is b
    with pytest.raises(ValueError):
        tracker.bind("right", b)
    with pytest.raises(ValueError):
        tracker.bind("left", a)
    assert tracker.bind("right", a)["right"].observation is a
    matching_copy = cube(-.13)
    with pytest.raises(ValueError):
        tracker.bind("left", matching_copy)
    tracker.update(2 * FRAME_NS, [matching_copy, a])
    with pytest.raises(ValueError):
        tracker.bind("left", b)


def test_click_delayed_by_preview_tracks_anonymous_candidate_to_latest_frame():
    t = BilateralTracker()
    old_a, old_b = at(44, .2), at(210, -.2)
    t.update(FRAME_NS, [old_a, old_b])
    t.update(2 * FRAME_NS, [at(190, -.19), at(67, .18)])
    new_a, new_b = at(97, .15), at(173, -.17)
    t.update(3 * FRAME_NS, [new_a, new_b])
    assert t.bind_at("right", (46, 81), FRAME_NS)["right"].observation is new_a
    assert t.bind_at("left", (210, 80), FRAME_NS)["left"].observation is new_b


def test_dropped_or_ambiguous_candidate_cannot_be_rebound_by_a_delayed_click():
    t = BilateralTracker()
    t.update(FRAME_NS, [at(90, -.06), at(160, .06)])
    t.update(2 * FRAME_NS, [at(113, .0)])
    latest_a = at(90, -.05)
    t.update(3 * FRAME_NS, [latest_a, at(160, .05)])
    for stamp in [FRAME_NS, 2*FRAME_NS]:
        with pytest.raises(ValueError):
            t.bind_at("left", (90 if stamp == FRAME_NS else 113, 80), stamp)
    with pytest.raises(ValueError):
        t.bind("left", latest_a)
    t.update(4 * FRAME_NS, [])
    t.update(5 * FRAME_NS, [at(90, -.06)])
    with pytest.raises(ValueError):
        t.bind_at("left", (90, 80), FRAME_NS)
    assert t.bind_at("left", (90, 80), 5*FRAME_NS)["left"].observation is not None


def test_stale_wrong_frame_nonfinite_background_overlapping_and_reset_clicks_rejected():
    t = BilateralTracker()
    # Optical centres overlap in the preview; 3D observations remain distinct.
    past = [at(90, -.2), at(112, .2)]
    for i in range(1, 17):
        t.update(i*FRAME_NS, past)
    for xy, stamp in [((90, 80), FRAME_NS), ((101, 80), 16*FRAME_NS),
                      ((90, 80), 16*FRAME_NS+1), ((500, 80), 16*FRAME_NS),
                      ((float("nan"), 80), 16*FRAME_NS), ((90,), 16*FRAME_NS)]:
        with pytest.raises(ValueError):
            t.bind_at("left", xy, stamp)
    t.reset()
    with pytest.raises(ValueError):
        t.bind_at("left", (90, 80), 16*FRAME_NS)
