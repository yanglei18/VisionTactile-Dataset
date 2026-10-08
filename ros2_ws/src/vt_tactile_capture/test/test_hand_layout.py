"""Candidate geometry must never imply that all raw addresses were calibrated."""

from dataclasses import FrozenInstanceError
import json
import math
from pathlib import Path

import pytest

from vt_tactile_capture.hand_layout import load_layout
import vt_tactile_capture.hand_layout as module


def original_config():
    return json.loads((Path(module.__file__).parent / "layouts" / "right_glove_candidate.json").read_text())


def load_modified(tmp_path, config):
    path = tmp_path / "layout.json"
    path.write_text(json.dumps(config), encoding="utf-8")
    return load_layout(path)


def test_default_address_coverage_and_candidate_provenance():
    layout = load_layout()
    assert layout.hand == "right" and layout.status == "candidate"
    assert layout.raw_rows == layout.raw_cols == 20
    assert len(layout.points) == 400
    assert {(p.row, p.col) for p in layout.points} == {(r, c) for r in range(20) for c in range(20)}
    assert len(set(layout.covered_indices)) == 400
    assert layout.covered_indices == tuple(p.row * 20 + p.col for p in layout.points)
    assert all(math.isfinite(p.x) and math.isfinite(p.y) for p in layout.points)
    assert len({(p.x, p.y) for p in layout.points}) == 400
    assert not any(p.row == 20 for p in layout.points)
    assert "未经逐点验证" in layout.note and "其余协议地址无效" in layout.note
    assert original_config()["source"]["date"] == "2026-10-05"


def test_observed_anchors_and_region_sizes():
    layout = load_layout()
    assert {(p.row, p.col) for p in layout.points if p.verified_anchor} == {
        (19, 19), (18, 15), (19, 11), (18, 7), (18, 1),
        (4, 9), (4, 10), (0, 9), (0, 10),
    }
    assert sum(p.region == "palm" for p in layout.points) == 160
    for region in ("thumb", "index", "middle", "ring", "little"):
        assert sum(p.region == region for p in layout.points) == 48
    assert sum(p.verified_anchor for p in layout.points) == 9


def test_right_palm_facing_viewer_order_and_wrist_to_tip_direction():
    points = {(p.row, p.col): p for p in load_layout().points}
    tips = [points[key] for key in [(19, 19), (18, 15), (19, 11), (18, 7), (18, 1)]]
    assert [p.x for p in tips] == sorted(p.x for p in tips)
    assert points[0, 9].y < points[4, 9].y < points[7, 9].y < points[8, 9].y < points[19, 9].y
    for col in range(20):
        assert points[19, col].y > points[8, col].y
    assert points[19, 19].x < points[8, 19].x
    assert tips[2].y > tips[1].y > tips[4].y


def test_outline_closed_simple_and_contains_sensor_centers():
    layout = load_layout()
    polygon = layout.outline
    assert polygon[0] == polygon[-1]

    def cross(a, b, c):
        return (b[0] - a[0]) * (c[1] - a[1]) - (b[1] - a[1]) * (c[0] - a[0])

    segments = list(zip(polygon, polygon[1:]))
    for i, (a, b) in enumerate(segments):
        for j, (c, d) in enumerate(segments):
            if j <= i + 1 or (i == 0 and j == len(segments) - 1):
                continue
            assert not (cross(a, b, c) * cross(a, b, d) < 0 and cross(c, d, a) * cross(c, d, b) < 0)
    for point in layout.points:
        intersections = sum(
            (a[1] > point.y) != (b[1] > point.y)
            and point.x < (b[0] - a[0]) * (point.y - a[1]) / (b[1] - a[1]) + a[0]
            for a, b in segments
        )
        assert intersections % 2 == 1, (point.row, point.col, point.x, point.y)


@pytest.mark.parametrize("key,value", [
    ("schema_version", 2), ("schema_version", True), ("hand", "both"),
    ("status", "verified"), ("raw_rows", 0), ("raw_cols", 81),
    ("raw_rows", True), ("name", ""), ("note", ""),
    ("regions", []), ("regions", {}), ("anchors", {}),
    ("outline", []), ("labels", {}), ("source", {}),
])
def test_invalid_top_config_rejected(tmp_path, key, value):
    config = original_config()
    config[key] = value
    with pytest.raises(ValueError):
        load_modified(tmp_path, config)


@pytest.mark.parametrize("key,value", [
    ("region", "unknown"), ("rows", [-1, 8]), ("rows", [0, 81]),
    ("cols", [20, 20]), ("cols", [0, 1.5]), ("rows", [False, 8]),
    ("origin", [float("nan"), 0]), ("row_step", [0, float("inf")]),
    ("col_step", [0, 0]), ("origin", [True, 0]), ("row_step", [0]),
])
def test_bad_grid_config_rejected(tmp_path, key, value):
    config = original_config()
    config["regions"][0][key] = value
    with pytest.raises(ValueError):
        load_modified(tmp_path, config)


@pytest.mark.parametrize("anchors", [[[80, 0]], [[20, 0]], [[0, 0], [0, 0]], [[True, 0]], [[0]], [[0.0, 0]]])
def test_bad_or_unmapped_anchors_rejected(tmp_path, anchors):
    config = original_config()
    config["anchors"] = anchors
    with pytest.raises(ValueError):
        load_modified(tmp_path, config)


def test_duplicate_raw_address_and_geometry_rejected(tmp_path):
    config = original_config()
    config["regions"][1]["rows"] = [0, 8]
    with pytest.raises(ValueError, match="duplicate raw address"):
        load_modified(tmp_path, config)
    config = original_config()
    config["regions"][1]["origin"] = config["regions"][0]["origin"].copy()
    with pytest.raises(ValueError, match="duplicate schematic position"):
        load_modified(tmp_path, config)


def test_unknown_keys_and_duplicate_json_keys_rejected(tmp_path):
    config = original_config()
    config["unexpected"] = "probably a typo"
    with pytest.raises(ValueError):
        load_modified(tmp_path, config)
    path = tmp_path / "duplicate.json"
    path.write_text('{"schema_version": 1, "schema_version": 3}')
    with pytest.raises(ValueError, match="duplicate JSON key"):
        load_layout(path)


def test_geometry_is_immutable_and_bad_path_is_not_silently_defaulted(tmp_path):
    layout = load_layout()
    with pytest.raises(FrozenInstanceError):
        layout.points[0].x = 0
    with pytest.raises(FileNotFoundError):
        load_layout(tmp_path / "does_not_exist.json")


def test_finger_cells_are_separated_and_thumb_does_not_overlap_palm():
    layout = load_layout()
    thumb = [point for point in layout.points if point.region == "thumb"]
    palm = [point for point in layout.points if point.region == "palm"]
    assert max(point.x for point in thumb) < min(point.x for point in palm)
    assert min(math.hypot(a.x - b.x, a.y - b.y) for a in thumb for b in palm) >= 1.3
    # Same-region point spacing supports visible gaps between rendered cells.
    assert min(math.hypot(a.x - b.x, a.y - b.y)
               for i, a in enumerate(layout.points) for b in layout.points[i + 1:]) >= 0.79
    xs, ys = zip(*layout.outline)
    assert 27 <= max(ys) - min(ys) <= 28
    assert max(xs) - min(xs) < 30
    assert len(layout.outline) >= 70  # Rounded fingertip caps, not four-corner blocks.


def test_affine_cells_with_16_percent_gutters_stay_inside_outline():
    layout = load_layout()
    segments = tuple(zip(layout.outline, layout.outline[1:]))
    regions = {region["region"]: region for region in original_config()["regions"]}
    for point in layout.points:
        row_step = regions[point.region]["row_step"]
        col_step = regions[point.region]["col_step"]
        for row_sign, col_sign in ((-1, -1), (-1, 1), (1, -1), (1, 1)):
            x = point.x + 0.42 * (row_sign * row_step[0] + col_sign * col_step[0])
            y = point.y + 0.42 * (row_sign * row_step[1] + col_sign * col_step[1])
            intersections = sum(
                (a[1] > y) != (b[1] > y)
                and x < (b[0] - a[0]) * (y - a[1]) / (b[1] - a[1]) + a[0]
                for a, b in segments
            )
            assert intersections % 2 == 1, (point.row, point.col, x, y)


def test_left_requires_explicit_profile_and_candidate_does_not_inherit_verified_anchors():
    with pytest.raises(ValueError, match="explicit left layout"):
        load_layout(expected_hand="left")
    path = Path(module.__file__).parent / "layouts/left_glove_unverified_candidate.json"
    layout = load_layout(path, expected_hand="left")
    assert layout.hand == "left" and len(layout.points) == 400
    assert not any(point.verified_anchor for point in layout.points)
    assert "未验证" in layout.note and "没有左手实测锚点" in layout.note
    assert dict(layout.source)["dataset"].startswith("none")
    with pytest.raises(ValueError, match="does not match"):
        load_layout(path, expected_hand="right")


def test_surface_address_directions_are_explicit_and_do_not_change_raw_indices(tmp_path):
    config = original_config()
    config["hand"] = "left"
    config["anchors"] = []
    config["regions"][0]["row_direction"] = -1
    config["regions"][0]["col_direction"] = -1
    layout = load_modified(tmp_path, config)
    assert layout.covered_indices == tuple(range(400))
    assert layout.regions[0].row_direction == layout.regions[0].col_direction == -1
    for invalid in (0, 2, True, "-1"):
        config["regions"][0]["row_direction"] = invalid
        with pytest.raises(ValueError):
            load_modified(tmp_path, config)
