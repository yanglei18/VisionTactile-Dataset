import math

import pytest

from vt_vive_tracker.visualization_model import (
    FIXED_ROLES,
    PoseValue,
    RoleSnapshot,
    VisualHealth,
)
from vt_vive_tracker_gui.canvas3d import TrackerCanvasRenderer
from vt_vive_tracker_gui.projection import Camera, camera_for_view


class FakeCanvas:
    def __init__(self):
        self.deleted = []
        self.events = []
        self.lines = []
        self.ovals = []
        self.polygons = []
        self.texts = []

    def delete(self, target):
        self.deleted.append(target)

    def _record(self, kind, collection, coordinates, options):
        call = {"coordinates": coordinates, **options}
        collection.append(call)
        self.events.append((kind, call))

    def create_line(self, *coordinates, **options):
        self._record("line", self.lines, coordinates, options)

    def create_oval(self, *coordinates, **options):
        self._record("oval", self.ovals, coordinates, options)

    def create_polygon(self, *coordinates, **options):
        self._record("polygon", self.polygons, coordinates, options)

    def create_text(self, *coordinates, **options):
        self._record("text", self.texts, coordinates, options)


def pose(position):
    return PoseValue(position, (0.0, 0.0, 0.0, 1.0))


def snapshots(values):
    return tuple(
        values.get(
            role,
            RoleSnapshot(role, VisualHealth.OFFLINE, None, (), 0.0, None),
        )
        for role in FIXED_ROLES
    )


def snapshot(role, health, position, *, trail=()):
    return RoleSnapshot(
        role,
        health,
        pose(position),
        tuple(pose(point) for point in trail),
        60.0,
        None,
    )


def three_fresh_snapshots():
    return snapshots(
        {
            "left_wrist": snapshot(
                "left_wrist",
                VisualHealth.FRESH,
                (-0.5, 0.0, 0.8),
                trail=((-0.7, 0.0, 0.8), (-0.6, 0.0, 0.8)),
            ),
            "right_wrist": snapshot(
                "right_wrist", VisualHealth.FRESH, (0.5, 0.0, 0.8)
            ),
            "torso": snapshot("torso", VisualHealth.FRESH, (0.0, 0.0, 1.3)),
        }
    )


def no_pose_snapshots():
    return snapshots({})


def one_offline_pose():
    return snapshots(
        {
            "torso": snapshot(
                "torso",
                VisualHealth.OFFLINE,
                (0.0, 0.0, 1.0),
                trail=((-0.2, 0.0, 1.0), (-0.1, 0.0, 1.0)),
            )
        }
    )


def test_render_draws_world_axes_three_role_labels_and_health_dots():
    canvas = FakeCanvas()
    renderer = TrackerCanvasRenderer(canvas)

    renderer.render(three_fresh_snapshots(), 800, 600)

    assert canvas.deleted == ["all"]
    assert {call["tags"] for call in canvas.lines} >= {
        "world-axis:x",
        "world-axis:y",
        "world-axis:z",
    }
    assert any(call["tags"] == "grid" for call in canvas.lines)
    assert {call["tags"] for call in canvas.texts} >= {
        "label:left_wrist",
        "label:right_wrist",
        "label:torso",
    }
    assert {call["tags"] for call in canvas.ovals} >= {
        "health:left_wrist",
        "health:right_wrist",
        "health:torso",
    }
    health_dots = {
        call["tags"]: call["fill"]
        for call in canvas.ovals
        if str(call["tags"]).startswith("health:")
    }
    assert set(health_dots.values()) == {"#34c759"}
    tracker_fills = {
        call["tags"]: call["fill"]
        for call in canvas.polygons
        if str(call["tags"]).startswith("tracker:")
    }
    assert tracker_fills == {
        "tracker:left_wrist": "#00d9ff",
        "tracker:right_wrist": "#ff00d9",
        "tracker:torso": "#ff8000",
    }


def test_never_seen_role_is_not_fabricated_at_world_origin():
    canvas = FakeCanvas()

    TrackerCanvasRenderer(canvas).render(no_pose_snapshots(), 800, 600)

    assert not any(
        str(call.get("tags", "")).startswith("tracker:")
        for call in canvas.polygons
    )
    assert not any(
        str(call.get("tags", "")).startswith(("label:", "health:"))
        for call in (*canvas.texts, *canvas.ovals)
    )


def test_offline_last_pose_is_gray_but_keeps_red_health_dot_and_gray_history():
    canvas = FakeCanvas()

    TrackerCanvasRenderer(canvas).render(one_offline_pose(), 800, 600)

    tracker = next(
        call for call in canvas.polygons if call["tags"] == "tracker:torso"
    )
    health = next(call for call in canvas.ovals if call["tags"] == "health:torso")
    history = [call for call in canvas.lines if call["tags"] == "trail:torso"]
    local_axes = [
        call
        for call in canvas.lines
        if str(call["tags"]).startswith("local-axis:torso:")
    ]
    assert tracker["fill"] == "#666666"
    assert health["fill"] == "#ff3b30"
    assert history and {call["fill"] for call in history} == {"#666666"}
    assert len(local_axes) == 3
    assert {call["fill"] for call in local_axes} == {"#666666"}


def test_trail_points_use_role_color_and_local_axes_use_xyz_colors():
    canvas = FakeCanvas()

    TrackerCanvasRenderer(canvas).render(three_fresh_snapshots(), 800, 600)

    trail = [call for call in canvas.lines if call["tags"] == "trail:left_wrist"]
    local_axes = [
        call
        for call in canvas.lines
        if str(call["tags"]).startswith("local-axis:left_wrist:")
    ]
    assert len(trail) == 1
    assert {call["fill"] for call in trail} == {"#00d9ff"}
    assert trail[0]["width"] == 4.0
    assert {call["fill"] for call in local_axes} == {"red", "green", "blue"}


def test_long_trail_uses_one_bounded_canvas_primitive():
    canvas = FakeCanvas()
    trail = tuple((index / 1000.0, 0.0, 0.8) for index in range(375))
    roles = snapshots(
        {
            "left_wrist": snapshot(
                "left_wrist",
                VisualHealth.FRESH,
                (0.4, 0.0, 0.8),
                trail=trail,
            )
        }
    )

    TrackerCanvasRenderer(canvas).render(roles, 800, 600)

    trail_lines = [
        call for call in canvas.lines if call["tags"] == "trail:left_wrist"
    ]
    trail_ovals = [
        call for call in canvas.ovals if call["tags"] == "trail:left_wrist"
    ]
    assert len(trail_lines) == 1
    assert len(trail_lines[0]["coordinates"]) <= 192
    assert trail_ovals == []


def test_trail_does_not_bridge_across_an_unprojectable_sample():
    canvas = FakeCanvas()
    camera = Camera(yaw=0.0, pitch=0.0, distance=1.0)
    roles = snapshots(
        {
            "left_wrist": snapshot(
                "left_wrist",
                VisualHealth.FRESH,
                (-0.3, 0.0, 0.0),
                trail=(
                    (-0.2, -0.1, 0.0),
                    (1.0, 0.0, 0.0),
                    (-0.2, 0.1, 0.0),
                ),
            )
        }
    )

    TrackerCanvasRenderer(canvas, camera=camera).render(roles, 800, 600)

    trail_lines = [
        call for call in canvas.lines if call["tags"] == "trail:left_wrist"
    ]
    trail_ovals = [
        call for call in canvas.ovals if call["tags"] == "trail:left_wrist"
    ]
    assert trail_lines == []
    assert len(trail_ovals) == 2
    assert {call["fill"] for call in trail_ovals} == {"#00d9ff"}


def test_long_trail_keeps_raw_unprojectable_sample_as_a_run_boundary():
    canvas = FakeCanvas()
    camera = Camera(yaw=0.0, pitch=0.0, distance=1.0)
    trail = tuple(
        (
            (1.0, 0.0, 0.0)
            if index == 48
            else (-0.2, (index - 48) / 1000.0, 0.0)
        )
        for index in range(97)
    )
    roles = snapshots(
        {
            "left_wrist": snapshot(
                "left_wrist",
                VisualHealth.FRESH,
                (-0.3, 0.0, 0.0),
                trail=trail,
            )
        }
    )

    TrackerCanvasRenderer(canvas, camera=camera).render(roles, 800, 600)

    trail_lines = [
        call for call in canvas.lines if call["tags"] == "trail:left_wrist"
    ]
    trail_ovals = [
        call for call in canvas.ovals if call["tags"] == "trail:left_wrist"
    ]
    assert len(trail_lines) == 2
    assert [len(call["coordinates"]) for call in trail_lines] == [96, 96]
    assert trail_ovals == []


def test_long_trail_evenly_samples_at_most_96_points_and_keeps_endpoints():
    canvas = FakeCanvas()
    camera = Camera(yaw=0.0, pitch=0.0, distance=5.0)
    trail = tuple((0.0, index / 1000.0, 0.0) for index in range(375))
    roles = snapshots(
        {
            "left_wrist": snapshot(
                "left_wrist",
                VisualHealth.FRESH,
                (0.0, 0.4, 0.0),
                trail=trail,
            )
        }
    )

    TrackerCanvasRenderer(canvas, camera=camera).render(roles, 800, 600)

    trail_lines = [
        call for call in canvas.lines if call["tags"] == "trail:left_wrist"
    ]
    trail_ovals = [
        call for call in canvas.ovals if call["tags"] == "trail:left_wrist"
    ]
    assert len(trail_lines) == 1
    screen_coordinates = trail_lines[0]["coordinates"]
    assert len(screen_coordinates) == 192
    assert len(trail_lines) + len(trail_ovals) <= 96

    focal_length = 300.0 / math.tan(math.radians(25.0))
    sampled_indices = tuple(
        round((screen_x - 400.0) * 5.0 / focal_length * 1000.0)
        for screen_x in screen_coordinates[::2]
    )
    assert sampled_indices[0] == 0
    assert sampled_indices[-1] == 374
    assert set(
        later - earlier
        for earlier, later in zip(sampled_indices, sampled_indices[1:])
    ) == {3, 4}
    assert tuple(screen_coordinates[1::2]) == pytest.approx((300.0,) * 96)


def test_zero_one_and_partially_unprojectable_trail_points_are_safe():
    canvas = FakeCanvas()
    camera = Camera(yaw=0.0, pitch=0.0, distance=1.0)
    roles = snapshots(
        {
            "left_wrist": snapshot(
                "left_wrist", VisualHealth.FRESH, (-0.3, 0.0, 0.0)
            ),
            "right_wrist": snapshot(
                "right_wrist",
                VisualHealth.DELAYED,
                (-0.3, 0.0, 0.0),
                trail=((1.0, 0.0, 0.0), (-0.2, 0.1, 0.2)),
            ),
            "torso": snapshot(
                "torso",
                VisualHealth.FRESH,
                (-0.3, 0.0, 0.0),
                trail=((1.0, 0.0, 0.0), (2.0, 0.0, 0.0)),
            ),
        }
    )

    TrackerCanvasRenderer(canvas, camera=camera).render(roles, 800, 600)

    trail_lines = [
        call
        for call in canvas.lines
        if str(call["tags"]).startswith("trail:")
    ]
    trail_ovals = [
        call
        for call in canvas.ovals
        if str(call["tags"]).startswith("trail:")
    ]
    assert trail_lines == []
    assert len(trail_ovals) == 1
    assert trail_ovals[0]["tags"] == "trail:right_wrist"
    assert trail_ovals[0]["fill"] == "#ff00d9"
    assert trail_ovals[0]["stipple"] == "gray50"
    x0, y0, x1, y1 = trail_ovals[0]["coordinates"]
    assert ((x0 + x1) * 0.5, (y0 + y1) * 0.5) != (400.0, 300.0)


def test_delayed_geometry_keeps_role_color_with_reduced_emphasis():
    canvas = FakeCanvas()
    delayed = snapshots(
        {
            "right_wrist": snapshot(
                "right_wrist", VisualHealth.DELAYED, (0.0, 0.0, 1.0)
            )
        }
    )

    TrackerCanvasRenderer(canvas).render(delayed, 800, 600)

    tracker = next(
        call
        for call in canvas.polygons
        if call["tags"] == "tracker:right_wrist"
    )
    health = next(
        call for call in canvas.ovals if call["tags"] == "health:right_wrist"
    )
    assert tracker["fill"] == "#ff00d9"
    assert tracker["stipple"] == "gray50"
    assert health["fill"] == "#ffcc00"


def test_depth_sorted_trackers_draw_far_to_near():
    canvas = FakeCanvas()
    camera = Camera(yaw=0.0, pitch=0.0, distance=5.0)
    roles = snapshots(
        {
            "left_wrist": snapshot(
                "left_wrist", VisualHealth.FRESH, (1.0, 0.0, 0.0)
            ),
            "right_wrist": snapshot(
                "right_wrist", VisualHealth.FRESH, (-1.0, 0.0, 0.0)
            ),
        }
    )

    TrackerCanvasRenderer(canvas, camera=camera).render(roles, 800, 600)

    tracker_tags = [
        call["tags"]
        for kind, call in canvas.events
        if kind == "polygon" and str(call["tags"]).startswith("tracker:")
    ]
    assert tracker_tags == ["tracker:right_wrist", "tracker:left_wrist"]


def test_set_view_orbit_zoom_and_reset_replace_the_camera():
    renderer = TrackerCanvasRenderer(FakeCanvas())
    original = renderer.camera

    renderer.set_view("side")
    assert renderer.camera == camera_for_view("side")
    assert renderer.camera is not original

    side = renderer.camera
    renderer.orbit(0.2, -0.1)
    assert renderer.camera == side.orbit(0.2, -0.1)

    orbited = renderer.camera
    renderer.zoom(2.0)
    assert renderer.camera == orbited.zoom(2.0)

    renderer.reset_view()
    assert renderer.camera == Camera()


def test_fit_all_ignores_roles_without_a_pose():
    renderer = TrackerCanvasRenderer(
        FakeCanvas(), camera=Camera(target=(-4.0, -5.0, -6.0), distance=9.0)
    )
    roles = snapshots(
        {
            "left_wrist": RoleSnapshot(
                "left_wrist",
                VisualHealth.OFFLINE,
                None,
                (pose((100.0, 100.0, 100.0)),),
                0.0,
                None,
            ),
            "torso": snapshot("torso", VisualHealth.FRESH, (2.0, 3.0, 4.0)),
        }
    )

    renderer.fit_all(roles)

    assert renderer.camera.target == pytest.approx((2.0, 3.0, 4.0))
    assert renderer.camera.distance == pytest.approx(1.0)


def test_grid_is_bounded_and_world_axes_are_z_up_xyz_colors():
    canvas = FakeCanvas()

    TrackerCanvasRenderer(canvas).render(no_pose_snapshots(), 800, 600)

    grid = [call for call in canvas.lines if call["tags"] == "grid"]
    world_axes = {
        call["tags"]: call
        for call in canvas.lines
        if str(call["tags"]).startswith("world-axis:")
    }
    assert 0 < len(grid) <= 50
    assert {
        tag: call["fill"] for tag, call in world_axes.items()
    } == {
        "world-axis:x": "red",
        "world-axis:y": "green",
        "world-axis:z": "blue",
    }
    z_coordinates = world_axes["world-axis:z"]["coordinates"]
    assert z_coordinates[3] < z_coordinates[1]
