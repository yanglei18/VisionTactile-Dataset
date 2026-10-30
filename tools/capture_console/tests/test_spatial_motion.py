"""Camera choreography never advances, rewrites, or refits a measured scene."""
from copy import deepcopy
import math

import pytest
from PySide6.QtCore import Qt, QPoint, QRectF
from PySide6.QtTest import QTest

from vt_capture_console.backend.scene_demo import DemoSceneBackend, demo_scene
from vt_capture_console.ui.widgets.spatial_page import SpatialPage
from vt_capture_console.ui.widgets.spatial_scene import scene_bounds_points, scene_lines, scene_surfaces
from vt_capture_console.viewmodels.spatial import SpatialViewModel, entity_color


def test_hub_y_up_grid_and_ceiling_share_display_coordinate_conversion():
    from vt_capture_console.viewmodels.spatial import display_point
    frame = dict(demo_scene(0), world_frame='vive_hub_map', trackers=[], cameras=[],
                 display_from_world=dict(translation_m=[0,0,0],
                    quaternion_xyzw=[math.sqrt(.5),0,0,math.sqrt(.5)]))
    original = deepcopy(frame)
    model = SpatialViewModel()
    model.layers = {'grid'}
    model.update(frame)
    grid = [line for line in scene_lines(model) if line[2] == '#303030']
    assert len(grid) == 42
    assert all(abs(point[2]) < 1e-10 for a,b,*_ in grid for point in (a,b))
    # A native +Y ceiling at 2 m appears overhead, not as a vertical front wall.
    assert display_point(frame, (.2,2.,.3)) == pytest.approx((.2,-.3,2.))
    assert frame == original


def test_non_hub_reference_grid_retains_original_xy_plane():
    frame = dict(demo_scene(0), world_frame='vive_map', trackers=[], cameras=[])
    model = SpatialViewModel()
    model.layers = {'grid'}
    model.update(frame)
    grid = [line for line in scene_lines(model) if line[2] == '#303030']
    assert all(point[2] == 0 for a,b,*_ in grid for point in (a,b))


def test_fit_includes_cloud_extent_and_respects_depth_and_visibility():
    import numpy as np
    frame=dict(demo_scene(0),trackers=[],cameras=[])
    model=SpatialViewModel()
    model.layers={'cloud'}
    model.update(frame)
    cloud=dict(positions=np.array([[0.,0.,1.],[2.,3.,4.],[0.,0.,float('nan')]]),
               header=dict(world_from_color=dict(translation_m=[10.,0.,0.],quaternion_xyzw=[0,0,0,1])))
    bounds=scene_bounds_points(model,{'camera':cloud})
    assert len(bounds)==8
    assert max(p[0] for p in bounds)==12.
    assert max(p[2] for p in bounds)==4.
    clipped=scene_bounds_points(model,{'camera':cloud},max_depth=2.)
    assert max(p[2] for p in clipped)==1.
    model.hidden_entities.add('camera')
    assert scene_bounds_points(model,{'camera':cloud})==[]


def test_full_display_does_not_decimate_received_points():
    import numpy as np
    from vt_capture_console.ui.widgets.spatial_scene import display_cloud_data
    from vt_capture_core.spatial.cloud_protocol import MAX_POINTS
    cloud=dict(positions=np.ones((100001,3),dtype='f4'),colors=np.ones((100001,4),dtype='f4'))
    positions,_=display_cloud_data(cloud,max_points=MAX_POINTS)
    assert len(positions)==100001


def test_cloud_fit_supports_all_points_without_rotated_empty_box_margin():
    import numpy as np
    from vt_capture_console.ui.widgets.spatial_scene import scene_fit_points,camera_fit_bounds,camera_components
    from vt_capture_console.viewmodels.spatial import transform,display_point
    frame=dict(demo_scene(0),trackers=[],cameras=[],display_from_world=dict(
        translation_m=[.4,-.2,.1],quaternion_xyzw=[math.sqrt(.5),0,0,math.sqrt(.5)]))
    model=SpatialViewModel();model.layers={'cloud'};model.update(frame)
    # A sloping thin patch: its local XYZ box contains much empty space.
    t=np.linspace(-2,2,1001)
    xyz=np.column_stack((t,t*.7,t*.4+2.))
    pose=dict(translation_m=[2,-1,.5],quaternion_xyzw=[0,0,math.sin(.3),math.cos(.3)])
    cloud=dict(positions=xyz,header=dict(world_from_color=pose))
    ht,vt=.8*math.tan(math.pi/6),.8*math.tan(math.pi/6)/2.5
    support=scene_fit_points(model,{'cam':cloud},0.,-55.,25.,horizontal_tan=ht,vertical_tan=vt)
    center,projected=camera_fit_bounds(support,-55.,25.)
    distance=max(max(z+abs(x)/ht,z+abs(y)/vt) for x,y,z in projected)
    all_points=[display_point(frame,transform(p,pose['translation_m'],pose['quaternion_xyzw'])) for p in xyz]
    exact_center,exact=camera_fit_bounds(all_points,-55.,25.)
    exact_distance=max(max(z+abs(x)/ht,z+abs(y)/vt) for x,y,z in exact)
    assert center==pytest.approx(exact_center)
    assert distance==pytest.approx(exact_distance)
    assert len(support)<=14
    for x,y,z in exact:
        assert distance-z>=max(abs(x)/ht,abs(y)/vt)-1e-8
    # Nothing is deleted, even an extreme isolated measured point.
    assert len(cloud['positions'])==1001


@pytest.fixture
def scene_page(qapp):
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    page.resize(1200, 760)
    page.show()
    page.apply_frame(demo_scene(0, scenario="static"), {"state": "ready"})
    qapp.processEvents()
    yield page
    page.close()
    backend.close()


def test_initial_fit_uses_visible_geometry_and_preserves_manual_camera(scene_page, qapp):
    page = scene_page
    page.cruise_button.setChecked(False)
    viewport = page.scene.viewport
    points = [viewport._project(point) for point in scene_bounds_points(page.model)]
    coverage = max((max(p.x() for p in points)-min(p.x() for p in points))/viewport.width(),
                   (max(p.y() for p in points)-min(p.y() for p in points))/viewport.height())
    assert .55 <= coverage <= .65

    viewport.scale *= 1.6
    viewport.target = (.2, .5, 1.)
    manual_scale = viewport.scale
    page.apply_frame(demo_scene(2), {"state": "ready"})
    assert viewport.scale == manual_scale
    assert viewport.target == (.2, .5, 1.)

    frame = demo_scene(2)
    frame["coordinate_epoch"] = "new-space"
    page.apply_frame(frame, {"state": "ready"})
    assert viewport.target != (.2, .5, 1.)
    viewport.scale = 30.
    page.scene.standard_view("reset")
    assert viewport.scale > 30.


def test_cruise_moves_only_camera_and_obeys_freeze_visibility_and_motion(scene_page, qapp):
    page = scene_page
    viewport = page.scene.viewport
    assert page.cruise_button.isChecked()
    assert page.scene.motion_timer.isActive()
    frame = page.model.frame
    angle = viewport.azimuth
    QTest.qWait(95)
    assert viewport.azimuth != angle
    assert page.model.frame is frame

    page.freeze_button.setChecked(True)
    angle = viewport.azimuth
    QTest.qWait(80)
    assert viewport.azimuth == angle
    assert not page.scene.motion_timer.isActive()
    page.freeze_button.setChecked(False)
    assert page.scene.motion_timer.isActive()

    page.set_motion_enabled(False)
    assert not page.scene.motion_timer.isActive()
    page.set_motion_enabled(True)
    assert page.scene.motion_timer.isActive()
    page.hide()
    assert not page.scene.motion_timer.isActive()
    page.show()
    qapp.processEvents()
    assert page.scene.motion_timer.isActive()
    page.showMinimized()
    qapp.processEvents()
    assert not page.scene.motion_timer.isActive()
    page.showNormal()
    qapp.processEvents()
    assert page.scene.motion_timer.isActive()

    QTest.mousePress(viewport, Qt.MouseButton.LeftButton, pos=QPoint(30, 30))
    QTest.mouseRelease(viewport, Qt.MouseButton.LeftButton, pos=QPoint(30, 30))
    assert not page.cruise_button.isChecked()
    assert not page.scene.motion_timer.isActive()


def test_live_camera_starts_still_and_global_motion_reaches_detached_window(qapp):
    backend = DemoSceneBackend()
    backend.mode = "live"
    page = SpatialPage(backend)
    try:
        page.show()
        frame = demo_scene(0)
        frame["mode"] = "live"
        page.apply_frame(frame, {"state": "ready"})
        qapp.processEvents()
        assert not page.cruise_button.isChecked()
        assert not page.scene.motion_timer.isActive()
        page.set_motion_enabled(False)
        page.open_detail()
        detail = page.detail_window.page
        detail.cruise_button.setChecked(True)
        assert not detail.scene.motion_timer.isActive()
        page.set_motion_enabled(True)
        assert detail.scene.motion_timer.isActive()
    finally:
        page.close()
        backend.close()


def test_frusta_are_outlines_without_occluding_surfaces(scene_page):
    model = scene_page.model
    frame = model.frame
    surfaces = scene_surfaces(model)
    lines = scene_lines(model)
    source = repr(frame)  # Published frames are recursively frozen mappings.
    assert all(0 < alpha <= 1 for _vertices, _color, alpha in surfaces)
    assert model.frame is frame
    model.layers.discard("frustum")
    # Frusta add calibrated edges, never filled faces / depth-buffer occluders.
    assert surfaces == scene_surfaces(model)
    assert len(lines) - len(scene_lines(model)) == 3*12
    assert model.frame is frame and repr(model.frame) == source


@pytest.mark.parametrize('vertices', [
    [(0,0,0),(-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)],
    [(-.1,-.1,.1),(.1,-.1,.1),(.1,.1,.1),(-.1,.1,.1),
     (-1,-1,1),(1,-1,1),(1,1,1),(-1,1,1)],
])
def test_frustum_wireframe_preserves_geometry_bounds_and_visibility(vertices):
    from vt_capture_console.viewmodels.spatial import display_point, transform
    frame = demo_scene(0, scenario='static')
    frame['trackers'] = []
    frame['cameras'] = [frame['cameras'][0]]
    camera = frame['cameras'][0]
    camera.update(frustum_vertices=vertices, position_m=[1,2,3],
                  quaternion_xyzw=[0,0,math.sqrt(.5),math.sqrt(.5)])
    original = deepcopy(frame)
    model = SpatialViewModel(); model.layers = {'camera'}; model.update(frame)
    frozen_frame = model.frame
    body_surfaces = scene_surfaces(model); body_lines = scene_lines(model)
    model.layers.add('frustum')
    lines = scene_lines(model)
    assert scene_surfaces(model) == body_surfaces
    edges = lines[len(body_lines):]
    assert len(edges) == (8 if len(vertices)==5 else 12)
    expected = [display_point(frame,transform(p,camera['position_m'],camera['quaternion_xyzw'])) for p in vertices]
    assert {tuple(p) for a,b,*_ in edges for p in (a,b)} == {tuple(p) for p in expected}
    bounds = scene_bounds_points(model)
    assert all(tuple(p) in bounds for p in expected)
    model.hidden_entities.add(camera['id'])
    assert scene_lines(model) == [] and scene_surfaces(model) == []
    assert frame == original and model.frame is frozen_frame


def test_tracker_shell_and_outline_follow_pose_and_display_transform_without_mutation():
    frame = demo_scene(0, scenario="static")
    frame["cameras"] = []
    frame["trackers"] = [frame["trackers"][0]]
    tracker = frame["trackers"][0]
    tracker.update(position_m=[0., 0., 0.], quaternion_xyzw=[0., 0., 0., 1.])
    frame.pop("display_from_world", None)
    model = SpatialViewModel()
    model.layers = {"tracker"}
    model.update(frame)
    original_surfaces = scene_surfaces(model)
    original_lines = scene_lines(model, include_grid=False)

    tracker.update(position_m=[1., 2., 3.], quaternion_xyzw=[0., 0., math.sqrt(.5), math.sqrt(.5)])
    frame["display_from_world"] = {"translation_m": [4., 5., 6.], "quaternion_xyzw": [1., 0., 0., 0.]}
    source_before = deepcopy(frame)
    model.update(frame)
    rendered_surfaces = scene_surfaces(model)
    rendered_lines = scene_lines(model, include_grid=False)
    bounds = scene_bounds_points(model)

    # Independently composed +90 degrees about Z, measured translation, then
    # 180 degrees about X and the display translation. No glyph vertex offsets
    # or topology are assumed, so this catches a shell detached from its pose.
    def expected(point):
        x, y, z = point
        return 5. - y, 3. - x, 3. - z

    for before, after in zip(original_surfaces, rendered_surfaces, strict=True):
        assert before[1:] == after[1:]
        for source, rendered in zip(before[0], after[0], strict=True):
            assert rendered == pytest.approx(expected(source))
            assert all(min(p[axis] for p in bounds) - 1e-10 <= rendered[axis]
                       <= max(p[axis] for p in bounds) + 1e-10 for axis in range(3))
    for before, after in zip(original_lines, rendered_lines, strict=True):
        assert before[2:] == after[2:]
        assert after[0] == pytest.approx(expected(before[0]))
        assert after[1] == pytest.approx(expected(before[1]))
    assert frame == source_before


@pytest.mark.parametrize("state", ["stale", "invalid", "offline"])
def test_tracker_status_changes_only_identity_accents_to_gray(state):
    frame = demo_scene(0, scenario="static")
    frame["cameras"] = []
    frame["trackers"] = [frame["trackers"][0]]
    tracker = frame["trackers"][0]
    tracker["state"] = "fresh"
    model = SpatialViewModel()
    model.layers = {"tracker"}
    model.update(frame)
    fresh_surfaces = scene_surfaces(model)
    fresh_lines = scene_lines(model, include_grid=False)
    accent = entity_color(tracker)
    assert any(color == accent for _vertices, color, _alpha in fresh_surfaces)
    assert any(color == accent for _a, _b, color, _width in fresh_lines)

    tracker["state"] = state
    model.update(frame)
    for before, after in zip(fresh_surfaces, scene_surfaces(model), strict=True):
        assert before[0] == after[0]
        assert after[1] == ("#adadad" if before[1] == accent else before[1])
    for before, after in zip(fresh_lines, scene_lines(model, include_grid=False), strict=True):
        assert before[:2] == after[:2]
        assert after[2] == ("#adadad" if before[2] == accent else before[2])


@pytest.mark.parametrize("size", [(1200, 760), (900, 600)])
def test_annotation_panels_stay_in_view_and_select_the_attached_entity(scene_page, qapp, size):
    page = scene_page
    page.cruise_button.setChecked(False)
    page.resize(*size)
    qapp.processEvents()
    page.scene.fit()
    qapp.processEvents()
    viewport = page.scene.viewport
    placements = viewport._label_placements
    assert len(placements) == len(page.model.entities())
    safe_bounds = QRectF(viewport.rect()).adjusted(10., 10., -10., -30.)
    assert all(safe_bounds.contains(placement.rect) for placement in placements)
    plate = next(p for p in placements if p.identifier != page.model.selected_id)
    QTest.mouseClick(viewport, Qt.MouseButton.LeftButton, pos=plate.rect.center().toPoint())
    qapp.processEvents()
    assert page.model.selected_id == plate.identifier
