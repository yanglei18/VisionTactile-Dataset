import numpy as np

from vt_capture_console.backend.cloud_display import WristCloudDisplay


def sample():
    z = np.array([.069, .07, .25, .50, .501, 2.], dtype='f4')
    return dict(positions=np.column_stack((np.ones(6), np.zeros(6), z)).astype('f4'),
                colors=np.arange(24, dtype='f4').reshape(6, 4),
                header=dict(worker_instance_id='worker', cloud_sequence=1,
                            world_from_color=dict(translation_m=[10, 20, 30], quaternion_xyzw=[1, 0, 0, 0])))


CAMERAS = [dict(id=name, tracker_role=role) for name, role in
           [('left', 'left_wrist'), ('right', 'right_wrist'), ('head', 'torso')]]


def test_both_wrists_crop_local_z_inclusively_keep_rgb_pose_and_head():
    source = {c['id']: sample() for c in CAMERAS}
    display = WristCloudDisplay()
    filtered = display.select(source, CAMERAS, True)
    for name in ('left', 'right'):
        np.testing.assert_array_equal(filtered[name]['positions'], source[name]['positions'][1:4])
        np.testing.assert_array_equal(filtered[name]['colors'], source[name]['colors'][1:4])
        assert filtered[name]['header'] is source[name]['header']
        assert len(source[name]['positions']) == 6
        assert 'display_range_m' not in source[name]
    assert filtered['head'] is source['head']
    again = display.select(source, CAMERAS, True)
    assert again['left']['positions'] is filtered['left']['positions']
    restored = display.select(source, CAMERAS, False)
    assert restored['left'] is source['left']
    assert not display._cache


def test_empty_range_and_role_changes_release_cached_wrist_data():
    source = {'left': sample()}
    source['left']['positions'][:, 2] = 2.
    display = WristCloudDisplay()
    assert display.select(source, CAMERAS, True)['left']['positions'].shape == (0, 3)
    assert display.select(source, [dict(id='left', tracker_role='torso')], True)['left'] is source['left']
    assert not display._cache


def test_toggle_updates_existing_frame_renderer_and_fit(qapp):
    from vt_capture_console.backend.scene_demo import DemoSceneBackend, demo_scene
    from vt_capture_console.ui.widgets.spatial_page import SpatialPage
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    page.resize(1200, 760)
    page.show()
    frame = demo_scene(0)
    page.apply_frame(frame, dict(state='ready'))
    page.cloud_check.setChecked(True)
    page.model.update(frame)
    page.scene.set_model(page.model)
    wrist = next(c['id'] for c in frame['cameras'] if c['tracker_role'] == 'left_wrist')
    cloud = sample()
    cloud['header'].update(source_realtime_ns=frame['generated_realtime_ns'], point_count=6)
    page.model.set_frozen(True)
    page._clouds = {wrist: cloud}
    page._update_clouds()
    qapp.processEvents()
    assert len(page.scene.viewport._cloud_cache[wrist][1]) == 6
    page.wrist_depth_check.setChecked(True)
    qapp.processEvents()
    assert len(page.scene.viewport._cloud_cache[wrist][1]) == 3
    fitted = page.scene.viewport._fit_clouds[wrist]
    assert max(fitted['positions'][:, 2]) == .5
    assert '3' in page.cloud_limits.text()
    reopened = SpatialPage(backend)
    assert reopened.wrist_depth_check.isChecked()
    reopened.close()
    page.wrist_depth_check.setChecked(False)
    qapp.processEvents()
    assert len(page.scene.viewport._cloud_cache[wrist][1]) == 6
    assert cloud['header']['point_count'] == 6
    page.close()
    backend.close()
