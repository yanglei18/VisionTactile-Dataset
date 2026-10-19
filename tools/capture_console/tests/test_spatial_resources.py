"""Window demand and source-owned arrays end with their actual consumers."""
import gc
import weakref

import numpy as np
from PySide6.QtWidgets import QVBoxLayout, QWidget

from vt_capture_console.backend.scene_backend import SceneBackend
from vt_capture_console.backend.scene_demo import DemoSceneBackend, demo_scene
from vt_capture_console.ui.widgets.spatial_page import SpatialPage
from vt_capture_console.ui.widgets.spatial_scene import SpatialScene, create_gl_view
from vt_capture_console.viewmodels.spatial import SpatialViewModel


def cloud(sequence=1):
    return {
        'header': {'worker_instance_id': 'worker', 'cloud_sequence': sequence,
                   'world_from_color': {'translation_m': [0., 0., 0.],
                                        'quaternion_xyzw': [0., 0., 0., 1.]}},
        'positions': np.ones((100, 3), dtype='f4'),
        'colors': np.ones((100, 4), dtype='f4'),
    }


def test_each_window_releases_demand_without_stopping_other_consumers(qapp):
    backend = SceneBackend(autostart=False)
    host = QWidget()
    embedded = SpatialPage(backend, host)
    QVBoxLayout(host).addWidget(embedded)
    detached = SpatialPage(backend)
    try:
        for page in (embedded, detached):
            page.cloud_check.setChecked(True)
            page.apply_frame(demo_scene(0), {'state': 'ready'})
        host.show()
        detached.show()
        qapp.processEvents()
        assert backend.cloud_backend._interest == {embedded.viewer_id, detached.viewer_id}

        held = detached.model.frame
        detached.freeze_button.setChecked(True)
        assert backend.cloud_backend._interest == {embedded.viewer_id}
        assert detached.model.frame is held

        host.showMinimized()
        qapp.processEvents()
        assert embedded.isVisible()  # This alone is not a consumption contract.
        assert not backend._interests
        assert not backend.cloud_backend._interest

        detached.freeze_button.setChecked(False)
        assert backend.cloud_backend._interest == {detached.viewer_id}
        host.showNormal()
        qapp.processEvents()
        assert backend.cloud_backend._interest == {embedded.viewer_id, detached.viewer_id}
        detached.close()
        assert backend.cloud_backend._interest == {embedded.viewer_id}
        host.hide()
        assert not backend.cloud_backend._interest
        host.show()
        qapp.processEvents()
        assert backend.cloud_backend._interest == {embedded.viewer_id}
    finally:
        embedded.close()
        detached.close()
        host.close()
        backend.close()


def test_cloud_reset_releases_displayed_and_hidden_pending_arrays(qapp):
    scene = SpatialScene()
    model = SpatialViewModel()
    model.update(demo_scene(0))
    scene.set_model(model)
    scene.show()
    qapp.processEvents()
    first = cloud()
    first_array = weakref.ref(first['positions'])
    scene.set_clouds({'first': first})
    scene.hide()
    second = cloud(2)
    second_array = weakref.ref(second['positions'])
    scene.set_clouds({'second': second})
    del first, second
    assert first_array() is not None and second_array() is not None
    scene.clear_clouds()
    gc.collect()
    assert first_array() is None and second_array() is None
    scene.show()
    qapp.processEvents()
    assert not scene.viewport._clouds
    scene.close()


def test_hidden_source_switch_and_close_clear_resources_immediately(qapp, monkeypatch):
    backend = DemoSceneBackend()
    replacement = DemoSceneBackend()
    page = SpatialPage(backend)
    cleared = []
    original = page.scene.clear_clouds
    monkeypatch.setattr(page.scene, 'clear_clouds', lambda: (cleared.append(True), original()))
    try:
        page._activate_backend(replacement)
        assert len(cleared) == 1
        page.close()
        assert len(cleared) == 2
    finally:
        page.close()
        backend.close()
        replacement.close()


def test_gl_cloud_items_are_reused_for_live_sources_and_retired_for_old_ids(qapp):
    # Construct the actual pyqtgraph widget without showing it: no GPU context
    # is created. This exercises scene ownership, not graphics-driver behavior.
    view = create_gl_view(None)
    model = SpatialViewModel()
    model.update(demo_scene(0))
    view.model = model
    base_count = len(view.items)
    retired = []
    try:
        for index in range(100):
            key = f'camera-{index}'
            source = cloud(index)
            view.set_clouds({key: source})
            item = view._cloud_items[key]
            original_data = item.pos
            view.set_clouds({key: source})
            assert view._cloud_items[key] is item and item.pos is original_data
            assert len(view.items) == base_count + 1
            if index:
                assert retired[-1].pos.size == 0
                assert retired[-1] not in view.items
            retired.append(item)
        view.clear_clouds()
        assert len(view.items) == base_count
        assert all(item.pos.size == 0 for item in retired)
        assert not view._cloud_items
    finally:
        view.close()


def test_gl_point_sprite_texture_is_deleted_with_owning_context_current(qapp, monkeypatch):
    from OpenGL import GL
    view = create_gl_view(None)
    model = SpatialViewModel()
    model.update(demo_scene(0))
    view.model = model
    calls = []
    try:
        view.set_clouds({'camera': cloud()})
        item = view._cloud_items['camera']
        item.pointTexture = 123
        monkeypatch.setattr(view, 'isValid', lambda: True)
        monkeypatch.setattr(view, 'makeCurrent', lambda: calls.append('current'))
        monkeypatch.setattr(view, 'doneCurrent', lambda: calls.append('done'))
        monkeypatch.setattr(GL, 'glDeleteTextures', lambda ids: calls.append(ids))
        view.clear_clouds()
        assert calls == ['current', [123], 'done']
        assert item.pointTexture is None
        view.clear_clouds()
        assert len(calls) == 3
    finally:
        view.close()
