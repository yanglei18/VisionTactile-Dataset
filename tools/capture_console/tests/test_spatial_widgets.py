"""Shared spatial windows and fallback drawing work without OpenGL or ROS."""
from PySide6.QtCore import Qt

from vt_capture_console.backend.scene_demo import DemoSceneBackend
from vt_capture_console.ui.widgets.spatial_page import SpatialPage


def test_page_freeze_independent_window_and_no_new_backend(qapp):
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    page.resize(1000, 650)
    page.show()
    page.refresh()
    qapp.processEvents()
    assert len(page.model.entities()) == 6
    assert page.scene.render_mode == "projection"
    page.open_detail()
    detail = page.detail_window
    assert detail is not None and not detail.isModal()
    assert detail.page.backend is backend
    assert detail.testAttribute(Qt.WidgetAttribute.WA_DeleteOnClose)
    detail.page.freeze_button.setChecked(True)
    saved = detail.page.model.frame
    page.refresh()
    assert detail.page.model.frame is saved
    assert "合成数据" in page.status.text()
    page.close()
    assert page.detail_window is None
    backend.close()


def test_spatial_navigation_preserves_device_tools_page_id(window):
    assert window.pages.count() == 5
    window.navigation.button(3).click()
    window.refresh()
    assert window.pages.currentWidget() is window.spatial_page
    assert window.page_title.text() == "空间"
    assert window.spatial_page.model.frame["mode"] == "demo"
    window.navigation.button(2).click()
    assert window.pages.currentIndex() == 2


def test_settings_preserves_configuration_and_uses_runtime_apply(qapp, tmp_path):
    from vt_capture_console.ui.widgets.spatial_settings import SpatialSettingsDialog
    class Runtime:
        configuration = {"process_groups": [], "disk_path": "/tmp"}
        calls = []
        def apply(self, configuration, snapshot):
            self.calls.append((configuration, snapshot))
            return True
    runtime = Runtime()
    path = tmp_path / "scene.yaml"
    path.write_text("schema_version: 1\n")
    dialog = SpatialSettingsDialog(runtime, lambda: {"instance_id": "test"})
    dialog.path.setText(str(path))
    dialog.save.click()
    assert runtime.calls[0][0]["scene_config_path"] == str(path)
    assert "scene_config_path" not in runtime.configuration
    dialog.close()


def test_replay_selection_does_not_show_live_camera_previews(qapp, tmp_path):
    import json
    import time
    from vt_capture_console.backend.scene_demo import demo_scene
    path = tmp_path / "scene.json"
    path.write_text(json.dumps(dict(schema_version=1, kind="vt.spatial.archive", frames=[demo_scene(0)])))
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    page.show()
    page.load_replay(path)
    try:
        deadline = time.monotonic()+2
        while page.backend.status()["state"] == "loading" and time.monotonic() < deadline:
            qapp.processEvents()
            time.sleep(.005)
        page.refresh()
        assert page.backend.mode == "replay"
        assert "历史帧" in page.status.text()
        assert page.replay_bar.isVisible()
        assert not page.camera_strip.isVisible()
        page.return_live()
        assert page.backend is backend
        assert not page.replay_bar.isVisible()
    finally:
        page.close()
        backend.close()


def test_cloud_display_limits_filter_local_depth_without_mutating_source():
    import numpy as np
    from vt_capture_console.ui.widgets.spatial_scene import display_cloud_data
    positions = np.array([[0., 0., z] for z in (.1, .2, .3, .4, 5.)], dtype="f4")
    colors = np.ones((5, 4), dtype="f4")
    positions.setflags(write=False)
    cloud = dict(positions=positions, colors=colors)
    displayed, displayed_colors = display_cloud_data(cloud, max_points=2, max_depth=1.)
    assert len(displayed) <= 2 and len(displayed_colors) == len(displayed)
    assert all(displayed[:, 2] <= 1.)
    assert len(positions) == 5 and positions[-1, 2] == 5.
