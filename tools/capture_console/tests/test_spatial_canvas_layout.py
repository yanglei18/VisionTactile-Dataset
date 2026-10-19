"""Canvas tools change presentation without changing the shared scene source."""
from vt_capture_console.backend.scene_demo import DemoSceneBackend
from vt_capture_console.ui.widgets.spatial_page import SpatialPage


def test_inspector_tabs_and_collapse_preserve_entities_and_subscription(qapp):
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    try:
        page.resize(1200, 640)
        page.show()
        page.refresh()
        qapp.processEvents()
        entities = tuple(entity["id"] for entity in page.model.entities())
        layers = frozenset(page.model.layers)
        interest = backend._interests[page.viewer_id]
        assert not page.inspector.isVisible()
        width_without_inspector = page.scene.width()
        page.inspector_toggle.click()
        qapp.processEvents()
        assert page.inspector.isVisible()
        width_with_inspector = page.scene.width()
        assert width_without_inspector > width_with_inspector + 200

        for index in range(page.inspector_tabs.count()):
            page.inspector_tabs.setCurrentIndex(index)
            qapp.processEvents()
            assert tuple(entity["id"] for entity in page.model.entities()) == entities
            assert frozenset(page.model.layers) == layers
            assert backend._interests[page.viewer_id] == interest

        page.inspector_toggle.click()
        qapp.processEvents()
        assert not page.inspector.isVisible()
        assert page.scene.width() > width_with_inspector + 200
        assert tuple(entity["id"] for entity in page.model.entities()) == entities
        assert backend._interests[page.viewer_id] == interest

        page.scene.selected.emit(entities[0])
        qapp.processEvents()
        assert page.inspector.isVisible()
        assert page.model.selected_id == entities[0]
        assert page.detail.isVisible()
        assert backend._interests[page.viewer_id] == interest
    finally:
        page.close()
        assert not backend._interests.get(page.viewer_id)
        backend.close()


def test_camera_strip_is_optional_and_respects_tracker_preset(qapp):
    backend = DemoSceneBackend()
    page = SpatialPage(backend)
    try:
        page.resize(1200, 640)
        page.show()
        page.refresh()
        qapp.processEvents()
        assert not page.camera_strip.isVisible()

        page.camera_toggle.click()
        qapp.processEvents()
        assert page.camera_strip.isVisible()
        page.preset.setCurrentIndex(page.preset.findData("tracker"))
        assert not page.camera_strip.isVisible()
        assert not page.camera_toggle.isEnabled()
        page.preset.setCurrentIndex(page.preset.findData("combined"))
        assert page.camera_strip.isVisible()
        assert page.camera_toggle.isChecked()
    finally:
        page.close()
        backend.close()
