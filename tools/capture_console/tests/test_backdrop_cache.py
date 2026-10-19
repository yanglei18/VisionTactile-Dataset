"""Static artwork should stay correct without rerasterizing on child updates."""

from unittest.mock import Mock

import pytest
from PySide6 import QtCore, QtGui

from vt_capture_console.ui.widgets.motion import AmbientBackdrop


@pytest.fixture
def backdrop(qapp):
    widget = AmbientBackdrop()
    widget.resize(640, 360)
    widget.set_motion_enabled(False)
    yield widget
    widget.close()
    widget.deleteLater()
    qapp.processEvents()


def test_static_repaints_reuse_artwork_and_preserve_pixels(backdrop, monkeypatch):
    paint = Mock(wraps=backdrop._paint_scene)
    monkeypatch.setattr(backdrop, "_paint_scene", paint)
    first = backdrop.grab().toImage()
    for _ in range(20):
        assert backdrop.grab().toImage() == first
    assert paint.call_count == 1

    # Render the same phase directly to check that caching changes no artwork.
    backdrop.set_motion_enabled(True)
    assert backdrop.grab().toImage() == first
    assert backdrop._static_background is None


def test_resize_and_composition_refresh_static_artwork(backdrop, monkeypatch):
    paint = Mock(wraps=backdrop._paint_scene)
    monkeypatch.setattr(backdrop, "_paint_scene", paint)
    original = backdrop.grab().toImage()
    backdrop.set_mode("spatial")
    spatial = backdrop.grab().toImage()
    assert spatial != original
    assert paint.call_count == 2
    backdrop.set_mode("spatial")
    assert backdrop.grab().toImage() == spatial
    assert paint.call_count == 2
    backdrop.resize(800, 450)
    resized = backdrop.grab()
    assert resized.width() == round(800 * backdrop.devicePixelRatioF())
    assert paint.call_count == 3


def test_pixel_ratio_change_rebuilds_native_resolution_surface(backdrop, monkeypatch):
    backdrop.grab()
    monkeypatch.setattr(backdrop, "devicePixelRatioF", lambda: 2.0)
    backdrop.grab()
    surface = backdrop._static_background
    assert surface.devicePixelRatioF() == 2.0
    assert surface.width() == backdrop.width() * 2
    assert surface.height() == backdrop.height() * 2


def test_oversized_cache_falls_back_to_direct_render(backdrop, monkeypatch):
    original = backdrop.grab().toImage()
    monkeypatch.setattr(backdrop, "_MAX_STATIC_CACHE_PIXELS", 1)
    backdrop.set_motion_enabled(True)
    backdrop.set_motion_enabled(False)
    assert backdrop.grab().toImage() == original
    assert backdrop._static_background is None


def test_motion_toggle_caches_current_phase(backdrop):
    original = backdrop.grab().toImage()
    backdrop.set_motion_enabled(True)
    backdrop._phase = 80.0
    moving = backdrop.grab().toImage()
    assert moving != original
    backdrop.set_motion_enabled(False)
    assert backdrop.grab().toImage() == moving


def test_moving_orbits_reuse_light_field_until_next_sample(backdrop, monkeypatch):
    backdrop.set_motion_enabled(True)
    light = Mock(wraps=backdrop._paint_lighting)
    monkeypatch.setattr(backdrop, "_paint_lighting", light)
    initial = backdrop.grab().toImage()
    for phase in (.1, .3, .5, .8):
        backdrop._phase = phase
        backdrop.grab()
    assert light.call_count == 1
    assert backdrop.grab().toImage() != initial
    backdrop._phase = 1.01
    backdrop.grab()
    assert light.call_count == 2


def test_soft_light_is_sampled_small_but_cached_at_display_resolution(backdrop, monkeypatch):
    backdrop.resize(2494, 1371)
    backdrop.set_motion_enabled(True)
    original = backdrop._paint_lighting
    sampled_sizes = []

    def observe(painter, *args):
        sampled_sizes.append(painter.device().width() * painter.device().height())
        return original(painter, *args)

    monkeypatch.setattr(backdrop, "_paint_lighting", observe)
    backdrop.grab()
    assert len(sampled_sizes) == 1
    assert sampled_sizes[0] <= backdrop._MAX_LIGHT_CACHE_PIXELS
    assert backdrop._light_background.width() == round(2494 * backdrop.devicePixelRatioF())
    assert backdrop._light_background.height() == round(1371 * backdrop.devicePixelRatioF())


@pytest.mark.parametrize("size,phase,ratio", [
    ((640, 360), 0., 1.), ((640, 360), 20.1, 2.),
    ((2494, 1371), 51.23, 1.), ((2494, 1371), 120.3, 1.),
    ((360, 640), 20.1, 1.),
])
def test_partial_motion_repaint_matches_full_frame_without_trails(backdrop, size, phase, ratio,
                                                                monkeypatch):
    backdrop.resize(*size)
    monkeypatch.setattr(backdrop, "devicePixelRatioF", lambda: ratio)

    def render(value):
        backdrop._phase = value
        frame = QtGui.QImage(round(size[0] * ratio), round(size[1] * ratio),
                             QtGui.QImage.Format.Format_RGB32)
        frame.setDevicePixelRatio(ratio)
        painter = QtGui.QPainter(frame)
        try:
            backdrop._paint_scene(painter)
        finally:
            painter.end()
        return frame

    incremental = render(phase)
    # Repeated clipped updates catch old rim/star pixels left behind by damage
    # regions that include only the destination instead of both footprints.
    for step in range(1, 5):
        previous, current = phase + (step - 1) * .067, phase + step * .067
        expected = render(current)
        region = backdrop._motion_region(previous) | backdrop._motion_region(current)
        painter = QtGui.QPainter(incremental)
        try:
            painter.setClipRegion(region)
            backdrop._paint_scene(painter)
        finally:
            painter.end()
        assert incremental == expected


def test_animation_only_invalidates_whole_window_when_soft_light_changes(backdrop, monkeypatch):
    class Clock:
        def isValid(self):
            return True

        def restart(self):
            return 67

        def invalidate(self):
            pass

    monkeypatch.setattr(backdrop, "_clock", Clock())
    updates = Mock()
    monkeypatch.setattr(backdrop, "update", updates)
    backdrop._phase = .2
    backdrop._advance()
    region = updates.call_args.args[0]
    assert isinstance(region, QtGui.QRegion)
    assert not region.isEmpty()
    assert region != QtGui.QRegion(backdrop.rect())
    backdrop._phase = .98
    backdrop._advance()
    assert updates.call_args.args == ()


def test_light_cache_rebuilds_for_mode_resize_and_pixel_ratio(backdrop, monkeypatch):
    backdrop.set_motion_enabled(True)
    backdrop.grab()
    first = backdrop._light_background.cacheKey()
    backdrop.set_mode("spatial")
    backdrop.grab()
    assert backdrop._light_background.cacheKey() != first
    backdrop.resize(800, 450)
    backdrop.grab()
    assert backdrop._light_background.size() == QtCore.QSize(800, 450)
    monkeypatch.setattr(backdrop, "devicePixelRatioF", lambda: 2.)
    backdrop.grab()
    assert backdrop._light_background.size() == QtCore.QSize(1600, 900)
    assert backdrop._light_background.devicePixelRatioF() == 2.
