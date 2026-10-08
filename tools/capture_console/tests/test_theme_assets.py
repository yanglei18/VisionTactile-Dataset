from PySide6.QtGui import QIcon

from vt_capture_console.theme import stylesheet
from vt_capture_console.ui.icons import icon


def test_stylesheet_resources_are_cwd_independent(tmp_path, monkeypatch):
    original = stylesheet()
    monkeypatch.chdir(tmp_path)
    assert stylesheet() == original
    assert "@THEME_DIR@" not in original
    assert "chevron-down.svg" in original


def test_native_vector_icons_render_enabled_and_disabled(qapp):
    names = ("dashboard", "trials", "devices", "settings", "play", "stop",
             "camera", "tracker", "glove", "tactile", "chevron", "orbit", "expand")
    for name in names:
        graphic = icon(name)
        for size in (16, 24, 40):
            for mode in (QIcon.Mode.Normal, QIcon.Mode.Disabled):
                image = graphic.pixmap(size, size, mode).toImage()
                assert not image.isNull()
                assert any(image.pixelColor(x, y).alpha() > 0
                           for y in range(image.height()) for x in range(image.width()))
