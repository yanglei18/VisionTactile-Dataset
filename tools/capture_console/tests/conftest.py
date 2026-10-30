import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtCore import QEvent, QSettings
from PySide6.QtWidgets import QApplication
from shiboken6 import isValid


@pytest.fixture(autouse=True)
def isolated_qt_widgets(isolated_presentation_settings):
    """Destroy this test's windows, including manually constructed dialogs.

    QWidget.close() only hides most widgets. A session-wide QApplication and
    Python signal connections otherwise retain them across tests; subsequent
    language/style changes then revisit every previous interface. Keep the
    application and any pre-existing windows, but deliver deferred destruction
    for the widgets owned by this test after its other fixtures finish.
    """
    app = QApplication.instance()
    # Hold the wrappers so Python cannot reuse a preserved widget's identity.
    existing = tuple(app.topLevelWidgets()) if app else ()
    existing_ids = {id(widget) for widget in existing}
    yield
    app = QApplication.instance()
    if app is None:
        return
    for widget in app.topLevelWidgets():
        if isValid(widget) and id(widget) not in existing_ids:
            widget.close()
            if isValid(widget):
                widget.deleteLater()
    app.sendPostedEvents(None, QEvent.Type.DeferredDelete)
    app.processEvents()
    app.sendPostedEvents(None, QEvent.Type.DeferredDelete)


@pytest.fixture(autouse=True)
def isolated_presentation_settings(tmp_path, monkeypatch):
    from vt_capture_console.ui import main_window, presentation_settings
    path = str(tmp_path / "presentation.ini")
    monkeypatch.setattr(main_window, "_presentation_settings",
                        lambda: QSettings(path, QSettings.Format.IniFormat))
    monkeypatch.setattr(presentation_settings, "presentation_settings",
                        lambda: QSettings(path, QSettings.Format.IniFormat))


@pytest.fixture(scope="session")
def qapp():
    app = QApplication.instance() or QApplication([])
    yield app


@pytest.fixture
def window(qapp):
    from vt_capture_console.backend.demo_backend import DemoBackend
    from vt_capture_console.ui.main_window import MainWindow
    instance = MainWindow(DemoBackend())
    instance.show()
    qapp.processEvents()
    instance.refresh()
    yield instance
    instance.close()
    qapp.processEvents()
