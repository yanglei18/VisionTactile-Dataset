import os

os.environ.setdefault("QT_QPA_PLATFORM", "offscreen")

import pytest
from PySide6.QtWidgets import QApplication


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
