"""Choose the supported Qt binding before PyInstaller imports the application."""
import os

os.environ["PYQTGRAPH_QT_LIB"] = "PySide6"
