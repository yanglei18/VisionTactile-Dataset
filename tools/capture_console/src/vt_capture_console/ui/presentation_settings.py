"""Shared display preferences, independent of the application window."""

from PySide6.QtCore import QSettings


def presentation_settings():
    # Keep the existing store so upgrades retain display and language choices.
    return QSettings(QSettings.Format.IniFormat, QSettings.Scope.UserScope,
                     "visiontactile", "capture-console")
