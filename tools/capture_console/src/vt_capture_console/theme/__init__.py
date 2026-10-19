"""Capture console design system and install-safe stylesheet resources."""

from importlib.resources import files


def stylesheet() -> str:
    """Load the theme with absolute icon paths, independent of process cwd.

    Qt's stylesheet URL loader does not resolve package-relative resources.
    Normal wheel installations expose these small packaged SVGs as files.
    """
    root = files(__package__)
    return root.joinpath("theme.qss").read_text(encoding="utf-8").replace(
        "@THEME_DIR@", str(root).replace("\\", "/").replace('"', '\\"')
    )
