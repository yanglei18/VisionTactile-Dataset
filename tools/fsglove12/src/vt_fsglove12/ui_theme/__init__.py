"""Shared workstation appearance, independent of Qt bindings and hardware.

PyQt5 calibration and PySide6 capture load the same packaged stylesheet. This
module returns data only: each application owns its QApplication and QFont.
Sensor and model colors deliberately remain with their renderers.
"""
from importlib.resources import files
from types import MappingProxyType


FONT_FAMILIES = ("Inter", "Noto Sans", "Noto Sans CJK SC")
FONT_POINT_SIZE = 10
PALETTE = MappingProxyType({
    "background": "#000000",
    "surface": "#080808",
    "raised": "#141414",
    "selection": "#303030",
    "border": "#555555",
    "disabled": "#888888",
    "muted": "#aaaaaa",
    "secondary": "#cccccc",
    "text": "#dddddd",
    "emphasis": "#ffffff",
})
CANVAS_BACKGROUND = PALETTE["background"]


def stylesheet() -> str:
    """Load the main theme with install-safe, cwd-independent icon paths.

    Normal wheel and frozen installations expose these packaged SVGs as files,
    which Qt's stylesheet URL loader needs rather than Python resource handles.
    """
    root = files(__package__)
    return root.joinpath("theme.qss").read_text(encoding="utf-8").replace(
        "@THEME_DIR@", str(root).replace("\\", "/").replace('"', '\\"')
    )


def calibration_stylesheet() -> str:
    """Shared appearance plus controls specific to the calibration workflow."""
    extra = files(__package__).joinpath("calibration.qss").read_text(encoding="utf-8")
    for name, value in PALETTE.items():
        extra = extra.replace(f"@{name}@", value)
    return stylesheet() + "\n" + extra


def status_style(state="neutral", *, compact=False) -> str:
    """Neutral status surfaces; visible text must also identify the state.

    A stronger leading edge distinguishes warnings and errors without competing
    with the measured colors on the hand. State is never communicated by color
    alone. Keep transient operational failures visible even with details folded.
    """
    styles = {
        "neutral": ("surface", "border", "secondary", 1, 400),
        "info": ("raised", "muted", "text", 2, 400),
        "success": ("raised", "secondary", "emphasis", 2, 500),
        "warning": ("raised", "emphasis", "emphasis", 3, 600),
        "error": ("selection", "emphasis", "emphasis", 4, 700),
        "muted": ("surface", "selection", "muted", 1, 400),
    }
    try:
        background, border, color, width, weight = styles[state]
    except KeyError as exc:
        raise ValueError(f"Unknown interface status: {state}") from exc
    padding = "5px 7px" if compact else "8px 10px"
    return (f"background: {PALETTE[background]}; color: {PALETTE[color]}; "
            f"border: 1px solid {PALETTE['selection']}; "
            f"border-left: {width}px solid {PALETTE[border]}; border-radius: 2px; "
            f"padding: {padding}; font-size: 12px; font-weight: {weight};")


def text_style(role="body") -> str:
    """Small semantic text styles for labels updated by the calibration view."""
    roles = {
        "title": ("emphasis", 22, 600),
        "instruction": ("text", 14, 400),
        "instruction_active": ("emphasis", 14, 600),
        "muted": ("muted", 11, 400),
        "body": ("text", 12, 400),
    }
    try:
        color, size, weight = roles[role]
    except KeyError as exc:
        raise ValueError(f"Unknown interface text role: {role}") from exc
    return f"color: {PALETTE[color]}; font-size: {size}px; font-weight: {weight};"
