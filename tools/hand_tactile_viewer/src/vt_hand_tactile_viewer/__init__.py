"""Independent application; importing this package does not load models or ports."""

__version__ = "0.2.4"


def product_id(hand):
    """Stable installation and preferences identity for a single glove."""
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    return f"{hand}-hand-display-row44"


PRODUCT_ID = product_id("right")
