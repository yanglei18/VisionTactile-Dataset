"""Dependency-free, explicit raw-address to schematic hand-point mapping.

This module does not crop, reconstruct or alter incoming tactile measurements.
An anchor confirms an anatomical response location, not a calibrated pressure or
an independently verified boundary around the rest of its candidate region.
"""

from dataclasses import dataclass
import json
import math
from pathlib import Path


@dataclass(frozen=True, slots=True)
class LayoutPoint:
    row: int
    col: int
    x: float
    y: float
    region: str
    verified_anchor: bool


@dataclass(frozen=True, slots=True)
class LayoutRegion:
    """Raw addresses plus explicit directions for candidate surface projection.

    Increasing normalized row runs wrist-to-tip; increasing normalized column
    runs toward the thumb. Directions affect display binding only, never ADC.
    """

    region: str
    rows: tuple[int, int]
    cols: tuple[int, int]
    row_direction: int = 1
    col_direction: int = 1


@dataclass(frozen=True, slots=True)
class HandLayout:
    name: str
    hand: str
    status: str
    raw_rows: int
    raw_cols: int
    points: tuple[LayoutPoint, ...]
    outline: tuple[tuple[float, float], ...]
    labels: tuple[tuple[str, float, float], ...]
    note: str
    regions: tuple[LayoutRegion, ...] = ()
    source: tuple[tuple[str, str], ...] = ()

    @property
    def covered_indices(self) -> tuple[int, ...]:
        """Indices into the physical 20x20 row-major ROS state array."""
        return tuple(point.row * self.raw_cols + point.col for point in self.points)


_REGIONS = frozenset({"palm", "thumb", "index", "middle", "ring", "little"})
_TOP_KEYS = frozenset({
    "schema_version", "name", "hand", "status", "raw_rows", "raw_cols",
    "regions", "anchors", "outline", "labels", "note", "source",
})
_GRID_KEYS = frozenset({"region", "rows", "cols", "origin", "row_step", "col_step"})


def _keys(value, expected, where):
    if not isinstance(value, dict) or set(value) != expected:
        raise ValueError(f"{where}: expected exactly keys {', '.join(sorted(expected))}")


def _integer(value, where):
    if isinstance(value, bool) or not isinstance(value, int):
        raise ValueError(f"{where}: expected an integer")
    return value


def _text(value, where):
    if not isinstance(value, str) or not value.strip():
        raise ValueError(f"{where}: expected nonempty text")
    return value


def _number(value, where):
    if isinstance(value, bool) or not isinstance(value, (int, float)):
        raise ValueError(f"{where}: expected a finite number")
    try:
        number = float(value)
    except OverflowError as exc:
        raise ValueError(f"{where}: expected a finite number") from exc
    if not math.isfinite(number):
        raise ValueError(f"{where}: expected a finite number")
    return number


def _pair(value, where):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{where}: expected [x, y]")
    return (_number(value[0], where), _number(value[1], where))


def _range(value, maximum, where):
    if not isinstance(value, list) or len(value) != 2:
        raise ValueError(f"{where}: expected a half-open [start, stop] range")
    start, stop = (_integer(item, where) for item in value)
    if not 0 <= start < stop <= maximum:
        raise ValueError(f"{where}: range must satisfy 0 <= start < stop <= {maximum}")
    return range(start, stop)


def _reject_constant(value):
    raise ValueError(f"layout contains non-finite JSON number: {value}")


def _unique_object(pairs):
    result = {}
    for key, value in pairs:
        if key in result:
            raise ValueError(f"layout contains duplicate JSON key: {key}")
        result[key] = value
    return result


def load_layout(path: str | Path | None = None, *, expected_hand: str | None = None) -> HandLayout:
    """Load an explicit per-hand candidate schematic, failing on mistakes.

    The default packaged layout has 400 candidate positions but only nine
    observed anchor pixels across seven anatomical areas. Only raw rows and
    columns 0..19 are physically valid. Coordinates are schematic units, with positive Y toward
    fingertips and the right palm facing the viewer (thumb at the left).
    """
    if expected_hand not in (None, "left", "right"):
        raise ValueError("expected_hand must be left or right")
    if path is None and expected_hand == "left":
        raise ValueError("Left-hand surface display requires an explicit left layout; use the raw matrix without one")
    location = Path(path) if path is not None else Path(__file__).with_name("layouts") / "right_glove_candidate.json"
    with location.open(encoding="utf-8") as stream:
        config = json.load(stream, parse_constant=_reject_constant, object_pairs_hook=_unique_object)
    _keys(config, _TOP_KEYS, "layout")
    if _integer(config["schema_version"], "schema_version") != 1:
        raise ValueError("schema_version: only version 1 is supported")
    if config["hand"] not in ("left", "right"):
        raise ValueError("hand must be left or right")
    if expected_hand is not None and config["hand"] != expected_hand:
        raise ValueError(f"layout hand {config['hand']} does not match expected hand {expected_hand}")
    if config["status"] != "candidate":
        raise ValueError("status: this schema supports candidate layouts only")
    name = _text(config["name"], "name")
    note = _text(config["note"], "note")
    _keys(config["source"], {"kind", "date", "dataset", "scope"}, "source")
    for key, value in config["source"].items():
        _text(value, f"source.{key}")
    raw_rows = _integer(config["raw_rows"], "raw_rows")
    raw_cols = _integer(config["raw_cols"], "raw_cols")
    if (raw_rows, raw_cols) != (20, 20):
        raise ValueError("raw_rows/raw_cols: must match the physical 20 x 20 grid")
    regions = config["regions"]
    if not isinstance(regions, list) or not regions:
        raise ValueError("regions: expected a nonempty list")
    positions = {}
    seen_xy = set()
    seen_regions = set()
    layout_regions = []
    for index, grid in enumerate(regions):
        where = f"regions[{index}]"
        optional = {"row_direction", "col_direction"}
        if not isinstance(grid, dict):
            raise ValueError(f"{where}: expected a region object")
        _keys({key: value for key, value in grid.items() if key not in optional}, _GRID_KEYS, where)
        region = _text(grid["region"], f"{where}.region")
        if region not in _REGIONS or region in seen_regions:
            raise ValueError(f"{where}.region: unknown or duplicate region {region!r}")
        seen_regions.add(region)
        rows = _range(grid["rows"], raw_rows, f"{where}.rows")
        cols = _range(grid["cols"], raw_cols, f"{where}.cols")
        directions = tuple(_integer(grid.get(key, 1), f"{where}.{key}") for key in ("row_direction", "col_direction"))
        if any(value not in (-1, 1) for value in directions):
            raise ValueError(f"{where}: row_direction/col_direction must be -1 or 1")
        layout_regions.append(LayoutRegion(region, (rows.start, rows.stop), (cols.start, cols.stop), *directions))
        origin = _pair(grid["origin"], f"{where}.origin")
        dr = _pair(grid["row_step"], f"{where}.row_step")
        dc = _pair(grid["col_step"], f"{where}.col_step")
        determinant = dr[0] * dc[1] - dr[1] * dc[0]
        if not math.isfinite(determinant) or abs(determinant) < 1e-12:
            raise ValueError(f"{where}: row_step and col_step must be independent finite vectors")
        for row in rows:
            for col in cols:
                address = (row, col)
                if address in positions:
                    raise ValueError(f"{where}: duplicate raw address {address}")
                x = _number(origin[0] + (row - rows.start) * dr[0] + (col - cols.start) * dc[0], where)
                y = _number(origin[1] + (row - rows.start) * dr[1] + (col - cols.start) * dc[1], where)
                xy = (round(x, 12), round(y, 12))
                if xy in seen_xy:
                    raise ValueError(f"{where}: duplicate schematic position {xy}")
                seen_xy.add(xy)
                positions[address] = (x, y, region)
    if not isinstance(config["anchors"], list):
        raise ValueError("anchors: expected a list")
    anchors = set()
    for index, address in enumerate(config["anchors"]):
        if not isinstance(address, list) or len(address) != 2:
            raise ValueError(f"anchors[{index}]: expected [row, col]")
        address = tuple(_integer(item, f"anchors[{index}]") for item in address)
        if address not in positions or address in anchors:
            raise ValueError(f"anchors[{index}]: unmapped or duplicate address {address}")
        anchors.add(address)
    if not isinstance(config["outline"], list) or len(config["outline"]) < 4:
        raise ValueError("outline: expected a closed polygon of at least three vertices")
    outline = tuple(_pair(item, "outline") for item in config["outline"])
    if outline[0] != outline[-1] or len(set(outline[:-1])) != len(outline) - 1:
        raise ValueError("outline: must be explicitly closed without repeated interior vertices")
    if not isinstance(config["labels"], list):
        raise ValueError("labels: expected a list")
    labels = []
    for index, item in enumerate(config["labels"]):
        if not isinstance(item, list) or len(item) != 3:
            raise ValueError(f"labels[{index}]: expected [text, x, y]")
        labels.append((_text(item[0], "label"), _number(item[1], "label.x"), _number(item[2], "label.y")))
    points = tuple(LayoutPoint(row, col, *positions[(row, col)], (row, col) in anchors)
                   for row, col in sorted(positions))
    return HandLayout(name, config["hand"], "candidate", raw_rows, raw_cols, points,
                      outline, tuple(labels), note, tuple(layout_regions), tuple(config["source"].items()))


def unmapped_layout(hand: str) -> HandLayout:
    """An empty UI placeholder, not a mirrored or inferred anatomical mapping."""
    if hand not in ("left", "right"):
        raise ValueError("hand must be left or right")
    return HandLayout(f"{hand} — no anatomical layout", hand, "unmapped", 20, 20, (),
                      ((0., 0.), (20., 0.), (20., 20.), (0., 20.), (0., 0.)), (),
                      "尚未选择此手的解剖布局；只显示原始 ADC 矩阵，不推断左右手地址映射。")
