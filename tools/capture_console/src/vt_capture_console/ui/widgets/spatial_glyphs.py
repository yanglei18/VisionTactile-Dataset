"""Small immutable schematic device meshes, independent of either renderer.

The tracker is a rounded, three-lobed housing with sensor apertures, a central
mount and a +X chevron.  Its origin is the incoming pose origin: these illustrative
metre coordinates neither describe a particular product nor supply calibration.
Both the OpenGL and projection views can transform exactly the same geometry.
"""
from __future__ import annotations

from ...i18n import tr

from dataclasses import dataclass
from functools import lru_cache
import math


Point = tuple[float, float, float]


@dataclass(frozen=True)
class GlyphSurface:
    """A convex, outward-wound polygon and a renderer-resolved colour token."""

    vertices: tuple[Point, ...]
    color: str
    alpha: float = 1.


@dataclass(frozen=True)
class GlyphLine:
    start: Point
    end: Point
    color: str
    width: float = 1.


@dataclass(frozen=True)
class LocalGlyph:
    surfaces: tuple[GlyphSurface, ...]
    lines: tuple[GlyphLine, ...]
    bounds: tuple[Point, ...]


def _circle(radius: float, height: float, count: int = 12,
            center: tuple[float, float] = (0., 0.)) -> tuple[Point, ...]:
    return tuple((center[0] + radius * math.cos(math.tau * i / count),
                  center[1] + radius * math.sin(math.tau * i / count), height)
                 for i in range(count))


def _housing_ring(scale: float, height: float) -> tuple[Point, ...]:
    # An eighteen-sided, gently rounded triangle reads as hardware even when
    # it occupies only a few screen pixels.  Its three lobes balance about zero.
    result = []
    for i in range(18):
        angle = math.tau * i / 18
        radius = .034 * (1. + .16 * math.cos(3 * angle)) * scale
        result.append((radius * math.cos(angle), radius * math.sin(angle), height))
    return tuple(result)


def _outline(lines: list[GlyphLine], ring: tuple[Point, ...], color: str,
             width: float = 1.) -> None:
    lines.extend(GlyphLine(point, ring[(i + 1) % len(ring)], color, width)
                 for i, point in enumerate(ring))


def _disk(surfaces: list[GlyphSurface], ring: tuple[Point, ...], color: str,
          *, upward: bool = True) -> None:
    center = tuple(sum(point[axis] for point in ring) / len(ring) for axis in range(3))
    for i, point in enumerate(ring):
        vertices = (center, point, ring[(i + 1) % len(ring)])
        surfaces.append(GlyphSurface(vertices if upward else vertices[::-1], color))


def _annulus(surfaces: list[GlyphSurface], outer: tuple[Point, ...],
             inner: tuple[Point, ...], color: str) -> None:
    for i, point in enumerate(outer):
        j = (i + 1) % len(outer)
        surfaces.append(GlyphSurface((point, outer[j], inner[j], inner[i]), color))


def _tracker() -> LocalGlyph:
    surfaces: list[GlyphSurface] = []
    lines: list[GlyphLine] = []

    # Thin role-colour belt, neutral shell and two bevels.  The complete housing
    # straddles the measured origin instead of shifting it to the base or top.
    rings = tuple(_housing_ring(scale, height) for scale, height in (
        (.89, -.016), (1., -.010), (1., -.003),
        (1., -.0012), (1., .007), (.88, .016)))
    for lower, upper, color in zip(rings, rings[1:],
                                    ("dark", "body", "accent", "body", "light")):
        for i, point in enumerate(lower):
            j = (i + 1) % len(lower)
            surfaces.append(GlyphSurface((point, lower[j], upper[j], upper[i]), color))
    _disk(surfaces, rings[0], "body", upward=False)
    _disk(surfaces, rings[-1], "body")
    _outline(lines, rings[-1], "light", .9)
    _outline(lines, rings[0], "light", .8)
    _outline(lines, rings[2], "accent", 1.1)

    # Three flush dark apertures, one in each lobe.  Raised by a fraction of a
    # millimetre solely to avoid coplanar depth flicker in the schematic mesh.
    for i in range(3):
        angle = math.tau * i / 3
        center = (.027 * math.cos(angle), .027 * math.sin(angle))
        outer = _circle(.0061, .01625, center=center)
        inner = _circle(.0048, .01625, center=center)
        _annulus(surfaces, outer, inner, "light")
        _disk(surfaces, inner, "dark")

    # A concentric mounting detail provides readable depth under overhead views.
    hub_outer = _circle(.0090, .01635)
    hub_inner = _circle(.0071, .01635)
    _annulus(surfaces, hub_outer, hub_inner, "light")
    _disk(surfaces, hub_inner, "dark")
    _disk(surfaces, _circle(.0021, .01645, count=8), "body")
    _outline(lines, hub_outer, "light", .9)

    # Split the concave chevron into two upward-wound triangles.  Its tip is +X,
    # giving rotations a visible heading even when coordinate axes are hidden.
    tip = (.0220, 0., .0166)
    upper = (.0110, .0090, .0166)
    inset = (.0145, 0., .0166)
    lower = (.0110, -.0090, .0166)
    surfaces.extend((GlyphSurface((tip, upper, inset), "accent"),
                     GlyphSurface((tip, inset, lower), "accent")))
    lines.extend((GlyphLine(lower, tip, "light", 1.1),
                  GlyphLine(tip, upper, "light", 1.1)))

    # Keep the underside identifiable when real poses turn the sensor face
    # away from the viewer: a neutral mounting ring and three recessed feet.
    underside = []
    mount_outer = _circle(.012, -.0163)
    mount_inner = _circle(.0095, -.0163)
    _annulus(underside, mount_outer, mount_inner, "light")
    surfaces.extend(GlyphSurface(face.vertices[::-1], face.color) for face in underside)
    _disk(surfaces, mount_inner, "dark", upward=False)
    _outline(lines, mount_outer, "light", .9)
    for index in range(3):
        angle = math.tau * index / 3
        foot = _circle(.0034, -.0164, count=8,
                       center=(.024*math.cos(angle), .024*math.sin(angle)))
        _disk(surfaces, foot, "dark", upward=False)
    for y in (-.003, .003):
        lines.append(GlyphLine((-.006, y, -.0165), (.006, y, -.0165), "light", .9))

    # Six small side apertures break up the shell without a dense wireframe.
    side = rings[1]
    for index in (17, 0, 5, 6, 11, 12):
        a, b = side[index], side[(index + 1) % len(side)]
        endpoints = tuple((a[0] * (1. - t) + b[0] * t,
                           a[1] * (1. - t) + b[1] * t) for t in (.24, .76))
        # Scale out very slightly so the inset graphic remains above the shell.
        left, right = tuple((x * 1.002, y * 1.002) for x, y in endpoints)
        surfaces.append(GlyphSurface(((left[0], left[1], -.008),
                                      (right[0], right[1], -.008),
                                      (right[0], right[1], .0045),
                                      (left[0], left[1], .0045)), "dark"))

    points = tuple(point for surface in surfaces for point in surface.vertices)
    minima = tuple(min(point[axis] for point in points) for axis in range(3))
    maxima = tuple(max(point[axis] for point in points) for axis in range(3))
    bounds = tuple((x, y, z) for x in (minima[0], maxima[0])
                   for y in (minima[1], maxima[1]) for z in (minima[2], maxima[2]))
    return LocalGlyph(tuple(surfaces), tuple(lines), bounds)


@lru_cache(maxsize=2)
def local_glyph(kind: str = "tracker") -> LocalGlyph:
    """Return shared immutable local geometry; callers apply measured poses.

    ``body``, ``light`` and ``dark`` are neutral material colour tokens.
    ``accent`` is the existing entity/state colour, used sparingly for identity.
    The tracker fits within a 0.09 m envelope and has fewer than 500 triangles.
    Unknown kinds are rejected rather than silently drawing the wrong device.
    """
    if kind != "tracker":
        raise ValueError(tr('Unsupported spatial glyph kind: {v0!r}', v0=kind))
    return _tracker()
