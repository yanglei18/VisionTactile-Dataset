from __future__ import annotations

import math
from typing import Iterable

import numpy as np
from PIL import Image, ImageDraw, ImageFont

from .errors import UnsupportedEncodingError
from .sdk_model import AlignedFrame, CameraSample, ImageData, TrackerPose
from .viewer_model import ViewerConfig


_BACKGROUND = (13, 17, 23)
_PANEL = (24, 31, 41)
_BORDER = (56, 68, 82)
_TEXT = (226, 232, 240)
_MUTED = (150, 162, 177)
_WARNING = (255, 184, 77)
_ROLE_COLORS = (
    (74, 144, 226),
    (72, 196, 121),
    (238, 114, 96),
    (190, 120, 230),
    (239, 196, 74),
)
_VIRIDIS_STOPS = np.array([0.0, 0.25, 0.5, 0.75, 1.0])
_VIRIDIS_RGB = np.array(
    [
        [68, 1, 84],
        [59, 82, 139],
        [33, 145, 140],
        [94, 201, 98],
        [253, 231, 37],
    ],
    dtype=np.float64,
)


def _font(size: int) -> ImageFont.FreeTypeFont | ImageFont.ImageFont:
    try:
        return ImageFont.truetype("DejaVuSans.ttf", size=size)
    except OSError:
        return ImageFont.load_default()


def _uint8_grayscale(array: np.ndarray) -> np.ndarray:
    value = np.asarray(array)
    if value.dtype == np.uint8:
        return value.copy()
    finite = np.isfinite(value)
    if not np.any(finite):
        return np.zeros(value.shape, dtype=np.uint8)
    observed = value[finite].astype(np.float64)
    minimum = float(observed.min())
    maximum = float(observed.max())
    if maximum <= minimum:
        return np.where(finite, 255, 0).astype(np.uint8)
    normalized = (value.astype(np.float64) - minimum) / (maximum - minimum)
    normalized = np.where(finite, normalized, 0.0)
    return np.rint(np.clip(normalized, 0.0, 1.0) * 255.0).astype(np.uint8)


def color_image_to_rgb(image: ImageData) -> np.ndarray:
    """Convert a decoded ROS color image into a writable RGB uint8 array."""
    value = np.asarray(image.array)
    if image.encoding == "rgb8":
        return value.astype(np.uint8, copy=True)
    if image.encoding == "bgr8":
        return value[..., ::-1].astype(np.uint8, copy=True)
    if image.encoding in {"mono8", "mono16", "16UC1", "32FC1"}:
        gray = _uint8_grayscale(value)
        return np.repeat(gray[..., np.newaxis], 3, axis=2)
    raise UnsupportedEncodingError(
        f"viewer does not support color encoding: {image.encoding}"
    )


def depth_image_to_rgb(image: ImageData, config: ViewerConfig) -> np.ndarray:
    """Colorize decoded metric depth with a fixed, session-stable range."""
    source = np.asarray(image.array)
    if image.encoding in {"mono16", "16UC1"}:
        meters = source.astype(np.float32) * 0.001
    elif image.encoding == "32FC1":
        meters = source.astype(np.float32, copy=True)
    else:
        raise UnsupportedEncodingError(
            f"viewer does not support depth encoding: {image.encoding}"
        )
    valid = np.isfinite(meters) & (meters > 0.0)
    span = config.depth_max_m - config.depth_min_m
    normalized = np.clip((meters - config.depth_min_m) / span, 0.0, 1.0)
    normalized = np.where(valid, normalized, 0.0)
    rgb = np.empty((*meters.shape, 3), dtype=np.uint8)
    for channel in range(3):
        rgb[..., channel] = np.rint(
            np.interp(normalized, _VIRIDIS_STOPS, _VIRIDIS_RGB[:, channel])
        ).astype(np.uint8)
    rgb[~valid] = 0
    return rgb


def _paste_fitted(
    canvas: Image.Image,
    array: np.ndarray,
    box: tuple[int, int, int, int],
    *,
    resample: Image.Resampling,
) -> None:
    left, top, right, bottom = box
    target_width = max(1, right - left)
    target_height = max(1, bottom - top)
    image = Image.fromarray(array, mode="RGB")
    scale = min(target_width / image.width, target_height / image.height)
    size = (
        max(1, int(round(image.width * scale))),
        max(1, int(round(image.height * scale))),
    )
    resized = image.resize(size, resample=resample)
    x = left + (target_width - size[0]) // 2
    y = top + (target_height - size[1]) // 2
    canvas.paste(resized, (x, y))


def _draw_camera_panel(
    canvas: Image.Image,
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    *,
    camera_name: str,
    modality: str,
    sample: CameraSample | None,
    config: ViewerConfig,
) -> None:
    left, top, right, bottom = box
    draw.rounded_rectangle(box, radius=8, fill=_PANEL, outline=_BORDER, width=1)
    label_font = _font(15)
    draw.text(
        (left + 10, top + 7),
        f"{camera_name}  {modality.upper()}",
        fill=_TEXT,
        font=label_font,
    )
    if sample is None:
        draw.text(
            (left + 10, top + 34),
            "MISSING ALIGNED CAMERA",
            fill=_WARNING,
            font=label_font,
        )
        return
    image = sample.color if modality == "color" else sample.depth
    if image is None:
        draw.text(
            (left + 10, top + 34),
            "NOT LOADED",
            fill=_MUTED,
            font=label_font,
        )
        return
    pixels = (
        color_image_to_rgb(image)
        if modality == "color"
        else depth_image_to_rgb(image, config)
    )
    _paste_fitted(
        canvas,
        pixels,
        (left + 5, top + 30, right - 5, bottom - 24),
        resample=(
            Image.Resampling.BILINEAR
            if modality == "color"
            else Image.Resampling.NEAREST
        ),
    )
    footer = f"delta={sample.delta_ns / 1_000_000.0:+.3f} ms"
    if sample.world_from_camera is not None:
        xyz = sample.world_from_camera.translation
        footer += f"  camera=({xyz[0]:+.2f},{xyz[1]:+.2f},{xyz[2]:+.2f}) m"
    draw.text(
        (left + 9, bottom - 20),
        footer,
        fill=_MUTED,
        font=_font(11),
    )


def _project(
    horizontal: float,
    vertical: float,
    box: tuple[int, int, int, int],
    tracker_range_m: float,
) -> tuple[int, int]:
    left, top, right, bottom = box
    half_width = (right - left) * 0.44
    half_height = (bottom - top) * 0.40
    center_x = (left + right) / 2.0
    center_y = (top + bottom) / 2.0
    return (
        int(round(center_x + horizontal / tracker_range_m * half_width)),
        int(round(center_y - vertical / tracker_range_m * half_height)),
    )


def _draw_tracker_plot(
    draw: ImageDraw.ImageDraw,
    box: tuple[int, int, int, int],
    *,
    title: str,
    horizontal_axis: int,
    vertical_axis: int,
    trackers: Iterable[tuple[str, TrackerPose | None]],
    tracker_range_m: float,
) -> None:
    left, top, right, bottom = box
    draw.rounded_rectangle(box, radius=8, fill=_PANEL, outline=_BORDER, width=1)
    draw.text((left + 9, top + 6), title, fill=_TEXT, font=_font(13))
    plot_box = (left + 8, top + 25, right - 8, bottom - 8)
    center = _project(0.0, 0.0, plot_box, tracker_range_m)
    draw.line((plot_box[0], center[1], plot_box[2], center[1]), fill=_BORDER)
    draw.line((center[0], plot_box[1], center[0], plot_box[3]), fill=_BORDER)
    for fraction in (-0.5, 0.5):
        horizontal = _project(
            fraction * tracker_range_m, 0.0, plot_box, tracker_range_m
        )[0]
        vertical = _project(
            0.0, fraction * tracker_range_m, plot_box, tracker_range_m
        )[1]
        draw.line((horizontal, plot_box[1], horizontal, plot_box[3]), fill=(40, 48, 59))
        draw.line((plot_box[0], vertical, plot_box[2], vertical), fill=(40, 48, 59))
    for role_index, (role, pose) in enumerate(trackers):
        if pose is None:
            continue
        transform = pose.world_from_tracker
        position = transform.translation
        matrix = transform.as_matrix()
        direction = matrix[:3, 0]
        start = _project(
            float(position[horizontal_axis]),
            float(position[vertical_axis]),
            plot_box,
            tracker_range_m,
        )
        arrow_scale = tracker_range_m * 0.14
        end = _project(
            float(position[horizontal_axis] + direction[horizontal_axis] * arrow_scale),
            float(position[vertical_axis] + direction[vertical_axis] * arrow_scale),
            plot_box,
            tracker_range_m,
        )
        color = _ROLE_COLORS[role_index % len(_ROLE_COLORS)]
        draw.line((start, end), fill=color, width=3)
        radius = 5
        draw.ellipse(
            (start[0] - radius, start[1] - radius, start[0] + radius, start[1] + radius),
            fill=color,
            outline=(255, 255, 255),
        )
        draw.text((start[0] + 7, start[1] - 8), role, fill=color, font=_font(10))


def render_aligned_frame(
    frame: AlignedFrame,
    *,
    camera_names: tuple[str, ...],
    total_frames: int,
    playing: bool,
    speed: float,
    config: ViewerConfig,
) -> Image.Image:
    """Render one aligned SDK frame as a complete RGB dashboard image."""
    if not camera_names:
        raise ValueError("camera_names must not be empty")
    if type(total_frames) is not int or total_frames <= frame.frame_index:
        raise ValueError("total_frames must include the rendered frame")
    if type(playing) is not bool:
        raise ValueError("playing must be bool")
    if not math.isfinite(speed) or speed <= 0.0:
        raise ValueError("speed must be a finite positive number")

    width = config.canvas_width
    height = config.canvas_height
    canvas = Image.new("RGB", (width, height), color=_BACKGROUND)
    draw = ImageDraw.Draw(canvas)
    header_height = 48
    footer_height = 30
    gutter = 8
    tracker_width = max(260, int(width * 0.25))
    camera_width = width - tracker_width - gutter
    camera_count = len(camera_names)
    column_width = (camera_width - gutter * (camera_count + 1)) // camera_count
    content_height = height - header_height - footer_height
    row_height = (content_height - gutter * 3) // 2

    state = "PLAY" if playing else "PAUSE"
    draw.text(
        (12, 11),
        f"ALIGNED DATASET  frame {frame.frame_index + 1}/{total_frames}  "
        f"{state} {speed:g}x  reference={frame.reference_camera}  "
        f"t={frame.reference_time_ns}",
        fill=_TEXT,
        font=_font(18),
    )
    for column, camera_name in enumerate(camera_names):
        left = gutter + column * (column_width + gutter)
        right = left + column_width
        sample = frame.cameras.get(camera_name)
        _draw_camera_panel(
            canvas,
            draw,
            (left, header_height + gutter, right, header_height + gutter + row_height),
            camera_name=camera_name,
            modality="color",
            sample=sample,
            config=config,
        )
        second_top = header_height + gutter * 2 + row_height
        _draw_camera_panel(
            canvas,
            draw,
            (left, second_top, right, second_top + row_height),
            camera_name=camera_name,
            modality="depth",
            sample=sample,
            config=config,
        )

    tracker_left = camera_width + gutter
    tracker_right = width - gutter
    tracker_items = tuple(frame.trackers.items())
    tracker_row_height = (content_height - gutter * 3) // 2
    _draw_tracker_plot(
        draw,
        (
            tracker_left,
            header_height + gutter,
            tracker_right,
            header_height + gutter + tracker_row_height,
        ),
        title=f"TRACKERS TOP  XY  range +/-{config.tracker_range_m:g} m",
        horizontal_axis=0,
        vertical_axis=1,
        trackers=tracker_items,
        tracker_range_m=config.tracker_range_m,
    )
    second_top = header_height + gutter * 2 + tracker_row_height
    _draw_tracker_plot(
        draw,
        (tracker_left, second_top, tracker_right, second_top + tracker_row_height),
        title=f"TRACKERS SIDE  XZ  range +/-{config.tracker_range_m:g} m",
        horizontal_axis=0,
        vertical_axis=2,
        trackers=tracker_items,
        tracker_range_m=config.tracker_range_m,
    )

    flags = " | ".join(frame.quality_flags) if frame.quality_flags else "quality flags: none"
    draw.text(
        (12, height - footer_height + 6),
        flags[:180],
        fill=_WARNING if frame.quality_flags else _MUTED,
        font=_font(12),
    )
    draw.text(
        (width - 420, height - footer_height + 6),
        "Space play/pause  Left/Right step  +/- speed  Home/End  Q quit",
        fill=_MUTED,
        font=_font(11),
    )
    return canvas
