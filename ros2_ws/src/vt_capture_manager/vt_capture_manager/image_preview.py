"""Bounded latest-only camera preview conversion, off the ROS callback thread."""
from __future__ import annotations

import base64
from collections import OrderedDict
import threading
import time


def jpeg_preview(message, *, max_width=320, max_height=240):
    """Respect stride and endian. Depth is visualized as relative raw units."""
    import cv2
    import numpy as np

    width, height, step = int(message.width), int(message.height), int(message.step)
    encoding = str(message.encoding).lower()
    formats = {"rgb8": (3, "u1"), "bgr8": (3, "u1"), "rgba8": (4, "u1"),
               "bgra8": (4, "u1"), "mono8": (1, "u1"), "8uc1": (1, "u1"),
               "16uc1": (1, ">u2" if message.is_bigendian else "<u2"),
               "mono16": (1, ">u2" if message.is_bigendian else "<u2"),
               "32fc1": (1, ">f4" if message.is_bigendian else "<f4")}
    if encoding not in formats:
        raise ValueError(f"unsupported preview encoding {encoding}")
    channels, dtype = formats[encoding]
    size = np.dtype(dtype).itemsize
    if not (0 < width <= 16384 and 0 < height <= 16384 and width*channels*size <= step):
        raise ValueError("invalid image dimensions/step")
    if step*height > 32*1024*1024 or len(message.data) > 32*1024*1024 or len(message.data) < step*height:
        raise ValueError("image data truncated or exceeds 32 MiB")
    rows = np.frombuffer(bytes(message.data), dtype=np.uint8, count=step*height).reshape(height, step)
    compact = rows[:, :width*channels*size].copy()
    pixels = compact.view(dtype).reshape(height, width, channels) if channels > 1 else compact.view(dtype).reshape(height, width)
    if size in (2, 4):
        pixels = pixels.astype(np.float32)
        mask = np.isfinite(pixels) & (pixels > 0)
        display = np.zeros((height, width), dtype=np.uint8)
        if mask.any():
            low, high = np.percentile(pixels[mask], [2, 98])
            display[mask] = np.clip((pixels[mask]-low) / max(float(high-low), 1.) * 254 + 1, 1, 255).astype(np.uint8)
        pixels = display
    elif encoding == "rgb8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGB2BGR)
    elif encoding == "rgba8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_RGBA2BGR)
    elif encoding == "bgra8":
        pixels = cv2.cvtColor(pixels, cv2.COLOR_BGRA2BGR)
    scale = min(1., max_width/width, max_height/height)
    w, h = max(1, round(width*scale)), max(1, round(height*scale))
    if (w, h) != (width, height):
        pixels = cv2.resize(pixels, (w, h), interpolation=cv2.INTER_AREA)
    ok, encoded = cv2.imencode(".jpg", pixels, [cv2.IMWRITE_JPEG_QUALITY, 70])
    if not ok:
        raise ValueError("JPEG encoding failed")
    return dict(width=w, height=h, format="jpeg", data_b64=base64.b64encode(encoded).decode("ascii"))


class ImagePreviewWorker:
    def __init__(self, model, *, converter=jpeg_preview):
        self.model, self.converter = model, converter
        self._pending = OrderedDict()
        self._condition = threading.Condition()
        self._stop = False
        self._last_submit = {}
        self._thread = threading.Thread(target=self._run, name="capture-camera-preview", daemon=True)

    def start(self):
        self._thread.start()
        return self

    def submit(self, device_id, message):
        if int(message.step) * int(message.height) > 32*1024*1024 or len(message.data) > 32*1024*1024:
            return
        with self._condition:
            now = time.monotonic()
            if now - self._last_submit.get(device_id, -1e9) < .1:
                return
            if device_id not in self._pending and len(self._pending) >= 8:
                return
            self._last_submit[device_id] = now
            self._pending[device_id] = message
            self._condition.notify()

    def _run(self):
        from .model import _stamp_ns
        while True:
            with self._condition:
                self._condition.wait_for(lambda: self._stop or bool(self._pending))
                if self._stop:
                    return
                device_id, message = self._pending.popitem(last=False)
            try:
                self.model.set_camera_preview(device_id, _stamp_ns(message), self.converter(message))
            except Exception as exc:
                self.model.event("warning", f"{device_id}: image preview unavailable: {exc}", key=f"preview:{device_id}")

    def close(self):
        with self._condition:
            self._stop = True
            self._pending.clear()
            self._condition.notify_all()
        self._thread.join(3.)
