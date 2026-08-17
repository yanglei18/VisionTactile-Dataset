from __future__ import annotations

from dataclasses import dataclass
import importlib
from typing import Any


@dataclass(frozen=True)
class HidInterface:
    vendor_id: int
    product_id: int
    interface_number: int
    path: str
    product: str


def enumerate_vive(
    hid_module: Any | None = None,
) -> tuple[HidInterface, ...]:
    module = (
        hid_module
        if hid_module is not None
        else importlib.import_module("hid")
    )
    result: list[HidInterface] = []
    for product_id in (0x0350, 0x06A3):
        for device in module.enumerate(0x0BB4, product_id):
            path = device.get("path", b"")
            if isinstance(path, bytes):
                path = path.decode("utf-8", errors="replace")
            result.append(
                HidInterface(
                    vendor_id=int(device["vendor_id"]),
                    product_id=int(device["product_id"]),
                    interface_number=int(
                        device.get("interface_number", -1)
                    ),
                    path=str(path),
                    product=str(device.get("product_string") or ""),
                )
            )
    return tuple(result)


def check_mode(
    mode: str,
    values: tuple[HidInterface, ...],
) -> tuple[str, ...]:
    products = {
        "TRACKER_USB": 0x06A3,
        "DONGLE_USB": 0x0350,
    }
    if mode not in products:
        raise ValueError(f"unsupported mode: {mode}")
    product_id = products[mode]
    if any(
        value.product_id == product_id
        and value.interface_number == 0
        for value in values
    ):
        return ()
    return (f"missing 0bb4:{product_id:04x} interface 0",)
