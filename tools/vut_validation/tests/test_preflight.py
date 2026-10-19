import unittest

from vut_validation.preflight import (
    HidInterface,
    check_mode,
    enumerate_vive,
)


class FakeHid:
    @staticmethod
    def enumerate(vendor_id, product_id):
        if product_id != 0x06A3:
            return []
        return [
            {
                "vendor_id": vendor_id,
                "product_id": product_id,
                "interface_number": 0,
                "path": b"/dev/hidraw2",
                "product_string": "VIVE Ultimate Tracker",
                "serial_number": "must-not-be-returned",
            }
        ]


class PreflightTests(unittest.TestCase):
    def test_enumeration_omits_serial_number(self) -> None:
        values = enumerate_vive(FakeHid)

        self.assertEqual(len(values), 1)
        self.assertEqual(values[0].path, "/dev/hidraw2")
        self.assertFalse(hasattr(values[0], "serial_number"))

    def test_dongle_preflight_accepts_interface_zero(self) -> None:
        values = (
            HidInterface(
                vendor_id=0x0BB4,
                product_id=0x0350,
                interface_number=0,
                path="/dev/hidraw1",
                product="VIVE Wireless Dongle",
            ),
        )

        self.assertEqual(check_mode("DONGLE_USB", values), ())

    def test_direct_preflight_requires_tracker_interface_zero(self) -> None:
        errors = check_mode("TRACKER_USB", ())

        self.assertEqual(
            errors,
            ("missing 0bb4:06a3 interface 0",),
        )

    def test_unknown_mode_is_rejected(self) -> None:
        with self.assertRaisesRegex(ValueError, "unsupported mode"):
            check_mode("AUTO", ())


if __name__ == "__main__":
    unittest.main()
