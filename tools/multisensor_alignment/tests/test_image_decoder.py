from types import SimpleNamespace
import unittest

import numpy as np

from vt_multisensor_alignment.errors import (
    DatasetFormatError,
    UnsupportedEncodingError,
)
from vt_multisensor_alignment.image_decoder import decode_image
from vt_multisensor_alignment.model import MessageRef


STAMP = 12_000_000_034
REF = MessageRef("/camera/image", 2, STAMP + 100, STAMP)


def stamp(value_ns: int = STAMP) -> SimpleNamespace:
    return SimpleNamespace(
        sec=value_ns // 1_000_000_000,
        nanosec=value_ns % 1_000_000_000,
    )


def image(
    *,
    width: int,
    height: int,
    encoding: str,
    step: int,
    data: bytes,
    is_bigendian: int = 0,
    source_stamp_ns: int = STAMP,
) -> SimpleNamespace:
    return SimpleNamespace(
        header=SimpleNamespace(stamp=stamp(source_stamp_ns), frame_id="camera_optical"),
        width=width,
        height=height,
        encoding=encoding,
        step=step,
        data=data,
        is_bigendian=is_bigendian,
    )


class ImageDecoderTests(unittest.TestCase):
    def test_decodes_rgb8_without_changing_channel_order(self) -> None:
        decoded = decode_image(
            image(
                width=2,
                height=1,
                encoding="rgb8",
                step=6,
                data=bytes([1, 2, 3, 4, 5, 6]),
            ),
            REF,
        )

        np.testing.assert_array_equal(
            decoded.array, [[[1, 2, 3], [4, 5, 6]]]
        )
        self.assertEqual(decoded.encoding, "rgb8")
        self.assertEqual(decoded.frame_id, "camera_optical")
        self.assertEqual(decoded.reference, REF)
        self.assertFalse(decoded.array.flags.writeable)

    def test_decodes_bgr8_without_changing_channel_order(self) -> None:
        decoded = decode_image(
            image(
                width=1,
                height=1,
                encoding="bgr8",
                step=3,
                data=bytes([9, 8, 7]),
            ),
            REF,
        )

        np.testing.assert_array_equal(decoded.array, [[[9, 8, 7]]])
        self.assertEqual(decoded.encoding, "bgr8")

    def test_decoder_removes_row_padding_without_mixing_rows(self) -> None:
        decoded = decode_image(
            image(
                width=2,
                height=2,
                encoding="mono8",
                step=4,
                data=bytes([1, 2, 99, 99, 3, 4, 88, 88]),
            ),
            REF,
        )

        np.testing.assert_array_equal(decoded.array, [[1, 2], [3, 4]])

    def test_decodes_little_and_big_endian_uint16_to_native_dtype(self) -> None:
        little = decode_image(
            image(
                width=2,
                height=1,
                encoding="16UC1",
                step=4,
                data=bytes([0x34, 0x12, 0xCD, 0xAB]),
            ),
            REF,
        )
        big = decode_image(
            image(
                width=2,
                height=1,
                encoding="mono16",
                step=4,
                data=bytes([0x12, 0x34, 0xAB, 0xCD]),
                is_bigendian=1,
            ),
            REF,
        )

        np.testing.assert_array_equal(little.array, [[0x1234, 0xABCD]])
        np.testing.assert_array_equal(big.array, [[0x1234, 0xABCD]])
        self.assertEqual(little.array.dtype, np.dtype(np.uint16))
        self.assertEqual(big.array.dtype, np.dtype(np.uint16))

    def test_decodes_native_float32(self) -> None:
        values = np.array([1.25, -2.5], dtype="<f4")

        decoded = decode_image(
            image(
                width=2,
                height=1,
                encoding="32FC1",
                step=8,
                data=values.tobytes(),
            ),
            REF,
        )

        np.testing.assert_array_equal(decoded.array, [[1.25, -2.5]])
        self.assertEqual(decoded.array.dtype, np.dtype(np.float32))

    def test_rejects_unknown_encoding(self) -> None:
        with self.assertRaisesRegex(UnsupportedEncodingError, "yuv422"):
            decode_image(
                image(
                    width=1,
                    height=1,
                    encoding="yuv422",
                    step=2,
                    data=b"\x00\x00",
                ),
                REF,
            )

    def test_rejects_step_smaller_than_pixel_row(self) -> None:
        with self.assertRaisesRegex(DatasetFormatError, "step"):
            decode_image(
                image(
                    width=2,
                    height=1,
                    encoding="rgb8",
                    step=5,
                    data=bytes(5),
                ),
                REF,
            )

    def test_rejects_truncated_or_extra_payload(self) -> None:
        for payload in (bytes(3), bytes(5)):
            with self.subTest(size=len(payload)):
                with self.assertRaisesRegex(DatasetFormatError, "data length"):
                    decode_image(
                        image(
                            width=2,
                            height=2,
                            encoding="mono8",
                            step=2,
                            data=payload,
                        ),
                        REF,
                    )

    def test_rejects_source_timestamp_mismatch(self) -> None:
        with self.assertRaisesRegex(DatasetFormatError, "source timestamp"):
            decode_image(
                image(
                    width=1,
                    height=1,
                    encoding="mono8",
                    step=1,
                    data=b"\x01",
                    source_stamp_ns=STAMP + 1,
                ),
                REF,
            )

    def test_rejects_invalid_shape_or_endianness_fields(self) -> None:
        cases = (
            {"width": 0, "height": 1, "is_bigendian": 0},
            {"width": 1, "height": -1, "is_bigendian": 0},
            {"width": 1, "height": 1, "is_bigendian": 2},
        )
        for values in cases:
            with self.subTest(values=values):
                with self.assertRaises(DatasetFormatError):
                    decode_image(
                        image(
                            width=values["width"],
                            height=values["height"],
                            encoding="mono8",
                            step=1,
                            data=b"\x01",
                            is_bigendian=values["is_bigendian"],
                        ),
                        REF,
                    )


if __name__ == "__main__":
    unittest.main()
