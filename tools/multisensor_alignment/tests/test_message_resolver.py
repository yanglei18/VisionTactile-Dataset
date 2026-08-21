from dataclasses import dataclass
from types import SimpleNamespace
import unittest

import numpy as np

from vt_multisensor_alignment.errors import (
    DatasetClosedError,
    DatasetFormatError,
    MissingMessageError,
)
from vt_multisensor_alignment.message_resolver import MessageResolver
from vt_multisensor_alignment.model import MessageRef


def stamp(value_ns: int) -> SimpleNamespace:
    return SimpleNamespace(
        sec=value_ns // 1_000_000_000,
        nanosec=value_ns % 1_000_000_000,
    )


@dataclass
class ValueMessage:
    source_ns: int
    value: str

    @property
    def header(self) -> SimpleNamespace:
        return SimpleNamespace(stamp=stamp(self.source_ns), frame_id="frame")


class InMemoryBackend:
    def __init__(self, records: list[tuple[str, object, int]]) -> None:
        self.records = sorted(records, key=lambda item: item[2])
        self.index = 0
        self.seek_calls: list[int] = []
        self.read_count = 0
        self.closed = False

    def seek(self, timestamp_ns: int) -> None:
        self.seek_calls.append(timestamp_ns)
        self.index = next(
            (
                index
                for index, (_, _, bag_time) in enumerate(self.records)
                if bag_time >= timestamp_ns
            ),
            len(self.records),
        )

    def has_next(self) -> bool:
        return self.index < len(self.records)

    def read_next(self) -> tuple[str, object, int]:
        result = self.records[self.index]
        self.index += 1
        self.read_count += 1
        return result

    def close(self) -> None:
        self.closed = True


def reference(
    topic: str, bag_timestamp_ns: int, source_timestamp_ns: int, sequence: int = 0
) -> MessageRef:
    return MessageRef(topic, sequence, bag_timestamp_ns, source_timestamp_ns)


def resolver(
    records: list[tuple[str, object, int]],
    *,
    timestamp_fields: dict[str, str] | None = None,
    reusable_topics: frozenset[str] = frozenset(),
) -> tuple[MessageResolver, InMemoryBackend]:
    backend = InMemoryBackend(records)
    value = MessageResolver(
        backend=backend,
        deserialize=lambda _topic, payload: payload,
        timestamp_fields=timestamp_fields
        or {topic: "header.stamp" for topic, _, _ in records},
        reusable_topics=reusable_topics,
    )
    return value, backend


class MessageResolverTests(unittest.TestCase):
    def test_matches_topic_bag_time_and_source_time(self) -> None:
        target = reference("/camera/color", 100, 10)
        value, _ = resolver(
            [
                ("/camera/color", ValueMessage(9, "wrong-source"), 100),
                ("/other", ValueMessage(10, "wrong-topic"), 100),
                ("/camera/color", ValueMessage(10, "exact"), 100),
                ("/camera/color", ValueMessage(10, "wrong-bag-time"), 101),
            ]
        )

        result = value.resolve_many((target,))

        self.assertEqual(result[target].value, "exact")

    def test_matches_distinct_topics_at_same_bag_time(self) -> None:
        left = reference("/left", 100, 10)
        right = reference("/right", 100, 20)
        value, _ = resolver(
            [
                ("/left", ValueMessage(10, "left"), 100),
                ("/right", ValueMessage(20, "right"), 100),
            ]
        )

        result = value.resolve_many((right, left))

        self.assertEqual(result[left].value, "left")
        self.assertEqual(result[right].value, "right")

    def test_reports_source_timestamp_mismatch(self) -> None:
        target = reference("/camera", 100, 10)
        value, _ = resolver(
            [("/camera", ValueMessage(11, "wrong"), 100)]
        )

        with self.assertRaisesRegex(MissingMessageError, "source_timestamp_ns=10"):
            value.resolve_many((target,))

    def test_reports_reference_missing_from_bag(self) -> None:
        target = reference("/camera", 100, 10)
        value, _ = resolver([],
            timestamp_fields={"/camera": "header.stamp"},
        )

        with self.assertRaisesRegex(MissingMessageError, "/camera"):
            value.resolve_many((target,))

    def test_reuses_forward_cursor_and_seeks_for_backward_access(self) -> None:
        first = reference("/camera", 100, 10)
        second = reference("/camera", 200, 20)
        value, backend = resolver(
            [
                ("/camera", ValueMessage(10, "first"), 100),
                ("/camera", ValueMessage(20, "second"), 200),
            ]
        )

        value.resolve_many((first,))
        value.resolve_many((second,))
        value.resolve_many((first,))

        self.assertEqual(backend.seek_calls, [100, 100])

    def test_reuses_extension_message_without_backward_seek(self) -> None:
        glove = reference("/glove", 100, 10)
        later = reference("/camera", 200, 20)
        value, backend = resolver(
            [
                ("/glove", ValueMessage(10, "glove"), 100),
                ("/camera", ValueMessage(20, "camera"), 200),
            ],
            reusable_topics=frozenset({"/glove"}),
        )

        first_result = value.resolve_many((glove, later))
        second_result = value.resolve_many((glove,))

        self.assertIs(first_result[glove], second_result[glove])
        self.assertEqual(backend.seek_calls, [100])

    def test_rejects_duplicate_exact_messages(self) -> None:
        target = reference("/camera", 100, 10)
        value, _ = resolver(
            [
                ("/camera", ValueMessage(10, "first"), 100),
                ("/camera", ValueMessage(10, "second"), 100),
            ]
        )

        with self.assertRaisesRegex(DatasetFormatError, "duplicate"):
            value.resolve_many((target,))

    def test_close_is_idempotent_and_prevents_resolution(self) -> None:
        value, backend = resolver([])

        value.close()
        value.close()

        self.assertTrue(backend.closed)
        with self.assertRaises(DatasetClosedError):
            value.resolve_many(())

    def test_camera_info_is_matched_by_optical_frame(self) -> None:
        first = SimpleNamespace(
            header=SimpleNamespace(frame_id="d405_color_optical_frame"),
            width=2,
            height=1,
            distortion_model="plumb_bob",
            d=[0.1, 0.2, 0.3, 0.4, 0.5],
            k=[1.0, 0.0, 0.5, 0.0, 2.0, 0.25, 0.0, 0.0, 1.0],
            r=[1.0, 0.0, 0.0, 0.0, 1.0, 0.0, 0.0, 0.0, 1.0],
            p=[1.0, 0.0, 0.5, 0.0, 0.0, 2.0, 0.25, 0.0, 0.0, 0.0, 1.0, 0.0],
            binning_x=0,
            binning_y=0,
            roi=SimpleNamespace(
                x_offset=0,
                y_offset=0,
                height=0,
                width=0,
                do_rectify=False,
            ),
        )
        second = SimpleNamespace(**{**first.__dict__})
        second.header = SimpleNamespace(frame_id="d436_color_optical_frame")
        second.width = 4
        value, backend = resolver(
            [
                ("/d436/camera_info", second, 80),
                ("/d405/camera_info", first, 90),
            ],
            timestamp_fields={},
        )

        result = value.load_camera_info(
            camera_frames={
                "d405_1": "d405_color_optical_frame",
                "d436": "d436_color_optical_frame",
            },
            camera_info_topics=frozenset(
                {"/d405/camera_info", "/d436/camera_info"}
            ),
        )

        self.assertEqual(result["d405_1"].width, 2)
        self.assertEqual(result["d436"].width, 4)
        np.testing.assert_array_equal(result["d405_1"].k[1], [0.0, 2.0, 0.25])
        self.assertFalse(result["d405_1"].k.flags.writeable)
        self.assertEqual(backend.seek_calls, [0, 0])

    def test_camera_info_rejects_missing_frame(self) -> None:
        value, _ = resolver([], timestamp_fields={})

        with self.assertRaisesRegex(MissingMessageError, "d436"):
            value.load_camera_info(
                camera_frames={"d436": "d436_color_optical_frame"},
                camera_info_topics=frozenset({"/d436/camera_info"}),
            )


if __name__ == "__main__":
    unittest.main()
