import math
import unittest

import numpy as np

from vt_multisensor_alignment.model import (
    CameraFrame,
    InterpolatedPose,
    MessageRef,
    Transform,
)


def message_ref(topic: str, sequence: int, stamp_ns: int) -> MessageRef:
    return MessageRef(topic, sequence, stamp_ns + 100, stamp_ns)


class TransformTests(unittest.TestCase):
    def test_compose_rotates_child_translation(self) -> None:
        half = math.sqrt(0.5)
        world_from_parent = Transform(
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, half, half]),
        )
        parent_from_child = Transform(
            np.array([1.0, 0.0, 0.0]),
            np.array([0.0, 0.0, 0.0, 1.0]),
        )

        result = world_from_parent.compose(parent_from_child)

        np.testing.assert_allclose(result.translation, [1.0, 1.0, 0.0], atol=1e-12)
        np.testing.assert_allclose(
            result.quaternion_xyzw, [0.0, 0.0, half, half], atol=1e-12
        )

    def test_interpolate_uses_shortest_quaternion_arc(self) -> None:
        start = Transform.identity()
        finish = Transform(
            np.array([2.0, 0.0, 0.0]),
            np.array([0.0, 0.0, -1.0, 0.0]),
        )

        result = start.interpolate(finish, 0.5)

        np.testing.assert_allclose(result.translation, [1.0, 0.0, 0.0])
        self.assertAlmostEqual(abs(result.quaternion_xyzw[2]), math.sqrt(0.5))
        self.assertAlmostEqual(abs(result.quaternion_xyzw[3]), math.sqrt(0.5))

    def test_camera_frame_rejects_mixed_source_stamps(self) -> None:
        with self.assertRaisesRegex(ValueError, "same source timestamp"):
            CameraFrame(
                camera_name="d405_1",
                source_timestamp_ns=10,
                host_realtime_ns=20,
                host_monotonic_ns=30,
                color=message_ref("/color", 0, 10),
                depth=message_ref("/depth", 0, 11),
                timing=message_ref("/timing", 0, 10),
            )

    def test_interpolated_pose_rejects_invalid_bracket(self) -> None:
        with self.assertRaisesRegex(ValueError, "bracket_gap_ns"):
            InterpolatedPose(
                timestamp_ns=10,
                transform=Transform.identity(),
                bracket_gap_ns=-1,
                before_sequence=1,
                after_sequence=2,
            )


if __name__ == "__main__":
    unittest.main()
