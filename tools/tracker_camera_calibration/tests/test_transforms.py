import math
import unittest

import numpy as np

from vt_tracker_camera_calib.transforms import (
    Transform,
    rotation_distance_rad,
    transform_mean,
)


class TransformTest(unittest.TestCase):
    def test_compose_inverse_and_matrix_round_trip(self) -> None:
        value = Transform.from_rvec_tvec(
            np.array([0.2, -0.1, 0.3]), np.array([0.4, -0.2, 1.1])
        )
        self.assertTrue(np.allclose((value @ value.inverse()).matrix, np.eye(4)))
        recovered = Transform.from_matrix(value.matrix)
        self.assertTrue(np.allclose(recovered.translation, value.translation))
        self.assertLess(rotation_distance_rad(recovered, value), 1e-7)

    def test_quaternion_interpolation(self) -> None:
        left = Transform.identity()
        right = Transform.from_rvec_tvec(
            np.array([0.0, 0.0, math.pi / 2.0]), np.array([2.0, 0.0, 0.0])
        )
        middle = left.interpolate(right, 0.5)
        self.assertTrue(np.allclose(middle.translation, [1.0, 0.0, 0.0]))
        self.assertAlmostEqual(
            math.degrees(rotation_distance_rad(left, middle)), 45.0, places=6
        )

    def test_mean_handles_quaternion_sign(self) -> None:
        values = [
            Transform.from_rvec_tvec([0.01, 0.0, 0.0], [1.0, 2.0, 3.0]),
            Transform.from_rvec_tvec([-0.01, 0.0, 0.0], [3.0, 2.0, 1.0]),
        ]
        mean = transform_mean(values)
        self.assertTrue(np.allclose(mean.translation, [2.0, 2.0, 2.0]))
        self.assertLess(rotation_distance_rad(mean, Transform.identity()), 1e-7)

    def test_rejects_reflection(self) -> None:
        with self.assertRaisesRegex(ValueError, "determinant"):
            Transform(np.diag([1.0, 1.0, -1.0]), np.zeros(3))


if __name__ == "__main__":
    unittest.main()
