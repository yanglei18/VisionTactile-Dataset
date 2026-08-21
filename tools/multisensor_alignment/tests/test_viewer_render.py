import unittest
import warnings

import numpy as np

from vt_multisensor_alignment.model import MessageRef, Transform
from vt_multisensor_alignment.sdk_model import (
    AlignedFrame,
    CameraSample,
    ImageData,
    TrackerPose,
)
from vt_multisensor_alignment.viewer_model import ViewerConfig
from vt_multisensor_alignment.viewer_render import (
    color_image_to_rgb,
    depth_image_to_rgb,
    render_aligned_frame,
)


def reference(topic: str = "/camera/image") -> MessageRef:
    return MessageRef(topic, 0, 100, 90)


def image(array: np.ndarray, encoding: str, topic: str) -> ImageData:
    return ImageData(
        array=array,
        encoding=encoding,
        frame_id="camera_optical_frame",
        source_timestamp_ns=90,
        reference=reference(topic),
    )


def tracker(role: str, xyz: tuple[float, float, float]) -> TrackerPose:
    return TrackerPose(
        role=role,
        tracker_id="a" * 64,
        timestamp_ns=1_000,
        bracket_gap_ns=20,
        before_sequence=0,
        after_sequence=1,
        world_from_tracker=Transform(
            np.asarray(xyz, dtype=np.float64),
            np.array([0.0, 0.0, 0.0, 1.0]),
        ),
    )


class ImageRenderingTests(unittest.TestCase):
    def test_bgr_color_is_converted_to_rgb_without_mutating_source(self) -> None:
        source = np.array([[[1, 2, 3], [4, 5, 6]]], dtype=np.uint8)
        rendered = color_image_to_rgb(image(source, "bgr8", "/color"))

        np.testing.assert_array_equal(
            rendered,
            np.array([[[3, 2, 1], [6, 5, 4]]], dtype=np.uint8),
        )
        np.testing.assert_array_equal(source[0, 0], np.array([1, 2, 3]))

    def test_depth_uses_meters_and_keeps_zero_invalid_pixels_black(self) -> None:
        source = np.array([[0, 200, 3_000]], dtype=np.uint16)
        rendered = depth_image_to_rgb(
            image(source, "16UC1", "/depth"),
            ViewerConfig(depth_min_m=0.2, depth_max_m=3.0),
        )

        np.testing.assert_array_equal(rendered[0, 0], np.array([0, 0, 0]))
        np.testing.assert_array_equal(rendered[0, 1], np.array([68, 1, 84]))
        np.testing.assert_array_equal(rendered[0, 2], np.array([253, 231, 37]))

    def test_float_depth_treats_nonfinite_and_nonpositive_values_as_invalid(self) -> None:
        source = np.array([[np.nan, -1.0, 1.0]], dtype=np.float32)
        with warnings.catch_warnings():
            warnings.simplefilter("error", RuntimeWarning)
            rendered = depth_image_to_rgb(
                image(source, "32FC1", "/depth"), ViewerConfig()
            )

        np.testing.assert_array_equal(rendered[0, 0], np.array([0, 0, 0]))
        np.testing.assert_array_equal(rendered[0, 1], np.array([0, 0, 0]))
        self.assertGreater(int(rendered[0, 2].sum()), 0)


class DashboardRenderingTests(unittest.TestCase):
    def test_renders_three_camera_dashboard_and_tracker_views(self) -> None:
        color = image(
            np.full((4, 6, 3), [20, 40, 60], dtype=np.uint8),
            "rgb8",
            "/color",
        )
        depth = image(
            np.full((4, 6), 1_000, dtype=np.uint16),
            "16UC1",
            "/depth",
        )
        sample = CameraSample(
            camera_name="d405_1",
            host_realtime_ns=1_000,
            source_timestamp_ns=90,
            delta_ns=-2_000_000,
            color=color,
            depth=depth,
            timing=None,
            timing_reference=reference("/timing"),
            attached_tracker=tracker("left_wrist", (0.1, 0.2, 0.3)),
            world_from_camera=Transform.identity(),
        )
        frame = AlignedFrame(
            frame_index=7,
            reference_camera="d405_1",
            reference_time_ns=1_000,
            cameras={"d405_1": sample, "d405_2": None, "d436": None},
            trackers={
                "left_wrist": tracker("left_wrist", (0.1, 0.2, 0.3)),
                "right_wrist": tracker("right_wrist", (-0.4, 0.5, 0.6)),
                "torso": tracker("torso", (0.0, 0.1, 1.0)),
            },
            additional_streams={},
            quality_flags=("camera:d436:missing",),
        )

        rendered = render_aligned_frame(
            frame,
            camera_names=("d405_1", "d405_2", "d436"),
            total_frames=100,
            playing=False,
            speed=1.0,
            config=ViewerConfig(canvas_width=1_200, canvas_height=600),
        )

        self.assertEqual(rendered.mode, "RGB")
        self.assertEqual(rendered.size, (1_200, 600))
        self.assertGreater(np.asarray(rendered).var(), 0.0)

    def test_out_of_range_tracker_cannot_draw_over_camera_panels(self) -> None:
        common = {
            "frame_index": 0,
            "reference_camera": "d405_1",
            "reference_time_ns": 1_000,
            "cameras": {"d405_1": None, "d405_2": None, "d436": None},
            "additional_streams": {},
            "quality_flags": (),
        }
        without_tracker = AlignedFrame(
            trackers={"left_wrist": None},
            **common,
        )
        outside_range = AlignedFrame(
            trackers={"left_wrist": tracker("left_wrist", (-10.0, 0.0, 0.0))},
            **common,
        )
        config = ViewerConfig(
            canvas_width=1_200,
            canvas_height=600,
            tracker_range_m=2.0,
        )

        baseline = np.asarray(
            render_aligned_frame(
                without_tracker,
                camera_names=("d405_1", "d405_2", "d436"),
                total_frames=1,
                playing=False,
                speed=1.0,
                config=config,
            )
        )
        rendered = np.asarray(
            render_aligned_frame(
                outside_range,
                camera_names=("d405_1", "d405_2", "d436"),
                total_frames=1,
                playing=False,
                speed=1.0,
                config=config,
            )
        )

        camera_area_right = 892
        np.testing.assert_array_equal(
            rendered[:, :camera_area_right],
            baseline[:, :camera_area_right],
        )


if __name__ == "__main__":
    unittest.main()
