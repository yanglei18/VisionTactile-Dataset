import unittest

from vt_multisensor_alignment.viewer_model import (
    PlaybackController,
    ViewerConfig,
)


class ViewerConfigTests(unittest.TestCase):
    def test_rejects_inverted_depth_range(self) -> None:
        with self.assertRaisesRegex(ValueError, "depth_max_m"):
            ViewerConfig(depth_min_m=2.0, depth_max_m=1.0)

    def test_rejects_canvas_that_cannot_fit_product_layout(self) -> None:
        with self.assertRaisesRegex(ValueError, "canvas_width"):
            ViewerConfig(canvas_width=799)


class PlaybackControllerTests(unittest.TestCase):
    def test_realtime_tick_skips_frames_when_rendering_is_behind(self) -> None:
        controller = PlaybackController(
            (1_000_000_000, 1_100_000_000, 1_200_000_000, 1_300_000_000),
            start_index=0,
            speed=1.0,
        )

        controller.play(now_ns=5_000_000_000)

        self.assertEqual(controller.tick(now_ns=5_250_000_000), 2)
        self.assertTrue(controller.playing)

    def test_tick_pauses_on_last_frame(self) -> None:
        controller = PlaybackController(
            (1_000_000_000, 1_100_000_000),
            start_index=0,
            speed=2.0,
        )

        controller.play(now_ns=5_000_000_000)

        self.assertEqual(controller.tick(now_ns=5_100_000_000), 1)
        self.assertFalse(controller.playing)

    def test_seek_while_playing_reanchors_data_time(self) -> None:
        controller = PlaybackController(
            (1_000_000_000, 1_100_000_000, 1_200_000_000),
            start_index=0,
            speed=1.0,
        )
        controller.play(now_ns=5_000_000_000)

        self.assertEqual(controller.seek(1, now_ns=8_000_000_000), 1)
        self.assertEqual(controller.tick(now_ns=8_050_000_000), 1)
        self.assertEqual(controller.tick(now_ns=8_100_000_000), 2)

    def test_speed_change_preserves_current_frame_anchor(self) -> None:
        controller = PlaybackController(
            (1_000_000_000, 1_100_000_000, 1_200_000_000),
            start_index=0,
            speed=1.0,
        )
        controller.play(now_ns=5_000_000_000)
        controller.tick(now_ns=5_100_000_000)

        controller.set_speed(2.0, now_ns=5_100_000_000)

        self.assertEqual(controller.tick(now_ns=5_149_000_000), 1)
        self.assertEqual(controller.tick(now_ns=5_150_000_000), 2)

    def test_speed_change_preserves_fractional_timeline_progress(self) -> None:
        controller = PlaybackController(
            (1_000_000_000, 1_100_000_000, 1_200_000_000),
            start_index=0,
            speed=1.0,
        )
        controller.play(now_ns=5_000_000_000)
        self.assertEqual(controller.tick(now_ns=5_050_000_000), 0)

        controller.set_speed(2.0, now_ns=5_050_000_000)

        self.assertEqual(controller.tick(now_ns=5_075_000_000), 1)

    def test_rejects_non_increasing_timeline(self) -> None:
        with self.assertRaisesRegex(ValueError, "strictly increasing"):
            PlaybackController((10, 10), start_index=0, speed=1.0)


if __name__ == "__main__":
    unittest.main()
