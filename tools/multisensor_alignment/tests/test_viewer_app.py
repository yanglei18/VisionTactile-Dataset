from types import ModuleType, SimpleNamespace
import sys
import unittest
from unittest import mock

import PIL

from vt_multisensor_alignment.sdk_model import AlignedFrame
from vt_multisensor_alignment.viewer_app import run_interactive
from vt_multisensor_alignment.viewer_model import ViewerConfig


class FakeLabel:
    def __init__(self, *args, **kwargs) -> None:
        self.image = None

    def pack(self) -> None:
        pass

    def configure(self, *, image) -> None:
        self.image = image


class FakeRoot:
    def __init__(self) -> None:
        self.key_callback = None
        self.after_callback = None
        self.destroyed = False
        self.swallowed = None

    def title(self, value: str) -> None:
        pass

    def resizable(self, width: bool, height: bool) -> None:
        pass

    def protocol(self, name: str, callback) -> None:
        pass

    def bind(self, name: str, callback) -> None:
        self.key_callback = callback

    def after(self, delay_ms: int, callback) -> None:
        self.after_callback = callback

    def destroy(self) -> None:
        self.destroyed = True

    def mainloop(self) -> None:
        self.key_callback(SimpleNamespace(keysym="Right"))
        try:
            self.after_callback()
        except Exception as error:
            # Real Tk reports callback errors but keeps mainloop alive. Returning
            # here models a user eventually closing that stale window.
            self.swallowed = error


class LateFailingDataset:
    camera_names = ("d405_1",)
    reference_times_ns = (1_000, 10_000_000_000)

    def __len__(self) -> int:
        return 2

    def frame(self, index: int, **kwargs) -> AlignedFrame:
        if index == 1:
            raise RuntimeError("late frame decode failed")
        return AlignedFrame(
            frame_index=0,
            reference_camera="d405_1",
            reference_time_ns=1_000,
            cameras={"d405_1": None},
            trackers={},
            additional_streams={},
            quality_flags=(),
        )


class ViewerAppTests(unittest.TestCase):
    def test_late_render_failure_closes_window_and_reaches_cli(self) -> None:
        root = FakeRoot()
        tkinter = ModuleType("tkinter")
        tkinter.Label = FakeLabel
        tkinter.TclError = RuntimeError
        tkinter.Tk = lambda: root
        image_tk = ModuleType("PIL.ImageTk")
        image_tk.PhotoImage = lambda image: object()

        with mock.patch.dict(
            sys.modules,
            {"tkinter": tkinter, "PIL.ImageTk": image_tk},
        ):
            with mock.patch.object(PIL, "ImageTk", image_tk, create=True):
                with self.assertRaisesRegex(
                    RuntimeError, "late frame decode failed"
                ):
                    run_interactive(
                        LateFailingDataset(),
                        start_index=0,
                        speed=1.0,
                        config=ViewerConfig(
                            canvas_width=800,
                            canvas_height=480,
                        ),
                    )

        self.assertTrue(root.destroyed)
        self.assertIsNone(root.swallowed)


if __name__ == "__main__":
    unittest.main()
