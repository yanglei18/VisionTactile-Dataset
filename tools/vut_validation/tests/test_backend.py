import unittest
from types import SimpleNamespace

from vut_validation.backend import PyVUTBackend


class FakeAPI:
    def __init__(self) -> None:
        self.callback = None
        self.started = 0
        self.stopped = 0
        self.tracker_group = SimpleNamespace(
            comms=SimpleNamespace(
                device_hid1=SimpleNamespace(nonblocking=0)
            )
        )

    def add_pose_callback(self, callback) -> None:
        self.callback = callback

    def start(self) -> None:
        self.started += 1

    def stop(self) -> None:
        self.stopped += 1


class BackendTests(unittest.TestCase):
    def test_backend_is_single_and_nonblocking(self) -> None:
        api = FakeAPI()
        backend = PyVUTBackend(
            "DONGLE_USB",
            api_factory=lambda mode: api,
        )

        backend.start(lambda sample: None)
        backend.stop()

        self.assertEqual(api.started, 1)
        self.assertEqual(api.stopped, 1)
        self.assertEqual(
            api.tracker_group.comms.device_hid1.nonblocking,
            1,
        )

    def test_callback_uses_both_host_clocks(self) -> None:
        api = FakeAPI()
        observed = []
        backend = PyVUTBackend(
            "DONGLE_USB",
            api_factory=lambda mode: api,
            monotonic_ns=lambda: 101,
            realtime_ns=lambda: 202,
        )
        backend.start(observed.append)

        api.callback(
            SimpleNamespace(
                mac="23:32:85:74:06:a3",
                position=(1.0, 2.0, 3.0),
                rotation=(1.0, 0.0, 0.0, 0.0),
                acceleration=(0.0, 0.0, 0.0),
                angular_velocity=(0.0, 0.0, 0.0, 0.0),
                tracking_status=2,
                buttons=0,
                timestamp_ms=303,
            )
        )

        self.assertEqual(observed[0].host_monotonic_ns, 101)
        self.assertEqual(observed[0].host_realtime_ns, 202)
        self.assertEqual(observed[0].upstream_timestamp_ms, 303)
        self.assertEqual(
            observed[0].tracker_id,
            "23:30:85:74:06:a3",
        )

    def test_backend_rejects_second_start(self) -> None:
        backend = PyVUTBackend(
            "DONGLE_USB",
            api_factory=lambda mode: FakeAPI(),
        )
        backend.start(lambda sample: None)

        with self.assertRaisesRegex(RuntimeError, "already running"):
            backend.start(lambda sample: None)


if __name__ == "__main__":
    unittest.main()
