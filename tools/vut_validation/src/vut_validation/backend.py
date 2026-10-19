from __future__ import annotations

import importlib
import time
from typing import Callable

from .model import PoseSample, canonical_tracker_id


class PyVUTBackend:
    def __init__(
        self,
        mode: str,
        api_factory=None,
        monotonic_ns=time.monotonic_ns,
        realtime_ns=time.time_ns,
    ) -> None:
        self.mode = mode
        self.api_factory = api_factory
        self.monotonic_ns = monotonic_ns
        self.realtime_ns = realtime_ns
        self.api = None

    def start(
        self,
        callback: Callable[[PoseSample], None],
    ) -> None:
        if self.api is not None:
            raise RuntimeError("backend is already running")
        if self.api_factory is not None:
            self.api = self.api_factory(self.mode)
        else:
            api_type = importlib.import_module(
                "pyvut"
            ).UltimateTrackerAPI
            self.api = api_type(mode=self.mode)

        self.api.tracker_group.comms.device_hid1.nonblocking = 1

        def receive(pose) -> None:
            callback(
                PoseSample(
                    tracker_id=canonical_tracker_id(str(pose.mac)),
                    host_monotonic_ns=self.monotonic_ns(),
                    host_realtime_ns=self.realtime_ns(),
                    upstream_timestamp_ms=int(pose.timestamp_ms),
                    position=tuple(
                        float(value) for value in pose.position
                    ),
                    quaternion_wxyz=tuple(
                        float(value) for value in pose.rotation
                    ),
                    acceleration=tuple(
                        float(value) for value in pose.acceleration
                    ),
                    angular_velocity=tuple(
                        float(value) for value in pose.angular_velocity
                    ),
                    tracking_status=int(pose.tracking_status),
                    buttons=int(pose.buttons),
                )
            )

        self.api.add_pose_callback(receive)
        self.api.start()

    def stop(self) -> None:
        if self.api is None:
            return
        self.api.stop()
        self.api = None
