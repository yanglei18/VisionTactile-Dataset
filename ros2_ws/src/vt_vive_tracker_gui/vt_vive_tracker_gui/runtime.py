"""Owned ROS executor lifecycle for the tracker GUI."""

from __future__ import annotations

import threading
from collections.abc import Callable
from typing import Any

from rclpy.executors import SingleThreadedExecutor


class RosRuntime:
    """Run and stop one GUI ROS node without touching other runtimes."""

    def __init__(
        self,
        *,
        executor: Any,
        node: Any,
        shutdown_context: Callable[[], None],
    ) -> None:
        self._executor = executor
        self._node = node
        self._shutdown_context = shutdown_context
        self._lock = threading.Lock()
        self._started = False
        self._stopped = False
        self._thread = threading.Thread(
            target=self._executor.spin,
            name="vive-gui-ros",
            daemon=False,
        )

    @classmethod
    def from_node(
        cls,
        node: Any,
        *,
        shutdown_context: Callable[[], None],
    ) -> RosRuntime:
        executor = SingleThreadedExecutor(context=node.context)
        executor.add_node(node)
        return cls(
            executor=executor,
            node=node,
            shutdown_context=shutdown_context,
        )

    def start(self) -> None:
        with self._lock:
            if self._started or self._stopped:
                return
            self._started = True
            self._thread.start()

    def stop(self) -> None:
        with self._lock:
            if self._stopped:
                return

            errors: list[BaseException] = []
            try:
                self._executor.shutdown()
            except BaseException as error:
                errors.append(error)

            if self._started:
                try:
                    self._thread.join(timeout=2.0)
                    if self._thread.is_alive():
                        errors.append(
                            RuntimeError(
                                "ROS executor thread vive-gui-ros did not "
                                "stop within 2 seconds"
                            )
                        )
                except BaseException as error:
                    errors.append(error)

            try:
                self._node.destroy_node()
            except BaseException as error:
                errors.append(error)

            try:
                self._shutdown_context()
            except BaseException as error:
                errors.append(error)

            self._stopped = True
            if len(errors) == 1:
                raise errors[0]
            if errors:
                raise BaseExceptionGroup(
                    "ROS runtime cleanup failed",
                    errors,
                )
