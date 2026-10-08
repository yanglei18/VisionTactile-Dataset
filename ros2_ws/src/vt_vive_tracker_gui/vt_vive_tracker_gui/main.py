"""Executable composition root for the standalone tracker monitor."""

from __future__ import annotations

import tkinter as tk

import rclpy

from .app import TrackerApplication
from .ros_node import TrackerGuiNode
from .runtime import RosRuntime
from .snapshot_store import LatestSnapshotStore
from .view import TrackerDashboard


def main(args=None) -> None:
    rclpy.init(args=args)
    store = LatestSnapshotStore()
    node = TrackerGuiNode(store)
    runtime = RosRuntime.from_node(node, shutdown_context=rclpy.shutdown)
    root = tk.Tk()
    root.title("VIVE Ultimate Tracker Monitor")
    root.geometry("1280x800")
    root.minsize(1000, 650)
    view = TrackerDashboard(root)
    app = TrackerApplication(root, store, view, shutdown=runtime.stop)
    runtime.start()
    app.start()
    try:
        root.mainloop()
    finally:
        app.close()
