"""GUI entry point: spin rclpy from the Qt event loop and own lifecycle."""

from __future__ import annotations

import sys

import rclpy
from PyQt6.QtWidgets import QApplication
from PyQt6.QtCore import QTimer

from ..ros_bridge import PendantBridge
from ..launcher import BackendProcess
from .main_window import MainWindow


def run_gui(no_backend: bool = False) -> int:
    rclpy.init()
    node = PendantBridge()
    backend = None if no_backend else BackendProcess()

    app = QApplication(sys.argv)
    window = MainWindow(node, backend)
    window.show()

    # Pump rclpy from Qt's loop. The rclpy.ok() guard prevents RCLError after
    # Ctrl+C tears down the context before Qt's loop exits (same pattern as
    # the project's drawing_ui_node).
    timer = QTimer()

    def _tick() -> None:
        if not rclpy.ok():
            timer.stop()
            app.quit()
            return
        try:
            rclpy.spin_once(node, timeout_sec=0)
        except Exception:
            timer.stop()
            app.quit()

    timer.timeout.connect(_tick)
    timer.start(10)  # 100 Hz

    exit_code = app.exec()

    timer.stop()
    if backend is not None:
        backend.stop()
    if rclpy.ok():
        node.destroy_node()
        rclpy.shutdown()
    return exit_code
