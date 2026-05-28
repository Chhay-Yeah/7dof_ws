"""Manage the ROS 2 backend as a child process.

The pendant GUI runs its own rclpy node in-process, but the heavy backend
(Gazebo, controllers, the IK/FK and drawing nodes) is started as a separate
``ros2 launch`` process so it can be torn down cleanly when the GUI exits.
"""

from __future__ import annotations

import os
import signal
import subprocess
from pathlib import Path

# Default backend launch composed for the pendant (see arm_bot/launch).
DEFAULT_LAUNCH = ("arm_bot", "pendant_backend.launch.py")


class BackendProcess:
    def __init__(self, package: str = DEFAULT_LAUNCH[0],
                 launch_file: str = DEFAULT_LAUNCH[1],
                 extra_args: list[str] | None = None) -> None:
        self.package = package
        self.launch_file = launch_file
        self.extra_args = extra_args or []
        self._proc: subprocess.Popen | None = None

    @property
    def running(self) -> bool:
        return self._proc is not None and self._proc.poll() is None

    def start(self, extra_args: list[str] | None = None) -> None:
        if self.running:
            return
        args = list(self.extra_args)
        if extra_args:
            args.extend(extra_args)
        cmd = ["ros2", "launch", self.package, self.launch_file, *args]
        # New process group so we can signal the whole launch tree on stop.
        self._proc = subprocess.Popen(
            cmd,
            start_new_session=True,
            stdout=None,
            stderr=None,
        )

    def stop(self, timeout: float = 10.0) -> None:
        if self._proc is None:
            return
        if self._proc.poll() is None:
            try:
                os.killpg(os.getpgid(self._proc.pid), signal.SIGINT)
                self._proc.wait(timeout=timeout)
            except (ProcessLookupError, subprocess.TimeoutExpired):
                try:
                    os.killpg(os.getpgid(self._proc.pid), signal.SIGKILL)
                except ProcessLookupError:
                    pass
        self._proc = None
