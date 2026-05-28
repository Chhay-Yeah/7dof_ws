"""Command-line entry point for ``7dof-pendant``.

Subcommands:
  (none) / launch   Bootstrap the environment and open the teach pendant GUI.
  build             colcon-build the resolved workspace and exit.
  bundle            Copy the dev workspace source into the package so the wheel
                    is self-contained.
  doctor            Print environment diagnostics (ROS, workspace, rclpy).
"""

from __future__ import annotations

import argparse
import shutil
import sys
from pathlib import Path

from . import __version__, bootstrap


def _build_parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        prog="7dof-pendant",
        description="Teach pendant GUI for the 7-DOF robotic arm.",
    )
    p.add_argument("--version", action="version", version=f"7dof-pendant {__version__}")
    sub = p.add_subparsers(dest="command")

    launch = sub.add_parser("launch", help="Open the teach pendant GUI (default).")
    launch.add_argument(
        "--no-backend",
        action="store_true",
        help="Do not auto-launch Gazebo/controllers; attach to a running backend.",
    )
    launch.add_argument(
        "--no-build",
        action="store_true",
        help="Do not colcon-build the workspace even if it looks unbuilt.",
    )

    sub.add_parser("build", help="colcon-build the resolved workspace and exit.")
    sub.add_parser("doctor", help="Print environment diagnostics and exit.")

    bundle = sub.add_parser(
        "bundle", help="Copy the dev workspace source into the package."
    )
    bundle.add_argument(
        "--from",
        dest="src_ws",
        default=None,
        help="Workspace to copy from (default: auto-resolved dev workspace).",
    )

    return p


def _cmd_doctor() -> int:
    ros_setup = bootstrap.find_ros_setup()
    ws = bootstrap.resolve_workspace()
    print(f"7dof-pendant {__version__}")
    print(f"  ROS distro       : {bootstrap.ros_distro()}")
    print(f"  ROS setup.bash   : {ros_setup or 'NOT FOUND'}")
    print(f"  Workspace        : {ws or 'NOT FOUND'}")
    if ws is not None:
        print(f"  Workspace built  : {bootstrap.workspace_is_built(ws)}")
    print(f"  rclpy importable : {bootstrap.rclpy_importable()}")
    ok = ros_setup is not None and ws is not None
    print("  Status           :", "READY" if ok else "INCOMPLETE")
    return 0 if ok else 1


def _cmd_build(no_build: bool = False) -> int:
    ros_setup = bootstrap.find_ros_setup()
    ws = bootstrap.resolve_workspace()
    if ros_setup is None:
        print("error: no ROS 2 install found under /opt/ros", file=sys.stderr)
        return 1
    if ws is None:
        print("error: no workspace found to build", file=sys.stderr)
        return 1
    return 0 if bootstrap.build_workspace(ws, ros_setup) else 1


def _cmd_bundle(src_ws: str | None) -> int:
    ws = Path(src_ws).expanduser() if src_ws else bootstrap.resolve_workspace()
    if ws is None or not (ws / "src").is_dir():
        print("error: could not resolve a source workspace to bundle", file=sys.stderr)
        return 1
    dest = Path(__file__).resolve().parent / "workspace"
    dest_src = dest / "src"
    print(f"[bundle] copying {ws / 'src'} -> {dest_src}")
    if dest_src.exists():
        shutil.rmtree(dest_src)
    dest.mkdir(parents=True, exist_ok=True)
    shutil.copytree(
        ws / "src",
        dest_src,
        ignore=shutil.ignore_patterns(
            "build", "install", "log", "__pycache__", "*.pyc", ".git"
        ),
    )
    print("[bundle] done. The wheel built from here is now self-contained.")
    return 0


def main(argv: list[str] | None = None) -> int:
    parser = _build_parser()
    args = parser.parse_args(argv)
    command = args.command or "launch"

    # These never touch rclpy, so run them without bootstrapping the env.
    if command == "doctor":
        return _cmd_doctor()
    if command == "bundle":
        return _cmd_bundle(args.src_ws)
    if command == "build":
        return _cmd_build()

    # launch: ensure ROS env is sourced (may re-exec the process) and the
    # workspace is built, then open the GUI.
    no_build = getattr(args, "no_build", False)
    bootstrap.ensure_environment(build_if_needed=not no_build)

    # Imported lazily: this line only runs once rclpy is importable.
    from .gui.app import run_gui

    return run_gui(no_backend=getattr(args, "no_backend", False))


if __name__ == "__main__":
    sys.exit(main())
