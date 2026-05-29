"""Environment bootstrap: locate ROS 2, locate/build the colcon workspace, and
re-exec the process inside a fully sourced shell so ``import rclpy`` works.

The hard constraint this module exists to solve: ``rclpy`` and the ROS 2
message libraries are only importable when the process was started with the
ROS 2 environment already exported (PYTHONPATH, LD_LIBRARY_PATH, AMENT_*).
You cannot source ROS from inside a running interpreter. So when the user just
types ``7dof-pendant`` from a plain shell, we detect the missing environment,
re-launch ourselves under ``bash`` with the right setup files sourced, and
guard against looping with a sentinel env var.
"""

from __future__ import annotations

import os
import shutil
import subprocess
import sys
from pathlib import Path

# Set on the child process so a re-exec never recurses forever.
_SENTINEL = "PENDANT7DOF_SOURCED"

# Override to point at an arbitrary colcon workspace.
_WS_ENV = "PENDANT7DOF_WS"


def ros_distro() -> str:
    return os.environ.get("ROS_DISTRO") or "humble"


def find_ros_setup() -> Path | None:
    """Return the path to the system ROS 2 ``setup.bash``, or None."""
    candidates = []
    distro = os.environ.get("ROS_DISTRO")
    if distro:
        candidates.append(Path(f"/opt/ros/{distro}/setup.bash"))
    # Fall back to scanning /opt/ros for any installed distro.
    opt_ros = Path("/opt/ros")
    if opt_ros.is_dir():
        for d in sorted(opt_ros.iterdir()):
            candidates.append(d / "setup.bash")
    for c in candidates:
        if c.is_file():
            return c
    return None


def resolve_workspace() -> Path | None:
    """Locate the colcon workspace holding the ROS 2 backend source.

    Resolution order:
      1. ``$PENDANT7DOF_WS`` if set.
      2. A workspace bundled inside the package (``pendant7dof/workspace``),
         populated by ``7dof-pendant bundle``.
      3. Auto-discovery: walk up from this file and from $HOME looking for a
         directory that contains ``src/arm_bot`` (the dev workspace).
    """
    env_ws = os.environ.get(_WS_ENV)
    if env_ws:
        p = Path(env_ws).expanduser()
        if (p / "src").is_dir():
            return p

    bundled = Path(__file__).resolve().parent / "workspace"
    if (bundled / "src" / "arm_bot").is_dir():
        return bundled

    # Search common locations for a dev workspace.
    search_roots = [
        Path(__file__).resolve().parent.parent.parent,  # repo checkout
        Path.home() / "7dof_ws",
        Path.home(),
    ]
    for root in search_roots:
        if not root.is_dir():
            continue
        if (root / "src" / "arm_bot").is_dir():
            return root
        # Shallow scan one level down (e.g. ~/something/7dof_ws).
        try:
            for child in root.iterdir():
                if child.is_dir() and (child / "src" / "arm_bot").is_dir():
                    return child
        except PermissionError:
            continue
    return None


def workspace_is_built(ws: Path) -> bool:
    return (ws / "install" / "setup.bash").is_file()


def workspace_is_sourced(ws: Path) -> bool:
    """True if the workspace's ``install/`` prefix is on AMENT_PREFIX_PATH.

    ``rclpy`` being importable only proves the system ROS environment is
    sourced — it says nothing about the workspace packages (``arm_bot`` etc.).
    If the workspace is built but its prefix isn't on the path, ``ros2 launch
    arm_bot ...`` fails with "package not found", so we must still re-exec to
    source ``install/setup.bash``.
    """
    install_prefix = str((ws / "install").resolve())
    entries = os.environ.get("AMENT_PREFIX_PATH", "").split(os.pathsep)
    return any(e and Path(e).resolve() == Path(install_prefix) for e in entries)


def build_workspace(ws: Path, ros_setup: Path) -> bool:
    """colcon build the workspace. Returns True on success.

    Requires the system ROS 2 environment, so we source it first inside the
    same bash invocation.
    """
    if shutil.which("bash") is None:
        print("error: bash not found; cannot build workspace", file=sys.stderr)
        return False
    print(f"[7dof-pendant] Building workspace at {ws} (first run, one time)…")
    cmd = f"source '{ros_setup}' && cd '{ws}' && colcon build"
    result = subprocess.run(["bash", "-lc", cmd])
    if result.returncode != 0:
        print(
            "[7dof-pendant] colcon build failed. Fix the build errors above "
            "and rerun.",
            file=sys.stderr,
        )
        return False
    return True


def setup_files_to_source(ws: Path, ros_setup: Path) -> list[Path]:
    files = [ros_setup]
    ws_setup = ws / "install" / "setup.bash"
    if ws_setup.is_file():
        files.append(ws_setup)
    return files


def reexec_sourced(ws: Path | None, ros_setup: Path) -> "None":
    """Re-launch this exact command under bash with ROS + workspace sourced.

    Does not return on success (replaces the process via ``os.execvp``).
    """
    files = setup_files_to_source(ws, ros_setup) if ws else [ros_setup]
    source_cmd = " && ".join(f"source '{f}'" for f in files)

    # Rebuild the original argv. sys.argv[0] is the console-script path; argv
    # carries the user's subcommand/flags.
    quoted_args = " ".join(_shquote(a) for a in sys.argv)
    inner = f"{source_cmd} && exec {quoted_args}"

    env = dict(os.environ)
    env[_SENTINEL] = "1"
    if ws:
        env[_WS_ENV] = str(ws)

    os.execvpe("bash", ["bash", "-lc", inner], env)


def _shquote(s: str) -> str:
    import shlex

    return shlex.quote(s)


def rclpy_importable() -> bool:
    try:
        import rclpy  # noqa: F401

        return True
    except Exception:
        return False


def ensure_environment(build_if_needed: bool = True) -> Path | None:
    """Make sure rclpy is importable and the workspace is built.

    Returns the resolved workspace path (may be None if nothing was found but
    rclpy is still importable, e.g. user only wants jog with an external
    backend). Re-execs the process if the environment is not yet sourced.
    """
    already_sourced = os.environ.get(_SENTINEL) == "1"
    ws = resolve_workspace()

    # The environment is fully ready only when rclpy imports AND the resolved
    # workspace is both built and on AMENT_PREFIX_PATH. A shell that sources
    # only /opt/ros (rclpy works) but not the workspace would otherwise fail to
    # launch arm_bot. Once we've re-exec'd (sentinel set) we never re-exec again
    # for this reason — trust the sourced child to avoid any loop.
    ws_ready = ws is None or (
        workspace_is_built(ws)
        and (already_sourced or workspace_is_sourced(ws))
    )
    if rclpy_importable() and ws_ready:
        return ws

    if already_sourced:
        # We already re-exec'd once. If rclpy still won't import or the ws is
        # unbuilt, build now (we are sourced) then report honestly.
        ros_setup = find_ros_setup()
        if ws is not None and not workspace_is_built(ws):
            if build_if_needed and ros_setup is not None:
                if build_workspace(ws, ros_setup):
                    # Need the freshly built install/ on the path -> re-exec once more.
                    os.environ.pop(_SENTINEL, None)
                    reexec_sourced(ws, ros_setup)
            else:
                print(
                    "[7dof-pendant] Workspace is not built. Run with build "
                    "enabled or `colcon build` it manually.",
                    file=sys.stderr,
                )
        if not rclpy_importable():
            print(
                "[7dof-pendant] rclpy still not importable after sourcing ROS. "
                "Is ROS 2 Humble installed under /opt/ros?",
                file=sys.stderr,
            )
        return ws

    # Not sourced yet — find ROS and re-exec under a sourced bash.
    ros_setup = find_ros_setup()
    if ros_setup is None:
        print(
            "[7dof-pendant] Could not find a ROS 2 install under /opt/ros.\n"
            "Install ROS 2 Humble (sudo apt install ros-humble-desktop) "
            "and try again.",
            file=sys.stderr,
        )
        sys.exit(1)

    # If the workspace exists but isn't built, build it before re-execing so
    # the sourced child sees install/setup.bash.
    if ws is not None and not workspace_is_built(ws) and build_if_needed:
        if not build_workspace(ws, ros_setup):
            sys.exit(1)

    reexec_sourced(ws, ros_setup)
    return ws  # unreachable
