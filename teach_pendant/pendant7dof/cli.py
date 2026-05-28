"""Command-line entry point for ``7dof-pendant``.

Subcommands:
  (none) / launch   Bootstrap the environment and open the teach pendant GUI.
  build             colcon-build the resolved workspace and exit.
  bundle            Copy the dev workspace source into the package so the wheel
                    is self-contained.
  release           Bundle + build a self-contained wheel correctly, then clean
                    up. Avoids the UNKNOWN-0.0.0 metadata trap.
  doctor            Print environment diagnostics (ROS, workspace, rclpy).
"""

from __future__ import annotations

import argparse
import shutil
import subprocess
import sys
from pathlib import Path

from . import __version__, bootstrap


def _package_dir() -> Path:
    """The installed/sourced ``pendant7dof`` package directory."""
    return Path(__file__).resolve().parent


def _project_root() -> Path | None:
    """The source checkout root (holds ``pyproject.toml``) when running from
    source; ``None`` when running from an installed wheel."""
    root = _package_dir().parent
    return root if (root / "pyproject.toml").is_file() else None


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

    release = sub.add_parser(
        "release",
        help="Bundle + build a self-contained wheel correctly, then clean up.",
    )
    release.add_argument(
        "--no-bundle",
        action="store_true",
        help="Build a GUI-only wheel without the ROS backend source.",
    )
    release.add_argument(
        "--keep-bundle",
        action="store_true",
        help="Leave the bundled workspace in place after building.",
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
    dest = _package_dir() / "workspace"
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


def _clean_build_artifacts(root: Path) -> None:
    """Remove stale build dirs and *.egg-info. A leftover ``UNKNOWN.egg-info``
    is exactly what makes setuptools emit an ``UNKNOWN-0.0.0`` wheel, so this is
    the key step that keeps the build honest."""
    for p in [root / "build", *root.glob("*.egg-info")]:
        shutil.rmtree(p, ignore_errors=True)


def _cmd_release(no_bundle: bool, keep_bundle: bool) -> int:
    root = _project_root()
    if root is None:
        print(
            "error: `release` must run from a source checkout (no pyproject.toml "
            "found next to the package); it is not available from an installed "
            "wheel.",
            file=sys.stderr,
        )
        return 1

    # 1. Build-tooling sanity. setuptools must be >=61 for the PEP 621 [project]
    #    table to be read — otherwise the wheel comes out as UNKNOWN-0.0.0.
    issues = []
    try:
        import setuptools

        if int(setuptools.__version__.split(".")[0]) < 61:
            issues.append(
                f"setuptools {setuptools.__version__} is too old (need >=61); "
                "run: pip install -U 'setuptools>=61,<80'"
            )
    except Exception:
        issues.append("setuptools is not importable; run: pip install 'setuptools>=61,<80'")
    try:
        import wheel  # noqa: F401
    except Exception:
        issues.append("the 'wheel' package is missing; run: pip install wheel")
    if issues:
        for msg in issues:
            print("error:", msg, file=sys.stderr)
        return 1

    # 2. Clear stale metadata so the build can't inherit a bad name.
    _clean_build_artifacts(root)

    # 3. Bundle the backend source into the package (unless GUI-only requested).
    bundled = False
    if not no_bundle:
        if _cmd_bundle(None) != 0:
            return 1
        bundled = True
    else:
        print("[release] --no-bundle: building a GUI-only wheel (no ROS backend).")

    # 4. Build the wheel WITHOUT build isolation, so it uses this environment's
    #    vetted setuptools instead of an arbitrary (possibly ancient) cached one.
    print("[release] building wheel (no build isolation)…")
    rc = subprocess.run(
        [sys.executable, "-m", "pip", "wheel", ".", "--no-deps",
         "--no-build-isolation", "-w", "dist"],
        cwd=str(root),
    ).returncode

    # 5. Always restore the dev tree: drop the bundle (unless asked to keep it)
    #    and the egg-info the build just regenerated.
    if bundled and not keep_bundle:
        shutil.rmtree(_package_dir() / "workspace", ignore_errors=True)
        print("[release] removed bundled workspace (dev launches use live ws again).")
    for p in root.glob("*.egg-info"):
        shutil.rmtree(p, ignore_errors=True)
    shutil.rmtree(root / "build", ignore_errors=True)

    if rc != 0:
        print("error: wheel build failed", file=sys.stderr)
        return rc

    # 6. Verify the artifact and report. Drop any UNKNOWN wheel that slipped by.
    dist = root / "dist"
    for bad in dist.glob("UNKNOWN-*.whl"):
        bad.unlink()
    wheels = sorted(dist.glob("7dof_pendant-*.whl"), key=lambda p: p.stat().st_mtime)
    if not wheels:
        print(
            "error: no 7dof_pendant wheel was produced — metadata problem "
            "(check setuptools version).",
            file=sys.stderr,
        )
        return 1
    whl = wheels[-1]
    kind = "GUI only" if no_bundle else "self-contained: GUI + ROS backend"
    size = whl.stat().st_size
    size_str = f"{size / 1e6:.1f} MB" if size >= 1e6 else f"{size / 1e3:.0f} KB"
    print(f"\n[release] OK -> {whl}")
    print(f"[release] {size_str} ({kind})")
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
    if command == "release":
        return _cmd_release(args.no_bundle, args.keep_bundle)
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
