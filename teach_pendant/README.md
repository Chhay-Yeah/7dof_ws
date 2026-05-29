# 7dof-pendant

A pip-installable **teach pendant GUI** for the 7-DOF robotic arm.

The app opens a PyQt6 pendant (joint jog, Cartesian jog, presets, drawing, live
status, E-stop) and drives the ROS 2 backend (Gazebo + ros2_control + the
custom IK/FK and drawing nodes).

## Install

ROS 2 **Humble** must already be installed (see [Requirements](#requirements)),
then install the pendant one of two ways:

**From PyPI (recommended for users):**

```bash
pip install --user 7dof-pendant
7dof-pendant            # ensure ~/.local/bin is on PATH
```

The published package is self-contained (GUI + bundled robot source). Project
page: <https://pypi.org/project/7dof-pendant/>.

**From source (for development):**

```bash
cd teach_pendant
pip install .
7dof-pendant
```

> The wheel ships only the GUI + robot source — it does **not** contain ROS.
> Install the system ROS dependencies first with `./install.sh` (see below).

## Requirements

ROS 2 **Humble** must be installed on the machine. The pendant **cannot** bring
ROS along inside the wheel — `rclpy`, MoveIt and `ros2_control` come from the
system install. The pendant *does* bundle the robot's ROS 2 source and builds it
for you on first launch.

### One-shot setup (`install.sh`)

`./install.sh` (run from `teach_pendant/`) installs every ROS dependency the
pendant needs to build and run via `rosdep`, plus the undeclared Python deps
(`numpy`, `scipy`, `urdf_parser_py`), then installs the pendant editable and
builds the workspace. It requires ROS 2 already installed under `/opt/ros`.
Use `./install.sh --no-build` to skip the colcon build.

It supports **real-hardware** control and the **`moveit` (RViz fake-hardware)
simulation** — RViz and MoveIt are installed. It deliberately **skips
Gazebo/Ignition**, so the `gazebo` physics-sim backend is not available; use
`moveit` mode (Settings → mode) for RViz-based testing.

## How it works

1. `7dof-pendant` checks whether the ROS 2 environment is sourced. If not, it
   re-execs itself under a shell with `/opt/ros/<distro>/setup.bash` and the
   workspace's `install/setup.bash` sourced — so you can launch from a plain
   terminal.
2. It locates the colcon workspace (`$PENDANT7DOF_WS`, a bundled copy, or your
   `~/7dof_ws`). On first run it `colcon build`s it once.
3. It opens the GUI. Use the **Settings** tab's **Launch Gazebo Simulation**
   button to bring up the backend (`ros2 launch arm_bot
   pendant_backend.launch.py`) as a managed child process — this is how you
   test the pendant end to end. Untick "Also open RViz" to skip RViz. The
   simulation is torn down when you close the app.

## Commands

```bash
7dof-pendant                 # launch the pendant (default)
7dof-pendant launch --no-backend   # attach to an already-running backend
7dof-pendant build           # colcon-build the workspace and exit
7dof-pendant bundle          # copy the dev workspace source into the package
7dof-pendant release         # build a shippable self-contained wheel
7dof-pendant doctor          # print environment diagnostics
```

## Building a self-contained wheel

By default the package finds your `~/7dof_ws` at runtime (no 49 MB of meshes
duplicated into git). To ship a wheel that carries the workspace inside it, run:

```bash
7dof-pendant release         # bundle + build + clean up, in one step
```

This bundles the backend source, builds the wheel **without pip build
isolation** (so it uses your vetted `setuptools>=61`, not an arbitrary cached
one), then removes the bundle so your dev launches keep using the live
`~/7dof_ws`. The wheel lands in `dist/7dof_pendant-<version>-py3-none-any.whl`.

> Do **not** use `python -m build` here: pip's build isolation can pull an
> ancient `setuptools` that ignores the PEP 621 `[project]` table and silently
> produces a broken `UNKNOWN-0.0.0` wheel. `7dof-pendant release` avoids that
> trap (and checks your `setuptools`/`wheel` versions first).

Flags: `--no-bundle` builds a GUI-only wheel (bring your own backend);
`--keep-bundle` leaves the bundled workspace in place after building.
