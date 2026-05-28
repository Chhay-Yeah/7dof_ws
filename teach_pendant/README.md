# 7dof-pendant

A pip-installable **teach pendant GUI** for the 7-DOF robotic arm. One install,
one command:

```bash
pip install .
7dof-pendant
```

The app opens a PyQt6 pendant (joint jog, Cartesian jog, presets, drawing, live
status, E-stop) and drives the ROS 2 backend (Gazebo + ros2_control + the
custom IK/FK and drawing nodes).

## Requirements

ROS 2 **Humble** must be installed on the machine (`sudo apt install
ros-humble-desktop`). The pendant **cannot** bring ROS along inside the wheel —
`rclpy`, MoveIt, Gazebo and `ros2_control` come from the system install. The
pendant *does* bundle the robot's ROS 2 source and builds it for you on first
launch.

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
