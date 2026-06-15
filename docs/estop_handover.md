# Handover — Hard E-stop redesign (7dof teach pendant)

**Date:** 2026-06-15  ·  **Scope:** the bottom red E-STOP / "RESET (E-stop active)" bar in the `7dof-pendant` GUI.

Hand this to whoever picks up the E-stop next. It explains what the E-stop used
to do, what it does now, exactly what changed, and what still needs verifying.

---

## 1. Context

- The GUI is `teach_pendant/pendant7dof/` (pip package `pendant7dof`, **editable**
  install → changes apply on a `7dof-pendant` app restart; **no** colcon build /
  backend restart).
- The GUI owns one rclpy node, `PendantBridge` in `ros_bridge.py`.
- It drives a ROS 2 backend: in **sim** that's `ros2 launch arm_bot
  pendant_backend.launch.py mode:=gazebo` (Gazebo + controllers + `ik_arm_v3` +
  `ik_to_trajectory` + `fk_arm_v3` + `drawing_batch_planner` + `go_to_start`); on
  **hardware** the user runs `pos_motor_sub`, which *subscribes* `/joint_states`
  to drive the DaMiao motors.
- The E-stop bar (`main_window.py::_toggle_estop`) toggles label **E-STOP** ⇄
  **RESET (E-stop active)** and calls `node.estop()` / `node.estop_reset()`.

## 2. Why it changed

The OLD E-stop (`estop()` → `freeze()`) was a **soft** stop: publish one hold
trajectory (current pose) to `/arm_controller/joint_trajectory` and block GUI
commands. It **failed when the arm wobbled near a singularity**: that wobble is
`ik_arm_v3` streaming oscillating `/joint_commands` → `ik_to_trajectory` flooding
`/arm_controller/joint_trajectory` at ~50 Hz; every new trajectory *preempts* the
GUI's hold, so the hold never sticks and the arm keeps wobbling ("refuses to take
any command until it stabilizes").

## 3. What's NEW — hard E-stop = kill the drivers, then hold

`estop()` (ros_bridge.py) now:

1. Builds the target node set =
   `DRIVER_NODES = ("ik_7dof_v3", "ik_to_trajectory", "drawing_batch_planner")`
   (command-side node names from `pendant_backend.launch.py`) **∪** the live
   publishers of `/joint_states` (`get_publishers_info_by_topic('/joint_states')`,
   excluding the GUI's own node — this covers hardware, where `pos_motor_sub`
   follows `/joint_states`).
2. `_find_node_procs(names)` scans `/proc/*/cmdline`, matches each target by its
   `__node:=NAME` remap or executable basename → `(pid, argv, cwd)`; saves argv+cwd
   to `self._killed_procs`.
3. **SIGKILL** each found process.
4. Publishes ONE hold trajectory (`_send_traj(self._last_q, 0.1)`). With the
   command flood now dead, the hold **sticks** and preempts any long drawing
   trajectory still loaded in the controller → arm freezes mid-stroke.

`estop_reset()` (the RESET button) re-runs each saved process with
`subprocess.Popen(argv, cwd=cwd, env=os.environ.copy(), start_new_session=True)`,
then clears `_estopped`. **No auto-home** — the arm stays exactly where it was
(the restarted IK re-seeds from the current `/joint_states` and won't move until
commanded). It homes only if the user presses **Home**.

## 4. Sim vs hardware (both handled)

- **Sim (Gazebo):** the 3 command drivers are separate processes → killed (they
  are the motion source → robot stops). `/joint_states`'s publisher is
  `joint_state_broadcaster`, which runs *inside* the Gazebo process (no separate
  process) → `_find_node_procs` finds nothing for it → **correctly skipped**
  (Gazebo is never killed). The hold preempts the loaded JTC trajectory.
- **Hardware:** the separate `/joint_states` publisher node is killed → the
  motors lose their command feed and hold. The hold-to-JTC step is a harmless
  no-op (there is no JointTrajectoryController on hardware).

## 5. Exact changes (`teach_pendant/pendant7dof/ros_bridge.py`)

- Added imports: `os, glob, signal, subprocess`.
- New module constant `DRIVER_NODES`.
- `__init__`: `self._killed_procs: list = []`.
- New method `_find_node_procs(self, node_names)`.
- Rewrote `estop(self)` — kill drivers + hold (was: `freeze()`).
- Rewrote `estop_reset(self)` — restart killed procs, clear estop, no home.
- `main_window.py::_toggle_estop` is **unchanged** (already toggles label + calls
  `estop()`/`estop_reset()`).

## 6. Verified vs NOT

- **Verified** (live headless backend): discovery + SIGKILL of the 3 command
  drivers (0 left afterwards); RESET re-ran all 3; `joint_state_broadcaster` was
  in the target set but skipped (in-Gazebo, no separate process).
- **NOT yet run end-to-end:** the hold-after-kill preempting a *long* stroke in a
  running sim (compiled, not executed); the full GUI button→behavior path in the
  live app (the bridge mechanism is validated piecewise).

## 7. How to verify next

1. Backend up (Simulation ON / gazebo) + `7dof-pendant`.
2. Send a long drawing, or jog into a wobble.
3. Press **E-STOP** → arm stops at once; `ros2 node list` shows `ik_7dof_v3`,
   `ik_to_trajectory`, `drawing_batch_planner` **gone**.
4. Press **RESET** → those nodes are back in `ros2 node list`; the arm **held its
   pose** (did not home).
5. On hardware: confirm the `/joint_states` publisher node is killed and the
   motors hold; RESET restarts it.

## 8. Caveats / follow-ups

- RESET re-spawns nodes as **independent** processes (not under the original
  `ros2 launch`). If you then do a full backend restart you can get duplicate
  nodes — restart everything cleanly to avoid.
- Node→PID matching needs `__node:=NAME` or the executable basename in the
  cmdline (true for launch-spawned nodes). A `/joint_states` publisher launched a
  different way might not match → wouldn't be killed.
- Restart relies on (a) the GUI's ROS environment (present — the pendant re-execs
  under `bash -lc` with ROS sourced) and (b) the original `--params-file
  /tmp/launch_params_*` in the saved argv still existing (it does while the
  original `ros2 launch` process lives; the E-stop does not kill the launch).
- All GUI-side and **not committed**. The PyPI wheel ships the pre-change GUI.
