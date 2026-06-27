# results_tools — drawing-run recorder + three thesis/video generators

A decoupled recorder and three generators that turn one recording of a drawing
run into:

1. **`gen_path_overlay.py`** — commanded vs executed pen path (thesis **Fig 4.1**).
   *executed* = FK of the recorded **encoder** `/joint_states`; *commanded* =
   the dispatched `/cartesian_path`. Reports RMS / max tracking error (mm).
2. **`gen_velocity.py`** — a **PyQt6** window of all seven joint velocities vs
   time, **live during a draw** and **replayed from a log**, with PNG+PDF export
   (thesis **Fig 4.5**).
3. **`gen_frontview_anim.py`** — a 2D front-view kinematic animation for video
   overlay: transparent PNG frame sequence + alpha video (ProRes 4444 / VP9) +
   the exact ffmpeg command to composite it over your recorded clip.

Every generator reads **either** a recorded rosbag2 dir **or** an encoder `.csv`
(`record_encoder.py`) — auto-detected by the `.csv` extension.

Plus **`record_draw.py`** (the observer/recorder), **`capture.py`** /
**`gen_from_csv.py`** (one-command orchestrators), **`config.yaml`** (every
knob), and the shared library: **`fk_chain.py`** (the *one* FK) and
**`data_io.py`** (bag + CSV reading, URDF resolution, time-alignment, velocity).

### Real-hardware flow (encoder CSV) — what you actually run

On the real arm, `pos_motor_sub` owns the single USB-CAN adapter, so the encoder
can only be read from inside that process. Use **`arm_bot_hw record_encoder`**
(it subclasses `pos_motor_sub`, drives the draw identically, and logs the
measured encoder of all 7 joints to a CSV):

```bash
source ~/7dof_ws/install/setup.bash
ros2 run arm_bot_hw record_encoder            # run INSTEAD of pos_motor_sub, then draw
#   ... Ctrl-C when the robot finishes  ->  recordings/draw_capture.csv  ( (2), (3) … )

python3 results_tools/gen_from_csv.py recordings/draw_capture.csv   # -> figures/draw_capture/
```

`gen_from_csv.py` renders all three artifacts into one run folder. Because the
CSV carries the **measured encoder velocity**, the velocity figure uses the real
motor velocity (no differentiation needed); the path overlay compares FK of the
**commanded** joints vs FK of the **measured** encoder; the animation is driven
by the measured encoder (`--drive commanded` to use the command instead). CSV
mode has no URDF, so FK uses the cached `results_tools/arm_bot.urdf` (regenerate
with `xacro src/arm_bot/urdf/arm_bot.urdf.xacro > results_tools/arm_bot.urdf`).

Everything uses **one FK** (`fk_chain.Chain`, the backend's URDF-driven chain),
**one joint order** (`joint_1..joint_7`, indexed by name), and **SI units**
(rad, m, s) — mm only in the paper-plane overlay. All streams are time-aligned by
their message stamps and **resampled onto a shared timeline**, so the three
artifacts and your video agree on `t`.

---

## Phase 0 — findings (read this first)

Established by inspecting the live ROS graph (hardware powered), the recorded
`draw_run/` bag, and the existing analysis toolchain:

- **No motor-reported velocity exists on ROS.** `pos_motor_sub.py` *subscribes*
  to `/joint_states` and drives the DaMiao motors over CAN — it never
  republishes motor feedback. In the recorded bag, `/joint_states.velocity` is
  numerically zero (~1e-19). The motors report velocity over CAN, but nothing
  surfaces it to ROS.
  → **The velocity figure differentiates the encoder position and lightly
  filters it** (Savitzky–Golay by default). The path taken is printed in the
  plot subtitle, e.g. *“numeric diff, Savitzky-Golay (win=21, poly=3)”*.
- **Live source = `/joint_states`** (encoder feedback), chosen in Phase 0. The
  recorder also captures `/cartesian_path`, `/robot_description`, `/motor_target`
  (MATLAB command), and `/joint_commands` when present.
- **The recorded "log" is a rosbag2 directory.** Replaying any generator =
  pointing it at a bag. The existing `draw_run/` bag works as-is.
- **FK** is the URDF-driven chain validated against `robot_state_publisher`/RViz
  (path overlay reproduces RMS **0.114 mm**, max **0.605 mm** on `draw_run/`).
- **The current live graph** (when this was built) was `/matlab_node` →
  `/motor_target` only; run a pendant-backend draw so `/joint_states` is
  published for live velocity + feedback-driven animation.

---

## Dependencies

Already present in the workspace Python 3.10 (system): `PyQt6`, `matplotlib`,
`numpy`, `scipy`, `urdf_parser_py`, `pyyaml`, `opencv` (`cv2`), plus a sourced
ROS 2 Humble (`rosbag2_py`, `rclpy`, message types).

**New, install once** (only needed for the Deliverable-3 alpha *video* — the
PNG frame sequence and the ffmpeg command text are produced without it):

```bash
sudo apt-get install -y ffmpeg
```

Use the workspace environment (`source ~/7dof_ws/install/setup.bash`); no
virtualenv is required.

---

## One command: record a draw → auto-generate everything

`capture.py` is the headline workflow. Run it **before** you hit Send; it records
(observe-only, so it can't perturb the draw). When the robot finishes, come back
and press **Ctrl-C** — it finalises the bag and renders all three artifacts into
one run folder.

```bash
source ~/7dof_ws/install/setup.bash
python3 results_tools/capture.py            # run FIRST, then hit Send
#   ... robot draws ...                     # then Ctrl-C when it finishes
```

Result — nothing is ever overwritten; a second run becomes `draw_capture (2)`, `(3)`, …:

```
figures/draw_capture/
    bag/                     the raw recording (rosbag2)
    figure_4_1.{png,pdf}     commanded vs executed path overlay
    figure_4_6.{png,pdf}     seven joint velocities
    frontview/               transparent frames + alpha .mov + ffmpeg_commands.txt
```

Options: `--name square` (run name), `--no-anim` (skip the slow animation),
`--drive auto|feedback|commanded`, `--video prores|webm|none`, `--fps 30`,
`--anim-max-frames N`. The front-view `--drive` auto-detects: `feedback` if
`/joint_states` was recorded, else `commanded`.

### Lower-level recorder

`record_draw.py` just records a bag (no auto-generate), if you want to run the
generators by hand later:

```bash
python3 results_tools/record_draw.py --name square_run    # -> draw_runs/square_run/
```

Topics/storage come from `config.yaml` (override with `--topics`, `--storage mcap`).

---

## Generate the three artifacts

```bash
source ~/7dof_ws/install/setup.bash

# 1) Commanded vs executed path overlay  -> figures/figure_4_1.{png,pdf}
python3 results_tools/gen_path_overlay.py draw_runs/square_run

# 2) Joint velocities (PyQt6 window). Replay a log:
python3 results_tools/gen_velocity.py --replay draw_runs/square_run
#    ...or live during a draw (rolling window):
python3 results_tools/gen_velocity.py --live
#    ...or headless, just write the figure (thesis Fig 4.5):
python3 results_tools/gen_velocity.py --replay draw_runs/square_run --export
#    -> figures/figure_4_5.{png,pdf}   (the in-window button does the same)

# 3) Front-view animation overlay  -> figures/frontview/
python3 results_tools/gen_frontview_anim.py draw_runs/square_run            # encoder feedback
python3 results_tools/gen_frontview_anim.py draw_runs/square_run --drive commanded
```

`--drive feedback` (default) proves the digital twin matches the encoders;
`--drive commanded` (from `/motor_target` or `/joint_commands`) proves the real
arm tracks the target. Useful flags: `--fps 30`, `--video prores|webm|none`,
`--start/--end <t_rel s>`, `--max-frames N`.

---

## Composite the overlay over your video

`gen_frontview_anim.py` writes `figures/frontview/frontview_<drive>.mov`
(ProRes 4444, alpha) and `figures/frontview/ffmpeg_commands.txt`. To lay it over
your recorded clip:

```bash
ffmpeg -i YOUR_CLIP.mp4 -i figures/frontview/frontview_feedback.mov \
  -filter_complex "[0:v][1:v]overlay=0:0:format=auto" -c:a copy -r 30 composited.mp4
```

Encode commands emitted (run automatically when ffmpeg is installed):

```bash
# ProRes 4444 (.mov, keeps alpha)
ffmpeg -y -framerate 30 -i figures/frontview/frames/frame_%05d.png \
  -c:v prores_ks -profile:v 4444 -pix_fmt yuva444p10le figures/frontview/frontview_feedback.mov
# VP9 (.webm, alpha)
ffmpeg -y -framerate 30 -i figures/frontview/frames/frame_%05d.png \
  -c:v libvpx-vp9 -pix_fmt yuva420p -b:v 0 -crf 24 figures/frontview/frontview_feedback.webm
```

**Lining the overlay up with the real arm** is all in `config.yaml`
→ `frontview.projection` / `frontview.render` (see below). Tweak, re-run, repeat
until the skeleton sits on the arm in your footage.

---

## config.yaml — key options

| Section | Option | Meaning |
|---|---|---|
| `fk` | `joint_names`, `base_link`, `tip_link` | the one chain + joint order |
| `timeline` | `resample_fps`, `zero_at_cartesian_path` | shared grid; t=0 at dispatch |
| `record` | `topics`, `feedback_topic`, `storage` | what the recorder captures |
| `velocity` | `filter` (`savgol`/`ema`/`none`), `savgol_window`/`polyorder`, `live_window_s` | velocity smoothing + live view |
| `path_overlay` | `begin_draw_joints`, `pen_offset_mm`, `pen_axis_local`, `paper_rotation_deg`, `paper_mirror_x`, timing (`move_to_begin`/`dwell`/`settle`/`approach`) | paper-frame mapping (defaults = `pendant_backend.launch.py`) |
| `frontview.projection` | `depth_axis` (axis into screen), `screen_x`, `screen_y`, `flip_x/y`, `rotation_deg` | match your camera angle |
| `frontview.render` | `width`/`height`, `origin_px`, `scale_px_per_m`, `line_thickness`, `joint_marker_size`, colors | placement + style in the output frame |
| `frontview` | `drive`, `fps`, `trace_pen_tip`, `export.video` | drive source, frame rate, pen trail, video codec |
| `io` | `figures_dir` | output root (relative paths anchor to the workspace root) |

---

## Notes

- The recorder is the only piece that touches the running system, and it only
  *subscribes* — the draw is never modified.
- Outputs (figures, frames, video) go to the workspace `figures/`; the tooling
  lives here in `results_tools/`.
- `gen_velocity.py --export` is the headless thesis-figure path; the live/replay
  window has an **Export PNG+PDF** button that writes the identical figure.
