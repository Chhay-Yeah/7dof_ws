# 7DOF Pendant → Real Hardware (DaMiao CAN motors)

Lets the teach pendant drive the physical arm through the friend's `arm_bot_hw`
DaMiao-CAN driver. Same pendant + IK/FK/drawing stack as simulation; Gazebo is
replaced by a hardware bridge that owns the U2CANFD adapter.

```
pendant jog/draw → /arm_controller/joint_trajectory → arm_hw_bridge ─┬→ DM motors (POS_VEL, CAN)
                                          /joint_states ← (live encoders) ┘
                                                  └→ IK / FK / pendant (track the real arm)
```

`arm_hw_bridge` (in `arm_bot_hw`, alongside the untouched `pos_motor_sub.py`):
- publishes `/joint_states` from the **live motor encoders**;
- drives the motors from `/arm_controller/joint_trajectory` (where every pendant
  command lands: joint jog, set, move, drawing, and Cartesian-jog IK);
- **SAFE START**: reads + holds the current pose on boot, only moves on command,
  caps velocity (`hw_max_vel`), disables motors on shutdown and `/hw_estop`.

There is **no `go_to_start`** here — the arm must not auto-move on launch.

## One-time setup
```bash
# 1. USB lib the native driver needs
python3 -m pip install --user pyusb           # already installed on this machine
sudo apt install -y libusb-1.0-0              # runtime backend (usually present)

# 2. Build (arm_bot_hw is symlinked into src/ as src/arm_bot_hw)
cd ~/7dof_ws && colcon build && source install/setup.bash

# 3. USB permissions for the adapter (if `dev_sn` shows nothing or "Access denied").
#    Either run with sudo -E, or add a udev rule for VID 0x34b7:
echo 'SUBSYSTEM=="usb", ATTR{idVendor}=="34b7", MODE="0666"' | \
  sudo tee /etc/udev/rules.d/99-u2canfd.rules
sudo udevadm control --reload-rules && sudo udevadm trigger
```

## Find your adapter serial
The serial is **hardcoded** in the original `pos_motor_sub.py`; the launch makes
it a parameter (`hw_sn`). Confirm yours:
```bash
ros2 run arm_bot_hw dev_sn        # prints SN for each U2CANFD device
```

## Bring-up (do this carefully the first time)
1. **Have a physical E-stop / power cutoff within reach.**
2. Place the arm in a safe, roughly mid-range posture (the bridge will read and
   hold *wherever it is*).
3. Launch the hardware backend with a **low** speed cap:
   ```bash
   source /opt/ros/humble/setup.bash && source ~/7dof_ws/install/setup.bash
   ros2 launch arm_bot pendant_hardware.launch.py hw_sn:=<your-serial> hw_max_vel:=1.0
   ```
4. **Watch the `SAFE START — holding current pose: …` line.** It must match the
   real arm's joint angles. If it reads ~0 or garbage, the encoder read failed —
   E-stop / cut power and check wiring, CAN ids (0x01..0x07), and the serial. Do
   not proceed.
5. Open the pendant with **Simulation OFF**: `7dof-pendant`. Jog gently in Joint
   mode first, then Cartesian. The arm should track the pendant.
6. Once confident, raise speed: relaunch with a higher `hw_max_vel` (e.g. 3.0).

## Parameters (`ros2 launch arm_bot pendant_hardware.launch.py <name>:=<val>`)
| arg | default | meaning |
|---|---|---|
| `hw_sn` | E067CA… | U2CANFD adapter serial (set yours) |
| `hw_max_vel` | 1.5 | velocity cap [rad/s] sent to every motor |
| `hw_rate` | 50.0 | `/joint_states` + command rate [Hz] |
| `hw_feedback` | encoder | `encoder` = live feedback (closed-loop); `command` = echo last cmd (open-loop fallback if reads are flaky) |
| `rviz` | false | open RViz |
| `jog_*` | (sim defaults) | Cartesian-jog IK tuning, same as the sim backend |

## Safety notes
- **Snap risk**: any position published on `/joint_states`/commanded is driven to
  *immediately* at up to `hw_max_vel`. The safe-start (read + hold) avoids the
  power-on jump; keep `hw_max_vel` low until you trust it.
- **Software E-stop**: the pendant's E-stop freezes (holds the current pose).
  For a hard motor cutoff, publish `ros2 topic pub --once /hw_estop std_msgs/Bool "{data: true}"`
  (disables the motors; `{data: false}` re-enables + re-reads the pose). **This
  is not a substitute for a physical E-stop.**
- Motors `joint_3` and `joint_5` are treated as direction-inverted (matching
  `pos_motor_sub.py`'s `inverted_motors`). If a joint moves the wrong way, adjust
  `INVERTED` in `arm_bot_hw/hw_bridge.py`.

## Status / what's verified
- ✅ Builds; `hw_bridge` runs and opens the adapter (fails cleanly "device not
  found" with no adapter attached); launch validated.
- ⚠️ **Not yet tested on the physical arm** (no motors/adapter on the dev
  machine). The motor motion, encoder-feedback path, and safe-start read need
  on-hardware validation — hence the careful first-power-on procedure above.
- If the encoder read proves unreliable, run with `hw_feedback:=command` (open-
  loop: `/joint_states` echoes the command) and rely on the procedural safe-start
  (place the arm at the pose you launch from).
