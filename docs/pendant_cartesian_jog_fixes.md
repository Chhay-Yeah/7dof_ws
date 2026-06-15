# Teach-pendant Cartesian-jog fixes

Thesis reference — the **pendant-side** changes that made the Cartesian jog
behave (no wobble/chatter, clean stops at reach limits, no stream-overriding of
Home/E-stop or drawings).

Everything here lives in the pip package **`pendant7dof`** (the teach pendant):
`ros_bridge.py`, with `gui/joystick.py` and `gui/main_window.py` for the
joystick-release path. The IK *solver* tuning is intentionally **out of scope**
(it lives in the backend node `ik_arm_v3`, `src/arm_bot`) — see the scope note
at the end.

Code is on `origin/main`; the relevant commits are `3c71152` (clamp / classify
/ hold), `ef8099d` (`_cart_jog_stop`), and `c56e59c` (joystick release tick);
all shipped in PyPI `7dof-pendant` v0.1.4.

---

## 1. How the pendant drives the jog

In Cartesian mode the pendant does **not** command joints. It publishes a
**target end-effector pose** on `/ee_target` that *leads* the actual EE by a
small "carrot" (`CART_LEAD_M = 0.01 m`) so the live IK keeps chasing it.

That carrot is the root of the original misbehaviour: near the reachable edge or
a singular posture the 1 cm lead is **unreachable**, so the IK never converges,
stays active, and the EE wobbles/chatters. Worse, the IK's continuous
`/joint_commands` stream overrode the one-shot Home/E-stop trajectories and
(later) preempted drawings.

## 2. The pendant fixes

**(a) Model-based workspace clamp** — `_clamp_target()`, `reach_r_max()`.
Before publishing, the target is projected into a precomputed **reachable dome**:
an `r_max(z)` outer boundary, shrunk by a safety margin `_WS_MARGIN_M`, plus
`z ∈ [_WS_Z_MIN, _WS_Z_MAX]`. The joystick therefore can **never** push
`/ee_target` outside what the arm can reach, which removes the
perpetual-unreachable-carrot. The dome is generated offline from URDF forward
kinematics + the jog's soft joint limits (`workspace_creation/jog_envelope.py`)
and baked into the bridge as a constant `(r, z)` table. The clamp never pulls
the target *inside* the current actual pose (that pose is reachable by
definition; the sampled model can under-reach by a few mm).

**(b) Closed-loop stall-latch + re-anchor** — `_cart_stall` / `_cart_stalled`.
The clamp is only a model; this is the feedback backstop. Each tick the bridge
measures how far the EE *actually* moved versus the commanded step. If an
outstanding lead is not being closed (`gap > ½·lead` **and**
`moved < STALL_FRAC·step`, with `STALL_FRAC = 0.25`) for `STALL_TICKS = 4`
consecutive ticks, it **latches and re-anchors `/ee_target` to the current
actual pose**. The IK then reaches that pose in ~1 tick and goes idle → the
wobble stops and Home/E-stop work again. The latch releases when the EE starts
moving or the new jog direction is steered >60° off the stuck direction
(`UNSTALL_DOT = 0.5`).

**(c) Boundary-vs-interior classification + hold message** —
`_at_workspace_boundary()`. When it latches, the pendant reports *why*: at the
dome edge it is a true reach limit ("at workspace limit — holding"); inside the
dome it is a local-IK limit ("reach limit here — jog another way to continue").
A stall becomes a clean, labelled stop instead of a silent wobble.

**(d) Release re-anchor** — `_cart_jog_stop()` + the joystick fix.
Releasing the joystick now emits a final `(0, 0, 0)` tick
(`joystick.py::mouseReleaseEvent` → `main_window._on_joy`), and the bridge
re-anchors `/ee_target` to the current pose so the jog IK **reaches it and
deactivates** instead of chasing the last 1 cm lead forever. `_cart_jog_stop`
also runs before a drawing is dispatched and on `freeze()` / E-stop
(`force=True`), guaranteeing the jog IK is idle so it cannot stream
`/joint_commands` over another motion — this is what had caused the post-jog
drawing twitch.

**(e) Cold-start guard** — `_jog_moved_once`. The IK takes a few ticks to
respond at the start of a jog; the stall counter only arms *after* the EE has
begun moving, so that startup latency is not mistaken for a stall (it had caused
a false latch on the very first jog).

## 3. Explored but reverted

A **global-IK "escape"** (auto-reconfigure the arm to reach an interior-stalled
target) was built in the bridge and then **removed**. Gazebo testing showed the
operator's stalls are *physical* limits (unreachable near the base / the ground
plane), not local-IK dead ends, so the escape could not help and only added
complexity. The kept behaviour is clamp → classify → hold.

## 4. Scope boundary (what is NOT the pendant)

The actual **IK solving** for the jog — position-only mode, singularity-robust
damping (`w_thresh` / `lambda_sing`), null-space handling, and the joint-7
snap-back fix — lives in the **backend node `ik_arm_v3` (`src/arm_bot`)**, not in
the pendant package. The pendant's contribution is the orchestration / safety
layer above the solver: the workspace clamp, the stall-latch re-anchor, the
boundary/interior classification, and the deactivate-on-release logic.

## 5. Key symbols / constants (in `ros_bridge.py`)

| symbol | meaning |
|---|---|
| `CART_LEAD_M = 0.01` | how far the target leads the actual EE (m) |
| `STALL_FRAC = 0.25` | progress below this × step ⇒ a stalled tick |
| `STALL_TICKS = 4` | consecutive stalled ticks ⇒ latch |
| `UNSTALL_DOT = 0.5` | steer >60° off the stalled dir ⇒ release |
| `_WS_MARGIN_M` | shrink the dome this far inside the boundary |
| `reach_r_max(z)` | sampled reachable-dome boundary `r_max(z)` |
| `_clamp_target()` | project a target into the dome |
| `_cart_jog_stop()` | re-anchor target → jog IK deactivates |
| `_at_workspace_boundary()` | edge (true limit) vs interior (local-IK) |
