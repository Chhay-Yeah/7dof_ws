# Cartesian-Jog Wobble Fix — Report Handover Brief

This is a **prompt brief**: hand the fenced block below to an AI agent (or use it
yourself) to write a full technical report documenting how the 7-DOF arm's
Cartesian-jog wobble was diagnosed and fixed, from first symptom to final
resolution.

**In-repo authoritative sources** (always available, verifiable):

- Commits, in order: `1bddee7` → `1953441` → `15de098` → `3c71152` (use `git show <hash>`).
- Code: `src/arm_bot/arm_bot/ik_arm_v3.py` (the live IK node), `teach_pendant/pendant7dof/ros_bridge.py`
  (the GUI's data path / `cartesian_jog_xyz`), `src/arm_bot/launch/pendant_backend.launch.py`
  (jog params + soft limits), `workspace_creation/jog_envelope.py` (reachable-dome generator).

**Richest narrative source** (the developer's blow-by-blow dev log, kept in Claude
Code's project memory — may not exist on every machine): `project_teach_pendant_app.md`,
the "Cartesian-jog …" entry and its `UPDATE 1`–`UPDATE 19`. The fenced prompt below
already inlines a condensed version of that arc, so it is usable without the memory file.

---

```
You are a technical writer/engineer producing a detailed engineering report titled
"Fixing the Cartesian-Jog Wobble on a 7-DOF Robot Arm: Diagnosis to Resolution."

CONTEXT
The repository at ~/7dof_ws is a 7-DOF robotic arm workspace (ROS 2 Humble, Gazebo
Ignition/Fortress simulation, MoveIt2). A PyQt6 teach pendant (package `pendant7dof`,
in teach_pendant/) lets an operator jog the arm. Its CARTESIAN jog (move the
end-effector in XYZ via an on-screen joystick) suffered from wobble/jerking/
oscillation — joints chattering and the arm fighting itself — while joint-by-joint
jog and the drawing mode worked fine. Fixing it took a long, multi-phase debugging
effort. Your job is to document that effort end-to-end: the symptom, every root
cause found, every fix tried (including the ones reverted), and the final solution.

PRIMARY SOURCE OF TRUTH (read first, in full, if present — it is the chronological dev log):
  project_teach_pendant_app.md → the "Cartesian-jog ..." entry and UPDATE 1 .. UPDATE 19.
  UPDATE 1-12 = the original wobble diagnostic arc; UPDATE 13-19 = workspace-clamp /
  range / snap-back / global-IK-escape work. (Developer's Claude Code memory.)

CROSS-CHECK against (memory may have minor imprecision — verify against code/git):
  - git commits (use `git show <hash>`):
      1bddee7  "Add Targets + Motion flowchart mode; fix live IK jog behaviour"  (first jog-IK fix)
      1953441  "arm_bot: position-only Cartesian jog + thesis Ch.4 toolchain"     (the structural fix)
      15de098  "workspace_creation: DH Monte-Carlo reachable-workspace generator" (reach tooling)
      3c71152  "arm_bot + pendant: Cartesian-jog workspace clamp, full-range limits, joint_7 snap-back fix" (final state)
  - code: src/arm_bot/arm_bot/ik_arm_v3.py (the IK node), teach_pendant/pendant7dof/ros_bridge.py
    (the GUI's data path), src/arm_bot/launch/pendant_backend.launch.py (params),
    workspace_creation/jog_envelope.py (reachable-dome generator).

THE STORY ARC TO COVER (use the memory UPDATEs for specifics/numbers):
  1. Symptom & why it's hard: 7-DOF redundancy, kinematic singularities, a live
     closed-loop resolved-rate IK; the wobble had MULTIPLE stacked root causes.
  2. Perpetually-active IK (1bddee7): a null-space pull left a permanent following
     error so the IK never converged/deactivated -> wobbled + overrode Home/E-stop.
     Fix: null_k=0 + parameterized tolerances so it converges and deactivates.
  3. The diagnostic arc (UPDATE 1-12): singularity damping (w_thresh); null-space
     velocity damping (null_damp); the all-zeros spawn pose is an EXACT singularity
     (go_to_start moves off it); elbow up/down branch FLIP through the straight-arm
     singularity (elbow-up soft limits); joint_7 wrist wobble; the front-end
     "carrot-vs-wall" feedback stall-latch; an EE-marker RViz debug viz.
  4. THE DECISIVE STRUCTURAL FIX (UPDATE 10-11, commit 1953441): holding EE
     ORIENTATION while translating is kinematically infeasible (joint_6's narrow
     limit) -> the wrist limit-cycled. Switching to POSITION-ONLY jog removed the
     conflicting objective and killed the wobble. Emphasize: the cure was
     structural, not parameter tuning.
  5. Reachable-workspace tooling (15de098) -> model-based WORKSPACE CLAMP in the GUI
     data path (UPDATE 13): clamp the jog target into the reachable dome before
     sending, so the joystick can't push past reach.
  6. Gazebo-hardening (UPDATE 14-18, commit 3c71152): joint_7 auto-escape RAN AWAY
     (removed); STARTUP FALSE-LATCH from IK cold-start (_jog_moved_once gate);
     a red-herring where the user was running an OLD bundled wheel not the live code
     (UPDATE 15); the q_ref SNAP-BACK fix (UPDATE 17); loosening soft limits to full
     URDF range for ~2x workspace (UPDATE 18).
  7. HONEST NEGATIVE RESULT (UPDATE 19): a global-IK "auto-branch-switching" escape
     was built, tested, and REVERTED — the arm's actual stalls are PHYSICAL limits
     (kinematic unreachability near the base; the ground/mounting plane at full
     extension), not local-IK dead ends an IK re-pose can beat.
  8. Final state & architecture: position-only jog + workspace clamp + clean hold at
     limits (no wobble/chatter/runaway) + snap-back fix + full-range limits. Note the
     architecture split: most logic is in the GUI data path (ros_bridge, solver-
     agnostic); one general IK-node fix (q_ref re-seed); the range is config values.

REPORT REQUIREMENTS
  - Audience: a robotics engineer / thesis reader. Technical but readable.
  - Structure: Abstract; Problem & Symptoms; Background (7-DOF redundancy, the
     control pipeline GUI->/ee_target->IK->controller->Gazebo); Diagnostic Journey
     (chronological, one subsection per root cause, each with hypothesis -> test ->
     result/revert); The Structural Fix (position-only jog); Workspace-Model Clamp;
     Hardening & the Snap-Back fix; Range Expansion; The Abandoned Global-IK Escape
     (why physical limits can't be solved); Final Architecture & Results; Lessons
     Learned.
  - EMPHASIZE: (a) multiple stacked root causes peeled apart iteratively; (b) the
     decisive fix was structural, not tuning; (c) many attempts were tried and
     reverted — and that the offline test rig did NOT reproduce Gazebo's velocity-
     controller dynamics (a real methodology lesson); (d) negative results
     (auto-escape) were correctly abandoned after empirical testing.
  - Include a concise timeline/table of root-cause -> fix -> outcome.
  - Be honest about dead-ends and reverts; the iterative, partly-failed path IS the story.
  - Output: a single Markdown document. Don't invent numbers — pull them from the
     memory log / code / `git show`, or state "not recorded".
```
