#!/usr/bin/env python3
"""
Layered system-architecture diagram of the 7-DOF teach-pendant stack running on
the Gazebo backend — from the operator's GUI down to the physics plant, with the
ROS 2 command path (down) and state-feedback path (up) annotated by topic.

    python3 make_architecture.py            # -> system_architecture.{png,pdf}
    python3 make_architecture.py --out figures/system_architecture

Pure matplotlib (no Graphviz needed).
"""
import argparse
import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
from matplotlib.patches import FancyBboxPatch, FancyArrowPatch

# Layer palette (top application -> bottom plant).
LAYERS = [
    dict(title="Operator  ·  Teach-Pendant GUI",
         sub="7dof-pendant (PyQt6) — node /pendant7dof_bridge",
         items=["Jogging\n(joint · Cartesian)", "Drawing\n(canvas)",
                "Motion\n(target sequences)", "Targets / Status", "E-STOP"],
         y=0.845, h=0.125, fc="#e8eefc", ec="#3a6ea5"),
    dict(title="Motion generation  ·  ROS 2 nodes",
         sub="Cartesian jog, drawing planner, command bridge",
         items=["ik_arm_v3\n(jog IK)", "fk_arm_v3\n(EE pose)",
                "ik_to_trajectory\n(cmd→traj)", "drawing_batch_planner\n(strokes→traj)",
                "go_to_start\n(boot pose)"],
         y=0.625, h=0.155, fc="#eaf3ea", ec="#2e7d32"),
    dict(title="ros2_control  ·  controller_manager",
         sub="100 Hz real-time control loop",
         items=["arm_controller\n(JointTrajectoryController)",
                "joint_state_broadcaster"],
         y=0.445, h=0.120, fc="#fdf3e3", ec="#b5650f"),
    dict(title="Hardware abstraction layer",
         sub="ros2_control SystemInterface",
         items=["ign_ros2_control / IgnitionSystem\n"
                "position command  ·  position+velocity state"],
         y=0.295, h=0.095, fc="#f4ecf7", ec="#7b3f9e"),
    dict(title="Simulation plant",
         sub="Ignition Gazebo physics @ 1 kHz",
         items=["Gazebo (Ignition) — robot model from URDF/SDF\n"
                "7 revolute joints  ·  /clock (sim time)"],
         y=0.135, h=0.105, fc="#eceff1", ec="#455a64"),
]

X0, X1 = 0.06, 0.94          # band horizontal extent


def _band(ax, ly):
    y, h = ly["y"], ly["h"]
    ax.add_patch(FancyBboxPatch((X0, y), X1 - X0, h,
                 boxstyle="round,pad=0.004,rounding_size=0.012",
                 fc=ly["fc"], ec=ly["ec"], lw=1.8, zorder=2))
    ax.text(X0 + 0.015, y + h - 0.022, ly["title"], fontsize=12.5,
            fontweight="bold", color=ly["ec"], va="center", zorder=4)
    ax.text(X0 + 0.015, y + h - 0.046, ly["sub"], fontsize=8.5,
            color="#444", va="center", style="italic", zorder=4)
    # component boxes along the bottom of the band
    n = len(ly["items"])
    gap = 0.012
    avail = (X1 - X0) - 0.03 - gap * (n - 1)
    bw = avail / n
    bx = X0 + 0.015
    by = y + 0.012
    bh = h - 0.072
    for it in ly["items"]:
        ax.add_patch(FancyBboxPatch((bx, by), bw, bh,
                     boxstyle="round,pad=0.003,rounding_size=0.008",
                     fc="white", ec=ly["ec"], lw=1.2, alpha=0.95, zorder=3))
        ax.text(bx + bw / 2, by + bh / 2, it, ha="center", va="center",
                fontsize=8.0, zorder=4)
        bx += bw + gap


def _arrow(ax, x, y0, y1, color, label, lx, down=True, label_side="right"):
    a = FancyArrowPatch((x, y0), (x, y1), arrowstyle="-|>", mutation_scale=20,
                        lw=2.4, color=color, zorder=5,
                        shrinkA=0, shrinkB=0)
    ax.add_patch(a)
    ha = "left" if label_side == "right" else "right"
    dx = 0.011 if label_side == "right" else -0.011
    ax.text(x + dx, (y0 + y1) / 2, label, fontsize=7.6, color=color,
            ha=ha, va="center", zorder=6,
            bbox=dict(boxstyle="round,pad=0.18", fc="white", ec=color, lw=0.8))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--out", default="system_architecture")
    args = ap.parse_args()

    fig, ax = plt.subplots(figsize=(11, 11.6))
    ax.set_xlim(0, 1); ax.set_ylim(0, 1); ax.axis("off")
    ax.set_title("7-DOF teach-pendant — layered system architecture (Gazebo backend)",
                 fontsize=15, fontweight="bold", pad=14)

    for ly in LAYERS:
        _band(ax, ly)

    # gaps between consecutive bands -> command (down, left) + feedback (up, right)
    CMD, FB = "#c0392b", "#1f7a3a"
    cmd_labels = [
        "/ee_target · /drawing/strokes\n/arm_controller/joint_trajectory",
        "/arm_controller/joint_trajectory",
        "position commands",
        "joint torques / motion",
    ]
    fb_labels = [
        "/joint_states · /ee_pose",
        "/joint_states",
        "joint position + velocity",
        "measured joint state",
    ]
    for i in range(len(LAYERS) - 1):
        top = LAYERS[i]; bot = LAYERS[i + 1]
        y_top = top["y"]                 # bottom edge of upper band
        y_bot = bot["y"] + bot["h"]      # top edge of lower band
        _arrow(ax, 0.30, y_top, y_bot, CMD, cmd_labels[i], 0.30,
               down=True, label_side="left")
        _arrow(ax, 0.70, y_bot, y_top, FB, fb_labels[i], 0.70,
               down=False, label_side="right")

    # legend for the two flows
    ax.annotate("command path", xy=(0.30, 0.045), fontsize=10, color=CMD,
                fontweight="bold", ha="center")
    ax.annotate("state feedback", xy=(0.70, 0.045), fontsize=10, color=FB,
                fontweight="bold", ha="center")
    ax.text(0.5, 0.018,
            "robot_state_publisher latches /robot_description (URDF) → consumed by ik_arm_v3 / fk_arm_v3",
            ha="center", fontsize=7.8, color="#555", style="italic")

    fig.tight_layout(rect=(0, 0, 1, 0.98))
    for fmt in ("png", "pdf"):
        fig.savefig(f"{args.out}.{fmt}", dpi=160, bbox_inches="tight")
    plt.close(fig)
    print(f"wrote {args.out}.png/.pdf")


if __name__ == "__main__":
    main()
