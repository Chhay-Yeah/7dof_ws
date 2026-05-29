#!/usr/bin/env bash
#
# install.sh — one-shot setup for the 7dof-pendant on a fresh machine. Installs
# the ROS dependencies the pendant needs to build and run, then the pendant.
#
# Supports two of the pendant's three backends:
#   * REAL-HARDWARE control (the deployment target), and
#   * the RViz "moveit" simulation (MoveIt + fake ros2_control hardware + MoveIt
#     RViz) for testing the app's functions on a dev box.
#
#   * Does NOT install Gazebo/Ignition — the `gazebo` backend (physics sim) is
#     intentionally excluded. RViz + MoveIt ARE installed so the `moveit` mode
#     works. (rosdep skips only the Gazebo/Ignition keys below.)
#   * Installs MoveIt's planning interface + ros2_control because arm_bot's
#     CMakeLists does `find_package(moveit_ros_planning_interface REQUIRED)` and
#     the pendant colcon-builds the whole workspace on first launch.
#
# Prerequisite: ROS 2 (default: Humble) already installed under /opt/ros.
#
# IMPORTANT — real-hardware gap: this repo currently ships only a *simulation*
# ros2_control hardware interface (ign_ros2_control). To actually drive the
# physical arm you must add a real hardware_interface::SystemInterface driver and
# a non-sim bringup, then run the pendant with `7dof-pendant launch --no-backend`
# to attach to it. This script gets the software installed; it does not create
# that driver.
#
# Usage:
#   ./install.sh                # full install + build
#   ./install.sh --no-build     # install deps + pendant, skip colcon build
#   ROS_DISTRO=iron ./install.sh
#
set -euo pipefail

SCRIPT_DIR="$(cd -- "$(dirname -- "${BASH_SOURCE[0]}")" >/dev/null 2>&1 && pwd)"
WS_ROOT="$(dirname -- "$SCRIPT_DIR")"     # repo root = parent of teach_pendant/
ROS_DISTRO="${ROS_DISTRO:-humble}"
ROS_SETUP="/opt/ros/${ROS_DISTRO}/setup.bash"
DO_BUILD=1
[[ "${1:-}" == "--no-build" ]] && DO_BUILD=0

log() { printf '\n\033[1;34m[install]\033[0m %s\n' "$*"; }
die() { printf '\n\033[1;31m[install] error:\033[0m %s\n' "$*" >&2; exit 1; }

# ── 0. sanity checks ──────────────────────────────────────────────────────────
[[ -d "$WS_ROOT/src/arm_bot" ]] || die "run this from the repo: $WS_ROOT/src/arm_bot not found"
[[ -f "$ROS_SETUP" ]] || die "ROS 2 '$ROS_DISTRO' not found at $ROS_SETUP.
Install it first (e.g. 'sudo apt install ros-${ROS_DISTRO}-ros-base'), or set
ROS_DISTRO to your installed distro, then re-run."
command -v sudo >/dev/null || die "sudo is required for the apt steps"

log "workspace : $WS_ROOT"
log "ROS distro: $ROS_DISTRO ($ROS_SETUP)"

# ── 1. build tooling (colcon, rosdep, compilers) ──────────────────────────────
log "installing build tooling (colcon, rosdep, compilers)…"
sudo apt-get update
sudo apt-get install -y \
  python3-colcon-common-extensions python3-rosdep python3-pip \
  build-essential cmake git

# ── 2. workspace ROS deps via rosdep, minus Gazebo/Ignition only ──────────────
# rosdep reads every package.xml and installs the matching apt packages. We skip
# the Gazebo/Ignition keys (physics-sim backend, not used here). RViz + MoveIt
# are kept so the `moveit` (RViz fake-hardware) simulation works. The two extra
# skips are dev/heavy-only: moveit_setup_assistant (config-gen tool, not needed
# at runtime) and warehouse_ros_mongo (MongoDB trajectory store, optional).
SKIP_KEYS="ros_gz_sim ros_gz_bridge ign_ros2_control gazebo_ros gazebo_ros2_control \
moveit_setup_assistant warehouse_ros_mongo"

log "resolving workspace dependencies with rosdep (skipping Gazebo/Ignition)…"
sudo rosdep init 2>/dev/null || true     # harmless if already initialised
rosdep update
rosdep install --from-paths "$WS_ROOT/src" --ignore-src -y \
  --rosdistro "$ROS_DISTRO" \
  --skip-keys "$SKIP_KEYS"

# ── 3. Python deps the nodes import but package.xml doesn't declare ───────────
# ik_arm_v3/fk_arm_v3 use numpy + urdf_parser_py; drawing_batch_planner uses scipy.
log "installing undeclared Python deps (numpy, scipy, urdf_parser_py)…"
sudo apt-get install -y python3-numpy python3-scipy "ros-${ROS_DISTRO}-urdfdom-py"

# ── 4. the pendant itself (PyQt6 comes from PyPI automatically) ───────────────
# Editable install so the pendant resolves THIS live workspace at runtime.
log "installing 7dof-pendant (editable) + pinned build tooling…"
python3 -m pip install --user -U "setuptools>=61,<80" wheel
python3 -m pip install --user -e "$SCRIPT_DIR"

# ── 5. build the workspace once (so errors surface now, not on first launch) ──
if [[ "$DO_BUILD" == 1 ]]; then
  log "colcon-building the workspace (one time)…"
  # shellcheck disable=SC1090
  source "$ROS_SETUP"
  ( cd "$WS_ROOT" && colcon build )
fi

# ── 6. report ─────────────────────────────────────────────────────────────────
log "running diagnostics…"
PENDANT="$HOME/.local/bin/7dof-pendant"
[[ -x "$PENDANT" ]] || PENDANT="7dof-pendant"
"$PENDANT" doctor || true

cat <<EOF

[install] Done. Make sure ~/.local/bin is on your PATH, then launch with:

    7dof-pendant                  # open the pendant
      └ Settings → mode 'moveit', then Simulation ON  → RViz fake-hardware sim
      └ for REAL hardware: bring up your hardware stack, then
        7dof-pendant launch --no-backend                # attach to it

Notes:
  * The 'gazebo' physics-sim mode is NOT supported by this install (Gazebo was
    skipped on purpose). Use 'moveit' mode for RViz-based testing.
  * Driving the PHYSICAL arm needs a real ros2_control hardware driver + bringup
    that this repo does not yet provide.
EOF
