#!/usr/bin/env python3
"""
ik_arm_v3.py  —  7-DOF IK node, Damped Least Squares with null-space centering.

FK / Jacobian are now built directly from the URDF on /robot_description, so
the kinematic model matches what robot_state_publisher/RViz visualise exactly
(no DH idealisation, no per-joint axis-sign assumptions).

ROS interface:
  SUB  /joint_states      sensor_msgs/JointState     — current joint angles
  SUB  /ee_target         geometry_msgs/PoseStamped  — desired EE pose
  SUB  /robot_description std_msgs/String (latched)  — URDF
  PUB  /joint_commands    sensor_msgs/JointState     — IK solution
"""
import numpy as np
import rclpy
from rclpy.node import Node
from rclpy.qos import QoSProfile, QoSDurabilityPolicy, QoSReliabilityPolicy
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseStamped
from std_msgs.msg import String

from arm_bot.ik_lib import UrdfChain, quat_to_rot, rot_error


class IKNode(Node):
    RATE_HZ      = 50.0
    LAMBDA_MAX   = 0.05      # damping when far from goal (numerical safety)
    LAMBDA_MIN   = 5e-4      # damping when close (lets residual shrink to ~µm)
    LAMBDA_KNEE  = 0.05      # error norm at which damping starts to shrink
    POS_GAIN     = 1.0
    ROT_GAIN     = 1.0
    DQ_MAX       = 0.10       # per-step joint-velocity cap [rad]
    INNER_ITERS  = 4          # DLS sub-iterations per timer tick
    TOL_POS      = 1e-5       # 10 µm
    TOL_ROT      = 1e-4       # ~0.006°
    NULL_K       = 0.3
    NULL_FADE    = 0.02       # err-norm below which the null-space pull fades out
    NULL_DAMP_ALPHA = 0.1     # low-pass rate of the posture the damping tracks

    def __init__(self):
        super().__init__("ik_7dof_v3")

        self.declare_parameter("base_link", "base_link")
        self.declare_parameter("tip_link",  "ee")
        self._base = self.get_parameter("base_link").value
        self._tip  = self.get_parameter("tip_link").value

        # Null-space (redundancy) resolution is tunable so a drawing-specific
        # instance can pull the elbow toward the draw posture instead of the
        # joint-limit midpoint. Pulling toward q_mid is wrong for drawing:
        # the draw posture sits far from mid, so NULL_K*(q_mid - q) is large,
        # dominates the per-tick dq budget after the DQ_MAX clamp, and starves
        # task tracking (~15 mm following error). Pull toward the draw posture
        # (null_target = begin_draw_joints) to zero that term out.
        # Defaults reproduce the original jog behaviour (q_mid, NULL_K=0.3).
        self.declare_parameter("null_k", self.NULL_K)
        self.declare_parameter("inner_iters", self.INNER_ITERS)
        # Empty list (the ROS-friendly sentinel) => fall back to q_mid.
        self.declare_parameter("null_target", [0.0])
        # Per-joint weighted-DLS weights. High weight = that joint moves less.
        # For drawing, penalise the tight-limit wrist joints (joint_6, joint_7)
        # so the tracker doesn't drift them into their limits and pin (which
        # leaves a position residual). Sentinel [0.0] => uniform (jog default).
        self.declare_parameter("joint_weights", [0.0])
        self._null_k = float(self.get_parameter("null_k").value)
        self._inner_iters = int(self.get_parameter("inner_iters").value)
        nt = list(self.get_parameter("null_target").value)
        self._null_target = (np.array(nt, dtype=float)
                             if len(nt) > 1 else None)
        jw = list(self.get_parameter("joint_weights").value)
        self._joint_weights = (np.array(jw, dtype=float)
                               if len(jw) > 1 else None)
        # Convergence tolerances. The defaults are very tight (10 µm / ~0.006°);
        # for live jogging that's effectively unreachable, so the node would
        # never deactivate and would keep commanding the controller forever —
        # fighting direct joint commands and wobbling. Relax them for jog use.
        self.declare_parameter("tol_pos", self.TOL_POS)
        self.declare_parameter("tol_rot", self.TOL_ROT)
        self._tol_pos = float(self.get_parameter("tol_pos").value)
        self._tol_rot = float(self.get_parameter("tol_rot").value)

        # Singularity-robust damping. When manipulability w = sqrt(det(J W^-1 J^T))
        # falls below w_thresh, raise the DLS damping toward lambda_sing so the
        # arm eases into a singularity instead of the pseudo-inverse amplifying a
        # tiny Cartesian error into a huge dq (the overshoot). w_thresh=0 disables
        # it (default — keeps the FK/IK test launches unchanged; the pendant jog
        # instance turns it on).
        self.declare_parameter("w_thresh", 0.0)
        self.declare_parameter("lambda_sing", self.LAMBDA_MAX)
        self._w_thresh = float(self.get_parameter("w_thresh").value)
        self._lambda_sing = float(self.get_parameter("lambda_sing").value)
        # Per-tick joint-step cap [rad]; smaller = gentler jog / less overshoot.
        self.declare_parameter("dq_max", self.DQ_MAX)
        self._dq_max = float(self.get_parameter("dq_max").value)

        # Null-space VELOCITY damping. A 7-DoF arm holding a 6-DoF pose has a
        # redundant DoF (self-motion manifold): the EE stays put while the
        # joints can slide, and with no damping the live loop turns that into a
        # wobble. This term resists the self-motion by pulling toward a low-pass
        # of the actual posture (q_ref) — so it damps oscillation but fades to
        # zero at rest (q_ref -> q), unlike the fixed null_k pull which leaks and
        # blocks convergence. 0 disables (default). Enable it for the live jog.
        self.declare_parameter("null_damp_k", 0.0)
        self.declare_parameter("null_damp_alpha", self.NULL_DAMP_ALPHA)
        self._null_damp_k = float(self.get_parameter("null_damp_k").value)
        self._null_damp_alpha = float(self.get_parameter("null_damp_alpha").value)

        # Task-space gains. Lowering rot_gain reduces how hard the IK chases
        # orientation — useful for jog, where a wrist with a tight joint can't
        # hold orientation while the EE translates and otherwise limit-cycles.
        self.declare_parameter("pos_gain", self.POS_GAIN)
        self.declare_parameter("rot_gain", self.ROT_GAIN)
        self._pos_gain = float(self.get_parameter("pos_gain").value)
        self._rot_gain = float(self.get_parameter("rot_gain").value)

        # Position-only mode: track EE position only (3-DoF task), leaving
        # orientation free for the null-space to resolve. For a redundant arm
        # whose wrist can't hold orientation while translating, this removes the
        # orientation fight entirely — clean XYZ tracking, no wrist wobble. The
        # tool orientation drifts smoothly as a consequence. Default False.
        self.declare_parameter("position_only", False)
        self._position_only = bool(self.get_parameter("position_only").value)

        # Soft joint limits: tighter than the URDF limits, used to filter out
        # unwanted IK branches (e.g. pin joint_4 to one sign so the arm stays
        # elbow-UP and can't flip elbow-up/down — the flip passes through the
        # straight-arm singularity and causes wobble). Intersected with the URDF
        # limits in _cb_urdf. Sentinel [0.0] (len<=1) => use URDF limits as-is.
        self.declare_parameter("soft_q_min", [0.0])
        self.declare_parameter("soft_q_max", [0.0])
        smn = list(self.get_parameter("soft_q_min").value)
        smx = list(self.get_parameter("soft_q_max").value)
        self._soft_min = np.array(smn, dtype=float) if len(smn) > 1 else None
        self._soft_max = np.array(smx, dtype=float) if len(smx) > 1 else None
        self._q_lo = None   # effective (URDF ∩ soft) limits, set in _cb_urdf
        self._q_hi = None

        self._chain: UrdfChain | None = None
        self._joint_index = None

        self._q      = None
        self._q_ref  = None        # low-pass posture for null-space damping
        self._names_in = None
        self._T_des  = None
        self._active = False

        latched = QoSProfile(
            depth=1,
            durability=QoSDurabilityPolicy.TRANSIENT_LOCAL,
            reliability=QoSReliabilityPolicy.RELIABLE,
        )
        self.create_subscription(String, "/robot_description", self._cb_urdf, latched)
        self.create_subscription(JointState,  "/joint_states", self._cb_joints, 30)
        self.create_subscription(PoseStamped, "/ee_target",    self._cb_target, 10)
        self._pub = self.create_publisher(JointState, "/joint_commands", 20)
        self.create_timer(1.0 / self.RATE_HZ, self._step)

        self.get_logger().info(
            f"ik_arm_v3 ready — waiting for /robot_description (chain {self._base} → {self._tip})"
        )

    def _cb_urdf(self, msg: String):
        if self._chain is not None:
            return
        try:
            self._chain = UrdfChain(msg.data, self._base, self._tip)
        except Exception as e:
            self.get_logger().error(f"URDF parse failed: {e}")
            return
        self.get_logger().info(
            f"URDF loaded: {self._chain.n} DoF — joints: {self._chain.joint_names}"
        )
        if (self._null_target is not None and
                self._null_target.shape[0] != self._chain.n):
            self.get_logger().warn(
                f"null_target has {self._null_target.shape[0]} entries but "
                f"chain has {self._chain.n} DoF — ignoring, using q_mid")
            self._null_target = None
        if (self._joint_weights is not None and
                self._joint_weights.shape[0] != self._chain.n):
            self.get_logger().warn(
                f"joint_weights has {self._joint_weights.shape[0]} entries but "
                f"chain has {self._chain.n} DoF — ignoring (uniform)")
            self._joint_weights = None
        self._W_inv = (np.diag(1.0 / self._joint_weights)
                       if self._joint_weights is not None else np.eye(self._chain.n))

        # Effective joint limits = URDF limits ∩ soft limits (when sized right).
        self._q_lo = np.array(self._chain.q_min, dtype=float).copy()
        self._q_hi = np.array(self._chain.q_max, dtype=float).copy()
        if self._soft_min is not None and self._soft_min.shape[0] == self._chain.n:
            self._q_lo = np.maximum(self._q_lo, self._soft_min)
        if self._soft_max is not None and self._soft_max.shape[0] == self._chain.n:
            self._q_hi = np.minimum(self._q_hi, self._soft_max)
        if (self._soft_min is not None or self._soft_max is not None):
            self.get_logger().info(
                f"soft limits active: q_lo={np.round(self._q_lo, 2)} "
                f"q_hi={np.round(self._q_hi, 2)}")

        tgt = "begin/custom" if self._null_target is not None else "q_mid"
        self.get_logger().info(
            f"null-space: target={tgt}, NULL_K={self._null_k}, "
            f"inner_iters={self._inner_iters}, "
            f"joint_weights={'set' if self._joint_weights is not None else 'uniform'}")

    def _cb_joints(self, msg: JointState):
        if self._chain is None:
            return
        if self._joint_index is None:
            try:
                self._joint_index = [msg.name.index(n) for n in self._chain.joint_names]
            except ValueError as e:
                self.get_logger().warn(
                    f"/joint_states missing one of {self._chain.joint_names}: {e}"
                )
                return
        self._q = np.array([msg.position[i] for i in self._joint_index], dtype=float)
        self._names_in = list(self._chain.joint_names)

    def _cb_target(self, msg: PoseStamped):
        o = msg.pose.orientation
        T = np.eye(4)
        T[:3, :3] = quat_to_rot(o.x, o.y, o.z, o.w)
        T[:3,  3] = [msg.pose.position.x,
                     msg.pose.position.y,
                     msg.pose.position.z]
        self._T_des  = T
        self._active = True

    def _step(self):
        if self._chain is None or self._q is None or not self._active:
            return

        n = self._chain.n
        I6 = np.eye(6)
        I3 = np.eye(3)
        In = np.eye(n)

        q_null = (self._null_target if self._null_target is not None
                  else self._chain.q_mid)

        # Track a low-pass of the actual posture (updated from the measured _q,
        # before the inner loop mutates it). The null-space damping below pulls
        # toward this lagged posture, so it opposes fast self-motion (wobble)
        # but vanishes once the arm settles (q_ref converges to q).
        if self._q_ref is None:
            self._q_ref = self._q.copy()
        else:
            self._q_ref += self._null_damp_alpha * (self._q - self._q_ref)

        for _ in range(self._inner_iters):
            J, T_cur = self._chain.jacobian(self._q)
            p_ee = T_cur[:3, 3]

            e_p = self._pos_gain * (self._T_des[:3, 3] - p_ee)
            if self._position_only:
                # 3-DoF task: track position only; orientation left free for the
                # null-space (no wrist fight → no orientation-induced wobble).
                J_task, e_task, I_task = J[:3, :], e_p, I3
                err_norm = np.linalg.norm(e_p)
                converged = err_norm < self._tol_pos
            else:
                e_r = self._rot_gain * rot_error(T_cur[:3, :3], self._T_des[:3, :3])
                J_task, e_task, I_task = J, np.r_[e_p, e_r], I6
                err_norm = np.sqrt(np.linalg.norm(e_p)**2 + np.linalg.norm(e_r)**2)
                converged = (np.linalg.norm(e_p) < self._tol_pos and
                             np.linalg.norm(e_r) < self._tol_rot)
            if converged:
                self._active = False
                break

            lam = self.LAMBDA_MIN + (self.LAMBDA_MAX - self.LAMBDA_MIN) * \
                  min(1.0, err_norm / self.LAMBDA_KNEE)

            # Weighted damped least-squares: high-weight joints move less.
            W_inv = self._W_inv
            JWJ = J_task @ W_inv @ J_task.T

            # Singularity-robust floor: raise the damping as manipulability
            # drops so the arm slows smoothly into a singularity instead of the
            # pseudo-inverse amplifying a tiny Cartesian error into a large dq
            # (the overshoot). Disabled when w_thresh == 0.
            if self._w_thresh > 0.0:
                w = np.sqrt(max(np.linalg.det(JWJ), 0.0))
                engaged = w < self._w_thresh
                if engaged:
                    ratio = 1.0 - w / self._w_thresh
                    lam = max(lam, self._lambda_sing * ratio * ratio)
                # Log w on every active tick (throttled) so w_thresh can be
                # calibrated: watch typical w while jogging, set w_thresh just
                # above the w where overshoot starts.
                self.get_logger().info(
                    f"manip w={w:.4f} (thresh {self._w_thresh})"
                    + (f"  -> DAMP λ={lam:.4f}" if engaged else ""),
                    throttle_duration_sec=0.5)

            M  = JWJ + (lam ** 2) * I_task
            dq = W_inv @ J_task.T @ np.linalg.solve(M, e_task)

            # Null-space (redundancy) resolution, faded out near the target so
            # its leak through the damped projector can't hold a steady
            # following error that stops the node deactivating (the reason jog
            # used to run null_k=0). Full pull while moving, ~0 once converged.
            Jp  = W_inv @ J_task.T @ np.linalg.solve(M, J_task)
            N   = In - Jp                       # (damped) null-space projector
            null_gain = self._null_k * min(1.0, err_norm / self.NULL_FADE)
            dq += N @ (null_gain * (q_null - self._q))
            # Null-space velocity damping: resist self-motion (wobble) of the
            # redundant DoF by pulling toward the lagged posture; self-fading so
            # it doesn't block convergence/deactivation.
            if self._null_damp_k > 0.0:
                dq += N @ (self._null_damp_k * (self._q_ref - self._q))

            mag = np.linalg.norm(dq)
            if mag > self._dq_max:
                dq *= (self._dq_max / mag)

            self._q = np.clip(self._q + dq, self._q_lo, self._q_hi)

        cmd              = JointState()
        cmd.header.stamp = self.get_clock().now().to_msg()
        cmd.name         = self._names_in
        cmd.position     = self._q.tolist()
        self._pub.publish(cmd)


def main():
    rclpy.init()
    rclpy.spin(IKNode())
    rclpy.shutdown()


if __name__ == "__main__":
    main()
