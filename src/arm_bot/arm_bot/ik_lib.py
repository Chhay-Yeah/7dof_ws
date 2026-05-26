"""
ik_lib.py — URDF-driven FK / geometric Jacobian / DLS-IK building blocks.

Extracted from ik_arm_v3.py so both the real-time IK node and offline
batch planners (drawing_batch_planner.py) can share the same kinematic
model and solver.
"""
from __future__ import annotations

from typing import Optional

import numpy as np
from urdf_parser_py.urdf import URDF


def _rpy_to_R(r: float, p: float, y: float) -> np.ndarray:
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _origin_T(xyz, rpy) -> np.ndarray:
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(*rpy)
    T[:3,  3] = xyz
    return T


def _axis_angle_R(axis: np.ndarray, angle: float) -> np.ndarray:
    a = axis / np.linalg.norm(axis)
    x, y, z = a
    c, s = np.cos(angle), np.sin(angle)
    C = 1 - c
    return np.array([
        [c + x*x*C,     x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s,   c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s,   z*y*C + x*s, c + z*z*C ],
    ])


def rot_error(R_cur: np.ndarray, R_des: np.ndarray) -> np.ndarray:
    E = R_cur.T @ R_des
    return 0.5 * np.array([E[2, 1] - E[1, 2],
                           E[0, 2] - E[2, 0],
                           E[1, 0] - E[0, 1]])


def quat_to_rot(qx: float, qy: float, qz: float, qw: float) -> np.ndarray:
    n = np.sqrt(qx*qx + qy*qy + qz*qz + qw*qw)
    x, y, z, w = qx/n, qy/n, qz/n, qw/n
    return np.array([
        [1 - 2*(y*y + z*z),     2*(x*y - z*w),     2*(x*z + y*w)],
        [    2*(x*y + z*w), 1 - 2*(x*x + z*z),     2*(y*z - x*w)],
        [    2*(x*z - y*w),     2*(y*z + x*w), 1 - 2*(x*x + y*y)],
    ])


class UrdfChain:
    """Serial-chain FK + geometric Jacobian built from a URDF."""

    def __init__(self, urdf_xml: str, base: str, tip: str):
        robot = URDF.from_xml_string(urdf_xml)

        parent_of = {j.child: (j, j.parent) for j in robot.joints}

        chain = []
        link = tip
        while link != base:
            if link not in parent_of:
                raise RuntimeError(
                    f"link '{link}' has no parent (chain to '{base}' broken)")
            joint, parent = parent_of[link]
            chain.append(joint)
            link = parent
        chain.reverse()

        self.joints = []
        self.q_joints = []
        self.joint_names = []

        for j in chain:
            xyz = list(j.origin.xyz) if j.origin and j.origin.xyz else [0, 0, 0]
            rpy = list(j.origin.rpy) if j.origin and j.origin.rpy else [0, 0, 0]
            T_origin = _origin_T(xyz, rpy)
            axis = np.array(j.axis if j.axis is not None else [0, 0, 1],
                            dtype=float)
            self.joints.append({
                "name": j.name,
                "type": j.type,
                "T_origin": T_origin,
                "axis": axis,
            })
            if j.type in ("revolute", "continuous"):
                self.q_joints.append(len(self.joints) - 1)
                self.joint_names.append(j.name)

        self.n = len(self.q_joints)

        # Joint limits (revolute joints only)
        limits = {j.name: j.limit for j in robot.joints
                  if j.type in ("revolute", "continuous") and j.limit is not None}

        self.q_min = np.full(self.n, -np.pi)
        self.q_max = np.full(self.n,  np.pi)
        for i, name in enumerate(self.joint_names):
            lim = limits.get(name)
            if lim is not None and lim.lower is not None and lim.upper is not None:
                self.q_min[i] = lim.lower
                self.q_max[i] = lim.upper
        self.q_mid = 0.5 * (self.q_min + self.q_max)

    def fk(self, q: np.ndarray):
        """Return list of T_world at every joint origin (pre-rotation) and T_ee."""
        T = np.eye(4)
        T_at_origin = []
        qi = 0
        for j in self.joints:
            T = T @ j["T_origin"]
            T_at_origin.append(T.copy())
            if j["type"] in ("revolute", "continuous"):
                R = _axis_angle_R(j["axis"], q[qi])
                Rh = np.eye(4)
                Rh[:3, :3] = R
                T = T @ Rh
                qi += 1
        return T_at_origin, T

    def jacobian(self, q: np.ndarray):
        T_at_origin, T_ee = self.fk(q)
        p_ee = T_ee[:3, 3]
        J = np.zeros((6, self.n))
        for col, joint_idx in enumerate(self.q_joints):
            T_o = T_at_origin[joint_idx]
            z = T_o[:3, :3] @ self.joints[joint_idx]["axis"]
            z = z / np.linalg.norm(z)
            o = T_o[:3, 3]
            J[:3, col] = np.cross(z, p_ee - o)
            J[3:, col] = z
        return J, T_ee


# ── Iterative DLS solver ─────────────────────────────────────────────────────


DEFAULT_IK_PARAMS = dict(
    lambda_max=0.05,
    lambda_min=5e-3,
    lambda_knee=0.05,
    pos_gain=1.0,
    rot_gain=1.0,
    dq_max=0.10,
    tol_pos=1e-5,
    tol_rot=1e-4,
    null_k=0.3,
    max_iters=400,
    # If set, the null-space term pulls toward this joint configuration.
    # Defaults to chain.q_mid (centre of joint limits) when None.
    q_null_target=None,
    # Optional per-joint weights (length-n list). High weight = joint motion
    # is "expensive" in the weighted-DLS sense, so the IK prefers to use
    # other joints to achieve the task. Useful for parking redundant joints
    # of a 7-DOF arm at their seed values. Default = uniform (all 1.0).
    joint_weights=None,
    # Optional list of joint indices to FREEZE at the seed value. Stronger
    # than joint_weights: their Jacobian columns are zeroed before DLS so
    # they contribute nothing to the task solution, and dq for those
    # indices is forced to 0 each iteration (suppresses null-space drift
    # too). Locking N joints reduces the effective DOF to (chain.n - N) —
    # if that drops below the task DOF (6 for full pose, 3 for position
    # only) the IK becomes under-determined and gives a best-fit solution.
    locked_joints=None,
)


def solve_ik(
    chain: UrdfChain,
    T_des: np.ndarray,
    q_seed: np.ndarray,
    *,
    use_null_space: bool = True,
    params: Optional[dict] = None,
) -> tuple[np.ndarray, float, bool]:
    """Solve IK from a seed configuration to a target pose using DLS.

    Returns (q_solution, final_residual_norm, converged_flag).
    """
    P = {**DEFAULT_IK_PARAMS, **(params or {})}
    q = np.array(q_seed, dtype=float).copy()
    I6 = np.eye(6)
    In = np.eye(chain.n)
    p_des = T_des[:3, 3]
    R_des = T_des[:3, :3]

    q_null_target = (np.asarray(P["q_null_target"], dtype=float)
                     if P["q_null_target"] is not None else chain.q_mid)

    # Weighted-DLS inverse-weight matrix. W_inv on the dq path makes
    # high-weighted joints move less for the same task error. Default =
    # identity (unweighted IK).
    if P["joint_weights"] is None:
        W_inv = In
    else:
        w = np.asarray(P["joint_weights"], dtype=float)
        if w.shape != (chain.n,):
            raise ValueError(
                f'joint_weights must have {chain.n} entries, got {w.shape[0]}'
            )
        W_inv = np.diag(1.0 / w)

    # Resolve locked-joint indices (drop sentinels < 0 and out-of-range).
    locked_raw = P["locked_joints"] or []
    locked = [int(i) for i in locked_raw if 0 <= int(i) < chain.n]

    err_norm = float("inf")
    converged = False

    for _ in range(P["max_iters"]):
        J, T_cur = chain.jacobian(q)
        # Freeze locked joints by zeroing their Jacobian columns. They
        # contribute nothing to the task and won't be moved by DLS.
        if locked:
            J = J.copy()
            J[:, locked] = 0.0
        e_p = P["pos_gain"] * (p_des - T_cur[:3, 3])
        e_r = P["rot_gain"] * rot_error(T_cur[:3, :3], R_des)
        err_norm = float(np.sqrt(np.linalg.norm(e_p)**2 + np.linalg.norm(e_r)**2))

        if (np.linalg.norm(e_p) < P["tol_pos"] and
            np.linalg.norm(e_r) < P["tol_rot"]):
            converged = True
            break

        lam = P["lambda_min"] + (P["lambda_max"] - P["lambda_min"]) * \
              min(1.0, err_norm / P["lambda_knee"])

        # Weighted damped least-squares: minimises ||W^(1/2) dq||² s.t.
        # the task error reduction. dq direction biases away from
        # high-weight joints.
        JW_inv_JT = J @ W_inv @ J.T
        M  = JW_inv_JT + (lam ** 2) * I6
        dq = W_inv @ J.T @ np.linalg.solve(M, np.r_[e_p, e_r])

        if use_null_space:
            Jp = W_inv @ J.T @ np.linalg.solve(M, J)
            dq += (In - Jp) @ (P["null_k"] * (q_null_target - q))

        # Belt-and-suspenders: enforce zero motion on locked joints even
        # though their J columns were zeroed (null-space term can still
        # poke at them otherwise).
        if locked:
            dq[locked] = 0.0

        mag = float(np.linalg.norm(dq))
        if mag > P["dq_max"]:
            dq *= (P["dq_max"] / mag)

        q = np.clip(q + dq, chain.q_min, chain.q_max)

    return q, err_norm, converged
