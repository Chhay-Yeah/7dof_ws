#!/usr/bin/env python3
"""
fk_chain.py — the ONE forward-kinematics implementation for results_tools.

This is the backend's URDF-driven chain (a faithful port of
arm_bot/fk_arm_v3.py / plot_commanded_vs_executed.py::UrdfChain, validated to
agree with robot_state_publisher/RViz to ~0.1 mm). All three generators and the
recorder import THIS module so there is exactly one FK, one joint order, and one
set of units (radians in, metres out). The only addition over the analysis-dir
copy is `fk_links()`, which returns every link-frame origin so the front-view
animation can draw the arm as connected segments.
"""
import numpy as np


# ── rotation / transform primitives (verbatim from fk_arm_v3) ────────────────

def _rpy_to_R(r, p, y):
    cr, sr = np.cos(r), np.sin(r)
    cp, sp = np.cos(p), np.sin(p)
    cy, sy = np.cos(y), np.sin(y)
    Rz = np.array([[cy, -sy, 0], [sy, cy, 0], [0, 0, 1]])
    Ry = np.array([[cp, 0, sp], [0, 1, 0], [-sp, 0, cp]])
    Rx = np.array([[1, 0, 0], [0, cr, -sr], [0, sr, cr]])
    return Rz @ Ry @ Rx


def _origin_T(xyz, rpy):
    T = np.eye(4)
    T[:3, :3] = _rpy_to_R(*rpy)
    T[:3, 3] = xyz
    return T


def _axis_angle_R(axis, angle):
    a = axis / np.linalg.norm(axis)
    x, y, z = a
    c, s = np.cos(angle), np.sin(angle)
    C = 1 - c
    return np.array([
        [c + x*x*C,   x*y*C - z*s, x*z*C + y*s],
        [y*x*C + z*s, c + y*y*C,   y*z*C - x*s],
        [z*x*C - y*s, z*y*C + x*s, c + z*z*C],
    ])


class Chain:
    """Serial-chain FK built from a URDF string.

    Compatible with plot_commanded_vs_executed.PaperFrame (exposes `.fk`,
    `.joint_names`, `.n`, `.q_min`, `.q_max`).
    """

    def __init__(self, urdf_xml, base, tip):
        from urdf_parser_py.urdf import URDF
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

        self.base = base
        self.tip = tip
        self.joints = []
        self.joint_names = []
        q_min, q_max = [], []
        for j in chain:
            xyz = list(j.origin.xyz) if j.origin and j.origin.xyz else [0, 0, 0]
            rpy = list(j.origin.rpy) if j.origin and j.origin.rpy else [0, 0, 0]
            axis = np.array(j.axis if j.axis is not None else [0, 0, 1], dtype=float)
            self.joints.append({"name": j.name, "type": j.type, "child": j.child,
                                "T_origin": _origin_T(xyz, rpy), "axis": axis})
            if j.type in ("revolute", "continuous"):
                self.joint_names.append(j.name)
                if j.type == "revolute" and j.limit is not None:
                    q_min.append(float(j.limit.lower))
                    q_max.append(float(j.limit.upper))
                else:
                    q_min.append(-np.inf)
                    q_max.append(np.inf)
        self.n = len(self.joint_names)
        self.q_min = np.array(q_min)
        self.q_max = np.array(q_max)

    # -- tip pose (what PaperFrame and the overlay use) ----------------------
    def fk(self, q):
        """Full 4x4 transform base_link -> tip for joint vector q (rad)."""
        T = np.eye(4)
        qi = 0
        for j in self.joints:
            T = T @ j["T_origin"]
            if j["type"] in ("revolute", "continuous"):
                Rh = np.eye(4)
                Rh[:3, :3] = _axis_angle_R(j["axis"], q[qi])
                T = T @ Rh
                qi += 1
        return T

    # -- every link-frame origin (for the skeleton drawing) ------------------
    def fk_links(self, q):
        """Return (names, P) where P is (L+1, 3) of link-frame origins in the
        base frame: base_link first, then each child link down to the tip.
        Connecting consecutive rows draws the arm."""
        names = [self.base]
        pts = [np.zeros(3)]
        T = np.eye(4)
        qi = 0
        for j in self.joints:
            T = T @ j["T_origin"]
            if j["type"] in ("revolute", "continuous"):
                Rh = np.eye(4)
                Rh[:3, :3] = _axis_angle_R(j["axis"], q[qi])
                T = T @ Rh
                qi += 1
            names.append(j["child"])
            pts.append(T[:3, 3].copy())
        return names, np.array(pts)


def build_chain(urdf_xml, base_link, tip_link):
    return Chain(urdf_xml, base_link, tip_link)
