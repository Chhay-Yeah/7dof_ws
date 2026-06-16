"""A 2-axis joystick with a twist ring.

  * Centre knob: drag to set (x, y) in [-1, 1] each; springs back to centre on
    release.
  * Outer ring (two striped grips you grab and turn): drag around it to set
    twist in [-1, 1]; springs back to 0 on release. A read-only quarter-circle
    arrow sits just outside the ring on the right as a direction legend (turn
    toward the "+" arrow for the positive direction); it is not interactive.

Any axis whose label is empty ("" via :meth:`set_labels`) is hidden and idle —
e.g. joint-7 mode binds only the Y axis, so the X labels, the twist ring and
its arrow legend are not drawn.

While either control is active the widget calls `on_jog(x, y, twist)` at a
fixed rate, so the consumer can stream velocity-style jog commands. The values
are displacements (rate), not absolute positions.
"""

from __future__ import annotations

import math

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QPolygonF


class Joystick(QWidget):
    def set_labels(self, x_label: str, y_label: str, twist_label: str) -> None:
        self.x_label, self.y_label, self.twist_label = x_label, y_label, twist_label
        self.update()

    def __init__(self, on_jog=None, rate_hz: float = 15.0) -> None:
        super().__init__()
        self.on_jog = on_jog
        self.setMinimumSize(220, 220)
        self.x = 0.0          # -1..1
        self.y = 0.0          # -1..1 (up = +)
        self.twist = 0.0      # -1..1
        # Short labels for what each direction drives (set by the consumer).
        self.x_label = "X"
        self.y_label = "Y"
        self.twist_label = "Z"
        self._knob_active = False
        self._ring_active = False
        self._grab_angle = 0.0
        self._timer = QTimer(self)
        self._timer.timeout.connect(self._tick)
        self._tick_ms = int(1000.0 / rate_hz)

    # ── geometry helpers (recomputed from current size) ───────────────────
    def _metrics(self):
        s = min(self.width(), self.height())
        # Centre the round control on the shorter (vertical) dimension, so a
        # widget wider than tall leaves blank space on the RIGHT for the twist
        # legend. For a square widget this is just the centre.
        c = QPointF(s / 2.0, self.height() / 2.0)
        r_base = s * 0.22       # knob travel radius
        r_knob = s * 0.11
        r_ring = s * 0.40       # twist-ring centreline radius (offset out from knob)
        ring_w = s * 0.07
        return c, r_base, r_knob, r_ring, ring_w

    # ── painting ──────────────────────────────────────────────────────────
    def paintEvent(self, _e):
        c, r_base, r_knob, r_ring, ring_w = self._metrics()
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        s = min(self.width(), self.height())

        # twist ring + grips (the rotary "grab and turn" control) — behind the
        # knob, and only when a twist axis is bound (joint-7 mode leaves twist
        # idle/unlabelled, so the ring is hidden).
        if self.twist_label:
            self._paint_twist_ring(p, c, r_ring, ring_w)

        # base well
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor("#2b2f36")))
        p.drawEllipse(c, r_base + r_knob * 0.4, r_base + r_knob * 0.4)

        # knob at current (x,y)
        kx = c.x() + self.x * r_base
        ky = c.y() - self.y * r_base
        p.setBrush(QBrush(QColor("#6aa9ff" if self._knob_active else "#4f9bff")))
        p.drawEllipse(QPointF(kx, ky), r_knob, r_knob)

        # direction labels: which way drives +/- of each axis. Each label is
        # nudged toward the centre per the pendant layout (Y by 5 px, X by
        # 10 px); an axis with an empty label (e.g. X in joint-7 mode) is
        # skipped entirely.
        p.setPen(QColor("#ffa726"))   # orange
        f = p.font()
        f.setBold(True)
        f.setPointSize(max(7, int(s * 0.05)))
        p.setFont(f)
        rl = s * 0.345

        def lab(px, py, text, w=78, h=18):
            p.drawText(QRectF(px - w / 2, py - h / 2, w, h),
                       Qt.AlignmentFlag.AlignCenter, text)

        if self.y_label:
            lab(c.x(), c.y() - rl + 5, "+" + self.y_label)   # +Y lowered 5 px
            lab(c.x(), c.y() + rl - 5, "−" + self.y_label)   # −Y raised 5 px
        if self.x_label:
            lab(c.x() + rl - 10, c.y(), "+" + self.x_label)  # +X 10 px to left
            lab(c.x() - rl + 10, c.y(), "−" + self.x_label)  # −X 10 px to right
        # twist direction legend: a curve + arrows just outside the ring, right
        # side. Read-only — the ring is what you grab; this only shows +/−.
        if self.twist_label:
            self._paint_twist_arc(p, c, r_ring, ring_w, s)

    # ── twist ring + grips (rotary "grab and turn" twist control) ─────────
    def _paint_twist_ring(self, p, c, r_ring, ring_w):
        # ring track
        p.setPen(QPen(QColor("#3a3f47"), ring_w))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawEllipse(QRectF(c.x() - r_ring, c.y() - r_ring, 2 * r_ring, 2 * r_ring))
        # two striped rectangular grips at 3- and 9-o'clock, rotating with the
        # current twist so it reads as a rotary control (box with 90° knurl
        # stripes).
        p.save()
        p.translate(c)
        # Negated so the grip rotates *with* the mouse (Qt's rotate is CW-
        # positive with y-down, the opposite of the math-convention twist).
        p.rotate(-self.twist * 60.0)
        grip_col = QColor("#6aa9ff" if self._ring_active else "#4f9bff")
        pad_long = ring_w * 2.6        # tangential length of the grip
        pad_thick = ring_w * 1.6       # radial thickness
        for sign in (1, -1):
            p.save()
            p.translate(sign * r_ring, 0.0)
            pad = QRectF(-pad_thick / 2, -pad_long / 2, pad_thick, pad_long)
            p.setPen(QPen(QColor("#23272d"), 1))
            p.setBrush(QBrush(grip_col))
            p.drawRoundedRect(pad, 3, 3)
            # 90-degree stripes across the grip (knurl marks)
            p.setPen(QPen(QColor("#e8f1ff"), 2))
            for j in (-1, 0, 1):
                y = j * pad_long * 0.26
                p.drawLine(QPointF(-pad_thick * 0.32, y),
                           QPointF(pad_thick * 0.32, y))
            p.restore()
        # short curved marks hint at the rotation direction
        arc_pen = QPen(grip_col, max(2.0, ring_w * 0.35))
        arc_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(arc_pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        arc = QRectF(-r_ring, -r_ring, 2 * r_ring, 2 * r_ring)
        p.drawArc(arc, int(38 * 16), int(34 * 16))
        p.drawArc(arc, int(218 * 16), int(34 * 16))
        p.restore()

    # ── twist direction legend (read-only curve + arrows outside the ring) ──
    def _paint_twist_arc(self, p, c, r_ring, ring_w, s):
        # 90° BLUE arc (±45° about 3 o'clock) sitting detached to the right of
        # the ring, an arrowhead + sign at each end. It only shows which way to
        # turn the ring: the CW (bottom) end is "+" and the CCW (top) end is "−",
        # matching on_jog's twist negation (a clockwise turn drives +).
        r = r_ring + ring_w * 1.5      # detached, clear of the grips
        half = 45.0
        col = QColor("#4f9bff")        # blue, like the ring grips
        pen = QPen(col, max(2.5, s * 0.014))
        pen.setCapStyle(Qt.PenCapStyle.FlatCap)
        p.setPen(pen)
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawArc(QRectF(c.x() - r, c.y() - r, 2 * r, 2 * r),
                  int(-half * 16), int(2 * half * 16))

        h = math.radians(half)
        head = max(8.0, s * 0.05)
        lab_off = s * 0.06
        f = p.font()
        f.setBold(True)
        f.setPointSize(max(7, int(s * 0.05)))
        for sign in (+1, -1):          # +1 = top end (CCW, −); −1 = bottom (CW, +)
            ang = sign * h
            ex = c.x() + r * math.cos(ang)
            ey = c.y() - r * math.sin(ang)
            # outward tangent — the arrowhead grows out of the line end along it
            self._arrow_head(p, QPointF(ex, ey),
                             -math.sin(h), -sign * math.cos(h), head, col)
            lx = c.x() + (r + lab_off) * math.cos(ang)
            ly = c.y() - (r + lab_off) * math.sin(ang)
            p.setPen(QColor("#ffa726"))   # +/- labels stay orange
            p.setFont(f)
            p.drawText(QRectF(lx - 18, ly - 9, 36, 18),
                       Qt.AlignmentFlag.AlignCenter,
                       ("−" if sign > 0 else "+") + self.twist_label)

    @staticmethod
    def _arrow_head(p, base, ux, uy, size, col):
        # Filled triangle: apex `size` BEYOND the curve end along (ux,uy) (a unit
        # vector), back corners straddling the end, so the curve runs into it.
        px, py = -uy, ux
        apex = QPointF(base.x() + ux * size, base.y() + uy * size)
        w = size * 0.55
        b1 = QPointF(base.x() + px * w, base.y() + py * w)
        b2 = QPointF(base.x() - px * w, base.y() - py * w)
        p.setPen(QPen(col, 1))
        p.setBrush(QBrush(col))
        p.drawPolygon(QPolygonF([apex, b1, b2]))

    # ── interaction ───────────────────────────────────────────────────────
    def _hit(self, pos):
        c, r_base, r_knob, r_ring, ring_w = self._metrics()
        d = math.hypot(pos.x() - c.x(), pos.y() - c.y())
        if d <= r_base + r_knob:
            return "knob"
        # twist: anywhere on the ring band (grab and turn), only when bound
        if self.twist_label and r_ring - ring_w <= d <= r_ring + ring_w:
            return "ring"
        return None

    def mousePressEvent(self, e):
        what = self._hit(e.position())
        if what == "knob":
            self._knob_active = True
            self._update_knob(e.position())
        elif what == "ring":
            self._ring_active = True
            c, *_ = self._metrics()
            self._grab_angle = math.atan2(-(e.position().y() - c.y()),
                                          e.position().x() - c.x())
        if (self._knob_active or self._ring_active) and not self._timer.isActive():
            self._timer.start(self._tick_ms)
        self.update()

    def mouseMoveEvent(self, e):
        if self._knob_active:
            self._update_knob(e.position())
        elif self._ring_active:
            c, *_ = self._metrics()
            ang = math.atan2(-(e.position().y() - c.y()), e.position().x() - c.x())
            d = ang - self._grab_angle
            while d > math.pi:
                d -= 2 * math.pi
            while d < -math.pi:
                d += 2 * math.pi
            # ~90deg of drag = full-scale twist
            self.twist = max(-1.0, min(1.0, d / (math.pi / 2.0)))
        self.update()

    def mouseReleaseEvent(self, _e):
        self._knob_active = False
        self._ring_active = False
        self.x = self.y = self.twist = 0.0
        self._timer.stop()
        # Emit one final centered tick so the consumer learns the jog stopped
        # (the timer is now stopped, so _tick won't fire again). The Cartesian
        # bridge uses this zero tick to re-anchor and let the jog IK go idle;
        # without it the IK keeps chasing the last leading target.
        if self.on_jog is not None:
            self.on_jog(0.0, 0.0, 0.0)
        self.update()

    def _update_knob(self, pos):
        c, r_base, *_ = self._metrics()
        dx = (pos.x() - c.x()) / r_base
        dy = -(pos.y() - c.y()) / r_base
        mag = math.hypot(dx, dy)
        if mag > 1.0:
            dx, dy = dx / mag, dy / mag
        self.x, self.y = dx, dy

    def _tick(self):
        if self.on_jog is not None:
            # Negate twist so a clockwise turn drives the positive direction;
            # the grip still rotates with the finger (paint uses self.twist).
            self.on_jog(self.x, self.y, -self.twist)
