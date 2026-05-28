"""A 2-axis joystick with a twist ring.

  * Centre knob: drag to set (x, y) in [-1, 1] each; springs back to centre on
    release.
  * Outer ring (a circular arrow, offset out from the knob so it's easy to grab
    without touching the knob): drag around it to set twist in [-1, 1]; springs
    back to 0 on release.

While either control is active the widget calls `on_jog(x, y, twist)` at a
fixed rate, so the consumer can stream velocity-style jog commands. The values
are displacements (rate), not absolute positions.
"""

from __future__ import annotations

import math

from PyQt6.QtWidgets import QWidget
from PyQt6.QtCore import Qt, QTimer, QPointF, QRectF
from PyQt6.QtGui import QPainter, QPen, QBrush, QColor, QPainterPath


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
        c = QPointF(self.width() / 2.0, self.height() / 2.0)
        r_base = s * 0.22       # knob travel radius
        r_knob = s * 0.11
        r_ring = s * 0.40       # ring centreline radius (offset out from knob)
        ring_w = s * 0.07
        return c, r_base, r_knob, r_ring, ring_w

    # ── painting ──────────────────────────────────────────────────────────
    def paintEvent(self, _e):
        c, r_base, r_knob, r_ring, ring_w = self._metrics()
        p = QPainter(self)
        p.setRenderHint(QPainter.RenderHint.Antialiasing)

        # twist ring (track)
        p.setPen(QPen(QColor("#3a3f47"), ring_w))
        p.setBrush(Qt.BrushStyle.NoBrush)
        ring_rect = QRectF(c.x() - r_ring, c.y() - r_ring, 2 * r_ring, 2 * r_ring)
        p.drawEllipse(ring_rect)

        # circular arrow on the ring, rotated by current twist for feedback
        p.save()
        p.translate(c)
        p.rotate(self.twist * 60.0)  # visual feedback only
        arrow_pen = QPen(QColor("#4f9bff"), max(2.0, ring_w * 0.5))
        arrow_pen.setCapStyle(Qt.PenCapStyle.RoundCap)
        p.setPen(arrow_pen)
        span = 250.0
        arc = QRectF(-r_ring, -r_ring, 2 * r_ring, 2 * r_ring)
        p.drawArc(arc, int(20 * 16), int(span * 16))
        # arrowhead at the end of the arc
        end = math.radians(20 + span)
        ex, ey = r_ring * math.cos(end), -r_ring * math.sin(end)
        head = QPainterPath()
        ah = ring_w * 0.9
        head.moveTo(ex, ey)
        head.lineTo(ex - ah, ey - ah * 0.4)
        head.lineTo(ex - ah * 0.4, ey + ah)
        head.closeSubpath()
        p.fillPath(head, QBrush(QColor("#4f9bff")))
        p.restore()

        # base well
        p.setPen(Qt.PenStyle.NoPen)
        p.setBrush(QBrush(QColor("#2b2f36")))
        p.drawEllipse(c, r_base + r_knob * 0.4, r_base + r_knob * 0.4)

        # knob at current (x,y)
        kx = c.x() + self.x * r_base
        ky = c.y() - self.y * r_base
        p.setBrush(QBrush(QColor("#6aa9ff" if self._knob_active else "#4f9bff")))
        p.drawEllipse(QPointF(kx, ky), r_knob, r_knob)

        # direction labels: which way drives +/- of each axis, plus the twist
        s = min(self.width(), self.height())
        p.setPen(QColor("#ffa726"))   # orange
        f = p.font()
        f.setBold(True)
        f.setPointSize(max(7, int(s * 0.05)))
        p.setFont(f)
        rl = s * 0.345

        def lab(px, py, text, w=78, h=18):
            p.drawText(QRectF(px - w / 2, py - h / 2, w, h),
                       Qt.AlignmentFlag.AlignCenter, text)

        lab(c.x(), c.y() - rl, "+" + self.y_label)
        lab(c.x(), c.y() + rl, "−" + self.y_label)
        lab(c.x() + rl, c.y(), "+" + self.x_label)
        lab(c.x() - rl, c.y(), "−" + self.x_label)
        # twist label in the top-left corner (free space outside the ring)
        p.drawText(QRectF(6, 4, self.width() - 12, 18),
                   Qt.AlignmentFlag.AlignLeft, "↻ twist = " + self.twist_label)

    # ── interaction ───────────────────────────────────────────────────────
    def _hit(self, pos):
        c, r_base, r_knob, r_ring, ring_w = self._metrics()
        d = math.hypot(pos.x() - c.x(), pos.y() - c.y())
        if d <= r_base + r_knob:
            return "knob"
        if r_ring - ring_w <= d <= r_ring + ring_w:
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
            self.on_jog(self.x, self.y, self.twist)
