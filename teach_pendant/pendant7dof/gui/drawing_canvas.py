"""Drawing canvas widget — captures pen strokes and tracks the live pen tip.

Lifted from the project's drawing_ui_node CanvasView so the pendant's drawing
tab behaves identically to the original standalone UI.
"""

from __future__ import annotations

import time

from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPen, QPainterPath, QColor, QBrush

# Canvas in pixels; the planner uses a 1:1 px->mm mapping.
CANVAS_W = 60
CANVAS_H = 60


class CanvasView(QGraphicsView):
    def __init__(self) -> None:
        super().__init__()
        self.scene_ = QGraphicsScene(0, 0, CANVAS_W, CANVAS_H)
        self.setScene(self.scene_)
        self.setSceneRect(0, 0, CANVAS_W, CANVAS_H)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumSize(300, 300)
        self.strokes: list[dict] = []
        self.current_path: QPainterPath | None = None
        self.current_points: list[dict] = []
        self._t0: float | None = None

        self.pen_dot: QGraphicsEllipseItem | None = None
        self._pen_last_pos: tuple[float, float] | None = None
        self._ensure_pen_dot()

    # ── live pen-tip dot ──────────────────────────────────────────────────
    def _ensure_pen_dot(self) -> None:
        dot_r = max(0.4, CANVAS_W * 0.01)
        self.pen_dot = QGraphicsEllipseItem(-dot_r, -dot_r, 2 * dot_r, 2 * dot_r)
        self.pen_dot.setBrush(QBrush(QColor(135, 206, 235)))  # skyblue
        self.pen_dot.setPen(QPen(Qt.PenStyle.NoPen))
        self.pen_dot.setZValue(1000.0)
        self.scene_.addItem(self.pen_dot)
        if self._pen_last_pos is not None:
            self.pen_dot.setPos(*self._pen_last_pos)
            self.pen_dot.setVisible(True)
        else:
            self.pen_dot.setVisible(False)

    def set_pen_pos(self, norm_x: float, norm_y: float, _z_mm: float) -> None:
        if not (0.0 <= norm_x <= 1.0 and 0.0 <= norm_y <= 1.0):
            self._pen_last_pos = None
            if self.pen_dot is not None:
                self.pen_dot.setVisible(False)
            return
        x_px = norm_x * CANVAS_W
        y_px = (1.0 - norm_y) * CANVAS_H
        self._pen_last_pos = (x_px, y_px)
        self.pen_dot.setPos(x_px, y_px)
        self.pen_dot.setVisible(True)

    # ── rendering helpers ─────────────────────────────────────────────────
    @staticmethod
    def _cosmetic_pen() -> QPen:
        pen = QPen(Qt.GlobalColor.black, 2)
        pen.setCosmetic(True)
        return pen

    def resizeEvent(self, event):
        super().resizeEvent(event)
        self.fitInView(self.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def showEvent(self, event):
        super().showEvent(event)
        self.fitInView(self.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    # ── stroke capture ────────────────────────────────────────────────────
    def mousePressEvent(self, event):
        if self._t0 is None:
            self._t0 = time.time()
        self.current_points = []
        p = self.mapToScene(event.pos())
        self.current_path = QPainterPath(p)
        self.current_points.append(
            {"x": p.x(), "y": p.y(), "t": time.time() - self._t0, "p": 0.5}
        )

    def mouseMoveEvent(self, event):
        if self.current_path is None:
            return
        p = self.mapToScene(event.pos())
        self.current_path.lineTo(p)
        self.scene_.clear()
        for s in self.strokes:
            self.scene_.addPath(s["qpath"], self._cosmetic_pen())
        self.scene_.addPath(self.current_path, self._cosmetic_pen())
        self._ensure_pen_dot()  # scene_.clear() destroyed the dot's C++ object
        self.current_points.append(
            {"x": p.x(), "y": p.y(), "t": time.time() - self._t0, "p": 0.5}
        )

    def mouseReleaseEvent(self, event):
        if self.current_path is None:
            return
        self.strokes.append({"qpath": self.current_path, "points": self.current_points})
        self.current_path = None

    def get_drawing(self) -> dict:
        return {
            "canvas": {"width": CANVAS_W, "height": CANVAS_H, "units": "px"},
            "strokes": [
                {"id": i, "points": s["points"]} for i, s in enumerate(self.strokes)
            ],
        }

    def clear(self) -> None:
        self.strokes = []
        self.scene_.clear()
        self._ensure_pen_dot()
        self._t0 = None
