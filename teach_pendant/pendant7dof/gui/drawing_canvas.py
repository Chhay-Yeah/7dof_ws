"""Drawing canvas widget — captures pen strokes and tracks the live pen tip.

The scene is sized in millimetres: one scene unit == 1 mm of robot workspace,
and the scene rect equals the configured workspace (wx_mm x wy_mm). That keeps
the canvas aspect ratio locked to the workspace aspect ratio, so a square drawn
on screen is a square on the table, and the batch planner's px->mm scale is a
clean 1:1 (it divides the reported canvas width/height back out).
"""

from __future__ import annotations

import time

from PyQt6.QtWidgets import QGraphicsView, QGraphicsScene, QGraphicsEllipseItem
from PyQt6.QtCore import Qt
from PyQt6.QtGui import QPen, QPainterPath, QColor, QBrush

# Default workspace (mm). Matches the batch planner's default 40 mm box.
DEFAULT_WORKSPACE_MM = 40.0


class CanvasView(QGraphicsView):
    def __init__(self, workspace_x_mm: float = DEFAULT_WORKSPACE_MM,
                 workspace_y_mm: float = DEFAULT_WORKSPACE_MM) -> None:
        super().__init__()
        self.wx = float(workspace_x_mm)
        self.wy = float(workspace_y_mm)
        self.scene_ = QGraphicsScene(0, 0, self.wx, self.wy)
        self.setScene(self.scene_)
        self.setSceneRect(0, 0, self.wx, self.wy)
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

    # ── workspace size ────────────────────────────────────────────────────
    def set_workspace(self, wx_mm: float, wy_mm: float) -> None:
        """Resize the drawing area to a new workspace (mm). Clears strokes so
        old pixel coordinates aren't reinterpreted at the new scale."""
        self.wx = float(wx_mm)
        self.wy = float(wy_mm)
        self.scene_.setSceneRect(0, 0, self.wx, self.wy)
        self.setSceneRect(0, 0, self.wx, self.wy)
        self.clear()
        self.fitInView(self.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    # ── live pen-tip dot ──────────────────────────────────────────────────
    def _ensure_pen_dot(self) -> None:
        dot_r = max(0.4, self.wx * 0.015)
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
        x_px = norm_x * self.wx
        y_px = (1.0 - norm_y) * self.wy
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
            "canvas": {"width": self.wx, "height": self.wy, "units": "mm"},
            "strokes": [
                {"id": i, "points": s["points"]} for i, s in enumerate(self.strokes)
            ],
        }

    def clear(self) -> None:
        self.strokes = []
        self.scene_.clear()
        self._ensure_pen_dot()
        self._t0 = None
