"""Landscape FlexPendant-style teach pendant.

Launch screen: centred 2x4 grid of mode cards ("Coming soon" for empty slots).
Inside a mode: a full-height left sidebar switches modes, a top bar carries
Back/Forward history (top-left) and a persistent Simulation toggle (top-right),
and an E-STOP runs along the bottom of the content area.

Jogging merges joint and Cartesian jog behind one joystick (2 axes + a twist
ring). The simulation always runs as external RViz/Gazebo windows.
"""

from __future__ import annotations

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QStackedWidget, QButtonGroup, QVBoxLayout,
    QHBoxLayout, QGridLayout, QPushButton, QLabel, QComboBox, QLineEdit,
    QDoubleSpinBox, QGroupBox, QSizePolicy, QFrame, QDial, QStyle, QCheckBox,
    QListWidget, QListWidgetItem, QScrollArea, QGraphicsOpacityEffect,
    QGraphicsView, QGraphicsScene, QGraphicsItem, QGraphicsPathItem,
    QGraphicsLineItem, QRadioButton, QMenu,
)
from PyQt6.QtCore import (
    Qt, QTimer, QPropertyAnimation, QEasingCurve, QSize, QEvent, QRect, QRectF,
    QPointF, QMimeData,
)
from PyQt6.QtGui import (
    QDoubleValidator, QPixmap, QPainter, QIcon, QColor, QPen, QBrush, QDrag,
    QPainterPath, QPolygonF, QPainterPathStroker, QFontMetricsF,
)

from .. import bootstrap, store
from ..ros_bridge import PendantBridge, JOINT_NAMES
from .drawing_canvas import CanvasView, DEFAULT_WORKSPACE_MM
from .joystick import Joystick

MAX_WORKSPACE_MM = 50.0
DEFAULT_LIFT_MM = 0.0
DEFAULT_Z_PAPER_OFFSET_MM = 0.0

# Per-tick jog scale at the joystick rate (full stick deflection).
JOINT_STEP_PER_TICK = 0.012   # rad
CART_STEP_PER_TICK = 0.0015   # m

# Seconds commanded for each Motion-sequence move. The executor waits this long
# for one move to finish before issuing the next, so it must match the duration
# handed to move_to_joints().
MOTION_MOVE_S = 2.5

# Active modes (title, blurb). Order == sidebar order == mode_stack index.
_MODES = [
    ("Jogging", "Joint & Cartesian jog"),
    ("Motion", "Sequence targets"),
    ("Drawing", "Draw on a canvas"),
    ("Status", "Live joint & EE readouts"),
    ("Settings", "Backend & options"),
]
_COMING_SOON = [
    ("Teach", "Record waypoints"),
    ("Calibration", "Tool & base setup"),
    ("Vision", "Camera & detection"),
]

_SIDEBAR_QSS = """
#sidebar { background: #2b2b2b; }
#sidebar QPushButton {
    text-align: left; padding: 14px 16px; font-size: 15px;
    border: none; border-left: 4px solid transparent; background: #2b2b2b;
    color: #ddd;
}
#sidebar QPushButton:hover { background: #3a3a3a; }
#sidebar QPushButton:checked {
    background: #1e3a5f; color: white; border-left: 4px solid #4f9bff;
    font-weight: bold;
}
"""
_HEADER_QSS = "background: #1b1b1b;"
_LAUNCHER_QSS = "background: #202225;"
# Fat, rounded buttons everywhere in the content area.
_CONTENT_QSS = "QPushButton { min-height: 42px; padding: 8px 18px; border-radius: 7px; font-size: 14px; }"


class ModeCard(QFrame):
    """A clickable launcher card with a hover drop-shadow animation."""

    def __init__(self, title: str, blurb: str, enabled: bool, on_click) -> None:
        super().__init__()
        self._enabled = enabled
        self._on_click = on_click
        self.setFixedSize(200, 118)
        self.setObjectName("card")
        # No dark fill — just a faint outline so the card reads as a hit area;
        # enabled cards highlight on hover.
        if enabled:
            self.setCursor(Qt.CursorShape.PointingHandCursor)
            self.setStyleSheet(
                "#card { background: transparent; border: 1px solid #3a3f47; border-radius: 12px; }"
                "#card:hover { border-color: #4f9bff; background: rgba(79,155,255,0.10); }"
            )
            title_col = "#fff"
        else:
            self.setStyleSheet(
                "#card { background: transparent; border: 1px solid #303236; border-radius: 12px; }"
            )
            title_col = "#777"

        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 12, 12, 12)
        lay.addStretch(1)
        t = QLabel(title)
        t.setAlignment(Qt.AlignmentFlag.AlignCenter)
        t.setStyleSheet(
            f"color: {title_col}; font-size: 19px; font-weight: bold;"
            " border: none; background: transparent;"
        )
        lay.addWidget(t)
        if not enabled:
            sub = QLabel("Coming soon")
            sub.setAlignment(Qt.AlignmentFlag.AlignCenter)
            sub.setStyleSheet("color: #999; font-size: 12px; border: none; background: transparent;")
            lay.addWidget(sub)
        lay.addStretch(1)

    def mouseReleaseEvent(self, e):
        if self._enabled and self.rect().contains(e.pos()):
            self._on_click()
        super().mouseReleaseEvent(e)


class TargetRow(QWidget):
    """A saved-target list row: a name label that turns into an in-place edit
    field when the pencil is clicked (no popup), plus the pencil button."""

    def __init__(self, name: str, on_renamed=None) -> None:
        super().__init__()
        self._committing = False
        self._on_renamed = on_renamed
        lay = QHBoxLayout(self)
        lay.setContentsMargins(6, 0, 2, 0)
        self.label = QLabel(name)
        self.edit = QLineEdit(name)
        self.edit.hide()
        lay.addWidget(self.label, 1)
        lay.addWidget(self.edit, 1)
        self.pencil = QPushButton("✎")
        self.pencil.setFixedSize(26, 22)
        self.pencil.setToolTip("Rename")
        lay.addWidget(self.pencil)
        self.pencil.clicked.connect(self._start_edit)
        self.edit.editingFinished.connect(self._finish_edit)

    def name(self) -> str:
        return self.label.text()

    def _start_edit(self) -> None:
        self.edit.setText(self.label.text())
        self.label.hide()
        self.edit.show()
        self.edit.setFocus()
        self.edit.selectAll()

    def _finish_edit(self) -> None:
        if self._committing:   # editingFinished re-fires when we drop focus
            return
        self._committing = True
        text = self.edit.text().strip()
        changed = bool(text) and text != self.label.text()
        if text:
            self.label.setText(text)
        self.edit.hide()
        self.label.show()
        self._committing = False
        if changed and self._on_renamed is not None:
            self._on_renamed()


TARGET_MIME = "application/x-pendant-target"
CANVAS_BG = "#6b7178"     # Motion canvas background (gray)
NODE_BG = "#d8d8d8"       # saved-target fill (dimmed white)
NODE_BORDER = "#000000"   # saved-target outline (black)
JOG_AXIS_COLOR = "#ffa726"  # joystick axis label colour (orange)

# Black-outlined, faint-grey button so each Motion control reads as pressable.
# Shared by the editor's Save/New/Tasks and the task list's toolbar.
TASK_BTN_QSS = (
    "QPushButton { border: 1px solid #000000; border-radius: 4px;"
    " background: #e6e6e6; color: #1b1b1b; padding: 5px 14px; }"
    "QPushButton:hover { background: #f2f2f2; }"
    "QPushButton:pressed { background: #d0d0d0; }"
)


class EdgeItem(QGraphicsPathItem):
    """A directed arrow between two nodes. Routed with **orthogonal** segments
    only (horizontal/vertical, bending at right angles for diagonal pairs) and
    anchored to a **fixed** pair of ports chosen at creation — the ports never
    auto-switch as the nodes move. An arrow has a *kind*: "sequence" (advance
    immediately) or "timer" (wait ``delay`` seconds, shown as a circled "t")."""

    TIMER_COLOR = "#f0a020"
    STUB = 22.0   # how far the arrow steps straight out of each port
    HANDLE = 6.0  # half-size of the square endpoint handles (when selected)
    GRAB = 18.0   # how close a press must be to an endpoint to grab it

    _NORMALS = {
        "top": QPointF(0, -1), "bottom": QPointF(0, 1),
        "left": QPointF(-1, 0), "right": QPointF(1, 0),
    }

    def __init__(self, src: "NodeItem", dst: "NodeItem",
                 src_side: str = "right") -> None:
        super().__init__()
        self.src = src
        self.dst = dst
        self.src_side = src_side if src_side in self._NORMALS else "right"
        self.dst_side = self._pick_dst_side()   # fixed once, never auto-switches
        self.kind = "sequence"        # "sequence" | "timer"
        self.delay = 5.0              # seconds, used when kind == "timer"
        self._drag_end = None         # "src" | "dst" while an endpoint is dragged
        self._preview = None          # temp dashed line shown while reattaching
        self.setZValue(-1)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        src.scene().addItem(self)
        src.add_edge(self)
        dst.add_edge(self)
        self.update_path()

    def set_kind(self, kind: str) -> None:
        if kind not in ("sequence", "timer"):
            return
        self.kind = kind
        self.prepareGeometryChange()
        self.update()

    def set_delay(self, secs: float) -> None:
        self.delay = float(secs)
        self.update()

    def _pick_dst_side(self) -> str:
        """Choose the destination port once, as the dst port nearest the chosen
        source port. Frozen afterwards so the arrow keeps pointing at one dot."""
        a = self.src.port_scene(self.src_side)
        best, bd = "left", float("inf")
        for s in self._NORMALS:
            d = (self.dst.port_scene(s) - a).manhattanLength()
            if d < bd:
                bd, best = d, s
        return best

    def _waypoints(self):
        """Orthogonal polyline src-port → … → dst-port. Steps straight out of
        each port along its outward normal, then connects the stubs with at most
        one right-angle bend so every segment is axis-aligned."""
        a = self.src.port_scene(self.src_side)
        b = self.dst.port_scene(self.dst_side)
        na, nb = self._NORMALS[self.src_side], self._NORMALS[self.dst_side]
        d = self.STUB
        a1 = QPointF(a.x() + na.x() * d, a.y() + na.y() * d)
        b1 = QPointF(b.x() + nb.x() * d, b.y() + nb.y() * d)
        pts = [a, a1]
        if abs(a1.x() - b1.x()) > 0.5 and abs(a1.y() - b1.y()) > 0.5:
            if self.src_side in ("left", "right"):
                pts.append(QPointF(b1.x(), a1.y()))   # horizontal then vertical
            else:
                pts.append(QPointF(a1.x(), b1.y()))   # vertical then horizontal
        pts += [b1, b]
        return pts

    def update_path(self) -> None:
        pts = self._waypoints()
        path = QPainterPath(pts[0])
        for q in pts[1:]:
            path.lineTo(q)
        self.setPath(path)

    def shape(self):
        stroker = QPainterPathStroker()
        stroker.setWidth(14)          # fat clickable band around the thin line
        return stroker.createStroke(self.path())

    def boundingRect(self):
        r = self.path().boundingRect().adjusted(-14, -14, 14, 14)
        if self.kind == "timer":
            r = r.adjusted(-60, -24, 60, 24)   # room for the midpoint badge
        return r

    def paint(self, p, opt, widget=None):
        import math
        pts = self._waypoints()
        prev, b = pts[-2], pts[-1]    # last segment defines the arrowhead angle
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        col = QColor("#4f9bff") if self.isSelected() else QColor("#cfd6e0")
        p.setPen(QPen(col, 3 if self.isSelected() else 2))
        p.setBrush(Qt.BrushStyle.NoBrush)
        p.drawPath(self.path())
        ang = math.atan2(b.y() - prev.y(), b.x() - prev.x())
        s = 11
        p1 = QPointF(b.x() - s * math.cos(ang - math.pi / 6),
                     b.y() - s * math.sin(ang - math.pi / 6))
        p2 = QPointF(b.x() - s * math.cos(ang + math.pi / 6),
                     b.y() - s * math.sin(ang + math.pi / 6))
        p.setBrush(QBrush(col))
        p.drawPolygon(QPolygonF([b, p1, p2]))
        if self.kind == "timer":
            self._paint_timer_badge(p)
        if self.isSelected():
            self._paint_handles(p, pts[0], pts[-1])

    def _paint_handles(self, p, a, b) -> None:
        """Square grips at both endpoints — drag to reattach the arrow."""
        h = self.HANDLE
        p.setBrush(QBrush(QColor("white")))
        p.setPen(QPen(QColor("#4f9bff"), 2))
        for pt in (a, b):
            p.drawRect(QRectF(pt.x() - h, pt.y() - h, 2 * h, 2 * h))

    def _paint_timer_badge(self, p) -> None:
        path = self.path()
        mid = path.pointAtPercent(0.5)
        r = 11.0
        accent = QColor(self.TIMER_COLOR)
        # circled "t"
        p.setBrush(QBrush(QColor("white")))
        p.setPen(QPen(accent, 2))
        p.drawEllipse(mid, r, r)
        f = p.font()
        f.setBold(True)
        f.setPointSize(9)
        p.setFont(f)
        p.setPen(accent)
        p.drawText(QRectF(mid.x() - r, mid.y() - r, 2 * r, 2 * r),
                   Qt.AlignmentFlag.AlignCenter, "t")
        # "= Ns" pill to the right of the circle
        label = f"= {self.delay:g}s"
        fm = QFontMetricsF(f)
        tw = fm.horizontalAdvance(label) + 12
        th = fm.height() + 2
        pill = QRectF(mid.x() + r + 3, mid.y() - th / 2, tw, th)
        p.setBrush(QBrush(QColor("white")))
        p.setPen(QPen(accent, 1))
        p.drawRoundedRect(pill, 5, 5)
        p.setPen(QColor("#1b1b1b"))
        p.drawText(pill, Qt.AlignmentFlag.AlignCenter, label)

    # ── endpoint dragging (reattach / move the arrow) ─────────────────────
    def itemChange(self, change, value):
        # Raise a selected edge above the nodes so its endpoint handles sit on
        # top and stay grabbable even where they overlap a node's port.
        if change == QGraphicsItem.GraphicsItemChange.ItemSelectedHasChanged:
            self.setZValue(10 if value else -1)
        return super().itemChange(change, value)

    def _end_near(self, scene_pos):
        a = self.src.port_scene(self.src_side)
        b = self.dst.port_scene(self.dst_side)
        if (scene_pos - a).manhattanLength() <= self.GRAB:
            return "src"
        if (scene_pos - b).manhattanLength() <= self.GRAB:
            return "dst"
        return None

    def mousePressEvent(self, e):
        end = self._end_near(e.scenePos())
        if end is not None:
            self._drag_end = end
            self._preview = QGraphicsLineItem()
            self._preview.setPen(QPen(QColor("#2a7fff"), 2, Qt.PenStyle.DashLine))
            self.scene().addItem(self._preview)
            self._update_preview(e.scenePos())
            e.accept()
            return
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self._drag_end is not None:
            self._update_preview(e.scenePos())
            e.accept()
            return
        super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e):
        if self._drag_end is not None:
            if self._preview is not None:
                self.scene().removeItem(self._preview)
                self._preview = None
            node = next((it for it in self.scene().items(e.scenePos())
                         if isinstance(it, NodeItem)), None)
            self._reattach(self._drag_end, node, e.scenePos())
            self._drag_end = None
            e.accept()
            return
        super().mouseReleaseEvent(e)

    def _update_preview(self, scene_pos) -> None:
        anchor = (self.dst.port_scene(self.dst_side) if self._drag_end == "src"
                  else self.src.port_scene(self.src_side))
        self._preview.setLine(anchor.x(), anchor.y(),
                              scene_pos.x(), scene_pos.y())

    def _nearest_side(self, node, pt) -> str:
        best, bd = "left", float("inf")
        for s in self._NORMALS:
            d = (node.port_scene(s) - pt).manhattanLength()
            if d < bd:
                bd, best = d, s
        return best

    def _reattach(self, end, node, drop_pos) -> None:
        """Move the dragged endpoint onto ``node`` (re-picking the nearest
        port). Rejected if it would make the arrow start and end on the same
        node, or if dropped on empty canvas — the endpoint snaps back."""
        if node is None:
            self.update_path()
            self.update()
            return
        other = self.dst if end == "src" else self.src
        if node is other:
            self.update_path()
            self.update()
            return
        old = self.src if end == "src" else self.dst
        if self in old.edges:
            old.edges.remove(self)
        side = self._nearest_side(node, drop_pos)
        if end == "src":
            self.src, self.src_side = node, side
        else:
            self.dst, self.dst_side = node, side
        if self not in node.edges:
            node.add_edge(self)
        self.prepareGeometryChange()
        self.update_path()
        self.update()

    def mouseDoubleClickEvent(self, e):
        cb = getattr(self.scene(), "edge_double_clicked", None)
        if callable(cb):
            cb(self)
            e.accept()
            return
        super().mouseDoubleClickEvent(e)


class NodeItem(QGraphicsItem):
    """A flowchart node (a saved target): a white box outlined in the canvas
    colour. Hovering reveals 4 mid-side ports you can drag to another node."""
    W, H, PORT_R, MARGIN = 124, 46, 5, 12
    GRID = 8   # canvas snap grid (px)

    def __init__(self, name: str) -> None:
        super().__init__()
        self.name = name
        self._hover = False
        self._connecting = False
        self._temp = None
        self._start = None
        self._start_side = None
        self.edges: list = []
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsMovable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemIsSelectable, True)
        self.setFlag(QGraphicsItem.GraphicsItemFlag.ItemSendsGeometryChanges, True)
        self.setAcceptHoverEvents(True)

    def add_edge(self, e):
        self.edges.append(e)

    def boundingRect(self):
        m = self.MARGIN
        return QRectF(-m, -m, self.W + 2 * m, self.H + 2 * m)

    def _ports(self):
        return {
            "top": QPointF(self.W / 2, 0),
            "bottom": QPointF(self.W / 2, self.H),
            "left": QPointF(0, self.H / 2),
            "right": QPointF(self.W, self.H / 2),
        }

    def port_scene(self, side):
        return self.mapToScene(self._ports()[side])

    def paint(self, p, opt, widget=None):
        p.setRenderHint(QPainter.RenderHint.Antialiasing)
        p.setBrush(QBrush(QColor(NODE_BG)))
        p.setPen(QPen(QColor(NODE_BORDER), 2))
        p.drawRoundedRect(QRectF(0, 0, self.W, self.H), 7, 7)
        p.setPen(QColor("#1b1b1b"))
        p.drawText(QRectF(0, 0, self.W, self.H), Qt.AlignmentFlag.AlignCenter, self.name)
        if self._hover or self._connecting:
            p.setBrush(QBrush(QColor("#2a7fff")))
            p.setPen(QPen(QColor("white"), 1))
            for pt in self._ports().values():
                p.drawEllipse(pt, self.PORT_R, self.PORT_R)

    def hoverEnterEvent(self, e):
        self._hover = True
        self.update()

    def hoverLeaveEvent(self, e):
        self._hover = False
        self.update()

    def _port_at(self, pos):
        for side, pt in self._ports().items():
            if (pos - pt).manhattanLength() <= self.PORT_R * 3:
                return side
        return None

    def mousePressEvent(self, e):
        side = self._port_at(e.pos())
        if side is not None:               # start an arrow from this port
            self._connecting = True
            self._start_side = side
            self._start = self.port_scene(side)
            self._temp = QGraphicsLineItem()
            self._temp.setPen(QPen(QColor("#2a7fff"), 2, Qt.PenStyle.DashLine))
            self.scene().addItem(self._temp)
            e.accept()
            return
        super().mousePressEvent(e)

    def mouseMoveEvent(self, e):
        if self._connecting:
            self._temp.setLine(self._start.x(), self._start.y(),
                               e.scenePos().x(), e.scenePos().y())
            e.accept()
            return
        super().mouseMoveEvent(e)

    def mouseReleaseEvent(self, e):
        if self._connecting:
            if self._temp is not None:
                self.scene().removeItem(self._temp)
                self._temp = None
            self._connecting = False
            target = next((it for it in self.scene().items(e.scenePos())
                           if isinstance(it, NodeItem) and it is not self), None)
            if target is not None:
                EdgeItem(self, target, src_side=self._start_side or "right")
            self.update()
            e.accept()
            return
        super().mouseReleaseEvent(e)

    def itemChange(self, change, value):
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionChange:
            # Snap the box to the canvas grid as it moves.
            g = self.GRID
            return QPointF(round(value.x() / g) * g, round(value.y() / g) * g)
        if change == QGraphicsItem.GraphicsItemChange.ItemPositionHasChanged:
            for ed in self.edges:
                ed.update_path()
                ed.update()
        return super().itemChange(change, value)


class MotionCanvas(QGraphicsView):
    """Flowchart canvas: drop targets as nodes, drag mid-side ports to connect,
    Ctrl+wheel / pinch to zoom."""

    def __init__(self) -> None:
        super().__init__()
        self.scene_ = QGraphicsScene(self)
        self.scene_.setSceneRect(-2000, -2000, 4000, 4000)
        self.setScene(self.scene_)
        # Set by MainWindow; EdgeItem calls it on double-click.
        self.scene_.edge_double_clicked = None
        self.setRenderHint(QPainter.RenderHint.Antialiasing)
        self.setAcceptDrops(True)
        self.setTransformationAnchor(QGraphicsView.ViewportAnchor.AnchorUnderMouse)
        self.setStyleSheet(f"background: {CANVAS_BG};")
        self.setFocusPolicy(Qt.FocusPolicy.StrongFocus)  # receive Delete key
        self._zoom = 1.0

    def dragEnterEvent(self, e):
        if e.mimeData().hasFormat(TARGET_MIME):
            e.acceptProposedAction()

    def dragMoveEvent(self, e):
        if e.mimeData().hasFormat(TARGET_MIME):
            e.acceptProposedAction()

    def dropEvent(self, e):
        if not e.mimeData().hasFormat(TARGET_MIME):
            return
        name = bytes(e.mimeData().data(TARGET_MIME)).decode()
        p = self.mapToScene(e.position().toPoint())
        self.add_node(name, p.x(), p.y())
        e.acceptProposedAction()

    def add_node(self, name: str, x: float, y: float) -> "NodeItem":
        node = NodeItem(name)
        node.setPos(x - NodeItem.W / 2, y - NodeItem.H / 2)
        self.scene_.addItem(node)
        return node

    # ── deletion (arrows + target blocks) ─────────────────────────────────
    def remove_edge(self, edge) -> None:
        for n in (edge.src, edge.dst):
            if edge in n.edges:
                n.edges.remove(edge)
        if edge.scene() is self.scene_:
            self.scene_.removeItem(edge)

    def remove_node(self, node) -> None:
        for edge in list(node.edges):      # drop every arrow touching the block
            self.remove_edge(edge)
        if node.scene() is self.scene_:
            self.scene_.removeItem(node)

    def delete_selected(self) -> None:
        sel = list(self.scene_.selectedItems())
        for it in sel:                     # edges first so node cleanup is simple
            if isinstance(it, EdgeItem):
                self.remove_edge(it)
        for it in sel:
            if isinstance(it, NodeItem):
                self.remove_node(it)

    def keyPressEvent(self, e):
        if e.key() in (Qt.Key.Key_Delete, Qt.Key.Key_Backspace):
            self.delete_selected()
            e.accept()
            return
        super().keyPressEvent(e)

    def contextMenuEvent(self, e):
        target = next((it for it in self.items(e.pos())
                       if isinstance(it, (NodeItem, EdgeItem))), None)
        if target is None:
            return
        menu = QMenu(self)
        label = "Delete arrow" if isinstance(target, EdgeItem) else "Delete target"
        act = menu.addAction(label)
        if menu.exec(e.globalPos()) is act:
            if isinstance(target, EdgeItem):
                self.remove_edge(target)
            else:
                self.remove_node(target)
            e.accept()

    def _zoom_by(self, f: float) -> None:
        new = self._zoom * f
        if 0.25 <= new <= 4.0:
            self._zoom = new
            self.scale(f, f)

    def wheelEvent(self, e):
        if e.modifiers() & Qt.KeyboardModifier.ControlModifier:
            self._zoom_by(1.0015 ** e.angleDelta().y())
            e.accept()
        else:
            super().wheelEvent(e)

    def event(self, e):
        if (e.type() == QEvent.Type.NativeGesture and
                e.gestureType() == Qt.NativeGestureType.ZoomNativeGesture):
            self._zoom_by(1.0 + e.value())
            return True
        return super().event(e)


class TargetPaletteItem(QFrame):
    """A target box in the Motion palette: name + a hover-revealed blue '?'
    badge whose tooltip lists the target's X/Y/Z (or joints). Draggable onto
    the canvas."""

    def __init__(self, name: str, info_html: str) -> None:
        super().__init__()
        self._name = name
        self.setFixedHeight(38)
        self.setCursor(Qt.CursorShape.OpenHandCursor)
        # Dimmed-white box outlined in black, dark text (matches the nodes once
        # dropped on the canvas).
        self.setStyleSheet(
            f"QFrame {{ background: {NODE_BG}; border: 1px solid {NODE_BORDER};"
            f" border-radius: 6px; }}"
            " QLabel { color: #1b1b1b; background: transparent; border: none; }"
        )
        lay = QHBoxLayout(self)
        lay.setContentsMargins(10, 4, 6, 4)
        lay.addWidget(QLabel(name), 1)
        self.badge = QLabel("?")
        self.badge.setFixedSize(18, 18)
        self.badge.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.badge.setStyleSheet(
            "background: #2a7fff; color: white; border-radius: 9px; font-weight: bold;"
        )
        self.badge.setToolTip(info_html)
        self.badge.hide()
        lay.addWidget(self.badge)

    def enterEvent(self, e):
        self.badge.show()
        super().enterEvent(e)

    def leaveEvent(self, e):
        self.badge.hide()
        super().leaveEvent(e)

    def mouseMoveEvent(self, e):
        if e.buttons() & Qt.MouseButton.LeftButton:
            drag = QDrag(self)
            mime = QMimeData()
            mime.setData(TARGET_MIME, self._name.encode())
            drag.setMimeData(mime)
            drag.exec(Qt.DropAction.CopyAction)


class TaskCard(QFrame):
    """A saved-task tile in the Motion task list. Single click selects it
    (calls ``on_click``); double click opens it for editing (``on_double``)."""

    def __init__(self, index: int, name: str, subtitle: str,
                 on_click, on_double) -> None:
        super().__init__()
        self._index = index
        self._on_click = on_click
        self._on_double = on_double
        self._selected = False
        self.setFixedSize(180, 92)
        self.setCursor(Qt.CursorShape.PointingHandCursor)
        lay = QVBoxLayout(self)
        lay.setContentsMargins(12, 10, 12, 10)
        self._title = QLabel(name)
        self._title.setStyleSheet("font-weight: bold; font-size: 14px; background: transparent; border: none;")
        self._title.setWordWrap(True)
        lay.addWidget(self._title)
        sub = QLabel(subtitle)
        sub.setStyleSheet("color: #ccc; font-size: 11px; background: transparent; border: none;")
        lay.addWidget(sub)
        lay.addStretch(1)
        self._apply_style()

    def _apply_style(self) -> None:
        border = "#4f9bff" if self._selected else "#565b63"
        bg = "#343a44" if self._selected else "#2b2f36"
        self.setStyleSheet(
            f"TaskCard {{ background: {bg}; border: 2px solid {border};"
            f" border-radius: 8px; }}"
        )

    def set_selected(self, sel: bool) -> None:
        self._selected = sel
        self._apply_style()

    def mousePressEvent(self, e):
        self._on_click(self._index)
        super().mousePressEvent(e)

    def mouseDoubleClickEvent(self, e):
        self._on_double(self._index)
        super().mouseDoubleClickEvent(e)


class MainWindow(QMainWindow):
    def __init__(self, node: PendantBridge, backend) -> None:
        super().__init__()
        self.node = node
        self.backend = backend
        self.setWindowTitle("7-DOF Teach Pendant")
        self.resize(1100, 700)

        self._draw_cfg = {
            "workspace_x_mm": DEFAULT_WORKSPACE_MM,
            "workspace_y_mm": DEFAULT_WORKSPACE_MM,
            "lift_mm": DEFAULT_LIFT_MM,
            "z_paper_offset_mm": DEFAULT_Z_PAPER_OFFSET_MM,
        }
        self._history: list[int] = []
        self._hist_idx = -1
        self._page_anim: QPropertyAnimation | None = None
        self.jog_mode = "joint"     # 'joint' | 'cartesian'
        self.jog_group = 0          # 0 -> joints 1-3, 1 -> joints 4-6
        self._dial7_pending: float | None = None  # joint-7 dial target to flush
        self._tasks: list[dict] = []   # saved Motion tasks (in-memory only)
        self._task_seq = 0          # running counter for default task names
        self._editing_task: int | None = None  # index being edited (None = new)
        self._selected_task: int | None = None  # index selected in the list view
        # Motion sequence execution state (driven by a chained QTimer).
        self._motion_running = False
        self._motion_steps: list[dict] = []
        self._motion_i = 0
        self._motion_timer: QTimer | None = None
        self._loading = False       # suppress _persist() while restoring state

        # Full-height sidebar on the left; the top bar + page content sit in a
        # column to its right, so the back/forward buttons live inside the
        # content's top bar and never overlap the sidebar column.
        self.sidebar = self._build_sidebar()

        self.main_stack = QStackedWidget()
        self.launcher = self._build_launcher()
        self.main_stack.addWidget(self.launcher)             # page 0
        self.main_stack.addWidget(self._build_mode_content())  # page 1

        rcol = QVBoxLayout()
        rcol.setContentsMargins(0, 0, 0, 0)
        rcol.setSpacing(0)
        rcol.addWidget(self._build_top_bar())
        rcol.addWidget(self.main_stack, 1)
        right = QWidget()
        right.setLayout(rcol)

        body = QHBoxLayout()
        body.setContentsMargins(0, 0, 0, 0)
        body.setSpacing(0)
        body.addWidget(self.sidebar)
        body.addWidget(right, 1)

        container = QWidget()
        container.setStyleSheet(_CONTENT_QSS)
        container.setLayout(body)
        self.setCentralWidget(container)

        for i, btn in enumerate(self._nav_buttons):
            btn.clicked.connect(lambda _, idx=i: self._navigate(idx))

        self._navigate(-1)

        self._poll = QTimer(self)
        self._poll.timeout.connect(self._refresh_status)
        self._poll.start(100)

        self._load_persisted()

    def _white_icon(self, std: "QStyle.StandardPixmap") -> QIcon:
        """Standard arrow icons render dark; recolour them white so they show
        on the dark top-bar buttons."""
        src = self.style().standardIcon(std).pixmap(20, 20)
        out = QPixmap(src.size())
        out.fill(Qt.GlobalColor.transparent)
        pr = QPainter(out)
        pr.drawPixmap(0, 0, src)
        pr.setCompositionMode(QPainter.CompositionMode.CompositionMode_SourceIn)
        pr.fillRect(out.rect(), QColor("#ffffff"))
        pr.end()
        return QIcon(out)

    # ── top bar ────────────────────────────────────────────────────────────
    def _build_top_bar(self) -> QWidget:
        bar = QFrame()
        bar.setStyleSheet(_HEADER_QSS)
        bar.setFixedHeight(48)
        row = QHBoxLayout(bar)
        row.setContentsMargins(10, 0, 12, 0)
        row.setSpacing(8)

        self.back_btn = QPushButton()
        self.fwd_btn = QPushButton()
        self.back_btn.setIcon(self._white_icon(QStyle.StandardPixmap.SP_ArrowBack))
        self.fwd_btn.setIcon(self._white_icon(QStyle.StandardPixmap.SP_ArrowForward))
        for b in (self.back_btn, self.fwd_btn):
            b.setFixedSize(40, 34)
            b.setIconSize(QSize(18, 18))
            # Lighter "chip" so they read as buttons in the top bar, not as the
            # top of the (darker) sidebar directly below them.
            b.setStyleSheet(
                "QPushButton { background: #4a4f57; border: 1px solid #5a616b;"
                " border-radius: 6px; }"
                " QPushButton:hover { background: #565d68; }"
                " QPushButton:disabled { background: #2c2f34; border-color: #3a3f47; }"
            )
        self.back_btn.clicked.connect(self._go_back)
        self.fwd_btn.clicked.connect(self._go_forward)
        row.addWidget(self.back_btn)
        row.addWidget(self.fwd_btn)

        title = QLabel("7-DOF Teach Pendant")
        title.setStyleSheet("color: white; font-size: 16px; font-weight: bold;")
        row.addWidget(title)
        row.addStretch(1)

        self.header_estop_label = QLabel("E-stop: clear")
        self.header_estop_label.setStyleSheet("color: #6f6;")
        row.addWidget(self.header_estop_label)
        row.addSpacing(14)

        self.sim_btn = QPushButton("◌  Simulation: OFF")
        self.sim_btn.setFixedHeight(36)
        self._set_sim_btn(False)
        self.sim_btn.clicked.connect(self._toggle_backend)
        if self.backend is None:
            self.sim_btn.setEnabled(False)
            self.sim_btn.setText("Backend external")
        row.addWidget(self.sim_btn)
        return bar

    # ── launcher ─────────────────────────────────────────────────────────
    def _build_launcher(self) -> QWidget:
        w = QWidget()
        w.setStyleSheet(_LAUNCHER_QSS)
        outer = QVBoxLayout(w)
        outer.addStretch(1)
        hbox = QHBoxLayout()
        hbox.addStretch(1)
        grid = QGridLayout()
        grid.setHorizontalSpacing(20)
        grid.setVerticalSpacing(20)
        cells = [(t, b, True) for (t, b) in _MODES]
        cells += [(t, b, False) for (t, b) in _COMING_SOON]
        cells = cells[:8]
        for i, (title, blurb, enabled) in enumerate(cells):
            r, c = divmod(i, 4)
            on_click = (lambda idx=i: self._navigate(idx)) if enabled else (lambda: None)
            grid.addWidget(ModeCard(title, blurb, enabled, on_click), r, c)
        hbox.addLayout(grid)
        hbox.addStretch(1)
        outer.addLayout(hbox)
        outer.addStretch(1)
        return w

    # ── mode content: the mode stack over the E-stop bar (sidebar is a
    # separate, full-height column built in __init__) ─────────────────────
    def _build_mode_content(self) -> QWidget:
        w = QWidget()
        rcol = QVBoxLayout(w)
        rcol.setContentsMargins(0, 0, 0, 0)
        rcol.setSpacing(0)
        self.mode_stack = QStackedWidget()
        for builder in (self._build_jogging_tab, self._build_motion_tab,
                        self._build_drawing_tab, self._build_status_tab,
                        self._build_settings_tab):
            self.mode_stack.addWidget(builder())
        rcol.addWidget(self.mode_stack, 1)
        rcol.addWidget(self._build_estop_bar())
        return w

    def _build_sidebar(self) -> QWidget:
        side = QFrame()
        side.setObjectName("sidebar")
        side.setStyleSheet(_SIDEBAR_QSS)
        side.setFixedWidth(150)
        col = QVBoxLayout(side)
        col.setContentsMargins(0, 0, 0, 0)
        col.setSpacing(0)
        self._nav_group = QButtonGroup(self)
        self._nav_group.setExclusive(True)
        self._nav_buttons: list[QPushButton] = []
        for title, _ in _MODES:
            btn = QPushButton(title)
            btn.setCheckable(True)
            self._nav_group.addButton(btn)
            self._nav_buttons.append(btn)
            col.addWidget(btn)
        col.addStretch(1)
        return side

    # ── navigation ─────────────────────────────────────────────────────────
    def _navigate(self, view: int) -> None:
        self._history = self._history[: self._hist_idx + 1]
        self._history.append(view)
        self._hist_idx = len(self._history) - 1
        self._apply_view(view)

    def _go_back(self) -> None:
        if self._hist_idx > 0:
            self._hist_idx -= 1
            self._apply_view(self._history[self._hist_idx])

    def _go_forward(self) -> None:
        if self._hist_idx < len(self._history) - 1:
            self._hist_idx += 1
            self._apply_view(self._history[self._hist_idx])

    def _apply_view(self, view: int) -> None:
        if view == -1:
            self.main_stack.setCurrentIndex(0)
            self.sidebar.setVisible(False)   # home screen has no sidebar
            self._fade_in(self.launcher)
        else:
            self.main_stack.setCurrentIndex(1)
            self.sidebar.setVisible(True)
            self.mode_stack.setCurrentIndex(view)
            self._nav_buttons[view].setChecked(True)
            self._fade_in(self.mode_stack.currentWidget())
            if _MODES[view][0] == "Motion":
                self._refresh_motion_palette()
                # Default to the saved-task list when tasks already exist;
                # otherwise drop straight into a blank editor.
                if self._tasks:
                    self._show_task_list()
                else:
                    self._show_motion_editor()
        self.back_btn.setEnabled(self._hist_idx > 0)
        self.fwd_btn.setEnabled(self._hist_idx < len(self._history) - 1)

    def _fade_in(self, widget: QWidget) -> None:
        eff = QGraphicsOpacityEffect(widget)
        widget.setGraphicsEffect(eff)
        anim = QPropertyAnimation(eff, b"opacity", self)
        anim.setDuration(180)
        anim.setStartValue(0.0)
        anim.setEndValue(1.0)
        anim.setEasingCurve(QEasingCurve.Type.InOutQuad)
        anim.finished.connect(lambda: widget.setGraphicsEffect(None))
        anim.start()
        self._page_anim = anim

    # ── E-stop ───────────────────────────────────────────────────────────
    def _build_estop_bar(self) -> QWidget:
        bar = QWidget()
        row = QHBoxLayout(bar)
        row.setContentsMargins(8, 6, 8, 6)
        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setMinimumHeight(54)
        self.estop_btn.setStyleSheet(
            "background-color: #d33; color: white; font-size: 20px; font-weight: bold;"
        )
        self.estop_btn.clicked.connect(self._toggle_estop)
        row.addWidget(self.estop_btn)
        return bar

    def _toggle_estop(self) -> None:
        if self.node.estopped:
            self.node.estop_reset()
            self.estop_btn.setText("E-STOP")
            self.estop_btn.setStyleSheet(
                "background-color: #d33; color: white; font-size: 20px; font-weight: bold;"
            )
        else:
            self._finish_motion()   # abort any running Motion sequence
            self.node.estop()
            self.estop_btn.setText("RESET (E-stop active)")
            self.estop_btn.setStyleSheet(
                "background-color: #555; color: #f88; font-size: 18px; font-weight: bold;"
            )

    # ── simulation toggle ──────────────────────────────────────────────────
    def _selected_mode(self) -> str:
        return "gazebo" if self.mode_combo.currentIndex() == 0 else "moveit"

    def _toggle_backend(self) -> None:
        if self.backend is None:
            return
        if self.backend.running:
            self.backend.stop()
            self._set_sim_btn(False)
            self.mode_combo.setEnabled(True)
        else:
            mode = self._selected_mode()
            args = [f"mode:={mode}"]
            if mode == "gazebo":
                # Gazebo already shows its own 3D view; don't also open RViz.
                args.append("rviz:=false")
            self.backend.start(extra_args=args)
            self._set_sim_btn(True)
            self.mode_combo.setEnabled(False)

    def _set_sim_btn(self, on: bool) -> None:
        if on:
            self.sim_btn.setText("●  Simulation: ON")
            color = "#c62828"
        else:
            self.sim_btn.setText("◌  Simulation: OFF")
            color = "#2e7d32"
        self.sim_btn.setStyleSheet(
            f"background: {color}; color: white; font-size: 14px; font-weight: bold;"
            " border-radius: 6px; padding: 0 14px;"
        )

    # ── jogging page (joint + cartesian behind the joystick) ──────────────
    def _build_jogging_tab(self) -> QWidget:
        # Absolute placement aligned to the 80 px grid overlay so positions can
        # be dictated by cell (A1, C5, K1, …). Column letter -> index*80,
        # row number -> (n-1)*80.
        S = 80
        w = QWidget()

        # Live joint info, top-left starting at A1, one joint per row.
        info = QWidget(w)
        iv = QVBoxLayout(info)
        iv.setContentsMargins(4, 2, 4, 2)
        iv.setSpacing(2)
        head = QLabel("Joints")
        head.setStyleSheet("font-weight: bold;")
        iv.addWidget(head)
        self.joint_info_labels: list[QLabel] = []
        for name in JOINT_NAMES:
            lbl = QLabel(f"{name} = +0.000")
            lbl.setStyleSheet("font-family: monospace; font-size: 13px;")
            self.joint_info_labels.append(lbl)
            iv.addWidget(lbl)
        self.ee_info_label = QLabel("ee = —")
        self.ee_info_label.setStyleSheet("font-family: monospace; font-size: 11px; color: #aaa;")
        iv.addWidget(self.ee_info_label)
        iv.addStretch(1)
        info.setGeometry(4, 4, 168, 250)

        # Joystick centred in C6 → (200, 440).
        self.joystick = Joystick(on_jog=self._on_joy)
        self.joystick.setParent(w)
        self.joystick.setFixedSize(240, 240)
        self.joystick.move(200 - 120, 440 - 120)   # (80, 320)

        # Joint 7 dial centred in J6 → (760, 440).
        self.joint7_box = QGroupBox("Joint 7", w)
        v7 = QVBoxLayout(self.joint7_box)
        v7.setContentsMargins(6, 4, 6, 4)
        self.dial7 = QDial()
        self.dial7.setRange(-160, 160)   # 0.01 rad per step over [-1.6, 1.6]
        self.dial7.setNotchesVisible(True)
        self.dial7.setWrapping(False)
        self.dial7.setFixedSize(94, 94)
        self.dial7.valueChanged.connect(self._on_dial7)
        v7.addWidget(self.dial7, alignment=Qt.AlignmentFlag.AlignCenter)
        self.dial7_value_label = QLabel("+0.000 rad")
        self.dial7_value_label.setAlignment(Qt.AlignmentFlag.AlignCenter)
        self.dial7_value_label.setStyleSheet("font-family: monospace;")
        v7.addWidget(self.dial7_value_label)
        self.joint7_box.setFixedSize(126, 156)
        self.joint7_box.move(760 - 63, 440 - 78)    # (697, 362)

        # Control cluster anchored with the Mode selector at K1 (x = 800),
        # then Joint selector (K2), Home (K3), Speed (K4).
        kx = 10 * S
        sel_qss = (
            "QPushButton { border: 2px solid #6a7280; border-radius: 8px;"
            " background: #3a3f47; color: white; }"
            " QPushButton:hover { background: #454b54; }"
        )
        self.mode_toggle_btn = QPushButton("Mode: Joint", w)
        self.mode_toggle_btn.clicked.connect(self._toggle_jog_mode)
        self.mode_toggle_btn.setStyleSheet(sel_qss)
        self.mode_toggle_btn.setGeometry(kx, 6, 150, 42)          # K1
        self.group_toggle_btn = QPushButton("Joints 1–3", w)
        self.group_toggle_btn.clicked.connect(self._toggle_jog_group)
        self.group_toggle_btn.setStyleSheet(sel_qss)
        self.group_toggle_btn.setGeometry(kx, 6 + S, 150, 42)     # K2

        self.jog_home_btn = QPushButton("Home", w)                # K3
        self.jog_home_btn.clicked.connect(lambda: self.node.goto_preset("Home"))
        self.jog_home_btn.setStyleSheet(sel_qss)
        self.jog_home_btn.setGeometry(kx, 6 + 2 * S, 150, 42)

        speed = QWidget(w)                                        # K4
        sl = QHBoxLayout(speed)
        sl.setContentsMargins(0, 0, 0, 0)
        sl.addWidget(QLabel("Speed:"))
        self.jog_speed = QDoubleSpinBox()
        self.jog_speed.setRange(0.2, 3.0)
        self.jog_speed.setSingleStep(0.1)
        self.jog_speed.setValue(1.0)
        sl.addWidget(self.jog_speed)
        speed.setGeometry(kx, 6 + 3 * S, 150, 32)

        # Compact set-joint row at D1 (x = 240); joint mode only.
        self.joint_set_box = QWidget(w)
        self.joint_set_box.setStyleSheet(
            "QPushButton, QComboBox, QLineEdit { min-height: 0px; max-height: 30px; padding: 2px 8px; }"
        )
        jset = QHBoxLayout(self.joint_set_box)
        jset.setContentsMargins(0, 0, 0, 0)
        jset.addWidget(QLabel("Set joint:"))
        self.set_joint_combo = QComboBox()
        self.set_joint_combo.addItems(JOINT_NAMES)
        jset.addWidget(self.set_joint_combo)
        self.set_joint_input = QLineEdit()
        self.set_joint_input.setPlaceholderText("rad")
        self.set_joint_input.setValidator(QDoubleValidator(-6.283, 6.283, 4))
        self.set_joint_input.returnPressed.connect(self._do_set_joint)
        jset.addWidget(self.set_joint_input)
        jbtn = QPushButton("Set")
        jbtn.clicked.connect(self._do_set_joint)
        jset.addWidget(jbtn)
        self.joint_set_box.setGeometry(3 * S, 6, 380, 34)

        # Manual X/Y/Z set, same slot (D1), cartesian mode only.
        self.cart_set_box = QWidget(w)
        self.cart_set_box.setStyleSheet(
            "QPushButton, QLineEdit { min-height: 0px; max-height: 30px; padding: 2px 8px; }"
        )
        cset = QHBoxLayout(self.cart_set_box)
        cset.setContentsMargins(0, 0, 0, 0)
        cset.addWidget(QLabel("Set XYZ (m):"))
        self.xyz_inputs: list[QLineEdit] = []
        for axis in ("x", "y", "z"):
            cset.addWidget(QLabel(axis.upper()))
            e = QLineEdit()
            e.setPlaceholderText("m")
            e.setValidator(QDoubleValidator(-2.0, 2.0, 4))
            self.xyz_inputs.append(e)
            cset.addWidget(e)
        cbtn = QPushButton("Set")
        cbtn.clicked.connect(self._do_set_cartesian)
        cset.addWidget(cbtn)
        self.cart_set_box.setGeometry(3 * S, 6, 470, 34)

        # Targets window spanning D2 → H4: search / save / recall positions.
        self.targets_box = QGroupBox("Targets", w)
        self.targets_box.setStyleSheet(
            "QPushButton, QLineEdit { min-height: 0px; max-height: 30px; padding: 2px 10px; }"
        )
        tv = QVBoxLayout(self.targets_box)
        tv.setContentsMargins(8, 6, 8, 6)
        self.target_search = QLineEdit()
        self.target_search.setPlaceholderText("Search targets…")
        self.target_search.setClearButtonEnabled(True)
        self.target_search.textChanged.connect(self._apply_target_filter)
        tv.addWidget(self.target_search)
        self.targets_list = QListWidget()
        tv.addWidget(self.targets_list, 1)
        brow = QHBoxLayout()
        save_btn = QPushButton("Save current")
        save_btn.setStyleSheet(
            "border: 2px solid #6a7280; border-radius: 6px; background: #3a3f47;"
            " color: white; padding: 2px 10px;"
        )
        save_btn.clicked.connect(self._save_target)
        go_btn = QPushButton("Go to")
        go_btn.clicked.connect(self._goto_target)
        del_btn = QPushButton("Delete")
        del_btn.clicked.connect(self._delete_target)
        brow.addWidget(save_btn)
        brow.addWidget(go_btn)
        brow.addWidget(del_btn)
        tv.addLayout(brow)
        self.targets_box.setGeometry(3 * S, 1 * S, 5 * S, 3 * S)  # (240, 80, 400, 240)

        self._update_jog_ui()
        return w

    def _next_target_name(self) -> str:
        """Lowest free ``posN`` name — so after deleting pos1..pos3 the next
        save is pos1 again, not pos4."""
        used = {name for name, _ in self._iter_targets()}
        n = 1
        while f"pos{n}" in used:
            n += 1
        return f"pos{n}"

    def _save_target(self) -> None:
        name = self._next_target_name()   # default name; rename via the pencil
        joints = list(self.node.get_joints())
        xyz = self.node.get_ee_xyz()      # may be None if no /ee_pose yet
        self._add_target_row(name, {"joints": joints, "xyz": xyz})
        self._apply_target_filter()
        self._persist()

    def _add_target_row(self, name: str, data: dict) -> None:
        """Append a saved-target row backed by ``data`` (``{"joints", "xyz"}``).
        Shared by the live Save button and state restore on startup."""
        # No display text on the item itself — the row widget draws the name,
        # so the delegate doesn't double-draw it behind the widget.
        item = QListWidgetItem()
        item.setData(Qt.ItemDataRole.UserRole, data)
        joints = data.get("joints") or []
        item.setToolTip("  ".join(f"{q:+.3f}" for q in joints))
        item.setSizeHint(QSize(0, 30))
        self.targets_list.addItem(item)
        self.targets_list.setItemWidget(item, TargetRow(name, self._persist))

    def _apply_target_filter(self) -> None:
        q = self.target_search.text().strip().lower()
        for i in range(self.targets_list.count()):
            it = self.targets_list.item(i)
            row = self.targets_list.itemWidget(it)
            name = row.name() if isinstance(row, TargetRow) else ""
            it.setHidden(q not in name.lower())

    def _goto_target(self) -> None:
        item = self.targets_list.currentItem()
        if item is None:
            return
        data = item.data(Qt.ItemDataRole.UserRole) or {}
        self.node.move_to_joints(data.get("joints"))

    # ── shared target access (used by Motion) ─────────────────────────────
    def _iter_targets(self):
        out = []
        for i in range(self.targets_list.count()):
            it = self.targets_list.item(i)
            row = self.targets_list.itemWidget(it)
            name = row.name() if isinstance(row, TargetRow) else f"pos{i + 1}"
            out.append((name, it.data(Qt.ItemDataRole.UserRole) or {}))
        return out

    def _target_info_html(self, data: dict) -> str:
        as_joints = (getattr(self, "info_joints_check", None) is not None
                     and self.info_joints_check.isChecked())
        if as_joints and data.get("joints"):
            return "<br>".join(f"{n} = {q:+.3f}"
                               for n, q in zip(JOINT_NAMES, data["joints"]))
        xyz = data.get("xyz")
        if xyz:
            return f"X = {xyz[0]:+.3f}<br>Y = {xyz[1]:+.3f}<br>Z = {xyz[2]:+.3f}"
        return "(no pose captured)"

    # ── persistence (targets + tasks) ─────────────────────────────────────
    def _persist(self) -> None:
        """Write saved targets, Motion tasks and the task-name counter to disk.
        A no-op while state is being restored (``self._loading``)."""
        if self._loading:
            return
        targets = [{"name": name, "joints": data.get("joints"),
                    "xyz": data.get("xyz")}
                   for name, data in self._iter_targets()]
        store.save_state({
            "targets": targets,
            "tasks": self._tasks,
            "task_seq": self._task_seq,
        })

    def _load_persisted(self) -> None:
        state = store.load_state()
        if not state:
            return
        self._loading = True
        try:
            for t in state.get("targets", []):
                if not isinstance(t, dict) or "name" not in t:
                    continue
                xyz = t.get("xyz")
                self._add_target_row(t["name"], {
                    "joints": t.get("joints"),
                    "xyz": tuple(xyz) if xyz else None,
                })
            tasks = state.get("tasks")
            if isinstance(tasks, list):
                self._tasks = tasks
            self._task_seq = int(state.get("task_seq", self._task_seq))
        finally:
            self._loading = False
        self._apply_target_filter()

    def closeEvent(self, e) -> None:
        self._stop_motion()
        self._persist()
        super().closeEvent(e)

    # ── Motion page ────────────────────────────────────────────────────────
    def _build_motion_tab(self) -> QWidget:
        w = QWidget()
        main = QHBoxLayout(w)

        # Left side flips between the flowchart editor and the saved-task list.
        self.motion_left_stack = QStackedWidget()
        self.motion_left_stack.addWidget(self._build_motion_editor())     # page 0
        self.motion_left_stack.addWidget(self._build_task_list_view())    # page 1
        main.addWidget(self.motion_left_stack, 1)

        palette_box = QGroupBox("Targets")
        pv = QVBoxLayout(palette_box)
        hint = QLabel("Drag a target onto the canvas.")
        hint.setStyleSheet("color: #999; font-size: 11px;")
        hint.setWordWrap(True)
        pv.addWidget(hint)
        area = QScrollArea()
        area.setWidgetResizable(True)
        inner = QWidget()
        self.motion_palette_layout = QVBoxLayout(inner)
        self.motion_palette_layout.setSpacing(6)
        self.motion_palette_layout.addStretch(1)
        area.setWidget(inner)
        pv.addWidget(area, 1)

        self.motion_side = QStackedWidget()
        self.motion_side.setFixedWidth(220)
        self.motion_side.addWidget(palette_box)                 # page 0: targets
        self.motion_side.addWidget(self._build_arrow_picker())  # page 1: arrow type
        main.addWidget(self.motion_side)
        return w

    def _build_motion_editor(self) -> QWidget:
        w = QWidget()
        left = QVBoxLayout(w)
        left.setContentsMargins(0, 0, 0, 0)
        top = QHBoxLayout()
        top.addWidget(QLabel("Task:"))
        self.task_name = QLineEdit()
        self.task_name.setPlaceholderText("task name")
        top.addWidget(self.task_name)
        save_btn = QPushButton("Save")
        save_btn.setStyleSheet(TASK_BTN_QSS)
        save_btn.clicked.connect(self._save_task)
        top.addWidget(save_btn)
        new_btn = QPushButton("New")
        new_btn.setStyleSheet(TASK_BTN_QSS)
        new_btn.clicked.connect(self._motion_new_task)
        top.addWidget(new_btn)
        tasks_btn = QPushButton("Tasks")
        tasks_btn.setStyleSheet(TASK_BTN_QSS)
        tasks_btn.clicked.connect(self._show_task_list)
        top.addWidget(tasks_btn)
        top.addStretch(1)
        left.addLayout(top)
        self.motion_canvas = MotionCanvas()
        self.motion_canvas.scene_.edge_double_clicked = self._on_motion_edge_double
        left.addWidget(self.motion_canvas, 1)
        return w

    def _build_task_list_view(self) -> QWidget:
        w = QWidget()
        v = QVBoxLayout(w)
        v.setContentsMargins(0, 0, 0, 0)
        bar = QHBoxLayout()
        for label, slot in (
            ("Create new Task", self._task_create_new),
            ("Edit", self._task_edit_selected),
            ("Run", self._task_run_selected),
            ("Duplicate", self._task_duplicate_selected),
            ("Delete", self._task_delete_selected),
        ):
            b = QPushButton(label)
            b.setStyleSheet(TASK_BTN_QSS)
            b.clicked.connect(slot)
            bar.addWidget(b)
            if label == "Run":
                self._run_btn = b
        bar.addStretch(1)
        v.addLayout(bar)

        area = QScrollArea()
        area.setWidgetResizable(True)
        inner = QWidget()
        self.task_grid = QGridLayout(inner)
        self.task_grid.setSpacing(12)
        self.task_grid.setContentsMargins(8, 8, 8, 8)
        self.task_grid.setAlignment(
            Qt.AlignmentFlag.AlignTop | Qt.AlignmentFlag.AlignLeft)
        area.setWidget(inner)
        v.addWidget(area, 1)
        self._task_cards: list[TaskCard] = []
        return w

    # ── Motion view switching ─────────────────────────────────────────────
    def _show_motion_editor(self) -> None:
        self.motion_left_stack.setCurrentIndex(0)
        self.motion_side.setCurrentIndex(0)
        self.motion_side.show()

    def _show_task_list(self) -> None:
        self._refresh_task_list()
        self.motion_left_stack.setCurrentIndex(1)
        self.motion_side.hide()

    def _motion_new_task(self) -> None:
        """Reset the editor to a blank task (does not leave the editor)."""
        self._editing_task = None
        self._selected_task = None
        if hasattr(self, "task_name"):
            self.task_name.clear()
        self.motion_canvas.scene_.clear()
        self._motion_edge = None
        if hasattr(self, "motion_side"):
            self.motion_side.setCurrentIndex(0)

    # ── task (de)serialisation ────────────────────────────────────────────
    def _serialize_canvas(self):
        nodes = [it for it in self.motion_canvas.scene_.items()
                 if isinstance(it, NodeItem)]
        idx = {n: i for i, n in enumerate(nodes)}
        node_data = [{"name": n.name, "x": n.pos().x(), "y": n.pos().y()}
                     for n in nodes]
        edge_data = []
        for it in self.motion_canvas.scene_.items():
            if isinstance(it, EdgeItem) and it.src in idx and it.dst in idx:
                edge_data.append({"src": idx[it.src], "dst": idx[it.dst],
                                  "kind": it.kind, "delay": it.delay})
        return node_data, edge_data

    def _load_task_into_canvas(self, task: dict) -> None:
        self.motion_canvas.scene_.clear()
        self._motion_edge = None
        nodes = []
        for nd in task["nodes"]:
            n = NodeItem(nd["name"])
            self.motion_canvas.scene_.addItem(n)
            n.setPos(nd["x"], nd["y"])
            nodes.append(n)
        for ed in task["edges"]:
            if ed["src"] < len(nodes) and ed["dst"] < len(nodes):
                e = EdgeItem(nodes[ed["src"]], nodes[ed["dst"]])
                e.set_kind(ed["kind"])
                e.set_delay(ed["delay"])

    def _save_task(self) -> None:
        node_data, edge_data = self._serialize_canvas()
        name = self.task_name.text().strip()
        if not name:
            self._task_seq += 1
            name = f"Task {self._task_seq}"
        task = {"name": name, "nodes": node_data, "edges": edge_data}
        if self._editing_task is not None and self._editing_task < len(self._tasks):
            self._tasks[self._editing_task] = task
            self._selected_task = self._editing_task
        else:
            self._tasks.append(task)
            self._selected_task = len(self._tasks) - 1
        self._persist()
        self._show_task_list()

    # ── task list interactions ────────────────────────────────────────────
    def _refresh_task_list(self) -> None:
        while self.task_grid.count():
            it = self.task_grid.takeAt(0)
            wdg = it.widget()
            if wdg is not None:
                wdg.deleteLater()
        self._task_cards = []
        if not self._tasks:
            lbl = QLabel("No saved tasks yet — build one on the canvas and press Save.")
            lbl.setStyleSheet("color: #ccc;")
            self.task_grid.addWidget(lbl, 0, 0)
            return
        cols = 3
        for i, task in enumerate(self._tasks):
            n = len(task["nodes"])
            sub = f"{n} target{'' if n == 1 else 's'}"
            card = TaskCard(i, task["name"], sub,
                            self._select_task, self._task_open)
            self._task_cards.append(card)
            self.task_grid.addWidget(card, i // cols, i % cols)
        if (self._selected_task is not None
                and self._selected_task < len(self._task_cards)):
            self._task_cards[self._selected_task].set_selected(True)

    def _select_task(self, index: int) -> None:
        self._selected_task = index
        for i, c in enumerate(self._task_cards):
            c.set_selected(i == index)

    def _task_open(self, index: int) -> None:
        self._selected_task = index
        self._task_edit_selected()

    def _task_create_new(self) -> None:
        self._motion_new_task()
        self._show_motion_editor()

    def _task_edit_selected(self) -> None:
        if self._selected_task is None:
            return
        self._editing_task = self._selected_task
        task = self._tasks[self._selected_task]
        self.task_name.setText(task["name"])
        self._load_task_into_canvas(task)
        self._show_motion_editor()

    def _task_duplicate_selected(self) -> None:
        if self._selected_task is None:
            return
        import copy
        task = copy.deepcopy(self._tasks[self._selected_task])
        task["name"] = task["name"] + " copy"
        self._tasks.insert(self._selected_task + 1, task)
        self._selected_task += 1
        self._persist()
        self._refresh_task_list()

    def _task_delete_selected(self) -> None:
        if self._selected_task is None:
            return
        del self._tasks[self._selected_task]
        self._selected_task = None
        self._persist()
        self._refresh_task_list()

    # ── Motion sequence execution ─────────────────────────────────────────
    def _build_motion_plan(self, task: dict):
        """Linearise a task's flowchart into an ordered list of move steps.

        Returns ``(steps, error)``. ``steps`` is a list of
        ``{"name", "joints", "pre_delay"}`` where ``pre_delay`` is the number
        of seconds to dwell at the *previous* node before moving here (set by a
        Timer arrow; 0 for a Sequence arrow or the first node). ``error`` is a
        human-readable string when the plan can't be built (and ``steps`` is
        empty).

        Traversal: start at a node with no incoming edge (a fallback to node 0
        when every node has one, e.g. a loop), then follow the first
        not-yet-visited outgoing edge at each step — the common case is a single
        chain. Joint values are resolved by node name from the saved targets.
        """
        nodes = task.get("nodes", [])
        edges = task.get("edges", [])
        if not nodes:
            return [], "task has no targets"

        joints_by_name = {name: data.get("joints")
                          for name, data in self._iter_targets()}

        # adjacency: src index -> list of (dst, kind, delay)
        adj: dict[int, list] = {i: [] for i in range(len(nodes))}
        has_incoming = [False] * len(nodes)
        for ed in edges:
            s, d = ed.get("src"), ed.get("dst")
            if s is None or d is None or s >= len(nodes) or d >= len(nodes):
                continue
            adj[s].append((d, ed.get("kind", "sequence"), ed.get("delay", 0.0)))
            has_incoming[d] = True

        start = next((i for i in range(len(nodes)) if not has_incoming[i]), 0)

        steps = []
        visited = set()
        cur, pre_delay = start, 0.0
        while cur is not None and cur not in visited:
            visited.add(cur)
            name = nodes[cur].get("name", f"node{cur}")
            joints = joints_by_name.get(name)
            if joints is None or len(joints) != 7:
                return [], (f"target '{name}' has no saved joints — "
                            f"re-save it in Jogging")
            steps.append({"name": name, "joints": list(joints),
                          "pre_delay": pre_delay})
            nxt = next((e for e in adj[cur] if e[0] not in visited), None)
            if nxt is None:
                break
            cur = nxt[0]
            pre_delay = float(nxt[2]) if nxt[1] == "timer" else 0.0
        return steps, None

    def _task_run_selected(self) -> None:
        if self._motion_running:
            self._stop_motion()
            return
        if self._selected_task is None:
            self.statusBar().showMessage("Select a task to run.", 3000)
            return
        if self.node.estopped:
            self.statusBar().showMessage(
                "E-stop is engaged — clear it before running.", 4000)
            return
        task = self._tasks[self._selected_task]
        steps, error = self._build_motion_plan(task)
        if error:
            self.statusBar().showMessage(f"Cannot run '{task['name']}': {error}",
                                         5000)
            return
        self._motion_steps = steps
        self._motion_i = 0
        self._motion_running = True
        if hasattr(self, "_run_btn"):
            self._run_btn.setText("Stop")
        self._run_step()

    def _run_step(self) -> None:
        if not self._motion_running:
            return
        if self.node.estopped:
            self.statusBar().showMessage("Motion aborted — E-stop engaged.", 4000)
            self._finish_motion()
            return
        i = self._motion_i
        if i >= len(self._motion_steps):
            self.statusBar().showMessage("Motion complete.", 4000)
            self._finish_motion()
            return
        step = self._motion_steps[i]
        n = len(self._motion_steps)
        self.node.move_to_joints(step["joints"], MOTION_MOVE_S)
        self.statusBar().showMessage(
            f"Running '{self._tasks[self._selected_task]['name']}': "
            f"{i + 1}/{n} → {step['name']}")
        self._motion_i = i + 1
        # Wait for this move to finish, then dwell for the next edge's timer.
        wait_s = MOTION_MOVE_S
        if self._motion_i < n:
            wait_s += self._motion_steps[self._motion_i]["pre_delay"]
        self._motion_timer = QTimer(self)
        self._motion_timer.setSingleShot(True)
        self._motion_timer.timeout.connect(self._run_step)
        self._motion_timer.start(int(wait_s * 1000))

    def _stop_motion(self) -> None:
        if not self._motion_running:
            return
        self._finish_motion()
        self.node.freeze()
        self.statusBar().showMessage("Motion stopped.", 3000)

    def _finish_motion(self) -> None:
        self._motion_running = False
        if self._motion_timer is not None:
            self._motion_timer.stop()
            self._motion_timer = None
        if hasattr(self, "_run_btn"):
            self._run_btn.setText("Run")

    def _build_arrow_picker(self) -> QWidget:
        box = QGroupBox("Arrow")
        v = QVBoxLayout(box)
        hint = QLabel("Choose how this arrow advances to the next target.")
        hint.setStyleSheet("color: #999; font-size: 11px;")
        hint.setWordWrap(True)
        v.addWidget(hint)

        self._arrow_seq_radio = QRadioButton("Sequence — go immediately")
        self._arrow_timer_radio = QRadioButton("Timer — wait, then go")
        grp = QButtonGroup(box)
        grp.addButton(self._arrow_seq_radio)
        grp.addButton(self._arrow_timer_radio)
        self._arrow_seq_radio.setChecked(True)
        self._arrow_seq_radio.toggled.connect(self._on_arrow_kind_changed)
        v.addWidget(self._arrow_seq_radio)
        v.addWidget(self._arrow_timer_radio)

        row = QHBoxLayout()
        row.addWidget(QLabel("Wait:"))
        self._arrow_delay_spin = QDoubleSpinBox()
        self._arrow_delay_spin.setRange(0.1, 600.0)
        self._arrow_delay_spin.setSingleStep(0.5)
        self._arrow_delay_spin.setDecimals(1)
        self._arrow_delay_spin.setSuffix(" s")
        self._arrow_delay_spin.setValue(5.0)
        self._arrow_delay_spin.setEnabled(False)
        self._arrow_delay_spin.valueChanged.connect(self._on_arrow_delay_changed)
        row.addWidget(self._arrow_delay_spin, 1)
        v.addLayout(row)

        v.addStretch(1)
        done = QPushButton("Done")
        done.clicked.connect(lambda: self.motion_side.setCurrentIndex(0))
        v.addWidget(done)
        return box

    def _on_motion_edge_double(self, edge) -> None:
        self._motion_edge = edge
        self._arrow_delay_spin.setValue(edge.delay)
        if edge.kind == "timer":
            self._arrow_timer_radio.setChecked(True)
        else:
            self._arrow_seq_radio.setChecked(True)
        self._arrow_delay_spin.setEnabled(edge.kind == "timer")
        self.motion_side.setCurrentIndex(1)

    def _on_arrow_kind_changed(self, _checked: bool = False) -> None:
        kind = "timer" if self._arrow_timer_radio.isChecked() else "sequence"
        self._arrow_delay_spin.setEnabled(kind == "timer")
        edge = getattr(self, "_motion_edge", None)
        if edge is not None:
            try:
                edge.set_kind(kind)
            except RuntimeError:        # edge was deleted from the canvas
                self._motion_edge = None

    def _on_arrow_delay_changed(self, val: float) -> None:
        edge = getattr(self, "_motion_edge", None)
        if edge is not None:
            try:
                edge.set_delay(val)
            except RuntimeError:        # edge was deleted from the canvas
                self._motion_edge = None

    def _refresh_motion_palette(self) -> None:
        lay = self.motion_palette_layout
        while lay.count() > 1:                     # keep the trailing stretch
            it = lay.takeAt(0)
            wdg = it.widget()
            if wdg is not None:
                wdg.deleteLater()
        for name, data in self._iter_targets():
            lay.insertWidget(lay.count() - 1,
                             TargetPaletteItem(name, self._target_info_html(data)))

    def _delete_target(self) -> None:
        row = self.targets_list.currentRow()
        if row >= 0:
            self.targets_list.takeItem(row)
            self._persist()

    def _toggle_jog_mode(self) -> None:
        self.jog_mode = "cartesian" if self.jog_mode == "joint" else "joint"
        self._update_jog_ui()

    def _toggle_jog_group(self) -> None:
        self.jog_group = 1 - self.jog_group
        self._update_jog_ui()

    def _update_jog_ui(self) -> None:
        cartesian = self.jog_mode == "cartesian"
        self.mode_toggle_btn.setText("Mode: Cartesian" if cartesian else "Mode: Joint")
        self.group_toggle_btn.setVisible(not cartesian)
        self.joint7_box.setVisible(not cartesian)
        self.joint_set_box.setVisible(not cartesian)
        self.cart_set_box.setVisible(cartesian)
        if cartesian:
            self.joystick.set_labels("X", "Y", "Z")
        else:
            base = 0 if self.jog_group == 0 else 3
            self.group_toggle_btn.setText("Joints 1–3" if self.jog_group == 0 else "Joints 4–6")
            # Short labels on the joystick itself (J1..J6 = joint_1..joint_6).
            self.joystick.set_labels(
                f"J{base+1}", f"J{base+2}", f"J{base+3}"
            )

    def _on_joy(self, x: float, y: float, twist: float) -> None:
        spd = self.jog_speed.value()
        if self.jog_mode == "cartesian":
            k = CART_STEP_PER_TICK * spd
            self.node.cartesian_jog_xyz(x * k, y * k, twist * k)
        else:
            k = JOINT_STEP_PER_TICK * spd
            base = 0 if self.jog_group == 0 else 3
            self.node.jog_joints({base: x * k, base + 1: y * k, base + 2: twist * k})

    def _on_dial7(self, value: int) -> None:
        rad = value / 100.0
        self._dial7_pending = rad
        self.dial7_value_label.setText(f"{rad:+.3f} rad")

    def _do_set_joint(self) -> None:
        text = self.set_joint_input.text().strip()
        if not text:
            return
        try:
            target = float(text)
        except ValueError:
            return
        self.node.set_joint(self.set_joint_combo.currentIndex(), target)

    def _do_set_cartesian(self) -> None:
        try:
            x, y, z = (float(e.text()) for e in self.xyz_inputs)
        except ValueError:
            return
        self.node.set_cartesian(x, y, z)

    # ── drawing page ──────────────────────────────────────────────────────
    def _build_drawing_tab(self) -> QWidget:
        w = QWidget()
        outer = QHBoxLayout(w)

        self.canvas = CanvasView(self._draw_cfg["workspace_x_mm"],
                                 self._draw_cfg["workspace_y_mm"])
        self.canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.node.attach_pen_callback(self.canvas.set_pen_pos)
        outer.addWidget(self.canvas, 1)

        panel = QWidget()
        panel.setFixedWidth(300)
        col = QVBoxLayout(panel)

        cfg_box = QGroupBox("Drawing settings")
        form = QGridLayout(cfg_box)
        self.ws_x_spin = self._mm_spin(10.0, MAX_WORKSPACE_MM, self._draw_cfg["workspace_x_mm"])
        self.ws_y_spin = self._mm_spin(10.0, MAX_WORKSPACE_MM, self._draw_cfg["workspace_y_mm"])
        self.lift_spin = self._mm_spin(0.0, 60.0, self._draw_cfg["lift_mm"])
        self.zpaper_spin = self._mm_spin(0.0, 30.0, self._draw_cfg["z_paper_offset_mm"])
        form.addWidget(QLabel("Workspace X (mm)"), 0, 0)
        form.addWidget(self.ws_x_spin, 0, 1)
        form.addWidget(QLabel("Workspace Y (mm)"), 1, 0)
        form.addWidget(self.ws_y_spin, 1, 1)
        form.addWidget(QLabel("Pen lift (mm)"), 2, 0)
        form.addWidget(self.lift_spin, 2, 1)
        form.addWidget(QLabel("Z-paper offset (mm)"), 3, 0)
        form.addWidget(self.zpaper_spin, 3, 1)
        apply_btn = QPushButton("Apply")
        apply_btn.clicked.connect(self._apply_draw_cfg)
        form.addWidget(apply_btn, 4, 0, 1, 2)
        col.addWidget(cfg_box)

        warn = QLabel(
            f"Workspace clamped to ~{int(MAX_WORKSPACE_MM)} mm; lift/offset ≥ 0. "
            "Applying new settings sends the robot Home."
        )
        warn.setWordWrap(True)
        warn.setStyleSheet("color: #a60; font-size: 11px;")
        col.addWidget(warn)

        actions = QGridLayout()
        for i, (label, cb) in enumerate((
            ("Send", lambda: self.node.send_drawing(self._drawing_message())),
            ("Clear", self.canvas.clear),
            ("Resume", self.node.resend_last_drawing),
            ("Home", lambda: self.node.goto_preset("Home")),
        )):
            b = QPushButton(label)
            b.clicked.connect(cb)
            actions.addWidget(b, i // 2, i % 2)
        col.addLayout(actions)

        col.addStretch(1)
        outer.addWidget(panel)
        return w

    @staticmethod
    def _mm_spin(lo: float, hi: float, val: float) -> QDoubleSpinBox:
        s = QDoubleSpinBox()
        s.setRange(lo, hi)
        s.setDecimals(1)
        s.setSingleStep(1.0)
        s.setValue(val)
        return s

    def _apply_draw_cfg(self) -> None:
        self._draw_cfg["workspace_x_mm"] = self.ws_x_spin.value()
        self._draw_cfg["workspace_y_mm"] = self.ws_y_spin.value()
        self._draw_cfg["lift_mm"] = self.lift_spin.value()
        self._draw_cfg["z_paper_offset_mm"] = self.zpaper_spin.value()
        self.canvas.set_workspace(self._draw_cfg["workspace_x_mm"],
                                  self._draw_cfg["workspace_y_mm"])
        # New settings shift the drawing frame, so park at Home first.
        self.node.goto_preset("Home")

    def _drawing_message(self) -> dict:
        self._draw_cfg["lift_mm"] = self.lift_spin.value()
        self._draw_cfg["z_paper_offset_mm"] = self.zpaper_spin.value()
        msg = self.canvas.get_drawing()
        msg["config"] = dict(self._draw_cfg)
        return msg

    # ── settings page ──────────────────────────────────────────────────────
    def _build_settings_tab(self) -> QWidget:
        w = QWidget()
        outer = QVBoxLayout(w)

        sim_box = QGroupBox("Simulation backend")
        sim = QVBoxLayout(sim_box)
        intro = QLabel(
            "Choose the backend, then toggle 'Simulation' (top-right) to start "
            "it. 'Gazebo' is the physics sim; 'MoveIt demo' is lighter (fake "
            "hardware + MoveIt RViz). The 3D view opens as its own window."
        )
        intro.setWordWrap(True)
        sim.addWidget(intro)

        mode_row = QHBoxLayout()
        mode_row.addWidget(QLabel("Backend:"))
        self.mode_combo = QComboBox()
        self.mode_combo.addItems(["Gazebo (physics sim)", "MoveIt demo (fake hardware)"])
        mode_row.addWidget(self.mode_combo)
        mode_row.addStretch(1)
        sim.addLayout(mode_row)

        self.backend_status = QLabel("simulation: stopped")
        sim.addWidget(self.backend_status)
        outer.addWidget(sim_box)

        opt_box = QGroupBox("Targets")
        ov = QVBoxLayout(opt_box)
        self.info_joints_check = QCheckBox("Show target info as joints (else X/Y/Z)")
        ov.addWidget(self.info_joints_check)
        outer.addWidget(opt_box)

        info_box = QGroupBox("Environment")
        info = QVBoxLayout(info_box)
        ws = bootstrap.resolve_workspace()
        for text in (
            f"ROS distro:  {bootstrap.ros_distro()}",
            f"Workspace:   {ws or 'not found'}",
            f"Built:       {bootstrap.workspace_is_built(ws) if ws else False}",
        ):
            lbl = QLabel(text)
            lbl.setStyleSheet("font-family: monospace;")
            info.addWidget(lbl)
        outer.addWidget(info_box)
        outer.addStretch(1)
        return w

    # ── status page ──────────────────────────────────────────────────────
    def _build_status_tab(self) -> QWidget:
        w = QWidget()
        outer = QVBoxLayout(w)
        self.status_joint_label = QLabel("joints: —")
        self.status_joint_label.setStyleSheet("font-family: monospace;")
        self.status_joint_label.setWordWrap(True)
        self.status_ee_label = QLabel("ee: —")
        self.status_ee_label.setStyleSheet("font-family: monospace;")
        self.status_estop_label = QLabel("E-stop: clear")
        outer.addWidget(self.status_joint_label)
        outer.addWidget(self.status_ee_label)
        outer.addWidget(self.status_estop_label)
        outer.addStretch(1)
        return w

    # ── periodic refresh ───────────────────────────────────────────────────
    def _refresh_status(self) -> None:
        joints = self.node.get_joints()
        xyz = self.node.get_ee_xyz()
        jtxt = "  ".join(f"{n}={q:+.3f}" for n, q in zip(JOINT_NAMES, joints))
        if xyz is None:
            ee_txt = "ee: (waiting for /ee_pose)"
        else:
            ee_txt = f"ee: x={xyz[0]:+.3f}  y={xyz[1]:+.3f}  z={xyz[2]:+.3f} m"

        self.status_joint_label.setText("joints: " + jtxt)
        self.status_ee_label.setText(ee_txt)
        for lbl, name, q in zip(self.joint_info_labels, JOINT_NAMES, joints):
            lbl.setText(f"{name} = {q:+.3f}")
        self.ee_info_label.setText(ee_txt)

        # Joint-7 dial: flush a pending target (throttled to this 10 Hz tick),
        # otherwise track the live joint while the user isn't turning it.
        if self._dial7_pending is not None:
            if not self.node.estopped:
                self.node.set_joint(6, self._dial7_pending, duration_s=0.3)
            self._dial7_pending = None
        elif not self.dial7.isSliderDown():
            self.dial7.blockSignals(True)
            self.dial7.setValue(int(round(joints[6] * 100)))
            self.dial7.blockSignals(False)
            self.dial7_value_label.setText(f"{joints[6]:+.3f} rad")

        estopped = self.node.estopped
        self.status_estop_label.setText("E-stop: ACTIVE" if estopped else "E-stop: clear")
        self.header_estop_label.setText("E-stop: ACTIVE" if estopped else "E-stop: clear")
        self.header_estop_label.setStyleSheet("color: #f66;" if estopped else "color: #6f6;")

        if self.backend is not None:
            running = self.backend.running
            self.backend_status.setText(
                "simulation: running" if running else "simulation: stopped"
            )
            if not running and self.sim_btn.text().endswith("ON"):
                self._set_sim_btn(False)
                self.mode_combo.setEnabled(True)
