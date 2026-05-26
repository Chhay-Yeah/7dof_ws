#!/usr/bin/env python3
# drawing_ui_node.py
import sys, json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray, Point
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from PyQt6.QtWidgets import (QApplication, QMainWindow, QGraphicsView,
                              QGraphicsScene, QPushButton, QVBoxLayout, QWidget,
                              QHBoxLayout, QComboBox, QLabel, QLineEdit,
                              QGraphicsEllipseItem)
from PyQt6.QtCore import QTimer, Qt, pyqtSignal
from PyQt6.QtGui import QPen, QPainterPath, QDoubleValidator, QColor, QBrush


HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4',
               'joint_5', 'joint_6', 'joint_7']
JOG_STEP_RAD = 0.1   # increment per D-pad press
JOG_DURATION_S = 0.5 # how long the controller takes to reach the new target

# Canvas dimensions in pixels. With a 1 : 1 px → mm mapping in the planner
# (workspace_x_mm = workspace_y_mm = 500), one pixel here equals one
# millimetre on the paper, so the canvas is also the literal drawing target.
CANVAS_W = 60
CANVAS_H = 60

# Soft joint limits — kept just inside the URDF hard limits with a safety
# margin. The velocity-mode JTC running on ign_ros2_control gets stuck if
# Gazebo clamps a joint at the hard limit, so we never command past these.
# `None` = continuous joint (no limit).
LIMIT_MARGIN_RAD = 0.05
JOINT_LIMITS = [
    None,            # joint_1 — continuous
    (-1.6, 1.6),     # joint_2
    None,            # joint_3 — continuous
    (-1.6, 1.6),     # joint_4
    None,            # joint_5 — continuous
    (-0.48, 0.26),   # joint_6 — narrow range, watch out
    (-1.6, 1.6),     # joint_7
]


def _clamp_joint(idx: int, value: float) -> float:
    lim = JOINT_LIMITS[idx]
    if lim is None:
        return value
    lo, hi = lim
    return max(lo + LIMIT_MARGIN_RAD, min(hi - LIMIT_MARGIN_RAD, value))

class DrawingNode(Node):
    def __init__(self):
        super().__init__('drawing_ui_node')
        self.publisher = self.create_publisher(String, 'drawing/strokes', 10)
        self.path_pub = self.create_publisher(PoseArray, '/cartesian_path', 10)
        self.traj_pub = self.create_publisher(
            JointTrajectory, '/arm_controller/joint_trajectory', 10)

        # Track latest joint positions so jog commands start from where the robot is
        self._last_q = list(HOME_JOINTS)
        self.create_subscription(JointState, '/joint_states', self._cb_state, 20)

        # Live pen-tip position (normalized canvas coords) from the planner.
        # The MainWindow wires its slot in attach_pen_callback().
        self._pen_cb = None
        self.create_subscription(Point, '/pen_canvas_norm', self._cb_pen, 30)

        # Remember the most recently sent strokes JSON so Resume can re-publish it.
        self._last_drawing: str | None = None

        self.get_logger().info('Drawing UI node ready')

    def attach_pen_callback(self, cb):
        """Set the callback invoked on every /pen_canvas_norm message.
        Called by MainWindow once the canvas dot exists."""
        self._pen_cb = cb

    def _cb_pen(self, msg: Point):
        if self._pen_cb is not None:
            # msg.x, msg.y in [0..1]; msg.z = mm above paper
            self._pen_cb(float(msg.x), float(msg.y), float(msg.z))

    def _cb_state(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        for jn, p in zip(msg.name, msg.position):
            if jn in JOINT_NAMES:
                self._last_q[JOINT_NAMES.index(jn)] = float(p)

    def set_joint(self, joint_idx: int, target_rad: float, duration_s: float = 1.5):
        empty = PoseArray()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = 'base_link'
        self.path_pub.publish(empty)

        clamped = _clamp_joint(joint_idx, float(target_rad))
        if clamped != float(target_rad):
            self.get_logger().warn(
                f'{JOINT_NAMES[joint_idx]}: {target_rad:+.3f} clamped to '
                f'{clamped:+.3f} (URDF limit + {LIMIT_MARGIN_RAD} margin)'
            )

        q = list(self._last_q)
        q[joint_idx] = clamped

        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = q
        sec = int(duration_s)
        nsec = int((duration_s - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        self.traj_pub.publish(traj)

        self._last_q = q
        self.get_logger().info(
            f'SET {JOINT_NAMES[joint_idx]} → {clamped:+.3f} rad')

    def get_joint(self, joint_idx: int) -> float:
        return float(self._last_q[joint_idx])

    def jog_joint(self, joint_idx: int, delta: float):
        # Stop any drawing in progress so its IK output doesn't fight us
        empty = PoseArray()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = 'base_link'
        self.path_pub.publish(empty)

        q = list(self._last_q)
        q[joint_idx] = _clamp_joint(joint_idx, q[joint_idx] + delta)

        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = q
        sec = int(JOG_DURATION_S)
        nsec = int((JOG_DURATION_S - sec) * 1e9)
        pt.time_from_start = Duration(sec=sec, nanosec=nsec)
        traj.points.append(pt)
        self.traj_pub.publish(traj)

        self._last_q = q
        self.get_logger().info(
            f'JOG {JOINT_NAMES[joint_idx]} {delta:+.2f} → {q[joint_idx]:+.3f} rad')

    def send_drawing(self, strokes_dict):
        msg = String()
        msg.data = json.dumps(strokes_dict)
        self.publisher.publish(msg)
        # Remember so Resume can replay this when un-pausing.
        self._last_drawing = msg.data
        self.get_logger().info(f'Published drawing: {len(strokes_dict["strokes"])} strokes')

    def send_home(self):
        # 1) Clear any in-flight drawing path so the executor stops feeding /ee_target.
        empty = PoseArray()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = 'base_link'
        self.path_pub.publish(empty)

        # 2) Send the home trajectory directly to the controller. Because the
        #    JointTrajectoryController preempts on new goal, this aborts the
        #    current motion and drives all 7 joints to zero over 2 s.
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(HOME_JOINTS)
        pt.time_from_start = Duration(sec=2, nanosec=0)
        traj.points.append(pt)
        self.traj_pub.publish(traj)

        self.get_logger().info('HOME — cleared path queue, commanded zeros')

    def send_freeze(self):
        """Send a single-waypoint trajectory at the robot's CURRENT joint
        position. JTC preempts whatever it was doing and holds the arm
        still. Used by both Pause and Reset."""
        traj = JointTrajectory()
        traj.joint_names = list(JOINT_NAMES)
        pt = JointTrajectoryPoint()
        pt.positions = list(self._last_q)
        # 0.1 s is enough for JTC to accept; the goal is identity so the
        # arm stops in place.
        pt.time_from_start = Duration(sec=0, nanosec=100_000_000)
        traj.points.append(pt)
        self.traj_pub.publish(traj)

    def resend_last_drawing(self):
        """Re-publish the most recently sent strokes JSON so the planner
        replans from the robot's current state. Effectively 'resume' — the
        approach + drawing phases run again from wherever the robot is."""
        if self._last_drawing is None:
            self.get_logger().warn('No previous drawing to resume')
            return
        msg = String()
        msg.data = self._last_drawing
        self.publisher.publish(msg)
        self.get_logger().info('Resume — re-sent last drawing')

class CanvasView(QGraphicsView):
    def __init__(self, on_stroke_update):
        super().__init__()
        # Scene coordinates are in canvas pixels (=== millimetres because the
        # planner uses a 1 : 1 px → mm mapping). The view stretches the scene
        # to fill the widget so the drawing stays large enough to be usable
        # while the published JSON stays in 0..CANVAS_W × 0..CANVAS_H space.
        self.scene_ = QGraphicsScene(0, 0, CANVAS_W, CANVAS_H)
        self.setScene(self.scene_)
        self.setRenderHints(self.renderHints())
        self.setSceneRect(0, 0, CANVAS_W, CANVAS_H)
        self.setHorizontalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setVerticalScrollBarPolicy(Qt.ScrollBarPolicy.ScrollBarAlwaysOff)
        self.setMinimumSize(200, 200)
        self.strokes = []
        self.current_path = None
        self.current_points = []
        self.on_stroke_update = on_stroke_update
        self._t0 = None

        # Skyblue dot tracking the live pen-tip position from the planner.
        # Stored as a separate Python attribute so we can detect when
        # scene_.clear() has destroyed the underlying C++ object and
        # recreate it. ~2 px on a 60 px canvas keeps it visible.
        self.pen_dot = None
        self._pen_last_pos = None  # (x_px, y_px) so we can restore after a clear
        self._ensure_pen_dot()

    def _ensure_pen_dot(self):
        """Create the skyblue pen dot and add it to the scene. Call after
        any scene_.clear() because Qt deletes the underlying C++ object,
        leaving the Python wrapper invalid."""
        # ~0.6 px on a 60 px canvas (2.5× smaller than the previous 2 px).
        dot_r = max(0.4, CANVAS_W * 0.01)
        self.pen_dot = QGraphicsEllipseItem(-dot_r, -dot_r, 2 * dot_r, 2 * dot_r)
        self.pen_dot.setBrush(QBrush(QColor(135, 206, 235)))  # skyblue
        self.pen_dot.setPen(QPen(Qt.PenStyle.NoPen))
        # Very high Z so the dot always renders on top of strokes (Z=0
        # by default) regardless of insertion order in the scene.
        self.pen_dot.setZValue(1000.0)
        self.scene_.addItem(self.pen_dot)
        # Restore last known position if we had one, else hide.
        if self._pen_last_pos is not None:
            self.pen_dot.setPos(*self._pen_last_pos)
            self.pen_dot.setVisible(True)
        else:
            self.pen_dot.setVisible(False)

    def set_pen_pos(self, norm_x: float, norm_y: float, _z_mm: float):
        """Update the tracking dot from normalized canvas coords [0..1]^2.
        The planner publishes (norm_x = right, norm_y = up); we map to
        scene coords with Y flipped because Qt scenes have Y growing down."""
        if not (0.0 <= norm_x <= 1.0 and 0.0 <= norm_y <= 1.0):
            self._pen_last_pos = None
            self.pen_dot.setVisible(False)
            return
        x_px = norm_x * CANVAS_W
        y_px = (1.0 - norm_y) * CANVAS_H
        self._pen_last_pos = (x_px, y_px)
        self.pen_dot.setPos(x_px, y_px)
        self.pen_dot.setVisible(True)

    @staticmethod
    def _cosmetic_pen():
        pen = QPen(Qt.GlobalColor.black, 2)
        pen.setCosmetic(True)
        return pen

    def resizeEvent(self, event):
        # Rescale the scene to fit the new widget size, preserving 1 : 1
        # aspect so the drawing never gets squashed.
        super().resizeEvent(event)
        self.fitInView(self.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def showEvent(self, event):
        super().showEvent(event)
        self.fitInView(self.sceneRect(), Qt.AspectRatioMode.KeepAspectRatio)

    def mousePressEvent(self, event):
        import time
        if self._t0 is None: self._t0 = time.time()
        self.current_points = []
        p = self.mapToScene(event.pos())
        self.current_path = QPainterPath(p)
        self.current_points.append({
            "x": p.x(), "y": p.y(), "t": time.time() - self._t0, "p": 0.5
        })

    def mouseMoveEvent(self, event):
        import time
        if self.current_path is None: return
        p = self.mapToScene(event.pos())
        self.current_path.lineTo(p)
        self.scene_.clear()
        for s in self.strokes:
            self.scene_.addPath(s["qpath"], self._cosmetic_pen())
        self.scene_.addPath(self.current_path, self._cosmetic_pen())
        # scene_.clear() destroyed the pen dot's C++ object — recreate.
        self._ensure_pen_dot()
        self.current_points.append({
            "x": p.x(), "y": p.y(), "t": time.time() - self._t0, "p": 0.5
        })

    def mouseReleaseEvent(self, event):
        if self.current_path is None: return
        self.strokes.append({
            "qpath": self.current_path,
            "points": self.current_points
        })
        self.current_path = None

    def get_drawing(self):
        return {
            "canvas": {"width": CANVAS_W, "height": CANVAS_H, "units": "px"},
            "strokes": [
                {"id": i, "points": s["points"]}
                for i, s in enumerate(self.strokes)
            ]
        }

    def clear(self):
        self.strokes = []
        self.scene_.clear()
        # scene_.clear() destroyed the pen dot's C++ object — recreate.
        self._ensure_pen_dot()
        self._t0 = None

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle('Robot Drawing Pad')
        self.ros_node = ros_node
        self.canvas = CanvasView(self.on_stroke_update)
        # Forward planner's pen-position broadcasts to the canvas dot.
        # The ROS executor runs on the Qt event-loop pumper below, so
        # this is called on the GUI thread — safe to update items directly.
        self.ros_node.attach_pen_callback(self.canvas.set_pen_pos)

        send_btn = QPushButton('Send to Robot')
        send_btn.clicked.connect(self.send)
        clear_btn = QPushButton('Clear')
        clear_btn.clicked.connect(self.canvas.clear)

        # Pause/Resume — toggles between freezing the robot in place and
        # re-sending the last drawing so it picks up from where it stopped.
        self._is_paused = False
        self.pause_btn = QPushButton('Pause')
        self.pause_btn.setStyleSheet('background-color: #e8a317; color: white; font-weight: bold;')
        self.pause_btn.clicked.connect(self.pause_or_resume)

        # Reset — cancels the in-flight drawing task (freezes the robot and
        # clears the canvas). Does NOT move the robot to home.
        reset_btn = QPushButton('Reset')
        reset_btn.setStyleSheet('background-color: #888; color: white; font-weight: bold;')
        reset_btn.clicked.connect(self.reset)

        # Home — drives all 7 joints back to 0.
        home_btn = QPushButton('HOME')
        home_btn.setStyleSheet('background-color: #d33; color: white; font-weight: bold;')
        home_btn.clicked.connect(self.home)

        # ── Top-left D-pad (2 buttons) + joint selector ───────────────────
        up_btn   = QPushButton('▲')
        down_btn = QPushButton('▼')
        for b in (up_btn, down_btn):
            b.setFixedSize(48, 48)
            b.setStyleSheet('font-size: 18px; font-weight: bold;')
        up_btn.clicked.connect(lambda: self.jog(+JOG_STEP_RAD))
        down_btn.clicked.connect(lambda: self.jog(-JOG_STEP_RAD))

        dpad = QVBoxLayout()
        dpad.setSpacing(2)
        dpad.addWidget(up_btn)
        dpad.addWidget(down_btn)
        dpad_container = QWidget(); dpad_container.setLayout(dpad)

        self.joint_select = QComboBox()
        self.joint_select.addItems(JOINT_NAMES)
        self.joint_select.setFixedWidth(100)
        self.joint_select.currentIndexChanged.connect(self._refresh_joint_input)

        top_bar = QHBoxLayout()
        top_bar.addWidget(dpad_container)
        top_bar.addWidget(QLabel('Joint:'))
        top_bar.addWidget(self.joint_select)
        top_bar.addStretch(1)
        top_bar_container = QWidget(); top_bar_container.setLayout(top_bar)

        # ── Manual joint target (absolute, in radians) ────────────────────
        self.joint_input = QLineEdit()
        self.joint_input.setPlaceholderText('rad')
        self.joint_input.setFixedWidth(120)
        # Allow scientific or decimal; permissive bounds — controller will
        # clamp at hardware limits anyway.
        validator = QDoubleValidator(-6.283, 6.283, 4)
        validator.setNotation(QDoubleValidator.Notation.StandardNotation)
        self.joint_input.setValidator(validator)
        self.joint_input.returnPressed.connect(self.set_joint_target)
        set_btn = QPushButton('Set')
        set_btn.clicked.connect(self.set_joint_target)

        set_row = QHBoxLayout()
        set_row.addWidget(QLabel('Target (rad):'))
        set_row.addWidget(self.joint_input)
        set_row.addWidget(set_btn)
        set_row.addStretch(1)
        set_row_container = QWidget(); set_row_container.setLayout(set_row)

        # Bottom-row control buttons: Pause/Resume | Reset | Home
        ctrl_row = QHBoxLayout()
        ctrl_row.addWidget(self.pause_btn)
        ctrl_row.addWidget(reset_btn)
        ctrl_row.addWidget(home_btn)
        ctrl_row_container = QWidget(); ctrl_row_container.setLayout(ctrl_row)

        layout = QVBoxLayout()
        layout.addWidget(top_bar_container)
        layout.addWidget(set_row_container)
        layout.addWidget(self.canvas)
        layout.addWidget(send_btn)
        layout.addWidget(clear_btn)
        layout.addWidget(ctrl_row_container)
        container = QWidget(); container.setLayout(layout)
        self.setCentralWidget(container)
        # Generous default window size; the canvas stretches to fill
        # whatever space the user gives it (CanvasView.resizeEvent calls
        # fitInView). The underlying scene stays 150 × 150 — only the
        # visual zoom changes.
        self.resize(900, 1000)

        self._refresh_joint_input(0)

    def on_stroke_update(self): pass

    def send(self):
        self.ros_node.send_drawing(self.canvas.get_drawing())

    def home(self):
        self.ros_node.send_home()
        # Coming home implicitly ends a pause cycle.
        self._is_paused = False
        self.pause_btn.setText('Pause')

    def pause_or_resume(self):
        """Toggle between freezing the robot (Pause) and re-sending the
        last drawing so it picks up from the current position (Resume).
        Note: 'resume' replans from the robot's current state — it does
        NOT continue the original trajectory exactly, but the planner will
        approach + draw again from wherever the robot was when paused."""
        if not self._is_paused:
            self.ros_node.send_freeze()
            self._is_paused = True
            self.pause_btn.setText('Resume')
        else:
            self.ros_node.resend_last_drawing()
            self._is_paused = False
            self.pause_btn.setText('Pause')

    def reset(self):
        """Cancel the in-flight drawing task: freeze the robot in place.
        Does NOT clear the canvas (use the Clear button for that) and
        does NOT move the robot to home (use HOME for that)."""
        self.ros_node.send_freeze()
        self._is_paused = False
        self.pause_btn.setText('Pause')

    def jog(self, delta: float):
        idx = self.joint_select.currentIndex()
        self.ros_node.jog_joint(idx, delta)

    def set_joint_target(self):
        text = self.joint_input.text().strip()
        if not text:
            return
        try:
            target = float(text)
        except ValueError:
            return
        idx = self.joint_select.currentIndex()
        self.ros_node.set_joint(idx, target)

    def _refresh_joint_input(self, idx: int):
        self.joint_input.setText(f'{self.ros_node.get_joint(idx):+.3f}')

def main():
    rclpy.init()
    ros_node = DrawingNode()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    # The bridge: pump rclpy from Qt's event loop. On Ctrl+C, rclpy's signal
    # handler tears down the context before Qt's loop exits — without the
    # rclpy.ok() guard, every subsequent tick would raise an RCLError
    # because the wait set can't be built on a shut-down context.
    timer = QTimer()

    def _tick():
        if not rclpy.ok():
            timer.stop()
            app.quit()
            return
        try:
            rclpy.spin_once(ros_node, timeout_sec=0)
        except Exception:
            timer.stop()
            app.quit()

    timer.timeout.connect(_tick)
    timer.start(10)  # 100 Hz

    exit_code = app.exec()
    timer.stop()
    if rclpy.ok():
        ros_node.destroy_node()
        rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()