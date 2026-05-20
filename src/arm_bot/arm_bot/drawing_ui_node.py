#!/usr/bin/env python3
# drawing_ui_node.py
import sys, json
import rclpy
from rclpy.node import Node
from std_msgs.msg import String
from sensor_msgs.msg import JointState
from geometry_msgs.msg import PoseArray
from trajectory_msgs.msg import JointTrajectory, JointTrajectoryPoint
from builtin_interfaces.msg import Duration
from PyQt6.QtWidgets import (QApplication, QMainWindow, QGraphicsView,
                              QGraphicsScene, QPushButton, QVBoxLayout, QWidget,
                              QHBoxLayout, QComboBox, QLabel)
from PyQt6.QtCore import QTimer, Qt, QPointF
from PyQt6.QtGui import QPen, QPainterPath


HOME_JOINTS = [0.0, 0.0, 0.0, 0.0, 0.0, 0.0, 0.0]
JOINT_NAMES = ['joint_1', 'joint_2', 'joint_3', 'joint_4',
               'joint_5', 'joint_6', 'joint_7']
JOG_STEP_RAD = 0.1   # increment per D-pad press
JOG_DURATION_S = 0.5 # how long the controller takes to reach the new target

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

        self.get_logger().info('Drawing UI node ready')

    def _cb_state(self, msg: JointState):
        if not msg.name or not msg.position:
            return
        for jn, p in zip(msg.name, msg.position):
            if jn in JOINT_NAMES:
                self._last_q[JOINT_NAMES.index(jn)] = float(p)

    def jog_joint(self, joint_idx: int, delta: float):
        # Stop any drawing in progress so its IK output doesn't fight us
        empty = PoseArray()
        empty.header.stamp = self.get_clock().now().to_msg()
        empty.header.frame_id = 'base_link'
        self.path_pub.publish(empty)

        q = list(self._last_q)
        q[joint_idx] = q[joint_idx] + delta

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

class CanvasView(QGraphicsView):
    def __init__(self, on_stroke_update):
        super().__init__()
        self.scene_ = QGraphicsScene(0, 0, 800, 600)
        self.setScene(self.scene_)
        self.setRenderHints(self.renderHints())
        self.strokes = []
        self.current_path = None
        self.current_points = []
        self.on_stroke_update = on_stroke_update
        self._t0 = None

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
            self.scene_.addPath(s["qpath"], QPen(Qt.GlobalColor.black, 2))
        self.scene_.addPath(self.current_path, QPen(Qt.GlobalColor.black, 2))
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
            "canvas": {"width": 800, "height": 600, "units": "px"},
            "strokes": [
                {"id": i, "points": s["points"]}
                for i, s in enumerate(self.strokes)
            ]
        }

    def clear(self):
        self.strokes = []
        self.scene_.clear()
        self._t0 = None

class MainWindow(QMainWindow):
    def __init__(self, ros_node):
        super().__init__()
        self.setWindowTitle('Robot Drawing Pad')
        self.ros_node = ros_node
        self.canvas = CanvasView(self.on_stroke_update)

        send_btn = QPushButton('Send to Robot')
        send_btn.clicked.connect(self.send)
        clear_btn = QPushButton('Clear')
        clear_btn.clicked.connect(self.canvas.clear)
        home_btn = QPushButton('HOME (stop and reset)')
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

        top_bar = QHBoxLayout()
        top_bar.addWidget(dpad_container)
        top_bar.addWidget(QLabel('Joint:'))
        top_bar.addWidget(self.joint_select)
        top_bar.addStretch(1)
        top_bar_container = QWidget(); top_bar_container.setLayout(top_bar)

        layout = QVBoxLayout()
        layout.addWidget(top_bar_container)
        layout.addWidget(self.canvas)
        layout.addWidget(send_btn)
        layout.addWidget(clear_btn)
        layout.addWidget(home_btn)
        container = QWidget(); container.setLayout(layout)
        self.setCentralWidget(container)
        self.resize(820, 760)

    def on_stroke_update(self): pass

    def send(self):
        self.ros_node.send_drawing(self.canvas.get_drawing())

    def home(self):
        self.ros_node.send_home()

    def jog(self, delta: float):
        idx = self.joint_select.currentIndex()
        self.ros_node.jog_joint(idx, delta)

def main():
    rclpy.init()
    ros_node = DrawingNode()

    app = QApplication(sys.argv)
    window = MainWindow(ros_node)
    window.show()

    # The bridge: pump rclpy from Qt's event loop
    timer = QTimer()
    timer.timeout.connect(lambda: rclpy.spin_once(ros_node, timeout_sec=0))
    timer.start(10)  # 100 Hz

    exit_code = app.exec()
    ros_node.destroy_node()
    rclpy.shutdown()
    sys.exit(exit_code)

if __name__ == '__main__':
    main()