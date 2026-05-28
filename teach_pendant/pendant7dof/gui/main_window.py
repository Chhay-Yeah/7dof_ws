"""Main teach pendant window: tabbed jog / cartesian / drawing / status,
a persistent E-stop, and backend start/stop controls."""

from __future__ import annotations

from PyQt6.QtWidgets import (
    QMainWindow, QWidget, QTabWidget, QVBoxLayout, QHBoxLayout, QGridLayout,
    QPushButton, QLabel, QComboBox, QLineEdit, QDoubleSpinBox, QGroupBox,
    QSizePolicy, QCheckBox,
)
from PyQt6.QtCore import Qt, QTimer
from PyQt6.QtGui import QDoubleValidator

from .. import bootstrap
from ..ros_bridge import (
    PendantBridge, JOINT_NAMES, JOINT_LIMITS, PRESETS, JOG_STEP_RAD, CART_STEP_M,
)
from .drawing_canvas import CanvasView


class MainWindow(QMainWindow):
    def __init__(self, node: PendantBridge, backend) -> None:
        super().__init__()
        self.node = node
        self.backend = backend
        self.setWindowTitle("7-DOF Teach Pendant")
        self.resize(720, 880)

        root = QVBoxLayout()

        self.tabs = QTabWidget()
        self.tabs.addTab(self._build_jog_tab(), "Jog")
        self.tabs.addTab(self._build_cartesian_tab(), "Cartesian")
        self.tabs.addTab(self._build_drawing_tab(), "Drawing")
        self.tabs.addTab(self._build_status_tab(), "Status")
        self.tabs.addTab(self._build_settings_tab(), "Settings")
        root.addWidget(self.tabs, 1)

        root.addWidget(self._build_estop_bar())

        container = QWidget()
        container.setLayout(root)
        self.setCentralWidget(container)

        # Poll bridge state into the readouts at 10 Hz.
        self._poll = QTimer(self)
        self._poll.timeout.connect(self._refresh_status)
        self._poll.start(100)

    # ── settings tab: simulation / backend control ────────────────────────
    def _build_settings_tab(self) -> QWidget:
        w = QWidget()
        outer = QVBoxLayout()

        sim_box = QGroupBox("Simulation")
        sim = QVBoxLayout()

        intro = QLabel(
            "Launch the Gazebo simulation to test the pendant end to end. "
            "This brings up Gazebo + controllers + the IK/FK and drawing nodes "
            "so Jog, Cartesian and Drawing actually move the robot."
        )
        intro.setWordWrap(True)
        sim.addWidget(intro)

        self.gazebo_btn = QPushButton("▶  Launch Gazebo Simulation")
        self.gazebo_btn.setMinimumHeight(48)
        self.gazebo_btn.setStyleSheet(
            "background-color: #2e7d32; color: white; font-size: 16px; font-weight: bold;"
        )
        self.gazebo_btn.clicked.connect(self._toggle_backend)
        if self.backend is None:
            self.gazebo_btn.setEnabled(False)
            self.gazebo_btn.setText("Backend is external (--no-backend)")
        sim.addWidget(self.gazebo_btn)

        self.rviz_check = QCheckBox("Also open RViz")
        self.rviz_check.setChecked(True)
        sim.addWidget(self.rviz_check)

        self.backend_status = QLabel("simulation: stopped")
        sim.addWidget(self.backend_status)

        sim_box.setLayout(sim)
        outer.addWidget(sim_box)

        # Environment diagnostics (handy while testing).
        info_box = QGroupBox("Environment")
        info = QVBoxLayout()
        ws = bootstrap.resolve_workspace()
        for text in (
            f"ROS distro:  {bootstrap.ros_distro()}",
            f"Workspace:   {ws or 'not found'}",
            f"Built:       {bootstrap.workspace_is_built(ws) if ws else False}",
        ):
            lbl = QLabel(text)
            lbl.setStyleSheet("font-family: monospace;")
            info.addWidget(lbl)
        info_box.setLayout(info)
        outer.addWidget(info_box)

        outer.addStretch(1)
        w.setLayout(outer)
        return w

    def _toggle_backend(self) -> None:
        if self.backend is None:
            return
        if self.backend.running:
            self.backend.stop()
            self.gazebo_btn.setText("▶  Launch Gazebo Simulation")
            self.gazebo_btn.setStyleSheet(
                "background-color: #2e7d32; color: white; font-size: 16px; font-weight: bold;"
            )
            self.rviz_check.setEnabled(True)
        else:
            rviz_arg = "true" if self.rviz_check.isChecked() else "false"
            self.backend.start(extra_args=[f"rviz:={rviz_arg}", "gazebo:=true"])
            self.gazebo_btn.setText("■  Stop Gazebo Simulation")
            self.gazebo_btn.setStyleSheet(
                "background-color: #c62828; color: white; font-size: 16px; font-weight: bold;"
            )
            self.rviz_check.setEnabled(False)

    # ── jog tab ───────────────────────────────────────────────────────────
    def _build_jog_tab(self) -> QWidget:
        w = QWidget()
        outer = QVBoxLayout()

        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Jog step (rad):"))
        self.jog_step = QDoubleSpinBox()
        self.jog_step.setRange(0.005, 1.0)
        self.jog_step.setSingleStep(0.01)
        self.jog_step.setDecimals(3)
        self.jog_step.setValue(JOG_STEP_RAD)
        step_row.addWidget(self.jog_step)
        step_row.addStretch(1)
        outer.addLayout(step_row)

        grid = QGridLayout()
        grid.addWidget(QLabel("Joint"), 0, 0)
        grid.addWidget(QLabel("Position (rad)"), 0, 1)
        grid.addWidget(QLabel("Limits"), 0, 4)
        self.joint_value_labels: list[QLabel] = []
        for i, name in enumerate(JOINT_NAMES):
            grid.addWidget(QLabel(name), i + 1, 0)
            val = QLabel("+0.000")
            val.setStyleSheet("font-family: monospace;")
            self.joint_value_labels.append(val)
            grid.addWidget(val, i + 1, 1)

            minus = QPushButton("−")
            plus = QPushButton("+")
            minus.setFixedWidth(44)
            plus.setFixedWidth(44)
            minus.clicked.connect(lambda _, idx=i: self.node.jog_joint(idx, -self.jog_step.value()))
            plus.clicked.connect(lambda _, idx=i: self.node.jog_joint(idx, +self.jog_step.value()))
            grid.addWidget(minus, i + 1, 2)
            grid.addWidget(plus, i + 1, 3)

            lim = JOINT_LIMITS[i]
            lim_txt = "continuous" if lim is None else f"[{lim[0]:.2f}, {lim[1]:.2f}]"
            grid.addWidget(QLabel(lim_txt), i + 1, 4)
        outer.addLayout(grid)

        # Absolute set row
        set_box = QGroupBox("Set absolute target")
        set_row = QHBoxLayout()
        set_row.addWidget(QLabel("Joint:"))
        self.set_joint_combo = QComboBox()
        self.set_joint_combo.addItems(JOINT_NAMES)
        set_row.addWidget(self.set_joint_combo)
        self.set_joint_input = QLineEdit()
        self.set_joint_input.setPlaceholderText("rad")
        self.set_joint_input.setValidator(QDoubleValidator(-6.283, 6.283, 4))
        self.set_joint_input.returnPressed.connect(self._do_set_joint)
        set_row.addWidget(self.set_joint_input)
        set_btn = QPushButton("Set")
        set_btn.clicked.connect(self._do_set_joint)
        set_row.addWidget(set_btn)
        set_row.addStretch(1)
        set_box.setLayout(set_row)
        outer.addWidget(set_box)

        # Presets
        preset_box = QGroupBox("Presets")
        preset_row = QHBoxLayout()
        for name in PRESETS:
            b = QPushButton(name)
            b.clicked.connect(lambda _, n=name: self.node.goto_preset(n))
            preset_row.addWidget(b)
        preset_row.addStretch(1)
        preset_box.setLayout(preset_row)
        outer.addWidget(preset_box)

        outer.addStretch(1)
        w.setLayout(outer)
        return w

    def _do_set_joint(self) -> None:
        text = self.set_joint_input.text().strip()
        if not text:
            return
        try:
            target = float(text)
        except ValueError:
            return
        self.node.set_joint(self.set_joint_combo.currentIndex(), target)

    # ── cartesian tab ─────────────────────────────────────────────────────
    def _build_cartesian_tab(self) -> QWidget:
        w = QWidget()
        outer = QVBoxLayout()

        note = QLabel(
            "Cartesian jog nudges /ee_target from the live /ee_pose.\n"
            "Requires the IK + FK nodes (backend) to be running."
        )
        note.setWordWrap(True)
        outer.addWidget(note)

        step_row = QHBoxLayout()
        step_row.addWidget(QLabel("Step (m):"))
        self.cart_step = QDoubleSpinBox()
        self.cart_step.setRange(0.001, 0.2)
        self.cart_step.setSingleStep(0.005)
        self.cart_step.setDecimals(3)
        self.cart_step.setValue(CART_STEP_M)
        step_row.addWidget(self.cart_step)
        step_row.addStretch(1)
        outer.addLayout(step_row)

        grid = QGridLayout()
        for col, axis in enumerate(("x", "y", "z")):
            grid.addWidget(QLabel(f"{axis.upper()}"), 0, col, alignment=Qt.AlignmentFlag.AlignCenter)
            plus = QPushButton(f"+{axis.upper()}")
            minus = QPushButton(f"−{axis.upper()}")
            plus.clicked.connect(lambda _, a=axis: self.node.cartesian_jog(a, +self.cart_step.value()))
            minus.clicked.connect(lambda _, a=axis: self.node.cartesian_jog(a, -self.cart_step.value()))
            grid.addWidget(plus, 1, col)
            grid.addWidget(minus, 2, col)
        outer.addLayout(grid)

        self.ee_label = QLabel("EE: (waiting for /ee_pose)")
        self.ee_label.setStyleSheet("font-family: monospace;")
        outer.addWidget(self.ee_label)

        outer.addStretch(1)
        w.setLayout(outer)
        return w

    # ── drawing tab ───────────────────────────────────────────────────────
    def _build_drawing_tab(self) -> QWidget:
        w = QWidget()
        outer = QVBoxLayout()
        self.canvas = CanvasView()
        self.canvas.setSizePolicy(QSizePolicy.Policy.Expanding, QSizePolicy.Policy.Expanding)
        self.node.attach_pen_callback(self.canvas.set_pen_pos)
        outer.addWidget(self.canvas, 1)

        row = QHBoxLayout()
        send = QPushButton("Send to Robot")
        send.clicked.connect(lambda: self.node.send_drawing(self.canvas.get_drawing()))
        clear = QPushButton("Clear")
        clear.clicked.connect(self.canvas.clear)
        resume = QPushButton("Resume Last")
        resume.clicked.connect(self.node.resend_last_drawing)
        for b in (send, clear, resume):
            row.addWidget(b)
        outer.addLayout(row)

        w.setLayout(outer)
        return w

    # ── status tab ────────────────────────────────────────────────────────
    def _build_status_tab(self) -> QWidget:
        w = QWidget()
        outer = QVBoxLayout()
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
        w.setLayout(outer)
        return w

    # ── E-stop bar ────────────────────────────────────────────────────────
    def _build_estop_bar(self) -> QWidget:
        bar = QWidget()
        row = QHBoxLayout()
        self.estop_btn = QPushButton("E-STOP")
        self.estop_btn.setMinimumHeight(56)
        self.estop_btn.setStyleSheet(
            "background-color: #d33; color: white; font-size: 20px; font-weight: bold;"
        )
        self.estop_btn.clicked.connect(self._toggle_estop)
        row.addWidget(self.estop_btn)
        bar.setLayout(row)
        return bar

    def _toggle_estop(self) -> None:
        if self.node.estopped:
            self.node.estop_reset()
            self.estop_btn.setText("E-STOP")
            self.estop_btn.setStyleSheet(
                "background-color: #d33; color: white; font-size: 20px; font-weight: bold;"
            )
        else:
            self.node.estop()
            self.estop_btn.setText("RESET (E-stop active)")
            self.estop_btn.setStyleSheet(
                "background-color: #555; color: #f88; font-size: 18px; font-weight: bold;"
            )

    # ── periodic status refresh ───────────────────────────────────────────
    def _refresh_status(self) -> None:
        joints = self.node.get_joints()
        for lbl, q in zip(self.joint_value_labels, joints):
            lbl.setText(f"{q:+.3f}")

        xyz = self.node.get_ee_xyz()
        if xyz is None:
            ee_txt = "EE: (waiting for /ee_pose)"
        else:
            ee_txt = f"EE: x={xyz[0]:+.3f}  y={xyz[1]:+.3f}  z={xyz[2]:+.3f} m"
        self.ee_label.setText(ee_txt)

        self.status_joint_label.setText(
            "joints: " + "  ".join(f"{n}={q:+.3f}" for n, q in zip(JOINT_NAMES, joints))
        )
        self.status_ee_label.setText(ee_txt.replace("EE:", "ee:"))
        self.status_estop_label.setText(
            "E-stop: ACTIVE" if self.node.estopped else "E-stop: clear"
        )

        if self.backend is not None:
            running = self.backend.running
            self.backend_status.setText(
                "simulation: running" if running else "simulation: stopped"
            )
            # If the launch exited on its own (crash / Ctrl+C in its terminal),
            # snap the button back to the launch state.
            if not running and self.gazebo_btn.text().startswith("■"):
                self.gazebo_btn.setText("▶  Launch Gazebo Simulation")
                self.gazebo_btn.setStyleSheet(
                    "background-color: #2e7d32; color: white; font-size: 16px; font-weight: bold;"
                )
                self.rviz_check.setEnabled(True)
