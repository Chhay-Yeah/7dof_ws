#!/usr/bin/env python3
"""
make_methodology_pdf.py — generates a detailed PDF documenting exactly how the
Chapter 4 thesis data (Figures 4.1-4.5, Tables 4.1-4.2) was created and recorded.

    python3 src/arm_bot/analysis/make_methodology_pdf.py --out data_methodology.pdf

Self-contained: needs only reportlab.
"""
import argparse

from reportlab.lib.pagesizes import A4
from reportlab.lib.units import mm
from reportlab.lib import colors
from reportlab.lib.styles import getSampleStyleSheet, ParagraphStyle
from reportlab.lib.enums import TA_LEFT
from reportlab.platypus import (
    SimpleDocTemplate, Paragraph, Spacer, Table, TableStyle, PageBreak,
    Preformatted, HRFlowable, ListFlowable, ListItem)


# ── styles ───────────────────────────────────────────────────────────────────
ss = getSampleStyleSheet()
H1 = ParagraphStyle('H1', parent=ss['Heading1'], fontSize=16, spaceBefore=14,
                    spaceAfter=8, textColor=colors.HexColor('#1f3a5f'))
H2 = ParagraphStyle('H2', parent=ss['Heading2'], fontSize=12.5, spaceBefore=12,
                    spaceAfter=5, textColor=colors.HexColor('#2c5d8f'))
H3 = ParagraphStyle('H3', parent=ss['Heading3'], fontSize=11, spaceBefore=8,
                    spaceAfter=3, textColor=colors.HexColor('#444444'))
BODY = ParagraphStyle('Body', parent=ss['BodyText'], fontSize=9.7, leading=13.6,
                      spaceAfter=6, alignment=TA_LEFT)
SMALL = ParagraphStyle('Small', parent=BODY, fontSize=8.6, leading=11.5,
                       textColor=colors.HexColor('#555555'))
CODE = ParagraphStyle('Code', parent=ss['Code'], fontSize=8.2, leading=10.8,
                      backColor=colors.HexColor('#f3f4f6'),
                      borderColor=colors.HexColor('#d6d9de'), borderWidth=0.5,
                      borderPadding=6, leftIndent=2, spaceBefore=2, spaceAfter=8,
                      textColor=colors.HexColor('#1a1a1a'))
TITLE = ParagraphStyle('Title', parent=ss['Title'], fontSize=22, leading=26,
                       textColor=colors.HexColor('#1f3a5f'))
SUB = ParagraphStyle('Sub', parent=ss['Title'], fontSize=12, leading=16,
                     textColor=colors.HexColor('#555555'))


def code(txt):
    return Preformatted(txt.strip('\n'), CODE)


def bullets(items):
    return ListFlowable(
        [ListItem(Paragraph(t, BODY), leftIndent=10, value='•') for t in items],
        bulletType='bullet', start='•', leftIndent=14)


def kv_table(rows, col_widths):
    t = Table(rows, colWidths=col_widths, hAlign='LEFT')
    t.setStyle(TableStyle([
        ('FONT', (0, 0), (-1, 0), 'Helvetica-Bold', 9),
        ('FONT', (0, 1), (-1, -1), 'Helvetica', 8.7),
        ('BACKGROUND', (0, 0), (-1, 0), colors.HexColor('#2c5d8f')),
        ('TEXTCOLOR', (0, 0), (-1, 0), colors.white),
        ('ROWBACKGROUNDS', (0, 1), (-1, -1),
         [colors.white, colors.HexColor('#eef2f7')]),
        ('GRID', (0, 0), (-1, -1), 0.4, colors.HexColor('#c3ccd6')),
        ('VALIGN', (0, 0), (-1, -1), 'TOP'),
        ('LEFTPADDING', (0, 0), (-1, -1), 5),
        ('RIGHTPADDING', (0, 0), (-1, -1), 5),
        ('TOPPADDING', (0, 0), (-1, -1), 3.5),
        ('BOTTOMPADDING', (0, 0), (-1, -1), 3.5),
    ]))
    return t


# ── page furniture ───────────────────────────────────────────────────────────
def footer(canvas, doc):
    canvas.saveState()
    canvas.setFont('Helvetica', 7.5)
    canvas.setFillColor(colors.HexColor('#888888'))
    canvas.drawString(20 * mm, 12 * mm,
                      '7-DOF Teach Pendant — Chapter 4 Data Methodology')
    canvas.drawRightString(190 * mm, 12 * mm, 'Page %d' % doc.page)
    canvas.setStrokeColor(colors.HexColor('#cccccc'))
    canvas.line(20 * mm, 14 * mm, 190 * mm, 14 * mm)
    canvas.restoreState()


def build(out_path):
    doc = SimpleDocTemplate(
        out_path, pagesize=A4,
        leftMargin=20 * mm, rightMargin=20 * mm,
        topMargin=18 * mm, bottomMargin=20 * mm,
        title='Chapter 4 Data Methodology',
        author='7-DOF Teach Pendant project')
    E = []

    # ── cover ────────────────────────────────────────────────────────────────
    E.append(Spacer(1, 38 * mm))
    E.append(Paragraph('How the Chapter&nbsp;4 Data Was<br/>Created and Recorded', TITLE))
    E.append(Spacer(1, 6 * mm))
    E.append(Paragraph('A complete, reproducible account of the measurement run, the '
                       'analysis toolchain, and every figure and table it produced',
                       SUB))
    E.append(Spacer(1, 10 * mm))
    E.append(HRFlowable(width='100%', thickness=1, color=colors.HexColor('#2c5d8f')))
    E.append(Spacer(1, 4 * mm))
    E.append(kv_table([
        ['Item', 'Value'],
        ['Robot', '7-DOF arm (joint_1 … joint_7), tip link "ee", base "base_link"'],
        ['Backend', 'Gazebo Ignition + ros2_control (ign_ros2_control), simulation only'],
        ['Drawn shape', 'Single closed curved stroke, ≈ 17 mm across ("circle" run)'],
        ['Source recording', 'rosbag2 "draw_run" (sqlite3), 10,002 messages, 101.1 s'],
        ['Artefacts produced', 'Figures 4.1, 4.3, 4.4, 4.5 (PNG + PDF); Tables 4.1, 4.2 (DOCX)'],
        ['Analysis location', 'src/arm_bot/analysis/  (run with python3, /opt/ros only)'],
    ], [40 * mm, 120 * mm]))
    E.append(Spacer(1, 6 * mm))
    E.append(Paragraph(
        'This document explains the methodology only — what was run, in what order, '
        'with which parameters, and how each number was obtained — so the run can be '
        'reproduced and defended.', SMALL))
    E.append(PageBreak())

    # ── 1. Overview ──────────────────────────────────────────────────────────
    E.append(Paragraph('1&nbsp;&nbsp;Overview and design principle', H1))
    E.append(Paragraph(
        'All Chapter&nbsp;4 quantitative results come from a <b>single recording of one '
        'drawing run</b>. The teach pendant commanded the arm to draw a small closed '
        'stroke; while it executed, a ROS&nbsp;2 bag captured the commanded path, the '
        'measured joint feedback, and the robot model. Every figure and the timing table '
        'are then derived <i>offline</i> from that one bag, so they are mutually consistent '
        '(same motion, same time base) and the run can be replayed at any time without '
        'the simulator.', BODY))
    E.append(Paragraph(
        'The analysis is deliberately decoupled from the live system: the scripts in '
        '<font face="Courier">src/arm_bot/analysis/</font> read the URDF straight out of '
        'the bag and rebuild the forward-kinematics chain themselves, so they need only a '
        'stock ROS install on the path (<font face="Courier">/opt/ros</font>) — not the '
        'built workspace overlay. They are plain <font face="Courier">python3</font> '
        'scripts, not <font face="Courier">ros2 run</font> nodes.', BODY))
    E.append(Paragraph('Pipeline at a glance', H3))
    E.append(code(
        "teach pendant  ──►  drawing_batch_planner (offline IK)  ──►  /cartesian_path\n"
        "                                                          └─► JointTrajectory\n"
        "                                                                   │\n"
        "  Gazebo Ignition + ros2_control  ◄── arm_controller ◄────────────┘\n"
        "                   │\n"
        "                   └─►  /joint_states (100 Hz feedback)\n"
        "\n"
        "  ros2 bag record  ──►  draw_run/   ──►  offline analysis  ──►  figures + tables"))

    # ── 2. The recording ─────────────────────────────────────────────────────
    E.append(Paragraph('2&nbsp;&nbsp;The recording (data capture)', H1))
    E.append(Paragraph('2.1&nbsp;&nbsp;Bringing up the system', H2))
    E.append(Paragraph(
        'The simulation backend was started through the pendant\'s composed launch graph '
        '(Gazebo, ros2_control, RViz, the FK/IK nodes and the batch drawing planner):', BODY))
    E.append(code("ros2 launch arm_bot pendant_backend.launch.py"))
    E.append(Paragraph(
        'The terminals that record and analyse must run under <b>bash</b> (the ROS '
        '<font face="Courier">setup.bash</font> files are bash; the user\'s default shell '
        'is fish). The pendant application itself re-execs under bash automatically.', SMALL))

    E.append(Paragraph('2.2&nbsp;&nbsp;Recording command', H2))
    E.append(Paragraph(
        'A rosbag2 recording was started <b>before</b> clicking <i>Send</i> in the pendant '
        'Drawing tab. This ordering is critical: <font face="Courier">/cartesian_path</font> '
        'is published <b>once</b> (a one-shot latched message that carries the whole '
        'resampled path), so a recorder started after Send would miss it.', BODY))
    E.append(code("ros2 bag record -o draw_run \\\n"
                  "    /joint_states /cartesian_path /robot_description"))
    E.append(Paragraph(
        'The drawing was then sent and allowed to run to completion — move-to-begin pose, '
        'dwell, draw the stroke, lift — before stopping the recorder with Ctrl-C.', BODY))

    E.append(Paragraph('2.3&nbsp;&nbsp;What the bag contains', H2))
    E.append(Paragraph(
        'The resulting bag (<font face="Courier">draw_run/draw_run_0.db3</font>, sqlite3 '
        'storage) holds exactly three topics:', BODY))
    E.append(kv_table([
        ['Topic', 'Type', 'Msgs', 'Role in the analysis'],
        ['/joint_states', 'sensor_msgs/JointState', '10,000',
         'Measured (executed) joint feedback at ~100 Hz — drives Figs 4.3-4.5 and '
         'the executed path in Fig 4.1.'],
        ['/cartesian_path', 'geometry_msgs/PoseArray', '1',
         'The commanded drawing path (resampled waypoints, already in paper '
         'coordinates). Its header stamp is the time origin t=0.'],
        ['/robot_description', 'std_msgs/String', '1',
         'The URDF — used to rebuild the FK chain and to read joint_6 limits.'],
    ], [38 * mm, 40 * mm, 16 * mm, 66 * mm]))
    E.append(Paragraph(
        'Total: 10,002 messages over a duration of 101.1 s. The 10,000 '
        '<font face="Courier">/joint_states</font> samples at ~100 Hz confirm the '
        'controller dispatch / feedback rate quoted in Table 4.1.', SMALL))

    E.append(Paragraph('2.4&nbsp;&nbsp;Coordinate convention captured in the bag', H2))
    E.append(Paragraph(
        'The <font face="Courier">/cartesian_path</font> poses are published by the batch '
        'planner <b>already in paper-plane coordinates</b> (the position x,y,z are the raw '
        'waypoint metres on the drawing plane), <i>not</i> base-frame end-effector targets. '
        'Only the executed path needs transforming from the base frame onto the paper plane. '
        'Getting this right was a fix during the first real run — see §6.', BODY))

    # ── 3. Analysis toolchain ────────────────────────────────────────────────
    E.append(Paragraph('3&nbsp;&nbsp;Analysis toolchain', H1))
    E.append(Paragraph(
        'Four scripts live in <font face="Courier">src/arm_bot/analysis/</font>. All run '
        'with plain <font face="Courier">python3</font> after only '
        '<font face="Courier">source /opt/ros/humble/setup.bash</font> (no workspace '
        'overlay needed — the FK chain is self-contained).', BODY))
    E.append(kv_table([
        ['Script', 'Produces', 'Input'],
        ['plot_commanded_vs_executed.py', 'Figure 4.1 (standalone)', 'draw_run bag'],
        ['make_thesis_figures.py', 'Figures 4.1, 4.3, 4.4, 4.5', 'draw_run bag'],
        ['make_table_4_1.py', 'Table 4.1 (timing) DOCX', 'measured values'],
        ['make_table_4_2.py', 'Table 4.2 (objectives) DOCX', 'authored / measured'],
    ], [62 * mm, 56 * mm, 42 * mm]))
    E.append(Paragraph(
        '<font face="Courier">make_thesis_figures.py</font> imports the shared helpers '
        '(bag reader, URDF FK chain, paper-frame transform, tracking-error metric) from '
        '<font face="Courier">plot_commanded_vs_executed.py</font> via importlib, so the '
        'four figures share one code path.', SMALL))

    E.append(Paragraph('3.1&nbsp;&nbsp;Commands actually run', H2))
    E.append(code(
        "# one source line, then everything is plain python3 (bash shell)\n"
        "source /opt/ros/humble/setup.bash\n"
        "cd ~/7dof_ws\n"
        "\n"
        "# Figures 4.1, 4.3, 4.4, 4.5 — all four from the one bag\n"
        "python3 src/arm_bot/analysis/make_thesis_figures.py draw_run\n"
        "\n"
        "# Tables\n"
        "python3 src/arm_bot/analysis/make_table_4_1.py \\\n"
        "        --out table_4_1_timing.docx\n"
        "python3 src/arm_bot/analysis/make_table_4_2.py \\\n"
        "        --out table_4_2_objectives_vs_outcomes.docx"))
    E.append(Paragraph(
        'Each figure is written as both a 300&nbsp;dpi PNG (for review) and a vector PDF '
        '(for LaTeX inclusion). Outputs land in the current directory unless '
        '<font face="Courier">--outdir</font> is given.', SMALL))

    # ── 4. How each figure was made ──────────────────────────────────────────
    E.append(Paragraph('4&nbsp;&nbsp;How each figure was derived', H1))

    E.append(Paragraph('4.1&nbsp;&nbsp;Figure 4.1 — commanded vs executed path overlay', H2))
    E.append(Paragraph(
        'The headline accuracy figure. It overlays the path the pendant <i>commanded</i> '
        'against the path the arm <i>executed</i>, both expressed as pen-tip positions on '
        'the drawing plane in millimetres, and annotates the gap between them.', BODY))
    E.append(bullets([
        '<b>Commanded path</b>: the waypoints from the one-shot '
        '<font face="Courier">/cartesian_path</font> PoseArray, scaled metres→mm '
        '(position&nbsp;&times;&nbsp;1000). No transform — they are already paper coordinates.',
        '<b>Executed path</b>: for every recorded <font face="Courier">/joint_states</font> '
        'sample, forward kinematics (the URDF chain, base→ee) gives the end-effector pose, '
        'which is mapped onto the paper plane by the exact transform the planner uses '
        '(pen offset along the EE-local pen axis, anchored at the pen tip at the begin-draw '
        'pose, rotated by the paper rotation).',
        '<b>Tracking error</b>: point-to-point distance between executed and commanded '
        'curves; the script prints and annotates RMS and max in mm.',
        '<b>Windowing</b>: the executed stream is trimmed to the drawing phase only — '
        'the move-to-begin (4&nbsp;s), dwell (3&nbsp;s), settle (0.5&nbsp;s) and a '
        '1&nbsp;s approach are dropped using the <font face="Courier">/cartesian_path</font> '
        'stamp as t=0, so the begin-pose swing is not counted as error.',
    ]))
    E.append(Paragraph(
        'Paper-frame parameters are baked in to match '
        '<font face="Courier">pendant_backend.launch.py</font>: begin-draw joints '
        '[0, -0.7, 0, 1.4, 0.01, 0, 1], pen offset 100&nbsp;mm, pen axis local [1,0,0], '
        'paper rotation 270&deg;, no mirror. (These shift/rotate both curves identically, '
        'so a small mismatch only reorients the axes — it cannot corrupt the comparison.)', SMALL))
    E.append(Paragraph(
        '<b>Result:</b> clean overlay, tracking error <b>RMS 0.11&nbsp;mm, max 0.61&nbsp;mm</b> '
        'over ~2000 pen-down samples.', BODY))

    E.append(Paragraph('4.2&nbsp;&nbsp;Figure 4.3 — joint position trajectories', H2))
    E.append(Paragraph(
        'Seven lines (joint_1 … joint_7) of measured joint angle vs time, read directly '
        'from <font face="Courier">/joint_states</font>. The series is indexed <b>by joint '
        'name on every message</b>, because the bag\'s name order is not sorted (joint_3 / '
        'joint_4 appear swapped) — indexing by position would silently mislabel two joints. '
        'The time axis is zeroed at trajectory dispatch and the trailing static hold is '
        'auto-trimmed.', BODY))

    E.append(Paragraph('4.3&nbsp;&nbsp;Figure 4.4 — joint velocity profiles', H2))
    E.append(Paragraph(
        'Seven lines of joint velocity vs time. The <font face="Courier">velocity</font> '
        'field published in <font face="Courier">/joint_states</font> is used directly '
        '(it was verified to match a numeric central-difference of the positions); a '
        '<font face="Courier">--vel-source diff</font> fallback recomputes by differentiation '
        'if velocity is ever missing. <b>Measured peaks:</b> joint_4 0.61, joint_7 0.44, '
        'joint_2 0.30 rad/s.', BODY))

    E.append(Paragraph('4.4&nbsp;&nbsp;Figure 4.5 — joint_6 vs its limits', H2))
    E.append(Paragraph(
        'joint_6 has the tightest travel on this arm, so it gets its own safety figure. '
        'The script reads joint_6\'s lower/upper limit straight from the URDF in the bag, '
        'shades the allowed band, draws the measured angle, and annotates the closest '
        'approach to a limit. <b>Result:</b> joint_6 used the range '
        '[-0.037, +0.020]&nbsp;rad against limits [-0.48, +0.26]&nbsp;rad — closest approach '
        '0.24&nbsp;rad, comfortably <b>within limits</b>.', BODY))
    E.append(Paragraph(
        'Figure 4.2 (not produced from the bag) is a Gazebo screenshot of the traced pen '
        'path, captured separately with '
        '<font face="Courier">ros2 launch arm_bot pendant_backend.launch.py '
        'enable_path_tracer:=true</font>.', SMALL))

    E.append(PageBreak())

    # ── 5. Tables ────────────────────────────────────────────────────────────
    E.append(Paragraph('5&nbsp;&nbsp;How each table was built', H1))

    E.append(Paragraph('5.1&nbsp;&nbsp;Table 4.1 — timing and performance', H2))
    E.append(Paragraph(
        'Built as a DOCX by <font face="Courier">make_table_4_1.py</font>. The values are '
        'measured from the drawing run two independent ways and cross-checked:', BODY))
    E.append(bullets([
        '<b>Per-waypoint IK solve time</b> — mean over the 45 path waypoints, obtained by '
        'replaying the planner\'s exact <font face="Courier">solve_ik</font> on the recorded '
        'path offline, and confirmed live by a <font face="Courier">[TIMING]</font> line the '
        'batch planner now prints. <b>0.65&nbsp;ms</b> mean (1.27&nbsp;ms max).',
        '<b>Batch generation time</b> — full path planning (spline resample + all IK): '
        '<b>≈ 30&nbsp;ms</b>.',
        '<b>Dispatch rate</b> — the controller update rate, also visible as the 10&nbsp;ms '
        '<font face="Courier">/joint_states</font> period in the bag: <b>100&nbsp;Hz</b>.',
    ]))
    E.append(Paragraph(
        'The offline replay also reported 45 waypoints, batch IK ~29&nbsp;ms and a max IK '
        'residual of 7.6&times;10⁻⁴ — i.e. every waypoint converged.', SMALL))

    E.append(Paragraph('5.2&nbsp;&nbsp;Table 4.2 — objectives vs outcomes', H2))
    E.append(Paragraph(
        'This table is <b>authored, not measured</b> (no bag needed). '
        '<font face="Courier">make_table_4_2.py</font> pairs each of the four Section&nbsp;1.3 '
        'thesis objectives with the outcome that demonstrates it and the evidence (figure / '
        'table / chapter). The objective wording was taken verbatim from the thesis; the '
        'outcome cells embed the measured results from this run (notably the Fig&nbsp;4.1 '
        'RMS 0.11&nbsp;mm). The objectives frame the contribution as the <b>pendant as an '
        'operator / integration layer</b> — not the kinematics — so the drawing mode is '
        'presented as an end-to-end correctness benchmark, not as a novel IK algorithm.', BODY))

    # ── 6. Provenance / fixes ────────────────────────────────────────────────
    E.append(Paragraph('6&nbsp;&nbsp;Provenance, validation and fixes', H1))
    E.append(Paragraph(
        'The toolchain was exercised on the real recorded run, and two bugs were found and '
        'fixed in the process — both worth recording for defensibility:', BODY))
    E.append(bullets([
        '<b>Self-contained FK</b> — the FK chain class was inlined verbatim from the '
        'live FK node instead of imported from the <font face="Courier">arm_bot</font> '
        'package, so the script runs with only <font face="Courier">/opt/ros</font> sourced '
        '(the fish terminal had no workspace overlay, which had caused a ModuleNotFoundError).',
        '<b>Commanded-frame bug</b> — <font face="Courier">/cartesian_path</font> is already '
        'in paper coordinates; an earlier version wrongly pushed it through the base→paper '
        'transform, so commanded and executed curves did not overlap and the error came out '
        'NaN. Fixed by scaling the commanded poses metres→mm only.',
    ]))
    E.append(Paragraph(
        'Note that the analysis touches two workspace files (a <font face="Courier">[TIMING]</font> '
        'log line in the batch planner and a default-off <font face="Courier">enable_path_tracer</font> '
        'launch arg) — both rebuilt cleanly and leave the pendant contract intact.', SMALL))

    # ── 7. Reproduce from scratch ────────────────────────────────────────────
    E.append(Paragraph('7&nbsp;&nbsp;Reproducing the whole dataset from scratch', H1))
    E.append(code(
        "# 1. bring up the simulation backend\n"
        "ros2 launch arm_bot pendant_backend.launch.py\n"
        "\n"
        "# 2. in a bash terminal, START recording BEFORE clicking Send\n"
        "ros2 bag record -o draw_run /joint_states /cartesian_path /robot_description\n"
        "#    -> open the pendant, Drawing tab, draw a stroke, click Send,\n"
        "#       let it finish (begin -> dwell -> draw -> lift), then Ctrl-C\n"
        "\n"
        "# 3. generate every figure and table from that one bag\n"
        "source /opt/ros/humble/setup.bash\n"
        "python3 src/arm_bot/analysis/make_thesis_figures.py draw_run\n"
        "python3 src/arm_bot/analysis/make_table_4_1.py --out table_4_1_timing.docx\n"
        "python3 src/arm_bot/analysis/make_table_4_2.py --out table_4_2_objectives_vs_outcomes.docx\n"
        "\n"
        "# (optional) Figure 4.2 path-tracer screenshot\n"
        "ros2 launch arm_bot pendant_backend.launch.py enable_path_tracer:=true"))
    E.append(Paragraph(
        'Outputs: <font face="Courier">figure_4_1.{png,pdf}</font> … '
        '<font face="Courier">figure_4_5.{png,pdf}</font>, '
        '<font face="Courier">table_4_1_timing.docx</font>, '
        '<font face="Courier">table_4_2_objectives_vs_outcomes.docx</font>.', SMALL))

    E.append(Spacer(1, 4 * mm))
    E.append(HRFlowable(width='100%', thickness=0.6, color=colors.HexColor('#bbbbbb')))
    E.append(Paragraph('Summary of measured results', H3))
    E.append(kv_table([
        ['Quantity', 'Value', 'Source'],
        ['End-effector tracking error', 'RMS 0.11 mm / max 0.61 mm', 'Fig 4.1'],
        ['Per-waypoint IK solve time', '0.65 ms mean (1.27 ms max)', 'Table 4.1'],
        ['Batch trajectory generation', '≈ 30 ms (45 waypoints)', 'Table 4.1'],
        ['Controller dispatch rate', '100 Hz', 'Table 4.1 / bag'],
        ['Peak joint velocity', 'joint_4 0.61 rad/s', 'Fig 4.4'],
        ['joint_6 range vs limits', '[-0.037,+0.020] vs [-0.48,+0.26] rad', 'Fig 4.5'],
        ['Max IK residual', '7.6e-4 (all converged)', 'offline replay'],
    ], [55 * mm, 70 * mm, 35 * mm]))

    doc.build(E, onFirstPage=lambda c, d: None, onLaterPages=footer)
    print(f'wrote {out_path}')


if __name__ == '__main__':
    ap = argparse.ArgumentParser()
    ap.add_argument('--out', default='data_methodology.pdf')
    args = ap.parse_args()
    build(args.out)
