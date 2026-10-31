# -*- coding: utf-8 -*-
"""Builds 'Project_Explanation_and_MATLAB_Files.docx': plain-language explanation of the report and of what every MATLAB file does / outputs."""
import os, json
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.enum.table import WD_TABLE_ALIGNMENT
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

HERE = os.path.dirname(os.path.abspath(__file__))
N = json.load(open(os.path.join(HERE, "numbers.json")))
ON, OFF = N["on"], N["off"]
OUT = os.path.join(HERE, "Project_Explanation_and_MATLAB_Files.docx")

doc = Document()
s = doc.sections[0]
s.page_width, s.page_height = Cm(21), Cm(29.7)
for a in ("left_margin", "right_margin"): setattr(s, a, Cm(2.0))
s.top_margin = s.bottom_margin = Cm(2.0)
st = doc.styles["Normal"]; st.font.name = "Calibri"; st.font.size = Pt(11)
st.element.rPr.rFonts.set(qn("w:eastAsia"), "Calibri"); st.paragraph_format.space_after = Pt(6)
for nm, sz, col in (("Heading 1", 16, (0x1F, 0x3A, 0x5F)), ("Heading 2", 13, (0x1F, 0x3A, 0x5F)), ("Title", 24, (0x1F, 0x3A, 0x5F))):
    h = doc.styles[nm]; h.font.name = "Calibri"; h.font.size = Pt(sz); h.font.bold = True; h.font.color.rgb = RGBColor(*col)
    h.element.rPr.rFonts.set(qn("w:eastAsia"), "Calibri")

fp = s.footer.paragraphs[0]; fp.alignment = WD_ALIGN_PARAGRAPH.CENTER
r = fp.add_run()
for t, txt in (("begin", None), (None, "PAGE"), ("end", None)):
    if t:
        fc = OxmlElement("w:fldChar"); fc.set(qn("w:fldCharType"), t); r._r.append(fc)
    else:
        it = OxmlElement("w:instrText"); it.set(qn("xml:space"), "preserve"); it.text = txt; r._r.append(it)


def P(text, bold=False, italic=False, size=None, align=None):
    p = doc.add_paragraph(); r = p.add_run(text); r.bold = bold; r.italic = italic
    if size: r.font.size = Pt(size)
    if align: p.alignment = align
    return p


def BL(items):
    for x in items:
        p = doc.add_paragraph(style="List Bullet")
        if isinstance(x, tuple):
            r = p.add_run(x[0]); r.bold = True; p.add_run(x[1])
        else:
            p.add_run(x)


def shade(cell, fill):
    tcPr = cell._tc.get_or_add_tcPr(); shd = OxmlElement("w:shd")
    shd.set(qn("w:val"), "clear"); shd.set(qn("w:fill"), fill); tcPr.append(shd)


def T(head, rows, widths, mono_cols=()):
    t = doc.add_table(rows=1, cols=len(head)); t.style = "Table Grid"; t.alignment = WD_TABLE_ALIGNMENT.CENTER
    for c, h in zip(t.rows[0].cells, head):
        c.text = ""; r = c.paragraphs[0].add_run(h); r.bold = True; r.font.size = Pt(10); r.font.color.rgb = RGBColor(255, 255, 255); shade(c, "1F3A5F")
    for row in rows:
        cells = t.add_row().cells
        for j, (c, v) in enumerate(zip(cells, row)):
            c.text = ""; r = c.paragraphs[0].add_run(v); r.font.size = Pt(9.5)
            if j in mono_cols: r.font.name = "Consolas"; r.font.size = Pt(9)
    for row in t.rows:
        for c, w in zip(row.cells, widths): c.width = Cm(17.0 * w)
        for c in row.cells: c.paragraphs[0].paragraph_format.space_after = Pt(2)
    doc.add_paragraph()


def f1(x): return f"{x:.1f}"
def f2(x): return f"{x:.2f}"


# ===================================================================================== title
doc.add_paragraph("Project Explanation and MATLAB File Guide", style="Title")
P("Design and Simulation of an Industry Standard AMR (AgileX Scout Mini) in a Warehouse", bold=True, size=13)
P("Sailakshmi B T (2024511019) | RA23502 Autonomous Mobile Robot", italic=True)
P("This document explains the project report in simple words and lists every MATLAB file, what it does and what output it "
  "gives. Use it to understand the work and to prepare for questions. The folder is "
  "D:\\academic_shit\\SEM5-III\\AMR ROS\\simulink_amr unless another path is given.")

# ===================================================================================== 1
doc.add_heading("1. The project in one page", 1)
P("What was built. A virtual warehouse robot. The robot is your own SolidWorks model of the AgileX Scout Mini (a four-wheel "
  "skid-steer robot). It is placed in a 3D warehouse inside MATLAB Simulink / Simscape Multibody, where real physics (mass, wheel "
  "torque, tyre friction) moves it. The robot drives by itself from a start dock to a pickup station, waits to load, drives to a delivery "
  "station and waits to unload, while avoiding racks, boxes, a worker, a second robot and a pallet truck.")
P("The chain of work, in order:", bold=True)
BL([("SolidWorks CAD model", " of the robot (your files Assem2, chassis, chassis_top, W, up)."),
    ("Export to URDF + STL meshes", ": a text description of the robot (links, joints, masses) plus the 3D shapes."),
    ("Import into MATLAB", ": importrobot (for viewing) and smimport (creates the Simscape Multibody / Simulink model)."),
    ("Make it a physical robot", ": free-moving chassis, torque-driven wheels, a tyre-ground friction model written by us."),
    ("Build the warehouse", ": racks with goods, walls, stations, floor boxes, oil/water patches, and four moving obstacles."),
    ("Add autonomy", ": LiDAR, occupancy grid, A* path planning, re-planning, path following and a mission state machine."),
    ("Add slip management", ": detect wheel slip and limit torque so the wheels do not spin or lock."),
    ("Run, log, plot, report", ": one script runs everything and draws all graphs.")])
P("Why it matters. Testing a warehouse robot in simulation is cheaper and safer than on a real floor. The project shows the full "
  "path from a CAD model to a robot that senses, plans, moves and handles slippery floors.")

# ===================================================================================== 2
doc.add_heading("2. The report explained section by section", 1)
T(["Section", "What it tells", "Key point to remember"],
  [["Abstract", "One-paragraph summary of the whole project and its results.", f"Mission {f1(ON['missionTime'])} s, {ON['reroutes']} route corrections, {ON['collisions']} collisions, minimum clearance {f2(ON['minClear'])} m."],
   ["1 Introduction", "What an AMR is, why simulate, the objectives and the scope.", "Simulation only. Real hardware was not tested."],
   ["2 Robot Specifications", "Numbers of the model: mass, wheel size, track, torque limit, tyre values.", "Values come from your CAD/URDF, not from measuring the real robot."],
   ["3 Sensors", "Every sensor, its configuration, why it is used and whether it is really simulated.", "Only the LiDAR (analytic) and the body-state / wheel-speed sensors are used in Simulink. IMU and camera are mounted in the model but give no signals."],
   ["4 SolidWorks Design", "Your CAD files and screenshots.", "SolidWorks files cannot be opened in MATLAB, so STL meshes carry the shape."],
   ["5 Conversion to URDF", "What URDF is and the values read from it.", "Only change made: package:// mesh paths replaced by meshes/."],
   ["6 Importing in MATLAB and Simulink", "importrobot and smimport.", "smimport is slow (minutes), so the result is cached in scout_import_raw.slx. Imported wheel joints have their Z axis along the axle."],
   ["7 Simscape Multibody model", "Steps that turn the import into a dynamic robot and the tyre model equations.", "6-DOF chassis, torque-driven wheels, spring-damper normal force + smoothed Coulomb friction (kc 12000 N/m, cc 700 N·s/m)."],
   ["8 Warehouse", "Layout table, moving obstacles, planning map.", "Racks and walls are known to the planner; boxes, workers, AMR and truck are found only by LiDAR."],
   ["9 Autonomous Navigation", "How to run, one full mission, the six scenarios, result figures and table.", "All six demo scenarios happen in one mission (box, worker, blocked aisle, cross-aisle, pickup to delivery, alternative path)."],
   ["10 Slip and Traction Management", "How slip is detected and limited; ON versus OFF comparison.", f"Peak slip {f2(ON['peakSlip'])} (ON) vs {f2(OFF['peakSlip'])} (OFF), but ON is about {f1(ON['missionTime']-OFF['missionTime'])} s slower. Both finished without collision."],
   ["11 Algorithms", "Table of every algorithm, file and parameters; A* and dynamic-obstacle handling.", "Reactive avoidance: moving obstacles are not predicted."],
   ["12 System Architecture", "Block diagrams: closed loop, tool-chain, state machine.", "The pose comes from the simulator (no localisation)."],
   ["13 What Happens When the Code Runs", "Flow of one 5 ms control step.", "Sense, map, plan, steer, limit slip, torque, physics."],
   ["14 Visual MATLAB Simulation", "The earlier script simulation with ISO 3691-4 style safety fields.", "Kinematic animation, not a multibody solver."],
   ["15 How to Set It Up", "Requirements, commands and a code-layout table.", "Run run_scout_warehouse_simulation."],
   ["16 Limitations and Future", "What is not modelled and what to do next.", "Analytic LiDAR and collisions, scripted obstacles, true pose, noisy torque."],
   ["17 Conclusion", "Summary of what was achieved and its limits.", "Shows the software chain works; not proof for the physical robot."]],
  [0.20, 0.42, 0.38])

# ===================================================================================== 3
doc.add_heading("3. MATLAB files and the output each one gives", 1)
P("The files are grouped by purpose. 'Output' means what you get when the file is run or called.")

doc.add_heading("3.1 Robot description (inputs, not code)", 2)
T(["File / folder", "What it is", "Output / use"],
  [["scout_mini.urdf", "URDF of the robot: links, joints, masses, inertias, sensor mounts (copied from the ROS 2 repository; mesh paths changed to meshes/).", "Read by importrobot and smimport."],
   ["meshes/ (base_link.STL, w1.STL ... w4.STL)", "3D shapes exported from your SolidWorks model.", "Drawn in the 3D scene and in the report renders."],
   ["../scout_mini_sw/ (Assem2.SLDASM, chassis.SLDPRT, chassis_top.SLDPRT, W.SLDPRT, up.SLDPRT)", "Your original SolidWorks files (cloned from GitHub).", "Source of the STL meshes. Not read by MATLAB."]],
  [0.30, 0.45, 0.25], mono_cols=(0,))

doc.add_heading("3.2 Model and physics", 2)
T(["File", "What it does", "Output"],
  [["scout_amr_params.m", "Holds every constant: robot, wheels, tyre contact, friction patches, warehouse layout, LiDAR, planning grid, controller gains. Also computes the wheel rotation signs from the URDF.", "Returns the struct P used by all other files. Edit this file to change the warehouse or the gains."],
   ["build_scout_amr_model.m", "Imports the URDF with smimport (or reuses scout_import_raw.slx), replaces the weld by a 6-DOF joint, makes wheels torque-driven, adds sensors, tyre forces, the 3D warehouse, the four movers, logging blocks, subsystems and the diagram layout.", "Creates scout_amr_sim.slx. Run it only to rebuild the model."],
   ["scout_import_raw.slx", "Raw result of smimport (cache).", "Speeds up rebuilds."],
   ["scout_amr_sim.slx", "The Simulink / Simscape Multibody simulation. Top level: World, Mechanism Configuration, Solver and four subsystems (Scout_Mini_Robot with the nested Tyre_Ground_Contact_Model, Warehouse, Moving_Obstacles, Mission_and_Slip_Controller).", "When run it logs ctrl_log, tire_log, lidar_log, path_log and mover_log to the workspace and shows the 3D Mechanics Explorer."],
   ["scout_amr_tires.m", "Tyre-ground model for one wheel (called four times): normal spring-damper force, smoothed Coulomb friction, floor friction map.", "11 values per wheel: force (3) and torque (3) in the wheel frame, normal load Fz, longitudinal force, lateral force, slip ratio, friction μ."]],
  [0.26, 0.45, 0.29], mono_cols=(0,))

doc.add_heading("3.3 Autonomy and control (called every 5 ms from the model)", 2)
T(["File", "What it does", "Output"],
  [["scout_amr_controller.m", "The brain inside the Controller block: gets robot pose and speed, calls the navigation, converts commands to wheel targets, estimates slip, applies slip management, runs the wheel-speed PI controller.", "tau_j: four wheel torques. lg: log vector of 44 signals (pose, speeds, slip ratios, torques, friction estimate, mission state, clearance, collisions...). mv: 8 mover positions. rng: 72 LiDAR ranges. pth: planned path."],
   ["scout_wh_nav.m", "Perception, planning and mission logic: world update, LiDAR, occupancy grid, A* replanning, pure pursuit, reactive repulsion, stop-before-contact limit, retreat, mission state machine, collision monitor.", "Speed and yaw-rate reference, mover positions, LiDAR ranges, path and an info vector (state, number of re-routes, clearance, collisions, goal distance)."],
   ["scout_wh_world.m", "Warehouse geometry and moving obstacles at time t (17 rectangles, 2 circles, mover positions). Workers react to the robot; the truck is released at x > 12 m.", "Rectangles, circles and the positions that move the 3D bodies."],
   ["scout_wh_lidar.m", "Casts 72 rays against the rectangles and circles.", "72 range values in metres (8 m means no return)."],
   ["scout_wh_plan.m", "8-connected A* on the inflated grid with line-of-sight smoothing.", "Path of up to 80 waypoints and their count."]],
  [0.24, 0.44, 0.32], mono_cols=(0,))

doc.add_heading("3.4 Running and plotting", 2)
T(["File", "How to run", "Output"],
  [["run_scout_warehouse_simulation.m", "run_scout_warehouse_simulation   (3D window)\nrun_scout_warehouse_simulation(false)   (headless)\nrun_scout_warehouse_simulation(false,true)   (slip ON and OFF)\nrun_scout_warehouse_simulation('replot')   (redraw from saved data)",
    "Runs the model, saves results/scout_warehouse_results.mat, draws figures 1 to 5 (6 with the comparison) as PNG in results/ and prints the summary (mission time, distance, speeds, re-routes, collisions, clearance, slip, energy)."],
   ["results/1_warehouse_trajectory.png", "(made by the file above)", "Map with driven path, planned paths, re-route events and mover trails."],
   ["results/2_velocity_wheel_speeds.png", "", "Linear and angular velocity (commanded and measured) and the four wheel speeds."],
   ["results/3_perception_safety.png", "", "Closest LiDAR range, ground-truth clearance, route corrections, collisions, mission state."],
   ["results/4_slip_management.png", "", "Slip ratio of each wheel, torques after limiting, friction estimate vs true friction, normal loads."],
   ["results/5_lidar_scenarios.png", "", "Four snapshots: avoiding the box, worker crossing, aisle blocked re-route, worker in aisle 3."],
   ["results/6_slip_on_vs_off.png", "(only with compareSlip = true)", "Worst-wheel slip ratio and speed with slip management on and off."],
   ["results/scout_warehouse_results.mat", "", "Saved logs of both runs (R) and the parameters (P)."]],
  [0.30, 0.34, 0.36], mono_cols=(0,))

doc.add_heading("3.5 Report-making files (folder report/)", 2)
T(["File", "Output"],
  [["make_report_figures.m", "robot_importrobot.png, robot_render.png (robot meshes with joint axes), warehouse3d_t17/t29/t41.png (3D warehouse renders with the robot and LiDAR rays) and the Simulink subsystem pictures simulink_Mission_and_Slip_Controller.png, simulink_Tyre_Ground_Contact_Model.png (in report/figs)."],
   ["report_numbers.m", "numbers.json: every number quoted in the report (ON and OFF run metrics, re-route times)."],
   ["capture_industry_full.m", "industry_01..05, industry_final.png (screenshots) and industry_telemetry.mat: runs simulate_industry_amr_warehouse.m unchanged and grabs its window and telemetry."],
   ["make_industry_graphs.m", "industry_g1_kinematics.png, industry_g2_safety_sensors.png, industry_g3_path.png redrawn from industry_telemetry.mat."],
   ["make_diagrams.py", "diag_toolchain.png, diag_architecture.png, diag_mission_fsm.png, diag_control_step.png (Python, matplotlib)."],
   ["content.py + build_report.py", "Scout_Mini_Warehouse_AMR_Report.pdf and .docx (Python; content.py holds the text, build_report.py the layout)."],
   ["build_explainer.py", "This document."]],
  [0.30, 0.70], mono_cols=(0,))

doc.add_heading("3.6 Earlier visual MATLAB simulation (folder AMR ROS)", 2)
T(["File", "What it does", "Output"],
  [["simulate_industry_amr_warehouse.m", "Script simulation: 26 x 16 m warehouse with racks, pallets, a ramp to a loading dock, the Scout Mini STL meshes, LiDAR, IMU and odometry drift, ISO 3691-4 style warning / braking / stop fields and automatic re-routing.", "An animated 3D figure with an occupancy grid, speed / wheel-speed / slip plots and safety telemetry. Not a multibody simulation."],
   ["plan_warehouse_roadmap.m", "Waypoint roadmap along aisle centre lines, avoiding hazard boxes.", "path: list of waypoints from the current position to the goal."],
   ["teleop_industry_amr_warehouse.m", "Same scene driven by hand (W/A/S/D, arrows, space).", "Interactive 3D figure."]],
  [0.30, 0.42, 0.28], mono_cols=(0,))

# ===================================================================================== 4
doc.add_heading("4. How the files work together", 1)
P("Order of events when you run run_scout_warehouse_simulation:", bold=True)
BL(["The runner loads scout_amr_sim.slx (builds it with build_scout_amr_model.m if missing) and sets tc_on (1 = slip management, 0 = off).",
    "Simulink runs. Every 5 ms the Controller block calls scout_amr_controller.m. That calls scout_wh_nav.m, which calls scout_wh_world.m (obstacle geometry), scout_wh_lidar.m (ranges) and scout_wh_plan.m (A* path).",
    "The controller returns four wheel torques. Four Tyre blocks call scout_amr_tires.m to turn wheel states into ground forces. Simscape Multibody applies the torques and forces and moves the chassis, wheels and movers.",
    "The sensors feed pose, velocity and wheel speeds back to the controller. The logs are stored.",
    "When the mission state reaches 'done' the simulation stops. The runner converts the logs, saves the .mat file, draws the figures and prints the summary.",
    "scout_amr_params.m is read by all of them, so changing a number there changes the whole simulation."])

# ===================================================================================== 5
doc.add_heading("5. Results to remember", 1)
T(["Quantity", "Slip management ON", "Slip management OFF"],
  [["Mission time", f"{f1(ON['missionTime'])} s", f"{f1(OFF['missionTime'])} s"],
   ["Reached pickup / left / delivered", f"{f1(ON['tPickup'])} / {f1(ON['tLeave'])} / {f1(ON['tDeliver'])} s", f"{f1(OFF['tPickup'])} / {f1(OFF['tLeave'])} / {f1(OFF['tDeliver'])} s"],
   ["Distance", f"{f1(ON['distance'])} m", f"{f1(OFF['distance'])} m"],
   ["Top speed", f"{f2(ON['topSpeed'])} m/s", f"{f2(OFF['topSpeed'])} m/s"],
   ["Route corrections", str(ON["reroutes"]), str(OFF["reroutes"])],
   ["Collisions", str(ON["collisions"]), str(OFF["collisions"])],
   ["Minimum clearance", f"{f2(ON['minClear'])} m", f"{f2(OFF['minClear'])} m"],
   ["Peak slip ratio", f2(ON["peakSlip"]), f2(OFF["peakSlip"])],
   ["Time with worst-wheel slip above 0.2", f"{100*ON['fracSlip02']:.1f} %", f"{100*OFF['fracSlip02']:.1f} %"],
   ["Wheel mechanical work", f"{ON['energy']:.0f} J", f"{OFF['energy']:.0f} J"]],
  [0.40, 0.30, 0.30])
P(f"Route corrections in the ON run happened at t = {N['rerouteTimes'][0]:.1f}, {N['rerouteTimes'][1]:.1f} and {N['rerouteTimes'][2]:.1f} s, when the pallet truck blocked aisle 2 and the robot re-planned.")

# ===================================================================================== 6
doc.add_heading("6. Likely questions and short answers", 1)
QA = [
    ("Why Simscape Multibody and not just a MATLAB animation?",
     "Because the wheels, chassis and tyre forces are simulated with real physics: torque on the wheels, friction at the ground, mass and inertia. The motion (and the slip) comes out of the physics, not from a drawn path."),
    ("How did the SolidWorks model get into MATLAB?",
     "SolidWorks to URDF + STL meshes with the exporter, then importrobot (Robotics System Toolbox) and smimport (Simscape Multibody) read the URDF."),
    ("What changed after the import?",
     "The weld between the footprint and the chassis was replaced by a 6-DOF joint so the robot can move; the wheel joints were set to accept torque; sensors, tyre forces, the warehouse and the controller were added."),
    ("How is wheel-ground friction modelled?",
     "Per wheel: a spring-damper normal force and a smoothed Coulomb friction force with μ from a floor map (0.9 concrete, 0.15 oil, 0.30 water). Written in scout_amr_tires.m."),
    ("What is slip ratio and how is it handled?",
     "Slip ratio = (wheel speed x radius - ground speed) / reference speed. Above 0.12 the controller limits the wheel-speed target to the ground speed window, cuts torque as slip grows and lowers its friction estimate, which also lowers the acceleration limit."),
    ("Did slip management help?",
     f"It reduced the peak slip ({f2(ON['peakSlip'])} vs {f2(OFF['peakSlip'])}) and the time with high slip, but made the mission about {f1(ON['missionTime']-OFF['missionTime'])} s slower. The robot also finished without collision with it off, so the benefit is smoother wheel behaviour, not success versus failure."),
    ("How does the robot see obstacles?",
     "A 72-beam, 8 m, 10 Hz LiDAR modelled by ray casting against the warehouse geometry. Racks and walls are known in advance; boxes, workers, the second AMR and the truck appear only when the LiDAR hits them."),
    ("How does it plan and avoid?",
     "A* on a 0.2 m occupancy grid inflated by 0.48 m, re-planned every 0.5 s or when the path is blocked, then pure pursuit with reactive repulsion and a stop-before-contact speed limit. If an obstacle closes in and the rear is free it backs away."),
    ("What does the mission state machine do?",
     "Five states: drive to pickup, load 3 s, drive to delivery, unload 3 s, done."),
    ("Is the pose estimated?",
     "No. The pose is the true simulator pose from the Transform Sensor blocks. There is no localisation or SLAM in the Simulink model."),
    ("Are collisions physical?",
     "No. Collisions are checked analytically with a 0.36 m robot circle against the geometry. There is no Simscape contact between the robot and obstacles, and the movers follow scripted paths."),
    ("Is the IMU or camera used?",
     "They are mounted on the robot (from the URDF) but give no signals in Simulink. Their specifications are listed in the report as part of the robot description; the IMU, odometry drift and LiDAR noise are modelled only in the earlier script simulation."),
    ("How do I change the warehouse or the gains?",
     "Edit scout_amr_params.m. If you change the racks, walls or movers, rebuild the model with build_scout_amr_model.m so the 3D scene matches."),
]
for q, a in QA:
    p = doc.add_paragraph(); r = p.add_run("Q: " + q); r.bold = True; p.paragraph_format.space_after = Pt(1)
    doc.add_paragraph("A: " + a)

# ===================================================================================== 7
doc.add_heading("7. Honest limits (say these before you are asked)", 1)
BL(["Simulation only. Nothing was tested on the physical Scout Mini.",
    "LiDAR and collision detection are analytic; there is no physical contact with obstacles.",
    "The moving obstacles follow scripts, and the workers pause when the robot is close.",
    "The pose is the true simulator pose (no localisation, no sensor noise in Simulink).",
    "The tyre model is a simple point-contact model; the motors are ideal torque sources limited to 17.5 N·m.",
    "The 3D warehouse pictures in the report are MATLAB renders of the same geometry; the live 3D view is in the Mechanics Explorer when the model runs.",
    "The slip comparison is one run per case on one layout."])

doc.save(OUT)
print("saved", OUT)
