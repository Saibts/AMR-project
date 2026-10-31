# -*- coding: utf-8 -*-
"""Builds Live_Demo_Cheat_Sheet.docx: the MATLAB command for every figure / result of the report."""
import os
from docx import Document
from docx.shared import Pt, Cm, RGBColor
from docx.enum.text import WD_ALIGN_PARAGRAPH
from docx.oxml.ns import qn
from docx.oxml import OxmlElement

HERE = os.path.dirname(os.path.abspath(__file__))
doc = Document()
for s in doc.sections:
    s.left_margin = s.right_margin = Cm(1.6); s.top_margin = s.bottom_margin = Cm(1.5)
st = doc.styles["Normal"]; st.font.name = "Times New Roman"; st.font.size = Pt(11)
st.element.rPr.rFonts.set(qn("w:eastAsia"), "Times New Roman")


def shade(cell, color):
    tcPr = cell._tc.get_or_add_tcPr(); sh = OxmlElement("w:shd")
    sh.set(qn("w:val"), "clear"); sh.set(qn("w:color"), "auto"); sh.set(qn("w:fill"), color); tcPr.append(sh)


def para(text, size=11, bold=False, align=None, after=4):
    p = doc.add_paragraph(); r = p.add_run(text); r.bold = bold; r.font.size = Pt(size)
    p.paragraph_format.space_after = Pt(after)
    if align: p.alignment = align
    return p


def table(head, rows, widths):
    t = doc.add_table(rows=1, cols=len(head)); t.style = "Table Grid"
    for c, h in zip(t.rows[0].cells, head):
        c.text = ""; r = c.paragraphs[0].add_run(h); r.bold = True; r.font.size = Pt(10); shade(c, "D9E2F3")
    for row in rows:
        cells = t.add_row().cells
        for i, (c, v) in enumerate(zip(cells, row)):
            c.text = ""; p = c.paragraphs[0]
            if i == 2:
                for k, line in enumerate(v.split("\n")):
                    if k: p = c.add_paragraph()
                    r = p.add_run(line); r.font.name = "Courier New"; r.font.size = Pt(9)
                    r._element.rPr.rFonts.set(qn("w:eastAsia"), "Courier New")
            else:
                r = p.add_run(v); r.font.size = Pt(10)
    for row in t.rows:
        for c, w in zip(row.cells, widths): c.width = Cm(w)
    doc.add_paragraph().paragraph_format.space_after = Pt(2)


para("LIVE DEMO CHEAT SHEET", 16, True, WD_ALIGN_PARAGRAPH.CENTER, 2)
para("Design and Simulation of Industry Standard AMRs (AgileX Scout Mini) - MATLAB R2025b", 11, False, WD_ALIGN_PARAGRAPH.CENTER, 8)

para("0. Before the demo", 12, True)
para("Type these once in the MATLAB Command Window. Every later command assumes this folder. A ready-made script, live_demo.m, holds all commands below in "
     "sections: open it in MATLAB and run one section at a time with Ctrl+Enter.")
table(["Step", "What it does", "Command"],
      [["1", "Go to the project folder and add the root folder (script simulation) to the path",
        "cd 'D:\\academic_shit\\SEM5-III\\AMR ROS\\simulink_amr'\naddpath(pwd); addpath('..');"],
       ["2", "Close old copies of the model and load the parameters (do this if anything looks wrong)",
        "close_system('scout_amr_sim',0); bdclose all; scout_amr_params;"],
       ["3", "Open live_demo.m (all commands, one section per figure)", "edit live_demo"]],
      [1.2, 7.0, 9.4])

para("1. Command for each figure of the report", 12, True)
rows = [
    ["Fig. 1-3  SolidWorks model", "The CAD assembly (needs SolidWorks); otherwise show the pictures in the report",
     "winopen('..\\scout_mini_sw\\Assem2.SLDASM')"],
    ["Fig. 4  importrobot", "URDF read by MATLAB as a rigidBodyTree, rotate with the mouse",
     "rbt = importrobot('scout_mini.urdf');\nshow(rbt, homeConfiguration(rbt), 'Visuals', 'on');\nshowdetails(rbt)"],
    ["Fig. 5  CAD meshes + wheel axes", "Meshes placed with the URDF transforms (saved picture)",
     "imshow('report/figs/robot_render.png')"],
    ["Fig. 6  Simulink top level", "World, Mechanism Configuration, Solver and the four colour-coded subsystems",
     "open_system('scout_amr_sim')"],
    ["Fig. 7  Mission_and_Slip_Controller", "Zero-order holds, Controller block, logging; tc_on switch",
     "open_system('scout_amr_sim/Mission_and_Slip_Controller')"],
    ["Section 7 subsystems", "Robot, tyre model (one MATLAB Function per wheel), warehouse, movers",
     "open_system('scout_amr_sim/Scout_Mini_Robot')\nopen_system('scout_amr_sim/Scout_Mini_Robot/Tyre_Ground_Contact_Model')\nopen_system('scout_amr_sim/Warehouse')\nopen_system('scout_amr_sim/Moving_Obstacles')"],
    ["Fig. 23-25  Multibody Explorer", "The 3D Simscape scene; the robot drives the mission live (about 30 s). Buttons Iso / Top change the view",
     "scout_amr_params; open_system('scout_amr_sim');\nset_param('scout_amr_sim','SimMechanicsOpenEditorOnUpdate','on');\nset_param('scout_amr_sim','SimulationCommand','update');\nset_param('scout_amr_sim','SimulationCommand','start');"],
    ["Fig. 8-10  3D renders", "Warehouse at t = 17.4, 29.3, 40.7 s (saved pictures)",
     "imshow('report/figs/warehouse3d_t17.png')   % t29, t41 likewise"],
    ["Fig. 11-15  Result graphs", "Trajectory, LiDAR scenarios, velocities, safety, slip: redrawn from the saved run (a few seconds)",
     "run_scout_warehouse_simulation('replot')"],
    ["Fig. 16  Slip ON vs OFF", "Comparison figure from the saved run",
     "imshow('results/6_slip_on_vs_off.png')"],
    ["Fig. 17-20  Diagrams", "Architecture, toolchain, mission state machine, control step",
     "imshow('report/figs/diag_architecture.png')   % diag_toolchain, diag_mission_fsm, diag_control_step"],
    ["Fig. 21  Trajectory map", "Top-down map with footprints and LiDAR fans",
     "make_example_style_outputs(false);\nimshow('results/trajectory_map.png')"],
    ["Fig. 22  Signals", "Clearance, commands, wheel speeds, chassis height / tilt",
     "imshow('results/signals.png')"],
    ["Animation (GIF / MP4)", "Top-down animation of the whole mission",
     "winopen('results/amr_warehouse_avoidance.mp4')\n% rebuild (a few minutes): make_example_style_outputs"],
    ["Fig. 26-29  Script simulation", "ISO 3691-4 style simulation: camera buttons, Replan Route, Emergency Stop (runs about 45 s)",
     "cd('..'); simulate_industry_amr_warehouse; cd('simulink_amr');"],
    ["Manual driving (optional)", "Same robot with the keyboard: W/A/S/D or arrows, Space = stop",
     "cd('..'); teleop_industry_amr_warehouse; cd('simulink_amr');"],
    ["Fig. 30-32  Script telemetry", "Speed, yaw rate, slip, clearance with ISO fields, drift, path",
     "addpath('report'); make_industry_graphs;\nimshow('report/figs/industry_g2_safety_sensors.png')"],
]
table(["Report item", "What it shows", "Command"], rows, [3.6, 5.4, 8.6])

para("2. Running the full simulation live", 12, True)
table(["Goal", "Command", "Note"],
      [["Mission with 3D Multibody Explorer, then all graphs", "run_scout_warehouse_simulation", "about 30 s; replaces results/ with the slip-management-ON run only"],
       ["Faster, no 3D window", "run_scout_warehouse_simulation(false)", "headless, about 30 s"],
       ["Slip management ON and OFF with comparison (Fig. 16)", "run_scout_warehouse_simulation(false, true)", "about 1 min; keeps both runs"],
       ["Slip management off by hand", "set_param('scout_amr_sim/Mission_and_Slip_Controller/tc_on','Value','0')\nsim('scout_amr_sim')", "set it back to '1' afterwards"],
       ["Rebuild the model from the URDF", "build_scout_amr_model", "reuses scout_import_raw.slx; the first import takes several minutes"],
       ["Open the finished report", "winopen('report/Scout_Mini_Warehouse_AMR_Report.pdf')", "36 pages"]],
      [5.2, 7.0, 5.4])

para("3. Suggested order for a 10-minute live demo", 12, True)
for t in ["Setup (step 0), then open_system('scout_amr_sim'): explain the four subsystems (Fig. 6), open the controller (Fig. 7) and the tyre model.",
          "Run the Multibody Explorer lines (Fig. 23-25): the robot drives the warehouse mission while you describe LiDAR, A* re-planning and the moving obstacles.",
          "run_scout_warehouse_simulation('replot'): walk through the trajectory, safety and slip graphs (Fig. 11-15).",
          "Show Fig. 16 and state the honest result: slip management lowers the peak slip (0.48 against 0.64) and costs about 4 s of mission time; both runs are collision-free.",
          "Show trajectory_map.png, signals.png and the MP4 animation (Fig. 21-22).",
          "Finish with simulate_industry_amr_warehouse (the earlier script simulation, Fig. 26-29): use the camera buttons and Emergency Stop."]:
    p = doc.add_paragraph(t, style="List Number"); p.paragraph_format.space_after = Pt(2)

para("4. If something goes wrong", 12, True)
table(["Problem", "Fix"],
      [["Multibody Explorer does not appear", "Run the four Fig. 23-25 lines (the 'update' command opens it); check the taskbar, it may be behind other windows."],
       ["Undefined variable / parameter errors in the model", "scout_amr_params   (then run again)"],
       ["Old model copy open or the model looks different", "close_system('scout_amr_sim',0); bdclose all; open_system('scout_amr_sim')"],
       ["Figures show the wrong run or OFF data is missing", "run_scout_warehouse_simulation(false, true)   (re-creates both runs)"],
       ["simulate_industry_amr_warehouse cannot find plan_warehouse_roadmap", "addpath('..')  or  cd('..') before running it"],
       ["Window stays open after the demo", "close all; bdclose all"]],
      [6.2, 11.4])


para("5. Explanation of every graph: what it shows and why it is used", 12, True)
para("Part A - Simscape Multibody model (sections 7 to 13 of the report). All numbers are from the reference run with slip management on: 49.6 s, 32.0 m, 3 route corrections, 0 collisions, minimum clearance 0.22 m.")
G = [
 ["Fig. 1-3 SolidWorks views", "Isometric, side and front view of the Scout Mini CAD (chassis, top cover, four wheels, axle brackets).", "Shows that the robot is the author's own design and where the geometry comes from.", "Say: the CAD cannot be opened in MATLAB, so it is exported to URDF and STL (section 5)."],
 ["Fig. 4 importrobot", "The robot as a rigidBodyTree in MATLAB.", "Checks that the URDF and the STL meshes were read correctly (links, joints, wheel positions).", "Rotate it live; four wheels and the base link must be at the right places."],
 ["Fig. 5 CAD meshes + wheel axes", "Meshes placed with the URDF transforms; red arrows are the wheel joint axes.", "Proves the wheel axis direction, which fixes the sign of wheel torque and slip ratio.", "The imported joint frames have their Z axis along the axle; this was found here and built into the tyre model."],
 ["Fig. 6 / 7 Simulink diagrams", "Top level with four subsystems; inside of Mission_and_Slip_Controller.", "Documents the model structure so that it is readable and repeatable.", "Top level has no signal wires; signals travel by Goto/From tags; tc_on switches slip management."],
 ["Fig. 8-10 3D renders", "Warehouse with racks, goods, pads, boxes and the robot at t = 17.4, 29.3 and 40.7 s.", "Shows the environment and three situations of the mission (floor box, worker + blocked aisle, final approach).", "Red lines in Fig. 9 are the LiDAR returns."],
 ["Fig. 11 Warehouse map with trajectory", "Top view of the hall; driven path (blue to the pickup, red to the delivery), all planned paths (grey), re-route events (stars), tracks of the moving obstacles (dotted).", "The main picture of the mission: shows that the robot reached both stations through the aisles and where it re-planned.", "Three stars = three route corrections (25.1, 30.2, 37.6 s). Pickup reached at 6.1 s, delivery at 46.6 s."],
 ["Fig. 12 LiDAR scenarios", "Four snapshots: LiDAR scan (red), re-planned path (green), driven path (blue), moving obstacles.", "Shows how the robot perceives and avoids: the floor box, the crossing worker, the blocked aisle, worker 2 in aisle 3.", "Perception (red) -> new plan (green) -> motion (blue): the planner and the LiDAR work together."],
 ["Fig. 13 Velocities and wheel speeds", "Linear speed and yaw rate (commanded and measured), and the four wheel speeds against time.", "Shows that the controller follows its commands and that left and right wheel speeds differ in turns (skid steering).", "Speed up to 1.2 m/s; the measured speed follows the command; flat parts are loading and unloading (3 s each)."],
 ["Fig. 14 Perception and safety", "LiDAR ranges, ground-truth clearance, route corrections and mission state (shaded bands).", "Safety evidence: clearance to every obstacle, counted collisions, and when each mission state is active.", "Clearance never reaches zero (minimum 0.22 m); 0 collisions; the closest LiDAR hit was 0.58 m."],
 ["Fig. 15 Slip management", "Wheel slip ratios with the +-0.12 threshold, wheel torques after limiting, friction estimate against the true friction, tyre normal loads.", "Core of the slip topic: shows how slip is detected, how the torque is limited and how the robot estimates the floor friction.", "The estimate falls to 0.10 on oil and water and recovers on dry floor with a lag of seconds; normal load is about 60-80 N per wheel (30 kg / 4); torque and load traces are noisy because the contact model is stiff."],
 ["Fig. 16 Slip ON vs OFF", "Worst-wheel slip ratio and forward speed for both runs (blue ON, orange OFF).", "The fair comparison: same world and mission, only tc_on changes.", "ON: peak slip 0.48, 2.1 % of the time above 0.2, 49.6 s. OFF: peak slip 0.64, 5.4 %, 45.5 s. Both collision-free: less slip costs about 4 s."],
 ["Fig. 17-20 Diagrams", "Architecture, toolchain, mission state machine, flow of one control step.", "Explain the closed loop and the software structure without code.", "Loop: sensors -> LiDAR / grid / A* -> controller + slip management -> tyre forces -> Simscape plant."],
 ["Fig. 21 Trajectory map", "Dark top-down map: path, robot footprints every 2.5 s, LiDAR fans, patches, re-route stars.", "Presentation version of Fig. 11 in the style of the reference examples.", "Footprints show the robot orientation along the route."],
 ["Fig. 22 Signals", "Four panels: clearance, controller commands, wheel-speed commands, chassis height and tilt.", "Shows safety, control and that the physics is sane in one figure.", "Chassis stays within 0.7 mm of the floor height and 0.6 deg of tilt: the contact model carries the robot without floating or sinking."],
 ["Fig. 23-25 Multibody Explorer", "Screenshots of the 3D Simscape scene at t = 0, 3.6 and 18.7 s.", "Proof that the model runs as a real multibody simulation, not an animation.", "Model tree: Moving_Obstacles, Scout_Mini_Robot, Warehouse, Mechanism Configuration, World."],
]
table(["Figure", "What is plotted", "Why it is used", "How to read it / what to say"], G, [2.6, 5.0, 4.6, 5.4])
para("Part B - MATLAB-script simulation simulate_industry_amr_warehouse.m (section 14). It is a kinematic and sensor-level model, not a multibody simulation: 44.7 s, 32.3 m, no collision reported.")
G2 = [
 ["Fig. 26-29 Screenshots", "The live window: 3D warehouse or top view on the left; occupancy grid, kinematics and safety telemetry on the right.", "Shows the ISO 3691-4 style behaviour and the camera views (isometric, top, chase, dock).", "Occupancy grid (black = occupied) fills up as the LiDAR scans; the final picture shows the full 44.7 s of telemetry."],
 ["Fig. 30 Kinematics", "Linear speed (limit 0.8 m/s), yaw rate (limit 1.4 rad/s), front-left motor speed, slip ratio.", "Shows the motion profile of the script robot.", "Speed drops to zero at the three route corners where the robot pivots; the slip ratio stays at zero because the script has only a drift model, not a tyre contact model."],
 ["Fig. 31 Driven path", "x-y path from the inbound depot to the shipping dock.", "Shows the route that the waypoint roadmap produced.", "Up the west aisle, east along y = 5 m, north at x = 14.6 m, east to the dock: 32.3 m."],
 ["Fig. 32 Safety and sensors", "LiDAR clearance with the 1.80 m warning, 1.05 m braking and 0.65 m stop fields; odometry drift; IMU yaw-rate magnitude.", "Shows how the safety fields are used and why wheel odometry alone is not enough.", "Clearance falls to 0.22 m between the racks; the 0.65 m stop field is not a hard limit here. Odometry drift grows to 0.42 m (2 % built-in drift); the IMU peaks are the three turns."],
]
table(["Figure", "What is plotted", "Why it is used", "How to read it / what to say"], G2, [2.6, 5.0, 4.6, 5.4])
para("Honest limits to mention when asked: both simulations use the true robot pose (no localisation), analytic sensing and scripted moving obstacles; slip management gives smoother wheel-ground behaviour, not the difference between success and failure; nothing was tested on the real robot.", 10)

doc.save(os.path.join(HERE, "Live_Demo_Cheat_Sheet_v2.docx"))
print("saved Live_Demo_Cheat_Sheet_v2.docx")
