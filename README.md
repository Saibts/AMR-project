# Design and Simulation of Industry Standard AMRs (AgileX Scout Mini)

Course RA23502 Autonomous Mobile Robot: Sailakshmi B T (2024511019).

A SolidWorks model of the AgileX Scout Mini is imported into MATLAB Simulink / Simscape Multibody and drives an
autonomous pickup-to-delivery mission in a simulated warehouse. The simulation includes a hub-motor torque-speed
limit, a battery calculation and a wheel-slip management on oil and water patches.

## Quick start (MATLAB R2025b)

```matlab
cd 'D:\academic_shit\SEM5-III\AMR ROS\simulink_amr'
live_graphs                                  % 3D Multibody Explorer + live graphs (map, speed, slip, clearance, motor current, battery)
live_graphs(false)                           % live graphs only, no 3D window
run_scout_warehouse_simulation               % run once, then draw all result figures (2 to 3 minutes)
run_scout_warehouse_simulation(false, true)  % also run with slip management off and compare
scout_amr_drive_sizing                       % hand calculation of the motor torque for 8 driving cases
runtests('test_scout_amr_drive')             % six unit tests of the drive model
```

Needs MATLAB, Simulink, Simscape and Simscape Multibody. See `simulink_amr/README.md` for the architecture, the
files and the results.

## Folders

| Path | Content |
|---|---|
| `simulink_amr/` | The Simulink / Simscape model `scout_amr_sim.slx`, the MATLAB code, the results figures and its own README |
| `AMR_report_documents/` | Full report (50 pages), second-review report (25 pages), project guide with cheat sheet, project explanation, live-demo cheat sheet |
| `simulate_industry_amr_warehouse.m`, `plan_warehouse_roadmap.m`, `teleop_industry_amr_warehouse.m` | The earlier script-only visual simulation (ISO 3691-4 style safety fields) |
| `meshes/` | STL shapes of the robot |
| `ROS_Report_Sailakshmi_Final.pdf` | The ROS 2 / Gazebo report of the same robot |

## Documents

- `AMR_report_documents/Scout_Mini_Warehouse_AMR_Second_Review.pdf`: second-review report (motor, torque, traction and battery work, with a short first-review recap).
- `AMR_report_documents/Scout_Mini_Warehouse_AMR_Report_Motor_Torque.pdf`: full report.
- `AMR_report_documents/AMR_Project_Guide.pdf`: plain-language explanation of the project, every MATLAB file, every graph, and the live-demo cheat sheet.

## Reference run (slip management on)

Mission time 51.5 s, 32.5 m driven, 5 route corrections, 0 collisions, minimum clearance 0.12 m, peak slip ratio 0.46,
peak wheel torque 5.9 N·m (rated 5, peak 15), peak motor current 9.2 A, 1.06 Wh from the 360 Wh battery.

## Limits

Simulation only; nothing was tested on the real robot. The pose is the true simulator pose, LiDAR and collision checks are
analytic, the moving obstacles follow scripts, and the motor and battery data are assumptions. The motor current and the
battery are calculated after the run and do not act on the dynamics.
