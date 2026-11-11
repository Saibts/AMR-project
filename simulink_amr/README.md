# AgileX Scout Mini: Autonomous Warehouse Robot (Simulink + Simscape Multibody)

Your SolidWorks robot (URDF + STL meshes from `AgileX_ScoutMini_JAZZY`) is imported into Simscape
Multibody and drives an autonomous pickup-to-delivery mission in a 3D warehouse that is part of the
physical simulation.

## Run it

```matlab
cd('D:\academic_shit\SEM5-III\AMR ROS\simulink_amr')
open_system('scout_amr_sim')              % the Simulink / Simscape model (press Run; 3D view opens)
run_scout_warehouse_simulation            % run + all graphs  (add (false) to skip the 3D window)
run_scout_warehouse_simulation(false,true)% also re-run with slip management OFF and compare (slow)
build_scout_amr_model                     % rebuild scout_amr_sim.slx from scratch
```
A full mission takes about 2 to 3 minutes of wall time with slip management on (1 to 2 minutes for the off run). Figures go to `results/`.

Presentation outputs (trajectory map, signals figure, MP4 animation) come from `make_example_style_outputs`. The older script-only visual simulation is `../simulate_industry_amr_warehouse.m` (with `../plan_warehouse_roadmap.m`).

## Architecture

| Layer | What | Where |
|---|---|---|
| Robot | URDF imported with `smimport`: chassis + 4 wheels (STL), masses/inertias from the URDF, chassis on a 6-DOF joint, 4 torque-driven revolute wheel joints | `scout_mini.urdf`, `meshes/`, `scout_amr_sim.slx` |
| Wheel-ground | Compliant vertical contact + smoothed Coulomb friction per wheel, applied with External Force and Torque blocks; floor friction map (dry concrete 0.9, oil 0.15, water 0.30) | `scout_amr_tires.m`, block group *Tyre_Ground_Contact_Model* |
| Warehouse (3D) | Floor, 3 rack rows with goods, walls, pickup/delivery/start pads, floor boxes, low-friction spills | subsystem `Warehouse` (Brick Solids) |
| Moving obstacles | 2 workers, a second AMR, a pallet truck: each on two prismatic joints driven by the controller block | `scout_wh_world.m` |
| LiDAR | 72-beam, 8 m, 10 Hz planar scan ray-cast against the world geometry | `scout_wh_lidar.m` |
| Planner | A* on an inflated occupancy grid (racks/walls known; boxes, workers and the truck only learned from LiDAR), line-of-sight smoothing, replan every 0.5 s or when the path is blocked | `scout_wh_plan.m`, `scout_wh_nav.m` |
| Navigation | Mission state machine (start -> pickup -> load 3 s -> delivery -> unload 3 s), pure pursuit, LiDAR repulsion, stop-before-contact speed limit, back-away behaviour | `scout_wh_nav.m` |
| Drive control | Skid-steer kinematics, wheel-speed PI with acceleration feed-forward, traction control (slip-ratio window, torque limiting, friction estimator) | `scout_amr_controller.m` |
| Motor / battery | 4 x 150 W hub motors, 24 V 15 Ah battery. Torque-speed limit `min(15 N m, Kt (V - Ke w) / R)` acts on every wheel in the controller; current, power, battery voltage and charge are calculated from the logged torques; hand calculation of the driving cases and the grip limit | `scout_amr_params.m`, `scout_amr_drive_sizing.m`, `scout_amr_drive_post.m` |
| Monitoring | Ground-truth clearance and collision counter (not used by the controller) | `scout_wh_nav.m` |
| Parameters | All constants and the warehouse layout | `scout_amr_params.m` |

The top level of `scout_amr_sim.slx` holds only World, Mechanism Configuration, Solver Configuration and four subsystems: `Scout_Mini_Robot` (imported robot, sensors, wheel drives and the nested `Tyre_Ground_Contact_Model`), `Warehouse`, `Moving_Obstacles` and `Mission_and_Slip_Controller`. Signals between them travel through Goto/From tags.
Inside the controller subsystem, the `tc_on` constant switches slip management (1 = on, 0 = off).

## Demonstration scenarios (all in one mission)

1. Driving along a warehouse aisle.
2. Navigating around shelves (cross-aisles between rack segments).
3. Detecting and avoiding the floor box in aisle 2.
4. Avoiding the worker who crosses aisle 2.
5. Travelling from the pickup to the delivery station.
6. Alternative path: a pallet truck drives into aisle 2 and blocks it; the robot detects it with LiDAR and re-routes through the cross-aisle and aisle 3. A second worker walks aisle 3.

## Results of the reference run (slip management ON)

Mission time 51.5 s, 32.5 m driven, 5 route corrections, **0 collisions**, minimum clearance 0.12 m,
peak slip ratio 0.46 (on the oil and water spills), top speed 1.2 m/s. Peak wheel torque 5.9 N m (rated 5 N m, peak 15 N m),
1.06 Wh taken from the 360 Wh battery.

| Figure | Content |
|---|---|
| `results/1_warehouse_trajectory.png` | Map, planned paths, driven trajectory, re-route events, mover trails |
| `results/2_velocity_wheel_speeds.png` | Linear / angular velocity and the four wheel speeds |
| `results/3_perception_safety.png` | LiDAR ranges, clearance, collisions, re-routes |
| `results/4_slip_management.png` | Slip ratios, torques, friction estimate vs truth, tyre loads |
| `results/5_lidar_scenarios.png` | LiDAR scans and replanned path at the four demonstration moments |
| `results/7_motor_battery.png` | Motor torque-speed limit with the run's operating points, motor current, battery power, voltage and charge |
| `results/8_drive_design_cases.png` | Hand-calculated driving cases on the motor torque-speed limit, with the grip limits of the three floors |

`scout_amr_drive_sizing` prints the sizing tables (force, torque, current, traction demand per case). The drive calculations are Section 17 of the report.

## Honest limitations

- Obstacle detection uses an analytic ray-cast of the world geometry, not Simscape contact; collisions are
  detected (counted) analytically and the robot does not physically collide with obstacles.
- Workers are scripted to wait while the robot is close to them (awareness of the AMR), but they still
  must be detected and avoided; the second AMR and truck follow fixed trajectories.
- Moving bodies are prescribed-motion bodies (they are not pushed by the robot).
