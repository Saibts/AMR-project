%% LIVE DEMO - one section per figure group of the report. Run a section with Ctrl+Enter (or the "Run Section" button).
%  Start here once per MATLAB session:
cd 'D:\academic_shit\SEM5-III\AMR ROS\simulink_amr'; addpath(pwd); addpath('..');
root = pwd; figs = fullfile(root, 'report', 'figs'); res = fullfile(root, 'results');

%% Fig. 2-4  SolidWorks model (needs SolidWorks; otherwise show the screenshots from the repository folder)
winopen(fullfile(root, '..', 'scout_mini_sw', 'Assem2.SLDASM'));
% images:  figure; imshow(fullfile(root, '..', 'scout_mini_sw', 'images', 'Screenshot 2026-08-17 122601.png'));

%% Fig. 6  URDF imported into MATLAB (importrobot -> rigidBodyTree)
rbt = importrobot('scout_mini.urdf');
figure('Color', 'w'); show(rbt, homeConfiguration(rbt), 'Visuals', 'on'); title('Scout Mini imported with importrobot');
showdetails(rbt)

%% Fig. 7  CAD meshes with the wheel axes
figure('Color', 'w', 'Position', [60 60 1100 700]); imshow(fullfile(figs, 'robot_render.png'));

%% Fig. 9  Simulink top level (World, Mechanism Configuration, Solver, four subsystems)
close_system('scout_amr_sim', 0); scout_amr_params; open_system('scout_amr_sim');

%% Fig. 10  Subsystem Mission_and_Slip_Controller (tc_on = slip management switch)
open_system('scout_amr_sim/Mission_and_Slip_Controller');

%% Section 7 subsystems: robot, tyre model, warehouse, moving obstacles
open_system('scout_amr_sim/Scout_Mini_Robot');
open_system('scout_amr_sim/Scout_Mini_Robot/Tyre_Ground_Contact_Model');
open_system('scout_amr_sim/Warehouse');
open_system('scout_amr_sim/Moving_Obstacles');

%% Fig. 11-13  Multibody Explorer: the 3D Simscape scene (press Play at the bottom, or run the lines below)
scout_amr_params; open_system('scout_amr_sim');
set_param('scout_amr_sim', 'SimMechanicsOpenEditorOnUpdate', 'on');
set_param('scout_amr_sim', 'SimulationCommand', 'update');    % opens the Multibody Explorer
set_param('scout_amr_sim', 'SimulationCommand', 'start');     % plays the mission live (about 30 s)

%% Fig. 15-18, 20  All result graphs of the Simscape run, redrawn from the saved results (fast, no simulation)
run_scout_warehouse_simulation('replot');       % Fig. 15 trajectory, 16 LiDAR scenarios, 17 velocities, 18 safety, 20 slip
% the ON / OFF comparison of Fig. 21:
figure('Color', 'w', 'Position', [60 60 1300 560]); imshow(fullfile(res, '6_slip_on_vs_off.png'));

%% Fig. 19 and the animation: map and signals of the run, GIF / MP4
make_example_style_outputs(false);              % rebuilds results/trajectory_map.png and results/signals.png (no video)
figure('Color', 'w', 'Position', [60 60 1400 700]); imshow(fullfile(res, 'trajectory_map.png'));
figure('Color', 'w', 'Position', [60 60 1000 1000]); imshow(fullfile(res, 'signals.png'));
winopen(fullfile(res, 'amr_warehouse_avoidance.mp4'));      % plays the animation in the video player

%% Run the whole mission live and print the numbers (30 s; overwrites results/, so the ON/OFF data of Fig. 21 is replaced by ON only)
% run_scout_warehouse_simulation;                % with the Multibody Explorer, then all graphs
% run_scout_warehouse_simulation(false, true);   % ON and OFF, both saved (about 1 minute) - use this to keep the comparison

%% Switch slip management by hand (tc_on: 1 = on, 0 = off) and run
set_param('scout_amr_sim/Mission_and_Slip_Controller/tc_on', 'Value', '0');   % OFF
% sim('scout_amr_sim');  ...then set it back:
set_param('scout_amr_sim/Mission_and_Slip_Controller/tc_on', 'Value', '1');   % ON

%% Fig. 1, 5, 8, 14, 22, 23  Diagrams (toolchain, URDF tree, robot subsystem, mission state machine, architecture, control step)
for f = {'diag_toolchain.png', 'diag_urdf_tree.png', 'diag_robot_subsystem.png', 'diag_mission_fsm.png', 'diag_architecture.png', 'diag_control_step.png'}
    figure('Color', 'w', 'Position', [60 60 1300 560]); imshow(fullfile(figs, f{1})); title(f{1}, 'Interpreter', 'none');
end

%% Fig. 24  MATLAB-script visual simulation (ISO 3691-4 style) - interactive window with camera buttons and E-stop
cd(fullfile(root, '..')); simulate_industry_amr_warehouse; cd(root);

%% Same robot driven by hand (W/A/S/D or arrow keys, Space = stop)
% cd(fullfile(root, '..')); teleop_industry_amr_warehouse; cd(root);

%% Fig. 25-27  Telemetry graphs of the script simulation (from the captured data)
addpath(fullfile(root, 'report')); make_industry_graphs;   % writes report/figs/industry_g1..g3 and prints the summary
for f = {'industry_g1_kinematics.png', 'industry_g2_safety_sensors.png', 'industry_g3_path.png'}
    figure('Color', 'w', 'Position', [60 60 1100 760]); imshow(fullfile(figs, f{1}));
end

%% Open the finished report
winopen(fullfile(root, 'report', 'Scout_Mini_Warehouse_AMR_Report.pdf'));
