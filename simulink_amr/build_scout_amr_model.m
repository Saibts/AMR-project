function build_scout_amr_model()
%BUILD_SCOUT_AMR_MODEL  Builds scout_amr_sim.slx: the SolidWorks Scout Mini (URDF/STL) imported into
%  Simscape Multibody, floating on a compliant friction ground with obstacles, low-friction
%  patches, mission controller and traction (slip) management.
here = fileparts(mfilename('fullpath')); cd(here); addpath(here);
P   = scout_amr_params();
mdl = 'scout_amr_sim';
if bdIsLoaded(mdl), close_system(mdl, 0); end
if bdIsLoaded('scout_import_raw'), close_system('scout_import_raw', 0); end

%% 1. Import the SolidWorks/URDF model into Simscape Multibody
if ~isfile('scout_import_raw.slx')
    m = smimport('scout_mini.urdf', 'ModelName', 'scout_import_raw');
    save_system(m, 'scout_import_raw'); close_system(m, 0);
end
load_system('scout_import_raw');
save_system('scout_import_raw', mdl);          % Save As -> scout_amr_sim
load_system('simscape'); load_system('nesl_utility');

%% 2. Free-floating chassis: replace the fixed footprint weld with a 6-DOF joint
w = [mdl '/base_footprint_joint']; pos = get_param(w, 'Position');
lh = get_param(w, 'LineHandles');
delete_line(lh.LConn); delete_line(lh.RConn);
delete_block(w);
J6 = add_block('sm_lib/Joints/6-DOF Joint', [mdl '/chassis_6dof'], 'Position', pos);
cn(mdl, [mdl '/base_footprint'], 'R', 1, J6, 'L', 1);
cn(mdl, J6, 'R', 1, [mdl '/base_link'], 'L', 1);
world = [mdl '/World'];
chassisFrame = J6;

%% 3. Wheel joints: torque input, velocity output
jn = {'j1','j2','j3','j4'};
for k = 1:4
    set_param([mdl '/' jn{k}], 'TorqueActuationMode', 'InputTorque', 'SenseVelocity', 'on');
end

%% 4. Sensors (World-frame pose/velocity of every wheel + chassis)
sens = struct();
for k = 1:4
    jb = [mdl '/' jn{k}];
    sens(k).p = addSensor(mdl, sprintf('w%d_xyz', k), world, jb, 'SenseXYZ',    'm');
    sens(k).v = addSensor(mdl, sprintf('w%d_vel', k), world, jb, 'SenseXYZDot', 'm/s');
    sens(k).R = addSensor(mdl, sprintf('w%d_R',   k), world, jb, 'SenseR',      '1');
    sens(k).w = addSensor(mdl, sprintf('w%d_omg', k), world, jb, 'SenseOmega',  'rad/s');
end
cs.p = addSensor(mdl, 'base_xyz', world, chassisFrame, 'SenseXYZ',    'm');
cs.v = addSensor(mdl, 'base_vel', world, chassisFrame, 'SenseXYZDot', 'm/s');
cs.R = addSensor(mdl, 'base_R',   world, chassisFrame, 'SenseR',      '1');
cs.w = addSensor(mdl, 'base_omg', world, chassisFrame, 'SenseOmega',  'rad/s');

% chassis pose logs (height and tilt of the robot)
bpl = add_block('simulink/Sinks/To Workspace', [mdl '/base_p_log'], 'VariableName', 'base_p_log', 'SaveFormat', 'Timeseries', 'SampleTime', '0.01');
brl = add_block('simulink/Sinks/To Workspace', [mdl '/base_R_log'], 'VariableName', 'base_R_log', 'SaveFormat', 'Timeseries', 'SampleTime', '0.01');
sl(mdl, cs.p, 1, bpl, 1); sl(mdl, cs.R, 1, brl, 1);

%% 5. Wheel joint velocity -> controller
wjMux = add_block('simulink/Signal Routing/Mux', [mdl '/wj_mux'], 'Inputs', '4');
for k = 1:4
    pv = add_block(ps2sl(), [mdl sprintf('/wj%d_ps2sl', k)]);
    setIf(pv, 'Unit', 'rad/s');
    cn(mdl, [mdl '/' jn{k}], 'R', 2, pv, 'L', 1);
    sl(mdl, pv, 1, wjMux, k);
end

%% 6. Controller (discrete)
tcC = add_block('simulink/Sources/Constant', [mdl '/tc_on'], 'Value', '1');   % 1 = slip management ON, 0 = OFF (edit inside Mission_and_Slip_Controller)
zp = addZOH(mdl, 'zoh_p', cs.p, P.Ts);  zR = addZOH(mdl, 'zoh_R', cs.R, P.Ts);
zv = addZOH(mdl, 'zoh_v', cs.v, P.Ts);  zw = addZOH(mdl, 'zoh_w', cs.w, P.Ts);
zj = addZOH(mdl, 'zoh_wj', wjMux, P.Ts);
ctl = addMF(mdl, 'Controller', ['function [tau_j, lg, mv, rng, pth] = fcn(pb, Rb, vb, wb, wj, tc_on)' newline ...
    '[tau_j, lg, mv, rng, pth] = scout_amr_controller(pb, Rb, vb, wb, wj, tc_on);' newline 'end']);
sl(mdl, zp, 1, ctl, 1); sl(mdl, zR, 1, ctl, 2); sl(mdl, zv, 1, ctl, 3);
sl(mdl, zw, 1, ctl, 4); sl(mdl, zj, 1, ctl, 5); sl(mdl, tcC, 1, ctl, 6);

dmx = add_block('simulink/Signal Routing/Demux', [mdl '/tau_demux'], 'Outputs', '4');
sl(mdl, ctl, 1, dmx, 1);
for k = 1:4
    pc = add_block(sl2ps(), [mdl sprintf('/tau%d_sl2ps', k)]);
    setIf(pc, 'Unit', 'N*m');
    sl(mdl, dmx, k, pc, 1);
    cn(mdl, pc, 'R', 1, [mdl '/' jn{k}], 'L', 2);
end
lgBlk = add_block('simulink/Sinks/To Workspace', [mdl '/ctrl_log'], 'VariableName', 'ctrl_log', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '0.01');
sl(mdl, ctl, 2, lgBlk, 1);
done = add_block('simulink/Signal Routing/Selector', [mdl '/done_sel'], 'InputPortWidth', '44', ...
    'IndexMode', 'One-based', 'IndexOptionArray', {'Index vector (dialog)'}, 'IndexParamArray', {'33'}, ...
    'OutputSizeArray', {'1'});
sl(mdl, ctl, 2, done, 1);
% extra logs: LiDAR scan, planned path, mover positions
rngL = add_block('simulink/Sinks/To Workspace', [mdl '/lidar_log'], 'VariableName', 'lidar_log', 'SaveFormat', 'Timeseries', 'SampleTime', '0.2');
pthL = add_block('simulink/Sinks/To Workspace', [mdl '/path_log'],  'VariableName', 'path_log',  'SaveFormat', 'Timeseries', 'SampleTime', '0.5');
mvL  = add_block('simulink/Sinks/To Workspace', [mdl '/mover_log'], 'VariableName', 'mover_log', 'SaveFormat', 'Timeseries', 'SampleTime', '0.1');
sl(mdl, ctl, 4, rngL, 1); sl(mdl, ctl, 5, pthL, 1); sl(mdl, ctl, 3, mvL, 1);
mvDmx = add_block('simulink/Signal Routing/Demux', [mdl '/mover_demux'], 'Outputs', '8');
mvDly = add_block('simulink/Discrete/Unit Delay', [mdl '/mover_delay'], 'SampleTime', num2str(P.Ts), 'InitialCondition', '[14; 3.0; 20; 7.5; 13; -0.4; 27; 4.2]');   % breaks the algebraic loop of the prescribed-motion bodies
sl(mdl, ctl, 3, mvDly, 1); sl(mdl, mvDly, 1, mvDmx, 1);
stp = add_block('simulink/Sinks/Stop Simulation', [mdl '/stop_when_done']);
sl(mdl, done, 1, stp, 1);

%% 7. Tyre / ground contact: one MATLAB Function + two external wrenches per wheel
tireMux = add_block('simulink/Signal Routing/Mux', [mdl '/tire_mux'], 'Inputs', '4');
for k = 1:4
    jb = [mdl '/' jn{k}];
    tf = addMF(mdl, sprintf('Tire%d', k), ['function out = fcn(pO, vO, R, w, Rb)' newline ...
        sprintf('out = scout_amr_tires(pO, vO, R, w, Rb, %d);', k) newline 'end']);
    sl(mdl, sens(k).p, 1, tf, 1); sl(mdl, sens(k).v, 1, tf, 2);
    sl(mdl, sens(k).R, 1, tf, 3); sl(mdl, sens(k).w, 1, tf, 4); sl(mdl, cs.R, 1, tf, 5);
    selF = addSel(mdl, sprintf('F%d_sel', k), 11, '[1 2 3]');
    selT = addSel(mdl, sprintf('T%d_sel', k), 11, '[4 5 6]');
    sl(mdl, tf, 1, selF, 1); sl(mdl, tf, 1, selT, 1);
    pF = add_block(sl2ps(), [mdl sprintf('/F%d_sl2ps', k)]); setIf(pF, 'Unit', 'N');
    pT = add_block(sl2ps(), [mdl sprintf('/T%d_sl2ps', k)]); setIf(pT, 'Unit', 'N*m');
    sl(mdl, selF, 1, pF, 1); sl(mdl, selT, 1, pT, 1);
    eft = add_block('sm_lib/Forces and Torques/External Force and Torque', [mdl sprintf('/WheelWrench%d', k)]);
    set_param(eft, 'EnableForce', 'on', 'EnableTorque', 'on');
    cn(mdl, pF, 'R', 1, eft, 'L', 1);
    cn(mdl, pT, 'R', 1, eft, 'L', 2);
    cn(mdl, eft, 'R', 1, jb, 'R', 1);
    sl(mdl, tf, 1, tireMux, k);
end
tl = add_block('simulink/Sinks/To Workspace', [mdl '/tire_log'], 'VariableName', 'tire_log', ...
    'SaveFormat', 'Timeseries', 'SampleTime', '0.01');
sl(mdl, tireMux, 1, tl, 1);

%% 8. 3D warehouse (part of the Simscape model): floor, racks with goods, walls, boxes, stations, movers
setappdata(0, 'scout_env_names', {});
addVisual(mdl, world, 'Floor', 'Brick', [34 17 0.02], [12.5 4.9 -0.01], [0.72 0.72 0.74]);
for i = 1:size(P.patches,1)
    q = P.patches(i,:);
    addVisual(mdl, world, sprintf('LowMuPatch%d', i), 'Brick', [q(2)-q(1) q(4)-q(3) 0.012], ...
        [(q(1)+q(2))/2 (q(3)+q(4))/2 0.001], [0.25 0.55 0.90]);
end
addVisual(mdl, world, 'PickupPad',   'Brick', [1.6 1.6 0.014], [P.pickup(1)  P.pickup(2)  0.002], [0.15 0.70 0.30]);
addVisual(mdl, world, 'DeliveryPad', 'Brick', [1.6 1.6 0.014], [P.deliver(1) P.deliver(2) 0.002], [0.85 0.25 0.20]);
addVisual(mdl, world, 'StartPad',    'Brick', [1.2 1.2 0.014], [0 0 0.002], [0.95 0.80 0.15]);
for i = 1:size(P.racks,1)
    q = P.racks(i,:); cx = (q(1)+q(2))/2; cy = (q(3)+q(4))/2; L = q(2)-q(1); D = q(4)-q(3);
    addVisual(mdl, world, sprintf('Rack%d', i), 'Brick', [L D 2.4], [cx cy 1.2], [0.20 0.35 0.65], 0.30);
    for s = 1:3
        addVisual(mdl, world, sprintf('Rack%d_goods%d', i, s), 'Brick', [0.85*L 0.8*D 0.5], [cx cy 0.45 + (s-1)*0.75], ...
                  [0.78 0.58 0.36 + 0.08*mod(i+s,3)]);
    end
end
for i = 1:size(P.walls,1)
    q = P.walls(i,:);
    addVisual(mdl, world, sprintf('Wall%d', i), 'Brick', [q(2)-q(1) q(4)-q(3) 1.2], [(q(1)+q(2))/2 (q(3)+q(4))/2 0.6], [0.60 0.62 0.66]);
end
for i = 1:size(P.boxes,1)
    q = P.boxes(i,:);
    addVisual(mdl, world, sprintf('FloorBox%d', i), 'Brick', [q(2)-q(1) q(4)-q(3) 0.8], [(q(1)+q(2))/2 (q(3)+q(4))/2 0.4], [0.80 0.55 0.25]);
end

% Moving obstacles: two workers, a second AMR and a pallet truck. Each rides on two prismatic joints
% (X then Y) whose positions come from the controller block, so they move inside the multibody scene.
Rx = '[0 0 1; 0 1 0; -1 0 0]';  Ry = '[1 0 0; 0 0 1; 0 -1 0]';  Rw = '[0 0 -1; -1 0 0; 0 1 0]';
mvNames = {'Worker1', 'Worker2', 'SecondAMR', 'PalletTruck'};
mvKind  = {'Cyl', 'Cyl', 'Brick', 'Brick'};
mvDims  = {[0.3 1.7], [0.3 1.7], [0.9 0.6 0.45], [1.4 2.6 0.9]};
mvZ     = [0.85 0.85 0.225 0.45];
mvCol   = {[0.95 0.55 0.10], [0.95 0.80 0.10], [0.15 0.30 0.60], [0.90 0.75 0.10]};
axTag   = 'xy';
for k = 1:4
    nm = mvNames{k};
    rt0 = add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/' nm '_rtx']);
    set_param(rt0, 'RotationMethod', 'RotationMatrix', 'RotationMatrix', Rx);
    jx = add_block('sm_lib/Joints/Prismatic Joint', [mdl '/' nm '_jx']);
    rt1 = add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/' nm '_rty']);
    set_param(rt1, 'RotationMethod', 'RotationMatrix', 'RotationMatrix', Ry);
    jy = add_block('sm_lib/Joints/Prismatic Joint', [mdl '/' nm '_jy']);
    rt2 = add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/' nm '_rtw']);
    set_param(rt2, 'RotationMethod', 'RotationMatrix', 'RotationMatrix', Rw);
    rt3 = add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/' nm '_rtz']);
    set_param(rt3, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', sprintf('[0 0 %g]', mvZ(k)));
    if strcmp(mvKind{k}, 'Brick')
        body = add_block('sm_lib/Body Elements/Brick Solid', [mdl '/' nm]);
        set_param(body, 'BrickDimensions', mat2str(mvDims{k}), 'BasedOnType', 'Mass', 'Mass', '20');
    else
        body = add_block('sm_lib/Body Elements/Cylindrical Solid', [mdl '/' nm]);
        set_param(body, 'CylinderRadius', num2str(mvDims{k}(1)), 'CylinderLength', num2str(mvDims{k}(2)), 'BasedOnType', 'Mass', 'Mass', '20');
    end
    set_param(body, 'GraphicDiffuseColor', mat2str(mvCol{k}));
    set_param(jx, 'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque');
    set_param(jy, 'MotionActuationMode', 'InputMotion', 'TorqueActuationMode', 'ComputedTorque');
    cn(mdl, world, 'R', 1, rt0, 'L', 1);
    cn(mdl, rt0, 'R', 1, jx, 'L', 1);
    cn(mdl, jx, 'R', 1, rt1, 'L', 1);
    cn(mdl, rt1, 'R', 1, jy, 'L', 1);
    cn(mdl, jy, 'R', 1, rt2, 'L', 1);
    cn(mdl, rt2, 'R', 1, rt3, 'L', 1);
    cn(mdl, rt3, 'R', 1, body, 'R', 1);
    for ax = 1:2
        pc = add_block(sl2ps(), [mdl '/' nm '_pos' axTag(ax)]);
        setIf(pc, 'Unit', 'm'); setIf(pc, 'FilteringAndDerivatives', 'filter'); setIf(pc, 'SimscapeFilterOrder', '2'); setIf(pc, 'InputFilterTimeConstant', '0.02');
        sl(mdl, mvDmx, 2*(k-1) + ax, pc, 1);
        jj = jx; if ax == 2, jj = jy; end
        cn(mdl, pc, 'R', 1, jj, 'L', 2);
    end
    regEnv({[nm '_rtx'], [nm '_jx'], [nm '_rty'], [nm '_jy'], [nm '_rtw'], [nm '_rtz'], nm, [nm '_posx'], [nm '_posy']});
end

%% 9. Structure the diagram: signals travel by Goto/From tags, blocks are grouped into subsystems
tagSignals(mdl);
grp(mdl, 'Floor_and_Stations', '^(Floor|LowMuPatch\d|PickupPad|DeliveryPad|StartPad)(_tf)?$');
grp(mdl, 'Storage_Racks',      '^Rack\d+(_goods\d)?(_tf)?$');
grp(mdl, 'Walls',              '^Wall\d(_tf)?$');
grp(mdl, 'Floor_Boxes',        '^FloorBox\d(_tf)?$');
grp(mdl, 'Warehouse',          '^(Floor_and_Stations|Storage_Racks|Walls|Floor_Boxes)$');
mvSub = {'Worker1', 'Worker_1'; 'Worker2', 'Worker_2'; 'SecondAMR', 'Second_AMR'; 'PalletTruck', 'Pallet_Truck'};
for k = 1:4, grp(mdl, mvSub{k,2}, ['^' mvSub{k,1} '(_\w+)?$']); end
grp(mdl, 'Moving_Obstacles', '^(Worker_1|Worker_2|Second_AMR|Pallet_Truck|mover_demux)$');
grp(mdl, 'Mission_and_Slip_Controller', '^(tc_on|zoh_\w+|Controller|ctrl_log|lidar_log|path_log|mover_log|mover_delay|done_sel|stop_when_done)$');
grp(mdl, 'Tyre_Ground_Contact_Model',   '^(Tire\d|F\d_sel|T\d_sel|tire_mux|tire_log)$');
grp(mdl, 'Sensors', '^(w\d|base)_(xyz|vel|R|omg)(_ps2sl)?$');
grp(mdl, 'Wheel_Drives_and_Forces', '^(tau\d_sl2ps|wj\d_ps2sl|F\d_sl2ps|T\d_sl2ps|WheelWrench\d)$');
nm = get_param(find_system(mdl, 'SearchDepth', 1, 'Type', 'Block'), 'Name');
solverName = nm{find(startsWith(nm, 'Solver'), 1)};
keep = {'World', 'MechanismConfiguration', solverName, 'Warehouse', 'Moving_Obstacles', 'Mission_and_Slip_Controller'};
rest = nm(~ismember(nm, [keep, {mdl}]));
h = cellfun(@(n) get_param([mdl '/' n], 'Handle'), rest);
Simulink.BlockDiagram.createSubsystem(h, 'Name', 'Scout_Mini_Robot');
for sub = {'Warehouse', 'Moving_Obstacles', 'Mission_and_Slip_Controller', 'Scout_Mini_Robot'}
    tidy([mdl '/' sub{1}]);
end
layoutTopLevel(mdl, solverName);

%% 10. Solver settings
set_param(mdl, 'InitFcn', 'addpath(fileparts(which(''scout_amr_sim'')));');   % so the helper .m files are always found
set_param(mdl, 'Solver', 'ode23t', 'StopTime', num2str(P.tStop), 'MaxStep', '0.01', ...
    'RelTol', '1e-3', 'SaveOutput', 'off', 'ReturnWorkspaceOutputs', 'on');
save_system(mdl);
fprintf('Built %s.slx (%d top-level blocks)\n', mdl, numel(find_system(mdl, 'SearchDepth', 1)));
end

%% ------------------------------ helpers ---------------------------------
function cn(mdl, a, sa, ia, b, sb, ib)
    pa = get_param(a, 'PortHandles'); pb = get_param(b, 'PortHandles');
    ha = pa.([sa 'Conn'])(ia); hb = pb.([sb 'Conn'])(ib);
    add_line(mdl, ha, hb);
end
function sl(mdl, a, ia, b, ib)
    pa = get_param(a, 'PortHandles'); pb = get_param(b, 'PortHandles');
    add_line(mdl, pa.Outport(ia), pb.Inport(ib));
end
function setIf(blk, prm, val)
    try, set_param(blk, prm, val); catch e, warning('setIf %s.%s: %s', blk, prm, e.message); end
end
function blk = addSel(mdl, name, width, idx)
    blk = add_block('simulink/Signal Routing/Selector', [mdl '/' name], 'InputPortWidth', num2str(width), ...
        'IndexOptionArray', {'Index vector (dialog)'}, 'IndexParamArray', {idx}, ...
        'OutputSizeArray', {'1'});
end
function blk = addSensor(mdl, name, world, frameBlk, opt, unit)
    t = add_block('sm_lib/Frames and Transforms/Transform Sensor', [mdl '/' name]);
    set_param(t, opt, 'on');
    cn(mdl, world, 'R', 1, t, 'L', 1);
    cn(mdl, frameBlk, 'R', 1, t, 'R', 1);
    c = add_block(ps2sl(), [mdl '/' name '_ps2sl']);
    setIf(c, 'Unit', unit);
    cn(mdl, t, 'R', 2, c, 'L', 1);
    blk = c;
end
function blk = addZOH(mdl, name, src, Ts)
    blk = add_block('simulink/Discrete/Zero-Order Hold', [mdl '/' name], 'SampleTime', num2str(Ts));
    sl(mdl, src, 1, blk, 1);
end
function blk = addMF(mdl, name, code)
    blk = [mdl '/' name];
    add_block('simulink/User-Defined Functions/MATLAB Function', blk);
    ch = find(sfroot, '-isa', 'Stateflow.EMChart', 'Path', blk);
    ch.Script = code;
end
function addVisual(mdl, world, name, kind, dims, c, color, opacity)
    if nargin < 8, opacity = 1; end
    rt = add_block('sm_lib/Frames and Transforms/Rigid Transform', [mdl '/' name '_tf']);
    set_param(rt, 'TranslationMethod', 'Cartesian', 'TranslationCartesianOffset', mat2str(c));
    if strcmp(kind, 'Brick')
        s = add_block('sm_lib/Body Elements/Brick Solid', [mdl '/' name]);
        set_param(s, 'BrickDimensions', mat2str(dims));
    else
        s = add_block('sm_lib/Body Elements/Cylindrical Solid', [mdl '/' name]);
        set_param(s, 'CylinderRadius', num2str(dims(1)), 'CylinderLength', num2str(dims(2)));
    end
    set_param(s, 'GraphicDiffuseColor', mat2str(color), 'GraphicOpacity', num2str(opacity));
    cn(mdl, world, 'R', 1, rt, 'L', 1);
    cn(mdl, rt, 'R', 1, s, 'R', 1);
    regEnv({[name '_tf'], name});
end
function regEnv(names)
    c = getappdata(0, 'scout_env_names'); setappdata(0, 'scout_env_names', [c, names(:)']);
end
function p = ps2sl(), p = ['nesl_utility/PS-Simulink' newline 'Converter']; end
function p = sl2ps(), p = ['nesl_utility/Simulink-PS' newline 'Converter']; end

function place(mdl, name, x, y)
    b = [mdl '/' name]; p = get_param(b, 'Position');
    set_param(b, 'Position', [x y x + (p(3)-p(1)) y + (p(4)-p(2))]);
end
function note(mdl, text, x, y)
    try
        a = Simulink.Annotation(mdl, text);
        a.Position = [x y]; a.FontSize = 14; a.FontWeight = 'bold';
    catch
    end
end
function grp(mdl, name, pat)
%GRP  Group the top-level blocks whose names match pat into one subsystem.
    nm = get_param(find_system(mdl, 'SearchDepth', 1, 'Type', 'Block'), 'Name');
    hit = nm(~cellfun(@isempty, regexp(cellfun(@baseName, nm, 'UniformOutput', false), pat, 'once')));
    if isempty(hit), return; end
    h = cellfun(@(n) get_param([mdl '/' n], 'Handle'), hit);
    Simulink.BlockDiagram.createSubsystem(h, 'Name', name);
end
function matchOrder(parent, srcName, dstName)
%MATCHORDER  Renumber the output ports of subsystem srcName so they follow the order of the input ports of dstName.
    sp = [parent '/' srcName]; dp = [parent '/' dstName];
    sh = get_param(sp, 'Handle');
    ph = get_param(dp, 'PortHandles').Inport;
    want = [];
    for k = 1:numel(ph)
        ln = get_param(ph(k), 'Line');
        if ln > 0 && get_param(ln, 'SrcBlockHandle') == sh
            want(end+1) = get_param(get_param(ln, 'SrcPortHandle'), 'PortNumber'); %#ok<AGROW>
        end
    end
    ob = find_system(sp, 'SearchDepth', 1, 'BlockType', 'Outport');
    pn = cellfun(@(b) str2double(get_param(b, 'Port')), ob);
    hb = cellfun(@(b) get_param(b, 'Handle'), ob);
    for k = 1:numel(want)
        blk = hb(pn == want(k)); if isempty(blk), continue; end
        set_param(blk, 'Port', num2str(k));
        ob = find_system(sp, 'SearchDepth', 1, 'BlockType', 'Outport');   % numbers shifted: refresh
        pn = cellfun(@(b) str2double(get_param(b, 'Port')), ob);
        hb = cellfun(@(b) get_param(b, 'Handle'), ob);
        want = want;  %#ok<ASGSL>
        want(k+1:end) = arrayfun(@(w) renum(w, want(k), k), want(k+1:end));
    end
end
function w2 = renum(w, orig, newIdx)
    % a port numbered between newIdx and orig moves down by one when 'orig' is moved to 'newIdx'
    if w >= newIdx && w < orig, w2 = w + 1; else, w2 = w; end
end
function b = baseName(nm)
%BASENAME  Goto/From tag blocks belong to the group of the block they serve (name encoding to__<src>__<n>, from__<dst>__<n>).
    b = nm;
    if startsWith(nm, 'to__') || startsWith(nm, 'from__')
        t = strsplit(nm, '__'); b = t{2};
    end
end
function tagSignals(mdl)
%TAGSIGNALS  Replace every signal line by a Goto/From pair so that no signal wire crosses a subsystem boundary.
    names = get_param(find_system(mdl, 'SearchDepth', 1, 'Type', 'Block'), 'Name');
    names = names(~strcmp(names, mdl));
    for i = 1:numel(names)
        b = [mdl '/' names{i}];
        if any(strcmp(get_param(b, 'BlockType'), {'Goto', 'From'})), continue; end
        ph = get_param(b, 'PortHandles');
        for k = 1:numel(ph.Outport)
            ln = get_param(ph.Outport(k), 'Line');
            if ln <= 0, continue; end
            dst = get_param(ln, 'DstPortHandle'); dst = dst(dst > 0);
            delete_line(ln);
            tag = regexprep(sprintf('%s_%d', names{i}, k), '\W', '_');
            pp = get_param(ph.Outport(k), 'Position');
            g = add_block('simulink/Signal Routing/Goto', sprintf('%s/to__%s__%d', mdl, names{i}, k), ...
                'GotoTag', tag, 'TagVisibility', 'global', 'Position', [pp(1)+25 pp(2)-7 pp(1)+75 pp(2)+7]);
            add_line(mdl, ph.Outport(k), get_param(g, 'PortHandles').Inport(1));
            for d = 1:numel(dst)
                owner = get_param(dst(d), 'Parent'); on = get_param(owner, 'Name'); pn = get_param(dst(d), 'PortNumber');
                dp = get_param(dst(d), 'Position');
                f = add_block('simulink/Signal Routing/From', sprintf('%s/from__%s__%d', mdl, on, pn), ...
                    'GotoTag', tag, 'Position', [dp(1)-75 dp(2)-7 dp(1)-25 dp(2)+7]);
                add_line(mdl, get_param(f, 'PortHandles').Outport(1), dst(d));
            end
        end
    end
end
function tidy(sys)
    try, Simulink.BlockDiagram.arrangeSystem(sys); catch e, warning('arrange %s: %s', sys, e.message); end
    nm = get_param(find_system(sys, 'SearchDepth', 1, 'BlockType', 'SubSystem'), 'Name');
    for i = 1:numel(nm)
        if ~strcmp(nm{i}, get_param(sys, 'Name')) && ~strcmp(get_param([sys '/' nm{i}], 'Mask'), 'on')
            try, Simulink.BlockDiagram.arrangeSystem([sys '/' nm{i}]); catch, end
        end
    end
end
function layoutTopLevel(mdl, solverName)
%LAYOUTTOPLEVEL  Top level has only a handful of blocks: place them in clean columns.
    set_p = @(n, pos) set_param([mdl '/' n], 'Position', pos);
    set_p('World',                  [60 300 120 360]);
    set_p('MechanismConfiguration', [60 420 120 480]);
    set_p(solverName,               [60 540 120 600]);
    set_p('Scout_Mini_Robot',       [420 80 640 200]);
    set_p('Warehouse',              [420 300 640 380]);
    set_p('Moving_Obstacles',       [420 480 640 560]);
    set_p('Mission_and_Slip_Controller', [420 660 640 740]);
    col = struct('Scout_Mini_Robot', '[1.0 0.85 0.55]', 'Warehouse', '[0.75 0.92 0.75]', ...
                 'Moving_Obstacles', '[1.0 0.75 0.75]', 'Mission_and_Slip_Controller', '[0.75 0.85 1.0]');
    for f = fieldnames(col)', set_param([mdl '/' f{1}], 'BackgroundColor', col.(f{1})); end
    note(mdl, 'AgileX Scout Mini AMR in a warehouse (Simscape Multibody)', 300, 20);
    note(mdl, 'Robot: SolidWorks/URDF import + sensors + tyre-ground contact', 660, 130);
    note(mdl, 'Warehouse: racks, goods, walls, stations, floor boxes', 660, 330);
    note(mdl, 'Workers, second AMR, pallet truck (prescribed motion)', 660, 510);
    note(mdl, 'Mission, LiDAR, A*, slip management (signals via Goto/From)', 660, 690);
    try, Simulink.BlockDiagram.routeLine(find_system(mdl, 'SearchDepth', 1, 'FindAll', 'on', 'Type', 'line')); catch, end
    % keep the Mechanism/Solver configuration attached to the World frame
    pw = get_param([mdl '/World'], 'PortHandles');
    pm = get_param([mdl '/MechanismConfiguration'], 'PortHandles');
    if isempty(get_param(pm.RConn(1), 'Line')) || get_param(pm.RConn(1), 'Line') < 0
        add_line(mdl, pw.RConn(1), pm.RConn(1));
    end
end
