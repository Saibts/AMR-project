function capture_industry_full()
%CAPTURE_INDUSTRY_FULL  Runs simulate_industry_amr_warehouse unchanged, grabs screenshots in all four camera views while it
%  runs, then dumps the telemetry lines of the final figure to report/figs/industry_telemetry.mat.
root = 'D:\academic_shit\SEM5-III\AMR ROS'; cd(root);
out = fullfile(root, 'simulink_amr', 'report', 'figs');
setappdata(0, 'capN', 0);
t = timer('ExecutionMode', 'fixedSpacing', 'Period', 9, 'StartDelay', 8, 'TimerFcn', @(~,~) grab(out));
start(t);
try, simulate_industry_amr_warehouse(); catch e, disp(e.message); end
stop(t); delete(t);
f = findobj('Type', 'figure'); f = f(1); drawnow; pause(1);
exportgraphics(f, fullfile(out, 'industry_final.png'), 'Resolution', 120);
D = struct('ax', {}, 'ttl', {}, 'xl', {}, 'yl', {}, 'name', {}, 'x', {}, 'y', {});
ax = findobj(f, 'Type', 'axes');
for a = ax(:)'
    L = findobj(a, 'Type', 'line');
    for l = L(:)'
        x = get(l, 'XData'); y = get(l, 'YData');
        if numel(x) > 50
            D(end+1) = struct('ax', a.Position, 'ttl', string(get(get(a, 'Title'), 'String')), 'xl', string(get(get(a, 'XLabel'), 'String')), ...
                'yl', string(get(get(a, 'YLabel'), 'String')), 'name', string(get(l, 'DisplayName')), 'x', x(:), 'y', y(:)); %#ok<AGROW>
        end
    end
end
save(fullfile(out, 'industry_telemetry.mat'), 'D');
fprintf('captured %d telemetry lines\n', numel(D));
close all force;
end
function grab(out)
    n = getappdata(0, 'capN') + 1; setappdata(0, 'capN', n);
    f = findobj('Type', 'figure'); if isempty(f), return; end
    f = f(1);
    views = {'Isometric View', 'Top View (Plan)', 'Chase Camera', 'Dock Camera'};
    names = {'iso', 'top', 'chase', 'dock'};
    k = mod(n - 1, 4) + 1;
    b = findobj(f, 'Style', 'pushbutton', 'String', views{k});
    if ~isempty(b), try, b.Callback(b, []); catch, end, end
    pause(0.8);
    try, exportgraphics(f, fullfile(out, sprintf('industry_%02d_%s.png', n, names{k})), 'Resolution', 110); catch e, disp(e.message); end
end
