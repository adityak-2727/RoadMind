function plotPlannedPath(path, egoState, axHandle)
% plotPlannedPath - renders the reference/planned path, the ego vehicle
% position, and its heading on the given axes.
%
% Inputs:
%   path     - Nx2 array of [x, y] waypoints
%   egoState - current ego state struct
%   axHandle - target axes handle

if isempty(axHandle) || ~ishghandle(axHandle)
    return;
end

cla(axHandle);
hold(axHandle, 'on');

if ~isempty(path)
    plot(axHandle, path(:, 1), path(:, 2), 'b--', 'LineWidth', 1);
end

plot(axHandle, egoState.x, egoState.y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

arrowLen = 2;
quiver(axHandle, egoState.x, egoState.y, ...
       arrowLen * cos(egoState.yaw), arrowLen * sin(egoState.yaw), 0, ...
       'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);

axis(axHandle, 'equal');
grid(axHandle, 'on');
xlabel(axHandle, 'x [m]');
ylabel(axHandle, 'y [m]');
hold(axHandle, 'off');

end
