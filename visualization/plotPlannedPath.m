function plotPlannedPath(path, egoState, axHandle)
% plotPlannedPath - renders the reference/planned path, the ego vehicle
% position, and its heading on the given axes. Does not clear the axes or
% manage hold state - the caller composes a frame from multiple plot*
% functions (see plotDetectedObjects, plotPredictedTrajectories) and is
% responsible for cla/hold/axis lifecycle around all of them.
%
% Inputs:
%   path     - Nx2 array of [x, y] waypoints
%   egoState - current ego state struct
%   axHandle - target axes handle

if isempty(axHandle) || ~ishghandle(axHandle)
    return;
end

if ~isempty(path)
    plot(axHandle, path(:, 1), path(:, 2), 'b--', 'LineWidth', 1);
end

plot(axHandle, egoState.x, egoState.y, 'ro', 'MarkerFaceColor', 'r', 'MarkerSize', 6);

arrowLen = 2;
quiver(axHandle, egoState.x, egoState.y, ...
       arrowLen * cos(egoState.yaw), arrowLen * sin(egoState.yaw), 0, ...
       'r', 'LineWidth', 1.5, 'MaxHeadSize', 2);

end
