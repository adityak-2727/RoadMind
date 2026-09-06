function smoothPath = pathSmoothing(rawPath, smoothingParams)
% pathSmoothing - smooths a raw waypoint path with a moving-average filter
% to remove the small discontinuities between adjacent lateral-offset
% candidates, producing a path the controller can track without jerky
% steering commands. The first point is pinned to the raw path's start so
% smoothing never pulls the path away from the vehicle's actual position.
%
% Inputs:
%   rawPath         - Nx2 array of raw [x, y] waypoints
%   smoothingParams - struct, optional field windowSize (default 5)
% Output:
%   smoothPath - Nx2 array of smoothed [x, y] waypoints

if isempty(rawPath) || size(rawPath, 1) < 5
    smoothPath = rawPath;
    return;
end

if isfield(smoothingParams, 'windowSize')
    windowSize = smoothingParams.windowSize;
else
    windowSize = 5;
end

smoothPath = smoothdata(rawPath, 1, 'movmean', windowSize);
smoothPath(1, :) = rawPath(1, :);

end
