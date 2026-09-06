function globalPath = globalPlanner(startPose, goalPose, mapData)
% globalPlanner - returns the scenario map's centerline as the coarse
% start-to-goal route when available, falling back to a straight line
% between start and goal otherwise (e.g. a scenario with no map geometry
% yet). Phase 5+ can replace the fallback with real A*/RRT over an
% occupancy grid without changing this function's contract.
%
% Inputs:
%   startPose - [x, y, yaw]
%   goalPose  - [x, y, yaw]
%   mapData   - scenario map struct with field centerline (Nx2), or []
% Output:
%   globalPath - Nx2 array of [x, y] waypoints

if isstruct(mapData) && isfield(mapData, 'centerline') && ~isempty(mapData.centerline)
    globalPath = mapData.centerline;
    return;
end

numPoints = 50;
x = linspace(startPose(1), goalPose(1), numPoints)';
y = linspace(startPose(2), goalPose(2), numPoints)';
globalPath = [x, y];

end
