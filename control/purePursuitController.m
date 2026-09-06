function steeringAngle = purePursuitController(egoState, path, lookaheadDist, vehicleConfig)
% purePursuitController - computes steering angle to track a path using the
% pure pursuit geometric method.
%
% Inputs:
%   egoState       - current ego state struct
%   path           - Nx2 array of [x, y] waypoints to track
%   lookaheadDist  - [m] lookahead distance
%   vehicleConfig  - struct from config/vehicleConfig.m (for wheelbase, limits)
% Output:
%   steeringAngle - [rad] steering command, clamped to vehicleConfig.maxSteerAngle

if isempty(path)
    steeringAngle = 0;
    return;
end

pos = [egoState.x, egoState.y];
dists = vecnorm(path - pos, 2, 2);

% Search forward from the point nearest the vehicle so a lookahead match
% can never lock onto a point already behind it (e.g. the path's start).
[~, nearestIdx] = min(dists);
forwardIdx = find(dists(nearestIdx:end) >= lookaheadDist, 1, 'first');
if isempty(forwardIdx)
    target = path(end, :);
else
    target = path(nearestIdx + forwardIdx - 1, :);
end

dx = target(1) - egoState.x;
dy = target(2) - egoState.y;
Ld = max(hypot(dx, dy), 1e-3);

alpha = atan2(dy, dx) - egoState.yaw;
steeringAngle = atan2(2 * vehicleConfig.wheelbase * sin(alpha), Ld);
steeringAngle = max(min(steeringAngle, vehicleConfig.maxSteerAngle), -vehicleConfig.maxSteerAngle);

end
