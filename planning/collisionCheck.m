function [isColliding, minTTC] = collisionCheck(egoTrajectory, predictedTrajectories, vehicleConfig)
% collisionCheck - checks a time-indexed candidate ego trajectory against
% all predicted agent trajectories for spatial overlap at matching
% timesteps, and returns the earliest time at which any overlap occurs.
%
% egoTrajectory and each predictedTrajectories{i} are assumed to share the
% same timestep spacing (row k of each corresponds to the same instant) -
% this holds for trajectories produced by localPlanner/adaptivePlanner and
% trajectoryPrediction, which are both driven by the same planning
% horizon/dt convention in main.m.
%
% Inputs:
%   egoTrajectory          - Nx2 array of candidate ego [x, y] waypoints
%   predictedTrajectories  - cell array of Mx3 [x, y, uncertaintyRadius]
%                            predicted agent trajectories
%   vehicleConfig          - struct from config/vehicleConfig.m (unused
%                            directly; ego footprint is approximated below,
%                            kept as a parameter to match the documented
%                            interface and allow a real footprint model later)
% Outputs:
%   isColliding - logical, true if any predicted overlap found
%   minTTC      - [s] time of the earliest predicted overlap (Inf if none)

isColliding = false;
minTTC = Inf;

if isempty(egoTrajectory) || isempty(predictedTrajectories)
    return;
end

egoRadius = 1.1; % approx vehicle half-width + safety margin
PLANNING_DT = 0.1; % must match the dt used by localPlanner/trajectoryPrediction

for i = 1:numel(predictedTrajectories)
    pred = predictedTrajectories{i};
    if isempty(pred)
        continue;
    end

    numSteps = min(size(egoTrajectory, 1), size(pred, 1));
    for k = 1:numSteps
        dist = hypot(egoTrajectory(k, 1) - pred(k, 1), egoTrajectory(k, 2) - pred(k, 2));
        threshold = egoRadius + pred(k, 3);
        if dist < threshold
            isColliding = true;
            minTTC = min(minTTC, k * PLANNING_DT);
            break; % earliest hit for this agent found; move to the next one
        end
    end
end

end
