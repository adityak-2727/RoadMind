function candidateTrajectories = localPlanner(egoState, globalPath, predictedTrajectories, plannerConfig) %#ok<INUSD>
% localPlanner - samples a lattice of candidate short-horizon trajectories
% as lateral offsets of the global path, time-indexed at the same dt/horizon
% convention collisionCheck and trajectoryPrediction use (PLANNING_DT,
% PLANNING_HORIZON below), so candidates line up index-for-index with
% predicted agent trajectories.
%
% Inputs:
%   egoState              - current ego state struct (see config/createEgoState.m)
%   globalPath             - Nx2 reference waypoints from globalPlanner
%   predictedTrajectories  - cell array of predicted agent trajectories (unused
%                            here; candidates are geometric offsets of the
%                            global path, scored against predictions later
%                            by adaptivePlanner)
%   plannerConfig          - struct from config/plannerConfig.m
% Output:
%   candidateTrajectories - cell array of Nx2 candidate waypoint arrays

PLANNING_DT = 0.1; % must match collisionCheck's PLANNING_DT
PLANNING_HORIZON = 4.0; % [s]
DEFAULT_HALF_WIDTH = 2.5; % [m] corridor half-width; scenario road width isn't
                          % passed into this fixed-signature function

candidateTrajectories = {};
if isempty(globalPath) || size(globalPath, 1) < 2
    return;
end

diffs = diff(globalPath, 1, 1);
segLen = hypot(diffs(:, 1), diffs(:, 2));
arcLen = [0; cumsum(segLen)];

pos = [egoState.x, egoState.y];
distToEgo = hypot(globalPath(:, 1) - pos(1), globalPath(:, 2) - pos(2));
[~, nearestIdx] = min(distToEgo);
startArc = arcLen(nearestIdx);

numSteps = max(3, round(PLANNING_HORIZON / PLANNING_DT));
speedEstimate = max(egoState.velocity, 1.0); % floor avoids a degenerate lookahead when stationary
targetArcs = startArc + speedEstimate * (1:numSteps)' * PLANNING_DT;
targetArcs = min(targetArcs, arcLen(end));

basePoints = [interp1(arcLen, globalPath(:, 1), targetArcs, 'linear', 'extrap'), ...
              interp1(arcLen, globalPath(:, 2), targetArcs, 'linear', 'extrap')];

tangents = zeros(numSteps, 2);
tangents(1, :) = basePoints(2, :) - basePoints(1, :);
tangents(end, :) = basePoints(end, :) - basePoints(end - 1, :);
tangents(2:end-1, :) = basePoints(3:end, :) - basePoints(1:end-2, :);
tangents = tangents ./ max(hypot(tangents(:, 1), tangents(:, 2)), 1e-6);
normals = [-tangents(:, 2), tangents(:, 1)];

numCandidates = max(round(plannerConfig.numCandidateTrajectories), 1);
offsets = linspace(-DEFAULT_HALF_WIDTH, DEFAULT_HALF_WIDTH, numCandidates);

candidateTrajectories = cell(1, numCandidates);
for c = 1:numCandidates
    candidateTrajectories{c} = basePoints + offsets(c) * normals;
end

end
