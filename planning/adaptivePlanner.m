function selectedTrajectory = adaptivePlanner(egoState, candidateTrajectories, predictedTrajectories, vehicleConfig, plannerConfig, scenarioContext) %#ok<INUSD>
% adaptivePlanner - scores each candidate trajectory from localPlanner using
% plannerConfig.costWeights and selects the lowest-cost trajectory that
% clears collisionCheck's TTC-critical threshold; if none clear it, falls
% back to whichever candidate buys the most time (least-bad emergency
% choice) rather than returning nothing.
%
% Note: this signature adds vehicleConfig versus the original Phase 0 draft
% in docs/architecture.md - adaptivePlanner must call collisionCheck
% internally to do its job, and collisionCheck requires vehicleConfig, so
% the original draft signature couldn't actually be implemented. Fixed here
% while the interface has no other callers besides main.m (also updated).
%
% Inputs:
%   egoState               - current ego state struct
%   candidateTrajectories  - cell array of Nx2 candidates from localPlanner
%   predictedTrajectories  - cell array of predicted agent trajectories
%   vehicleConfig          - struct from config/vehicleConfig.m
%   plannerConfig          - struct from config/plannerConfig.m
%   scenarioContext        - scenario-specific hints (road type, density, etc.)
% Output:
%   selectedTrajectory - Nx2 array of chosen [x, y] waypoints

if isempty(candidateTrajectories)
    selectedTrajectory = zeros(0, 2);
    return;
end

numCandidates = numel(candidateTrajectories);
centerline = candidateTrajectories{ceil(numCandidates / 2)};
w = plannerConfig.costWeights;

costs = zeros(numCandidates, 1);
minTTCs = Inf(numCandidates, 1);
isCollidingFlags = false(numCandidates, 1);

for c = 1:numCandidates
    candidate = candidateTrajectories{c};

    [isColliding, minTTC] = collisionCheck(candidate, predictedTrajectories, vehicleConfig);
    isCollidingFlags(c) = isColliding;
    minTTCs(c) = minTTC;

    riskCost = w.collisionRisk / max(minTTC, 0.1);
    deviationCost = w.pathDeviation * mean(vecnorm(candidate - centerline, 2, 2));

    heading = atan2(diff(candidate(:, 2)), diff(candidate(:, 1)));
    curvatureCost = w.curvature * sum(abs(diff(heading)));

    costs(c) = riskCost + deviationCost + curvatureCost;
end

safeIdx = find(~isCollidingFlags & minTTCs >= plannerConfig.ttcThresholds.critical);
if ~isempty(safeIdx)
    [~, bestLocal] = min(costs(safeIdx));
    bestIdx = safeIdx(bestLocal);
else
    [~, bestIdx] = max(minTTCs); % no safe candidate: take whichever buys the most time
end

selectedTrajectory = candidateTrajectories{bestIdx};

end
