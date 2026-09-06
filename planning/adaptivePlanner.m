function [selectedTrajectory, selectedIndex] = adaptivePlanner(egoState, candidateTrajectories, predictedTrajectories, vehicleConfig, plannerConfig, scenarioContext, previousIndex) %#ok<INUSD>
% adaptivePlanner - scores each candidate trajectory from localPlanner
% against all six weights in plannerConfig.costWeights and selects the
% lowest-cost trajectory that clears collisionCheck's TTC-critical
% threshold; if none clear it, falls back to whichever candidate buys the
% most time (least-bad emergency choice) rather than returning nothing.
%
% Cost terms:
%   collisionRisk      - w.collisionRisk / minTTC (from collisionCheck)
%   obstacleClearance  - w.obstacleClearance / minClearance (min distance
%                        to any predicted trajectory over the horizon;
%                        distinct from collisionRisk, which only sees the
%                        binary TTC-critical threshold - this rewards
%                        margin even when nothing is imminently critical)
%   pathDeviation      - w.pathDeviation * mean distance from the centerline candidate
%   curvature          - w.curvature * total heading change (ride comfort/feasibility)
%   speedChange        - w.speedChange * |egoState.velocity - impliedSafeSpeed|,
%                        where impliedSafeSpeed = sqrt(vehicleConfig.maxAccel /
%                        candidateCurvature) is the classic curvature speed
%                        limit - a tight candidate implies slowing down,
%                        so this penalizes candidates whose required speed
%                        differs a lot from the ego's current speed
%   uncertainty        - w.uncertainty * mean (predicted-agent uncertainty
%                        radius / distance) encountered along the candidate -
%                        distinct from collisionRisk/obstacleClearance:
%                        this penalizes brushing close past a
%                        fast-growing-uncertainty prediction specifically,
%                        not just proximity to any prediction
%   consistency        - not one of the six documented weights; a small
%                        fixed penalty (not tunable via costWeights) against
%                        switching away from previousIndex, since candidates
%                        are indexed in a fixed left-to-right lateral order
%                        by localPlanner. Without it, near-tied costs near
%                        an obstacle flip the pick every step, swinging the
%                        vehicle's heading wildly (found live in
%                        highwayMerge: index bounced 2->3->2->2->3, later
%                        5->8->8->8->8->4->4->6->8, +/-40 degree yaw swings,
%                        driving clearance down to 1.17m from 4.86m). A
%                        genuinely large safety cost difference still
%                        dominates it.
%
% Candidates vary only laterally (localPlanner does not generate distinct
% stop/emergency-brake candidates the way the project brief's example
% does) - speed control is handled by a separate decision layer
% (decision/behaviorDecision.m + decisionStateMachine.m modulating
% vehicleController's targetSpeed) rather than per-candidate speed
% profiles. speedChange above is a curvature-implied proxy within that
% decoupled design, not a literal per-candidate speed cost.
%
% Note: this signature adds vehicleConfig versus the original Phase 0 draft
% in docs/architecture.md - adaptivePlanner must call collisionCheck
% internally to do its job, and collisionCheck requires vehicleConfig, so
% the original draft signature couldn't actually be implemented. Fixed here
% while the interface has no other callers besides main.m (also updated).
% previousIndex/selectedIndex were added for the same reason as the
% consistency cost above.
%
% Inputs:
%   egoState               - current ego state struct
%   candidateTrajectories  - cell array of Nx2 candidates from localPlanner
%   predictedTrajectories  - cell array of predicted agent trajectories
%   vehicleConfig          - struct from config/vehicleConfig.m
%   plannerConfig          - struct from config/plannerConfig.m
%   scenarioContext        - scenario-specific hints (road type, density, etc.)
%   previousIndex          - index selected last step (optional; omit or
%                            pass [] on the first call, defaults to centerline)
% Outputs:
%   selectedTrajectory - Nx2 array of chosen [x, y] waypoints
%   selectedIndex       - the chosen candidate's index, to pass back in as
%                        previousIndex next step

if isempty(candidateTrajectories)
    selectedTrajectory = zeros(0, 2);
    selectedIndex = [];
    return;
end

numCandidates = numel(candidateTrajectories);
centerline = candidateTrajectories{ceil(numCandidates / 2)};
w = plannerConfig.costWeights;

if nargin < 7 || isempty(previousIndex)
    previousIndex = ceil(numCandidates / 2);
end

CONSISTENCY_WEIGHT = 0.3;

costs = zeros(numCandidates, 1);
minTTCs = Inf(numCandidates, 1);
isCollidingFlags = false(numCandidates, 1);

for c = 1:numCandidates
    candidate = candidateTrajectories{c};

    [isColliding, minTTC] = collisionCheck(candidate, predictedTrajectories, vehicleConfig);
    isCollidingFlags(c) = isColliding;
    minTTCs(c) = minTTC;

    [minClearance, uncertaintyExposure] = assessCandidateAgainstPredictions(candidate, predictedTrajectories);

    heading = atan2(diff(candidate(:, 2)), diff(candidate(:, 1)));
    segLengths = hypot(diff(candidate(:, 1)), diff(candidate(:, 2)));
    validSeg = segLengths > 1e-6;
    if any(validSeg)
        avgSegLength = mean(segLengths(validSeg));
    else
        avgSegLength = 1e-3;
    end
    maxHeadingChange = max(abs(diff(heading)));
    approxCurvature = maxHeadingChange / max(avgSegLength, 1e-3);
    impliedSafeSpeed = min(vehicleConfig.maxSpeed, sqrt(vehicleConfig.maxAccel / max(approxCurvature, 1e-3)));

    riskCost = w.collisionRisk / max(minTTC, 0.1);
    clearanceCost = w.obstacleClearance / max(minClearance, 0.1);
    deviationCost = w.pathDeviation * mean(vecnorm(candidate - centerline, 2, 2));
    curvatureCost = w.curvature * sum(abs(diff(heading)));
    speedChangeCost = w.speedChange * abs(egoState.velocity - impliedSafeSpeed);
    uncertaintyCost = w.uncertainty * uncertaintyExposure;
    consistencyCost = CONSISTENCY_WEIGHT * abs(c - previousIndex) / numCandidates;

    costs(c) = riskCost + clearanceCost + deviationCost + curvatureCost + speedChangeCost + uncertaintyCost + consistencyCost;
end

safeIdx = find(~isCollidingFlags & minTTCs >= plannerConfig.ttcThresholds.critical);
if ~isempty(safeIdx)
    [~, bestLocal] = min(costs(safeIdx));
    bestIdx = safeIdx(bestLocal);
else
    [~, bestIdx] = max(minTTCs); % no safe candidate: take whichever buys the most time
end

selectedTrajectory = candidateTrajectories{bestIdx};
selectedIndex = bestIdx;

end

function [minClearance, uncertaintyExposure] = assessCandidateAgainstPredictions(candidate, predictedTrajectories)
% Returns the minimum distance from this candidate to any predicted agent
% trajectory over the shared horizon, and a mean inverse-distance-weighted
% uncertainty-radius exposure (higher when the candidate passes close to a
% prediction that itself carries a large/fast-growing uncertainty radius).
minClearance = Inf;
totalExposure = 0;
exposureCount = 0;

for i = 1:numel(predictedTrajectories)
    pred = predictedTrajectories{i};
    if isempty(pred)
        continue;
    end
    numSteps = min(size(candidate, 1), size(pred, 1));
    for k = 1:numSteps
        dist = hypot(candidate(k, 1) - pred(k, 1), candidate(k, 2) - pred(k, 2));
        minClearance = min(minClearance, dist);
        totalExposure = totalExposure + pred(k, 3) / max(dist, 0.5);
        exposureCount = exposureCount + 1;
    end
end

if exposureCount > 0
    uncertaintyExposure = totalExposure / exposureCount;
else
    uncertaintyExposure = 0;
end

end
