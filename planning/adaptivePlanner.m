function [selectedTrajectory, selectedIndex] = adaptivePlanner(egoState, candidateTrajectories, predictedTrajectories, vehicleConfig, plannerConfig, scenarioContext, previousIndex, debugFallback) %#ok<INUSD>
% adaptivePlanner - scores each candidate trajectory from localPlanner
% against all six weights in plannerConfig.costWeights and selects the
% lowest-cost trajectory that clears collisionCheck's TTC-critical
% threshold; if none clear it, falls back to a feasibility-aware emergency
% choice (see the fallback block below) rather than returning nothing.
%
% debugFallback - optional, default false. When true and the fallback path
% is taken, prints a per-candidate diagnostic table (minTTC, minClearance,
% required/max feasible curvature and steering angle, feasible flag,
% ranking score) so the selected fallback candidate is always explainable.
% Kept as an optional trailing argument so every existing 7-argument call
% site is unaffected (same pattern as perception/sensorFusion.m's
% debugDedup).
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
%   debugFallback          - optional, default false; see above.
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
if nargin < 8 || isempty(debugFallback)
    debugFallback = false;
end

CONSISTENCY_WEIGHT = 0.3;

costs = zeros(numCandidates, 1);
minTTCs = Inf(numCandidates, 1);
isCollidingFlags = false(numCandidates, 1);
minClearances = Inf(numCandidates, 1);
curvatures = zeros(numCandidates, 1);

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

    minClearances(c) = minClearance;
    curvatures(c) = approxCurvature; % same curvature already used for curvatureCost/speedChangeCost above -
                                      % reused (not recomputed) by the fallback ranking below

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
    % No candidate clears the safety screen above - this is the existing,
    % unchanged emergency fallback path (isCollidingFlags/minTTCs/safeIdx
    % are exactly as before; nothing here can turn a colliding candidate
    % "safe" or move the critical threshold). The only change from the
    % previous plain max(minTTCs) rule: among the (still unsafe) candidates,
    % prefer ones the vehicle can actually steer into given
    % vehicleConfig.maxSteerAngle/wheelbase, per K2's root-cause finding
    % that some max-minTTC picks required more curvature than the bicycle
    % model can produce (up to ~4.96 1/m against a ~0.259 1/m limit).
    maxFeasibleCurvature = tan(vehicleConfig.maxSteerAngle) / vehicleConfig.wheelbase;
    isFeasible = curvatures <= maxFeasibleCurvature;

    feasibleIdx = find(isFeasible);
    if ~isempty(feasibleIdx)
        % feasible first, then max minTTC, then max clearance as tiebreak -
        % never invents safety: every candidate here is still the unsafe
        % set, this only orders which unsafe candidate to ride out.
        rankKeys = [minTTCs(feasibleIdx), minClearances(feasibleIdx)];
        [~, order] = sortrows(rankKeys, [-1, -2]);
        bestIdx = feasibleIdx(order(1));
    else
        % No candidate is even kinematically feasible - retain the exact
        % pre-K2 behavior rather than inventing a new rule (still an
        % emergency fallback, not a solved situation).
        [~, bestIdx] = max(minTTCs);
    end

    if debugFallback
        requiredSteerDeg = rad2deg(atan(curvatures * vehicleConfig.wheelbase));
        maxFeasibleSteerDeg = rad2deg(vehicleConfig.maxSteerAngle);
        % Display-only composite reflecting the actual lexicographic
        % ranking used above (minTTC primary, minClearance tiebreak) -
        % selection itself uses sortrows, not this scalar, but a single
        % number makes the printed ordering easy to eyeball.
        rankScore = minTTCs * 1e4 + min(minClearances, 1e3);
        fprintf('[adaptivePlanner fallback] t=%.2f no safe candidate (%d total), %d/%d kinematically feasible\n', ...
            egoState.timestamp, numCandidates, numel(feasibleIdx), numCandidates);
        fprintf('  %-4s %-8s %-10s %-12s %-12s %-10s %-12s\n', 'idx', 'minTTC', 'minClr', 'reqCurv', 'reqSteerDeg', 'feasible', 'rankScore');
        for k = 1:numCandidates
            if isFeasible(k)
                feasStr = 'yes';
            else
                feasStr = 'no';
            end
            if k == bestIdx
                selMark = '  <- selected';
            else
                selMark = '';
            end
            fprintf('  %-4d %-8.2f %-10.2f %-12.3f %-12.1f %-10s %-12.1f%s\n', ...
                k, minTTCs(k), minClearances(k), curvatures(k), requiredSteerDeg(k), feasStr, rankScore(k), selMark);
        end
        fprintf('  max feasible curvature=%.3f 1/m, max feasible steering=%.1f deg\n', maxFeasibleCurvature, maxFeasibleSteerDeg);
    end
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
