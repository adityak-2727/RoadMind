function [predictedTrajectories, behaviorInfo] = carlaPredictionStep(trackedAgents, egoState, horizon, dt)
% carlaPredictionStep - Phase 12: connects tracked agents to the existing,
% UNMODIFIED prediction/trajectoryPrediction.m (K1 lives entirely inside
% it and is never touched here), and separately reports a human-readable
% per-track behavior label for visualization/metrics.
%
% predictedTrajectories is returned EXACTLY as trajectoryPrediction.m
% produces it - a cell array, one Nx3 [x, y, uncertaintyRadius] per
% agent, same order as trackedAgents - which is already the interface
% main.m feeds straight into behaviorDecision.m / localPlanner.m /
% adaptivePlanner.m / collisionCheck.m unmodified. Phase 12 therefore
% reuses this existing interface rather than inventing a new one.
%
% BEHAVIOR LABEL DERIVATION (reporting only - never fed back into K1 or
% the planner): trajectoryPrediction.m does not expose which of its two
% motion models it used, or whether its private predictable-unknown
% relaxation fired for a given "unknown"-class track, because that
% decision depends on a `persistent trackHistory` buffer that is private
% to that file (see its header). Rather than duplicating that private
% state (risk of drift, double bookkeeping for a reporting-only need),
% this file:
%   1. Calls classifyBehavior.m directly (the exact same, unmodified
%      function trajectoryPrediction.m calls internally) to get
%      stopped/normal/crossing/merging - fully deterministic, no
%      inference needed for these four.
%   2. For the one remaining ambiguous case - an "unknown"-class agent
%      classified "normal" by motion, where K1's relaxation MAY have
%      fired - inspects the ALREADY-RETURNED trajectory's uncertainty
%      growth rate against the two frozen models' own documented,
%      distinct constants (constantVelocityPrediction.m: growthRate=0.15;
%      irregularMotionModel.m: growthRate=0.6) to tell which one
%      actually produced it. This is a post-hoc observation of a frozen
%      function's OUTPUT, not a reimplementation of its decision logic.
%
% Inputs:
%   trackedAgents - base config/createAgent.m struct array (from
%                   carlaTrackingStep.m's first output)
%   egoState      - current ego state (createEgoState schema)
%   horizon, dt   - [s] forwarded to trajectoryPrediction.m unchanged
% Outputs:
%   predictedTrajectories - EXACTLY trajectoryPrediction.m's own return
%                            value, untouched
%   behaviorInfo  - struct array, same order as trackedAgents:
%                     .trackId, .motionCategory ("stopped"|"normal"|
%                     "crossing"|"merging"), .isIrregularClass (logical),
%                     .k1Relaxed (logical - true only for an
%                     "unknown"-class track whose output trajectory
%                     matches the constant-velocity model's growth
%                     signature), .label (one human-readable string:
%                     "stopped"|"normal"|"crossing"|"merging"|
%                     "irregular/unknown"|"normal (K1-relaxed unknown)")

predictedTrajectories = trajectoryPrediction(trackedAgents, egoState, horizon, dt);

irregularClasses = ["pedestrian", "animal", "pushcart", "bicycle", "unknown"]; % identical list to trajectoryPrediction.m
GROWTH_RATE_MIDPOINT = 0.35; % between constantVelocityPrediction's 0.15 and irregularMotionModel's 0.6

behaviorInfo = repmat(struct('trackId', 0, 'motionCategory', "normal", 'isIrregularClass', false, ...
    'k1Relaxed', false, 'label', "normal"), 0, 0);

for i = 1:numel(trackedAgents)
    agent = trackedAgents(i);
    motionCategory = classifyBehavior(agent, egoState.yaw);
    isIrregularClass = any(strcmp(agent.class, irregularClasses));
    k1Relaxed = false;

    if motionCategory == "stopped"
        label = "stopped";
    elseif motionCategory == "crossing"
        label = "crossing";
    elseif motionCategory == "merging"
        label = "merging";
    elseif isIrregularClass
        traj = predictedTrajectories{i};
        if size(traj, 1) >= 2
            growthRate = (traj(2, 3) - traj(1, 3)) / dt;
        else
            growthRate = Inf; % single-step horizon: cannot infer, default to the conservative label
        end
        if growthRate < GROWTH_RATE_MIDPOINT
            label = "normal (K1-relaxed unknown)";
            k1Relaxed = true;
        else
            label = "irregular/unknown";
        end
    else
        label = "normal";
    end

    behaviorInfo(end + 1) = struct('trackId', agent.id, 'motionCategory', motionCategory, ...
        'isIrregularClass', isIrregularClass, 'k1Relaxed', k1Relaxed, 'label', label); %#ok<AGROW>
end

end
