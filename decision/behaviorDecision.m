function behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, plannerConfig, decisionState) %#ok<INUSD>
% behaviorDecision - maps current perception/prediction context into a
% high-level behavior intent by finding the worst-case time-to-collision
% (evaluation/calculateTTC.m) and minimum proximity across all tracked
% agents, then classifying against plannerConfig.ttcThresholds. Applies
% hysteresis keyed off decisionState so a TTC value that hovers right at a
% threshold doesn't flip the command every tick: leaving a more cautious
% state requires clearly more margin than entering it, and recovering from
% an emergency stop steps down to "yield" first rather than straight to
% "cruise".
%
% Note: plannerConfig was added versus the original Phase 0 draft signature
% - the TTC classification needs plannerConfig.ttcThresholds and there was
% no other way to receive it. See docs/architecture.md's interface notes.
%
% Inputs:
%   egoState               - current ego state struct
%   trackedAgents          - struct array of tracked agents
%   predictedTrajectories  - cell array of predicted agent trajectories
%                            (unused for now; current-state TTC is enough
%                            to classify behavior, kept for a future
%                            horizon-aware version)
%   plannerConfig          - struct from config/plannerConfig.m
%   decisionState          - current state from decisionStateMachine, used
%                            here for hysteresis
% Output:
%   behaviorCommand - one of: "cruise", "yield", "emergency_stop"

if isempty(trackedAgents)
    behaviorCommand = "cruise";
    return;
end

egoPos = [egoState.x, egoState.y];
minTTC = Inf;
minDist = Inf;

for i = 1:numel(trackedAgents)
    agent = trackedAgents(i);
    minTTC = min(minTTC, calculateTTC(egoState, agent));
    minDist = min(minDist, norm(agent.position - egoPos));
end

critical = plannerConfig.ttcThresholds.critical;
warning = plannerConfig.ttcThresholds.warning;
PROXIMITY_YIELD_DIST = 2.0; % [m] yield even on a non-closing course if this close - kept
                            % tight (near the actual graze distance) since these roads can be
                            % only ~5m wide; a wider threshold flags every roadside agent the
                            % planner has already safely routed around, and a stopped ego next
                            % to a stopped agent can never out-distance a static trigger, which
                            % deadlocked villageRoad/cattleCrossing at this value's old 5.0m
RECOVERY_MARGIN = 1.5; % must clear a threshold by this factor before de-escalating

switch decisionState
    case "emergency_stop"
        if minTTC > critical * RECOVERY_MARGIN && minDist > PROXIMITY_YIELD_DIST
            behaviorCommand = "yield"; % step down to yield first, never straight to cruise
        else
            behaviorCommand = "emergency_stop";
        end
    case "yield"
        if minTTC < critical
            behaviorCommand = "emergency_stop";
        elseif minTTC > warning * RECOVERY_MARGIN && minDist > PROXIMITY_YIELD_DIST * RECOVERY_MARGIN
            behaviorCommand = "cruise";
        else
            behaviorCommand = "yield";
        end
    otherwise % "cruise"
        if minTTC < critical
            behaviorCommand = "emergency_stop";
        elseif minTTC < warning || minDist < PROXIMITY_YIELD_DIST
            behaviorCommand = "yield";
        else
            behaviorCommand = "cruise";
        end
end

end
