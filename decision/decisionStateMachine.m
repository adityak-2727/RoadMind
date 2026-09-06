function nextState = decisionStateMachine(currentState, behaviorCommand, sensorEvents) %#ok<INUSD>
% decisionStateMachine - the FSM governing high-level behavior transitions
% across the project brief's state set (Hour 20-22): CRUISE, FOLLOW, MERGE,
% REPLAN, AVOID, WAIT, BRAKE, EMERGENCY_STOP.
%
% Escalation (behaviorDecision proposing an equal-or-more-restrictive
% state, by behaviorSeverity rank) is applied immediately - safety first,
% never delayed. Relaxation steps down at most one severity rank per
% transition and never below what behaviorDecision actually proposed, so
% the vehicle walks back up to speed rather than jumping:
%
%   emergency_stop -> wait -> brake -> avoid -> replan -> follow/merge -> cruise
%
% This realises the brief's CRUISE -> AVOID -> BRAKE -> REPLAN -> CRUISE
% recovery cycle as a monotonic speed ramp (see behaviorSeverity.m).
%
% The one-rank-at-a-time rule is load-bearing, not cosmetic. An earlier
% version jumped straight from "brake" to a fixed "replan" recovery hop
% regardless of what was proposed; that overshot far below the actual
% hazard level, so the very next tick re-escalated to "brake" (escalation
% being immediate by design), producing a sustained brake<->replan
% oscillation with a ~1.8s period whenever a hazard persisted - visible in
% villageRoad as repeated 0.3s flip-backs.
%
% Note: the minimum dwell time that rate-limits relaxation lives in main.m,
% not here - it is a scheduling concern about how often to commit a
% transition, not part of which transitions are legal. See
% docs/architecture.md's "Known behavior" note.
%
% Inputs:
%   currentState    - current behavior state
%   behaviorCommand - proposed command from behaviorDecision
%   sensorEvents    - struct of triggering events (unused for now;
%                     reserved for explicit external triggers)
% Output:
%   nextState - next behavior state

commandRank = behaviorSeverity(behaviorCommand);
currentRank = behaviorSeverity(currentState);

if commandRank >= currentRank
    nextState = behaviorCommand;
    return;
end

targetRank = max(commandRank, currentRank - 1);

% "wait" is a deliberate hold for a moving hazard to clear, not a rung on
% the recovery ladder: it shares emergency_stop's speed factor (0.0), so
% relaxing emergency_stop -> wait would just hold the vehicle at a dead
% stop for another dwell period achieving nothing. Skip it on the way down
% and resume creeping (brake) instead - measurably harmful otherwise on
% highwayMerge, where sitting stopped lets an 11 m/s car close on the ego.
% "wait" is still reachable whenever behaviorDecision explicitly proposes it.
WAIT_RANK = 6;
if targetRank == WAIT_RANK && commandRank < WAIT_RANK
    targetRank = WAIT_RANK - 1;
end

if commandRank == targetRank
    nextState = behaviorCommand; % the proposal itself sits at this rank
else
    nextState = canonicalStateForRank(targetRank);
end

end

function state = canonicalStateForRank(rank)
% Inverse of behaviorSeverity.m - the representative state to relax into at
% a given rank. Must be kept consistent with behaviorSeverity.m. "follow"
% represents rank 2 here; a proposal of "merge" reaches that rank directly
% via the commandRank == targetRank branch above.
switch rank
    case 1
        state = "cruise";
    case 2
        state = "follow";
    case 3
        state = "replan";
    case 4
        state = "avoid";
    case 5
        state = "brake";
    case 6
        state = "wait";
    otherwise
        state = "emergency_stop";
end
end
