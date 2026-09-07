function agents = carlaFusedAgentsToGlobal(agents, egoState)
% carlaFusedAgentsToGlobal - Phase 13 integration fix: converts
% carlaPerceptionStep.m's output (deliberately EGO-RELATIVE, so its own
% internal sensor-range gate can test norm(position)<=maxRange) back into
% the GLOBAL/world frame that the frozen decision/planning stack actually
% expects - the exact inverse of carlaPerceptionStep.m's internal
% worldToEgoFrame() helper.
%
% WHY THIS IS NEEDED (found live during Phase 13 development, root-caused
% by direct inspection, not guessed): main.m feeds objectTracking.m /
% trajectoryPrediction.m / behaviorDecision.m / localPlanner.m /
% adaptivePlanner.m / collisionCheck.m with egoState.x/y, globalPath, and
% every agent's .position ALL in ONE SHARED GLOBAL FRAME - confirmed by
% reading main.m directly (egoState.x/y is used as an absolute world
% position throughout, e.g. compared against scenario.egoGoal) and by
% behaviorDecision.m's own internal `relVec = agent.position - [egoState.x,
% egoState.y]` (decision/behaviorDecision.m), which is only meaningful if
% agent.position is in that SAME global frame.
%
% carlaPerceptionStep.m instead returns fusedAgents already translated
% and rotated into the EGO-RELATIVE frame (its own worldToEgoFrame()),
% which was never converted back before Phase 12's carlaClosedLoopStep.m
% handed those agents onward to the frozen stack. The result: every
% distance/bearing/TTC computation in behaviorDecision.m/collisionCheck.m
% was silently computing `smallEgoRelativeNumber - hugeAbsoluteEgoCoordinate`,
% producing a spurious ~100+ meter apparent separation for every real
% agent regardless of its true distance - confirmed live: minTTC was Inf
% and every candidate trajectory was rated "feasible" on every tick across
% 8 independent demo scenarios (470+ ticks total), including demos with
% no staged actor at all, even while a directly-measured ego-relative
% clearance metric (bypassing this whole chain) recorded agents within
% 0.3-1.6m of the ego at various points in the same runs.
%
% This function is called ONCE, immediately after carlaPerceptionStep.m
% returns and BEFORE carlaTrackingStep.m is called (see
% carlaClosedLoopStep.m) - so objectTracking.m's own Kalman filter
% operates ENTIRELY in the global frame from the start, exactly matching
% how main.m already validates it, rather than trying to un-mix a
% moving-frame velocity estimate after the fact.
%
% Inputs:
%   agents   - struct array of config/createFusedAgent.m agents, EGO-
%              RELATIVE (carlaPerceptionStep.m's raw output), or [].
%   egoState - this tick's egoState (project/global frame: .x, .y, .yaw).
% Output:
%   agents   - same struct array, .position/.velocity/.heading rotated
%              and translated into the GLOBAL frame. All other fields
%              (class, confidence, sources, simulatorActorId, ...)
%              untouched.

if isempty(agents)
    return;
end

c = cos(egoState.yaw);
s = sin(egoState.yaw);
egoPos = [egoState.x, egoState.y];

for i = 1:numel(agents)
    p = agents(i).position;
    % Inverse of worldToEgoFrame's [c*d(1)+s*d(2), -s*d(1)+c*d(2)] rotation
    % (a rotation by -yaw), i.e. rotate by +yaw then translate back by egoPos.
    agents(i).position = egoPos + [c * p(1) - s * p(2), s * p(1) + c * p(2)];

    v = agents(i).velocity;
    agents(i).velocity = [c * v(1) - s * v(2), s * v(1) + c * v(2)];

    agents(i).heading = agents(i).heading + egoState.yaw;
end

end
