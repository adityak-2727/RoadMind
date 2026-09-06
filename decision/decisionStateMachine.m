function nextState = decisionStateMachine(currentState, behaviorCommand, sensorEvents) %#ok<INUSD>
% decisionStateMachine - holds the FSM governing high-level behavior
% transitions. Escalates to a more cautious state immediately (safety
% first), but de-escalates only one severity level per tick even if
% behaviorDecision says it's safe to fully cruise again, so the vehicle
% doesn't snap straight from an emergency stop back to full speed the
% instant risk clears.
%
% Inputs:
%   currentState     - current behavior state, one of severityOrder below
%   behaviorCommand   - proposed command from behaviorDecision
%   sensorEvents      - struct of relevant triggering events (unused for
%                       now; reserved for explicit external triggers like
%                       track-lost events)
% Output:
%   nextState - next behavior state

severityOrder = ["cruise", "yield", "emergency_stop"];

curIdx = find(severityOrder == currentState, 1);
cmdIdx = find(severityOrder == behaviorCommand, 1);
if isempty(curIdx)
    curIdx = 1;
end
if isempty(cmdIdx)
    cmdIdx = 1;
end

if cmdIdx >= curIdx
    nextIdx = cmdIdx;
else
    nextIdx = curIdx - 1;
end

nextState = severityOrder(nextIdx);

end
