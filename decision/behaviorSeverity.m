function rank = behaviorSeverity(state)
% behaviorSeverity - how restrictive each behavior state is, as an ordinal
% rank. Single source of truth: decisionStateMachine.m uses it to tell
% escalation from de-escalation, and main.m uses it to decide whether a
% proposed transition is an escalation (applied immediately, safety-first)
% or a relaxation (rate-limited by MIN_DWELL_TIME). Duplicating the
% ordering in both places would be a real drift risk.
%
% The ranking is deliberately monotonic with main.m's decisionSpeedFactors -
% a higher rank never drives faster than a lower one:
%
%   rank 1  cruise          speed x1.0
%   rank 2  follow, merge   speed x0.6
%   rank 3  replan          speed x0.5
%   rank 4  avoid           speed x0.45
%   rank 5  brake           speed x0.15
%   rank 6  wait            speed x0.0
%   rank 7  emergency_stop  speed x0.0
%
% That monotonicity matters: decisionStateMachine relaxes one rank at a
% time, so the ranks are also the vehicle's speed ramp back up after a
% hazard. An earlier version ranked "avoid" above "follow" while giving it
% a *higher* speed factor, which made stepping down the ranks briefly
% speed the vehicle up.
%
% decisionStateMachine.m holds the inverse map (rank -> canonical state)
% and must be kept consistent with this function.
%
% Inputs:
%   state - behavior state string
% Output:
%   rank - 1 (least restrictive) to 7 (most restrictive); unknown states rank 1

switch state
    case "cruise"
        rank = 1;
    case {"follow", "merge"}
        rank = 2;
    case "replan"
        rank = 3;
    case "avoid"
        rank = 4;
    case "brake"
        rank = 5;
    case "wait"
        rank = 6;
    case "emergency_stop"
        rank = 7;
    otherwise
        rank = 1;
end

end
