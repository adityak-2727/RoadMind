function ttc = calculateTTC(egoState, agent)
% calculateTTC - computes time-to-collision between ego and a single agent
% from their current relative position and velocity (range / closing
% speed). Used both for the completed run's TTC metric and, live, by
% decision/behaviorDecision.m to judge risk before planning.
%
% Inputs:
%   egoState - current ego state struct
%   agent    - single agent struct
% Output:
%   ttc - [s] time-to-collision; Inf if not on a closing course

egoPos = [egoState.x, egoState.y];
egoVel = egoState.velocity * [cos(egoState.yaw), sin(egoState.yaw)];

relPos = agent.position - egoPos;
relVel = agent.velocity - egoVel;

range = norm(relPos);
if range < 1e-6
    ttc = 0;
    return;
end

closingSpeed = -dot(relPos, relVel) / range;

if closingSpeed <= 0
    ttc = Inf;
else
    ttc = range / closingSpeed;
end

end
