function category = classifyBehavior(agent, referenceHeading)
% classifyBehavior - classifies an agent's current motion relative to a
% reference direction (the road/ego heading) into one of: "stopped",
% "normal" (moving roughly parallel to the reference direction, either way),
% "crossing" (moving roughly perpendicular to it), or "merging" (an oblique
% angle - neither parallel nor perpendicular). This lets trajectoryPrediction
% treat a structured-class agent (a car, a motorcycle) that happens to be
% crossing or merging into the ego's path with the same heightened
% uncertainty an irregular-class agent gets, instead of only ever keying
% off class - a car cutting across an intersection is exactly as hard to
% predict precisely as a pedestrian doing the same thing.
%
% "erratic" (the fifth category the project brief lists) is not classified
% here: detecting genuinely erratic motion needs heading variance over
% several frames, which would require extending the tracked-agent schema
% with a history buffer - out of scope for this pass. Irregular-class
% agents already get conservative (fast-growing) uncertainty regardless of
% their instantaneous behavior category, which covers the same intent.
%
% Inputs:
%   agent            - single agent struct (see config/createAgent.m)
%   referenceHeading - [rad] the direction "parallel to the road" is
%                      measured against (main.m uses egoState.yaw)
% Output:
%   category - one of: "stopped", "normal", "crossing", "merging"

STOPPED_SPEED_THRESHOLD = 0.3; % [m/s]

speed = norm(agent.velocity);
if speed < STOPPED_SPEED_THRESHOLD
    category = "stopped";
    return;
end

agentHeading = atan2(agent.velocity(2), agent.velocity(1));
relativeAngle = atan2(sin(agentHeading - referenceHeading), cos(agentHeading - referenceHeading));
absDeg = abs(rad2deg(relativeAngle));

if absDeg <= 30 || absDeg >= 150
    category = "normal";
elseif absDeg >= 60 && absDeg <= 120
    category = "crossing";
else
    category = "merging";
end

end
