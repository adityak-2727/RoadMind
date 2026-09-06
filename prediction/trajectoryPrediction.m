function predictedTrajectories = trajectoryPrediction(trackedAgents, egoState, horizon, dt)
% trajectoryPrediction - dispatches each tracked agent to the appropriate
% motion model using both agent.class and its current behavior category
% (classifyBehavior.m): irregular-class agents (pedestrian/animal/pushcart/
% bicycle/unknown) always get the higher-uncertainty model, and so does any
% agent - regardless of class - currently classified "crossing" or
% "merging" relative to the ego's heading, since a car cutting across an
% intersection is exactly as hard to predict precisely as a pedestrian
% doing the same thing. A "stopped" agent always gets the lower-uncertainty
% model even if its class is normally treated as irregular (a parked cart
% isn't erratic just because carts can be).
%
% Note: egoState was added versus the original Phase 0 draft signature -
% classifying crossing/merging needs a reference direction ("parallel to
% the road"), and egoState.yaw is the only one available without adding a
% road-geometry parameter. See docs/architecture.md's interface change log.
%
% Inputs:
%   trackedAgents - struct array of tracked agents (see config/createAgent.m)
%   egoState      - current ego state struct, for its yaw as the reference
%                   direction classifyBehavior measures crossing/merging against
%   horizon       - [s] prediction horizon
%   dt            - [s] prediction timestep
% Output:
%   predictedTrajectories - cell array, one Nx3 [x, y, uncertaintyRadius]
%                           array per agent, same order as trackedAgents

irregularClasses = ["pedestrian", "animal", "pushcart", "bicycle", "unknown"];

predictedTrajectories = cell(1, numel(trackedAgents));
for i = 1:numel(trackedAgents)
    agent = trackedAgents(i);
    behaviorCategory = classifyBehavior(agent, egoState.yaw);

    isIrregularClass = any(strcmp(agent.class, irregularClasses));
    isUnpredictableMotion = behaviorCategory == "crossing" || behaviorCategory == "merging";

    if behaviorCategory ~= "stopped" && (isIrregularClass || isUnpredictableMotion)
        predictedTrajectories{i} = irregularMotionModel(agent, horizon, dt);
    else
        predictedTrajectories{i} = constantVelocityPrediction(agent, horizon, dt);
    end
end

end
