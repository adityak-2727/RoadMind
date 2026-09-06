function predictedTrajectories = trajectoryPrediction(trackedAgents, horizon, dt)
% trajectoryPrediction - dispatches each tracked agent to the appropriate
% motion model by agent.class and collects the results.
%
% Inputs:
%   trackedAgents - struct array of tracked agents (see config/createAgent.m)
%   horizon       - [s] prediction horizon
%   dt            - [s] prediction timestep
% Output:
%   predictedTrajectories - cell array, one Nx3 [x, y, uncertaintyRadius]
%                           array per agent, same order as trackedAgents

irregularClasses = ["pedestrian", "animal", "pushcart", "bicycle", "unknown"];

predictedTrajectories = cell(1, numel(trackedAgents));
for i = 1:numel(trackedAgents)
    agent = trackedAgents(i);
    if any(strcmp(agent.class, irregularClasses))
        predictedTrajectories{i} = irregularMotionModel(agent, horizon, dt);
    else
        predictedTrajectories{i} = constantVelocityPrediction(agent, horizon, dt);
    end
end

end
