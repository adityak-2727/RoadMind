function predictedTrajectory = irregularMotionModel(agent, horizon, dt)
% irregularMotionModel - projects an unpredictable agent forward using the
% same constant-velocity centerline as constantVelocityPrediction, but with
% a faster-growing uncertainty radius: these agents (irregular class, or
% currently crossing/merging per prediction/classifyBehavior.m - see
% trajectoryPrediction.m for the dispatch logic) can change heading/speed
% abruptly, so the planner should treat the space around their projected
% path as increasingly unsafe the further out it looks.
%
% trajectoryPrediction.m routes any "stopped" agent straight to
% constantVelocityPrediction instead of here, so this function can assume
% its caller has already established the agent is actually moving - no
% stationary-speed check is needed on this end too.
%
% Inputs:
%   agent   - single agent struct (see config/createAgent.m)
%   horizon - [s] prediction horizon
%   dt      - [s] prediction timestep
% Output:
%   predictedTrajectory - Nx3 array of [x, y, uncertaintyRadius]

numSteps = max(1, round(horizon / dt));
t = (1:numSteps)' * dt;

positions = agent.position + t * agent.velocity;

baseRadius = 0.8;
growthRate = 0.6;
radius = baseRadius + growthRate * t;

predictedTrajectory = [positions, radius];

end
