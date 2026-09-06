function predictedTrajectory = constantVelocityPrediction(agent, horizon, dt)
% constantVelocityPrediction - projects an agent forward assuming constant
% velocity/heading. Appropriate for structured movers (cars/buses/trucks/
% autos/motorcycles) whose near-term motion is well-approximated by their
% current velocity.
%
% Inputs:
%   agent   - single agent struct (see config/createAgent.m)
%   horizon - [s] prediction horizon
%   dt      - [s] prediction timestep
% Output:
%   predictedTrajectory - Nx3 array of [x, y, uncertaintyRadius], one row
%                         per timestep from dt to horizon. The radius column
%                         grows with time and is the safety margin collisionCheck
%                         adds around this point (small/slow-growing here since
%                         constant-velocity motion is comparatively predictable).

numSteps = max(1, round(horizon / dt));
t = (1:numSteps)' * dt;

positions = agent.position + t * agent.velocity;

baseRadius = 0.6;
growthRate = 0.15;
radius = baseRadius + growthRate * t;

predictedTrajectory = [positions, radius];

end
