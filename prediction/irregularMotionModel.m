function predictedTrajectory = irregularMotionModel(agent, horizon, dt)
% irregularMotionModel - projects an erratic/unpredictable agent (pedestrian,
% animal, pushcart, bicycle, unknown) forward using the same constant-velocity
% centerline as constantVelocityPrediction, but with a faster-growing
% uncertainty radius when the agent is actually moving: these classes can
% change heading/speed abruptly, so the planner should treat the space
% around their projected path as increasingly unsafe the further out it
% looks. A near-stationary agent (a parked cart, a pothole) gets the same
% slow growth as a structured mover instead - its class says it *could*
% move erratically, but zero current velocity means its near-term position
% is just as predictable as a parked car's, and treating it as fast-growing
% anyway made static obstacles effectively unavoidable once uncertainty
% ballooned past the local planner's corridor width.
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

STATIONARY_SPEED_THRESHOLD = 0.05; % [m/s]
if norm(agent.velocity) < STATIONARY_SPEED_THRESHOLD
    baseRadius = 0.6;
    growthRate = 0.15;
else
    baseRadius = 0.8;
    growthRate = 0.6;
end
radius = baseRadius + growthRate * t;

predictedTrajectory = [positions, radius];

end
