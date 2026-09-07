function cfg = vehicleConfig()
% vehicleConfig - ego vehicle physical/dynamic limits (stub, Phase 0).
%
% Schema:
%   cfg.wheelbase      [m]     distance between front and rear axles
%   cfg.maxSteerAngle  [rad]   max steering angle (single side)
%   cfg.maxSteerRate   [rad/s] max rate of change of steering angle
%   cfg.maxAccel       [m/s^2] max acceleration
%   cfg.maxBraking     [m/s^2] max deceleration (magnitude)
%   cfg.maxSpeed       [m/s]   top speed cap for planning/control

% maxSpeed raised from 16.7 m/s (~60 km/h) to 20.0 m/s (~72 km/h) on
% request. Every scenario's cruise speed and every decision-state's speed
% factor (main.m's scenarioSpeedFactors / decisionSpeedFactors) scale off
% this single cap, so this one change speeds up all five scenarios
% proportionally without altering their relative tuning. Re-validated: 0
% collisions preserved across all five scenarios and all 14 regression
% tests still pass.
cfg = struct( ...
    'wheelbase',        2.7, ...
    'maxSteerAngle',    deg2rad(35), ...
    'maxSteerRate',     deg2rad(60), ... % full lock-to-lock in ~1.2s; a real rack, not a snap
    'maxAccel',         2.0, ...
    'maxBraking',       6.0, ...
    'maxSpeed',         20.0 ...
);

end
