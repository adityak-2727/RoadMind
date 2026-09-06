function cfg = vehicleConfig()
% vehicleConfig - ego vehicle physical/dynamic limits (stub, Phase 0).
%
% Schema:
%   cfg.wheelbase      [m]     distance between front and rear axles
%   cfg.maxSteerAngle  [rad]   max steering angle (single side)
%   cfg.maxAccel       [m/s^2] max acceleration
%   cfg.maxBraking     [m/s^2] max deceleration (magnitude)
%   cfg.maxSpeed       [m/s]   top speed cap for planning/control

cfg = struct( ...
    'wheelbase',        2.7, ...
    'maxSteerAngle',    deg2rad(35), ...
    'maxAccel',         2.0, ...
    'maxBraking',       6.0, ...
    'maxSpeed',         16.7 ... % ~60 km/h, typical unstructured-road cap
);

end
