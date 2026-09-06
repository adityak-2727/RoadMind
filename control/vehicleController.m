function controlCommand = vehicleController(egoState, selectedTrajectory, vehicleConfig)
% vehicleController - top-level controller combining lateral (pure pursuit)
% and longitudinal (P speed control to a fixed cruise target) control into
% a single command applied to the vehicle model. Phase 1: fixed cruise
% target speed, no adaptive/planned speed profile yet.
%
% Inputs:
%   egoState           - current ego state struct
%   selectedTrajectory - Nx2 array of chosen [x, y] waypoints from planning
%   vehicleConfig      - struct from config/vehicleConfig.m
% Output:
%   controlCommand - struct with fields: throttle, brake, steeringAngle

lookaheadDist = max(3.0, 0.5 * egoState.velocity + 2.0);
steeringAngle = purePursuitController(egoState, selectedTrajectory, lookaheadDist, vehicleConfig);

targetSpeed = 0.5 * vehicleConfig.maxSpeed;
Kp = 1.0;
accelCmd = Kp * (targetSpeed - egoState.velocity);
accelCmd = max(min(accelCmd, vehicleConfig.maxAccel), -vehicleConfig.maxBraking);

throttle = max(accelCmd, 0) / vehicleConfig.maxAccel;
brake    = max(-accelCmd, 0) / vehicleConfig.maxBraking;

controlCommand = struct( ...
    'throttle',      throttle, ...
    'brake',         brake, ...
    'steeringAngle', steeringAngle ...
);

end
