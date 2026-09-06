function controlCommand = vehicleController(egoState, selectedTrajectory, vehicleConfig, targetSpeed)
% vehicleController - top-level controller combining lateral (pure pursuit)
% and longitudinal (P speed control to targetSpeed) control into a single
% command applied to the vehicle model.
%
% Note: targetSpeed was added versus the original Phase 0/1 draft signature
% (which hardcoded a fixed cruise speed internally) so the decision layer
% (decision/behaviorDecision.m + decisionStateMachine.m) can actually make
% the vehicle yield or stop - lateral planning alone can't resolve a
% perpendicular crossing. See docs/architecture.md's interface notes.
%
% Inputs:
%   egoState           - current ego state struct
%   selectedTrajectory - Nx2 array of chosen [x, y] waypoints from planning
%   vehicleConfig      - struct from config/vehicleConfig.m
%   targetSpeed        - [m/s] desired speed, set by the caller from the
%                        current decision state (e.g. 0 to stop)
% Output:
%   controlCommand - struct with fields: throttle, brake, steeringAngle

lookaheadDist = max(3.0, 0.5 * egoState.velocity + 2.0);
steeringAngle = purePursuitController(egoState, selectedTrajectory, lookaheadDist, vehicleConfig);

targetSpeed = max(0, min(targetSpeed, vehicleConfig.maxSpeed));
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
