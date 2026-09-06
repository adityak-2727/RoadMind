function controlCommand = vehicleController(egoState, selectedTrajectory, vehicleConfig, targetSpeed, dt)
% vehicleController - top-level controller combining lateral (pure pursuit)
% and longitudinal (P speed control to targetSpeed) control into a single
% command applied to the vehicle model.
%
% Rate-limits the steering command against egoState.steering (the angle
% actually applied last tick) before returning it. Measured need, not a
% precaution: validating the Phase 6 decision layer found adaptivePlanner's
% selected candidate index can jump substantially between ticks even with
% its consistency cost (a soft cost, not a hard constraint) - e.g.
% urbanIntersection candidate 15->1 produced a steering command swinging
% 70 degrees in one 0.1s tick (700 deg/s), and 4 of 5 scenarios saturated
% the absolute +-35 degree limit repeatedly. No real steering rack moves
% anywhere near that fast. The absolute clamp alone (still applied, via
% purePursuitController and again below) bounds *where* steering can be;
% it does nothing to bound *how fast* it gets there, which is what
% destabilized the vehicle when the selected path itself was discontinuous
% tick to tick.
%
% Note: targetSpeed was added versus the original Phase 0/1 draft signature
% (which hardcoded a fixed cruise speed internally) so the decision layer
% (decision/behaviorDecision.m + decisionStateMachine.m) can actually make
% the vehicle yield or stop - lateral planning alone can't resolve a
% perpendicular crossing. dt was added for the steering-rate limit above -
% see docs/architecture.md's interface notes for both.
%
% Inputs:
%   egoState           - current ego state struct (egoState.steering is
%                        the previously-applied steering angle)
%   selectedTrajectory - Nx2 array of chosen [x, y] waypoints from planning
%   vehicleConfig      - struct from config/vehicleConfig.m
%   targetSpeed        - [m/s] desired speed, set by the caller from the
%                        current decision state (e.g. 0 to stop)
%   dt                 - [s] time since the last control update
% Output:
%   controlCommand - struct with fields: throttle, brake, steeringAngle

lookaheadDist = max(3.0, 0.5 * egoState.velocity + 2.0);
desiredSteeringAngle = purePursuitController(egoState, selectedTrajectory, lookaheadDist, vehicleConfig);

maxSteerStep = vehicleConfig.maxSteerRate * dt;
steeringDelta = max(min(desiredSteeringAngle - egoState.steering, maxSteerStep), -maxSteerStep);
steeringAngle = egoState.steering + steeringDelta;
steeringAngle = max(min(steeringAngle, vehicleConfig.maxSteerAngle), -vehicleConfig.maxSteerAngle);

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
