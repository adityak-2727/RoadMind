function nextEgoState = bicycleModel(egoState, controlCommand, vehicleConfig, dt)
% bicycleModel - integrates the rear-axle kinematic bicycle model one
% timestep forward given a control command, closing the loop back to
% perception on the next iteration.
%
% Inputs:
%   egoState       - current ego state struct (see config/createEgoState.m)
%   controlCommand - struct from vehicleController (throttle, brake, steeringAngle)
%   vehicleConfig  - struct from config/vehicleConfig.m
%   dt             - [s] integration timestep
% Output:
%   nextEgoState - next ego state struct

steeringAngle = max(min(controlCommand.steeringAngle, vehicleConfig.maxSteerAngle), ...
                     -vehicleConfig.maxSteerAngle);
accel = controlCommand.throttle * vehicleConfig.maxAccel ...
        - controlCommand.brake * vehicleConfig.maxBraking;

nextEgoState = egoState;
nextEgoState.x        = egoState.x + egoState.velocity * cos(egoState.yaw) * dt;
nextEgoState.y        = egoState.y + egoState.velocity * sin(egoState.yaw) * dt;
nextEgoState.yaw      = egoState.yaw + (egoState.velocity / vehicleConfig.wheelbase) ...
                         * tan(steeringAngle) * dt;
nextEgoState.velocity = max(0, min(vehicleConfig.maxSpeed, egoState.velocity + accel * dt));
nextEgoState.steering = steeringAngle;
nextEgoState.timestamp = egoState.timestamp + dt;

end
