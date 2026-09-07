function applied = carlaApplyControl(steer, throttle, brake)
% carlaApplyControl - Task 7 interface: sends a raw steering/throttle/
% brake command to the CARLA ego vehicle (steer in [-1,1], throttle/brake
% in [0,1] - CARLA's own VehicleControl ranges). This is an
% integration-test control channel only (Phase 9 Task 7) - it is not a
% substitute for control/vehicleController.m, which still owns the
% project's real driving logic in later phases. Requires
% carlaSpawnEgoVehicle() first.
%
% Inputs:
%   steer, throttle, brake - scalars; clamped to CARLA's valid ranges by
%                             carla_adapter.py before being applied.
% Output:
%   applied - struct with the actually-applied (post-clamp) steer/
%             throttle/brake values, echoed back from carla_adapter.py.

session = getCarlaSession();
applied = session.applyControl(steer, throttle, brake);

end
