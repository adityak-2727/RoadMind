function actorId = carlaSpawnEgoVehicleAtTransform(egoBlueprint, x, y, z, yawDeg)
% carlaSpawnEgoVehicleAtTransform - Phase 11.5: spawns the ego vehicle at
% an explicit, deterministic world transform - used for the Indian hero
% scene, where the ego's approach point was resolved once from the map's
% real road waypoints (see config/carlaIndianSceneConfig.m) rather than
% picked from whichever spawn points CARLA's map happens to define.
% carlaSpawnEgoVehicle.m (Phase 9, unmodified) still exists for the
% spawn-point-index use case.
%
% Inputs:
%   egoBlueprint - CARLA blueprint id string (e.g. 'vehicle.tesla.model3')
%   x, y, z      - world position (meters)
%   yawDeg       - world heading (degrees, CARLA convention)
% Output:
%   actorId - CARLA actor id of the spawned ego vehicle

session = getCarlaSession();
actorId = session.spawnEgoVehicleAtTransform(egoBlueprint, x, y, z, yawDeg);

end
