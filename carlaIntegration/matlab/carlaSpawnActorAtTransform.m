function actorId = carlaSpawnActorAtTransform(blueprintId, x, y, z, yawDeg)
% carlaSpawnActorAtTransform - Phase 11.5: spawns a NON-EGO actor at an
% explicit, deterministic world transform. Used for the Indian hero
% scene's traffic/parked-vehicle/pedestrian manifest
% (config/carlaIndianSceneConfig.m), where every actor's position is
% resolved from the map's actual road geometry, not from the ego's
% current pose (contrast carlaSpawnActorRelativeToEgo.m, Phase 10,
% unmodified, still used where ego-relative placement is what's needed).
%
% Inputs:
%   blueprintId - CARLA blueprint id string
%   x, y, z     - world position (meters)
%   yawDeg      - world heading (degrees, CARLA convention)
% Output:
%   actorId - CARLA actor id, or [] if the spawn point was occupied/
%             colliding (never errors for that expected case).

session = getCarlaSession();
actorId = session.spawnActorAtTransform(blueprintId, x, y, z, yawDeg);

end
