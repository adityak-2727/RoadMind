function actorId = carlaSpawnActorRelativeToEgo(blueprintId, forwardM, rightM, upM, yawOffsetDeg)
% carlaSpawnActorRelativeToEgo - Phase 10: spawns one NON-EGO actor at a
% position given relative to the ego vehicle's current transform
% (forward/right/up in meters, ego-local frame; yawOffsetDeg added to
% the ego's current yaw). Used ONLY for coordinate-system verification
% and the sensor validation scene - never to drive or represent the ego
% vehicle itself.
%
% Inputs:
%   blueprintId  - CARLA blueprint id string (e.g. 'vehicle.audi.tt',
%                  'walker.pedestrian.0001').
%   forwardM, rightM, upM - offset from the ego vehicle, meters, in the
%                  ego's own local frame (upM defaults are handled by
%                  CarlaSession; pass e.g. 0.5 to clear the ground).
%   yawOffsetDeg - added to the ego's current yaw (degrees).
% Output:
%   actorId - CARLA actor id of the spawned actor, or [] if the spawn
%             failed (e.g. collision at that exact point - try a
%             different offset).

session = getCarlaSession();
actorId = session.spawnActorRelativeToEgo(blueprintId, forwardM, rightM, upM, yawOffsetDeg);

end
