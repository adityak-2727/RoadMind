function rawState = carlaGetActorState(actorId)
% carlaGetActorState - Phase 10: retrieves a non-ego test actor's raw
% CARLA-frame state (same shape as carlaGetEgoState.m's raw form),
% for coordinate-verification readback. actorId must have been returned
% by carlaSpawnActorRelativeToEgo().
%
% Output:
%   rawState - struct with .location, .rotation_deg, .velocity_mps,
%              .frame, .timestamp_s, .type_id, .actor_id (CARLA's own
%              units/frame, unconverted - pass through
%              carlaCoordToProject.m/carlaYawToProject.m for the
%              project-frame equivalent).

session = getCarlaSession();
rawState = session.getActorState(actorId);

end
