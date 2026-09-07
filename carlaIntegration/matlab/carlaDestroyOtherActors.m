function carlaDestroyOtherActors()
% carlaDestroyOtherActors - Phase 12: destroys every non-ego actor
% spawned via carlaSpawnActorRelativeToEgo() so far, without
% disconnecting or touching the ego vehicle/sensors. Lets a multi-section
% demo reset the scene to a clean slate between sections. Safe to call
% even if nothing was spawned, and safe to call more than once.

session = getCarlaSession();
session.destroyOtherActors();

end
