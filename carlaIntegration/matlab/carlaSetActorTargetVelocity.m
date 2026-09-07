function carlaSetActorTargetVelocity(actorId, vx, vy, vz)
% carlaSetActorTargetVelocity - Phase 10: sets a spawned non-ego test
% actor's velocity directly (world-frame m/s). Test-only determinism for
% the "one moving toward/away" coordinate-verification case - not
% physics-driven motion, and never applied to the ego vehicle (the ego
% is only ever driven via carlaApplyControl.m).
%
% Inputs:
%   actorId    - id returned by carlaSpawnActorRelativeToEgo().
%   vx, vy, vz - CARLA world-frame velocity components (m/s).

session = getCarlaSession();
session.setActorTargetVelocity(actorId, vx, vy, vz);

end
