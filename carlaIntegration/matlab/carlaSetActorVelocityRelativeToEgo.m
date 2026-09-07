function carlaSetActorVelocityRelativeToEgo(actorId, forwardMps, rightMps, upMps)
% carlaSetActorVelocityRelativeToEgo - Phase 13: sets a non-ego actor's
% velocity as components along the EGO's CURRENT forward/right axes
% (recomputed from the ego's live transform each call), instead of a raw
% CARLA world-frame (vx, vy) that the caller would have to derive by hand
% from the ego's heading.
%
% Use this for staged scenario actors (crossing/merging/etc.) whenever
% the intent is "close the gap toward the ego's path at rate X" - a
% world-frame vector picked by hand can accidentally have zero component
% in the direction that matters if the road isn't axis-aligned the way
% the picker assumed (see CarlaSession.m/carla_adapter.py for the live
% case this was built to fix: a crossing actor given a world-frame
% velocity that turned out to be purely longitudinal, so it never
% actually entered the ego's planning corridor).
%
% Inputs:
%   actorId    - CARLA actor id from carlaSpawnActorRelativeToEgo().
%   forwardMps - velocity component along the ego's current forward axis
%                (m/s). Positive = same direction the ego is facing.
%   rightMps   - velocity component along the ego's current right axis
%                (m/s). Positive = toward the ego's right.
%   upMps      - vertical velocity component (m/s), usually 0.

session = getCarlaSession();
session.setActorVelocityRelativeToEgo(actorId, forwardMps, rightMps, upMps);

end
