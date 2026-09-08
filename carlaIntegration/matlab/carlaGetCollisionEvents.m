function events = carlaGetCollisionEvents()
% carlaGetCollisionEvents - Phase 14: returns every REAL physical
% collision event CARLA's physics engine has reported for the ego vehicle
% since carlaAttachCollisionSensor() was called.
%
% Output:
%   events - struct array (0x0 if zero collisions - genuinely checked and
%            clean, not "never checked"), one entry per contact:
%            .frame, .timestamp, .otherActorId, .otherActorType,
%            .impulseMagnitude. Requires carlaAttachCollisionSensor()
%            first - raises otherwise, so a caller cannot silently
%            mistake "never attached" for "attached and clean".

session = getCarlaSession();
events = session.getCollisionEvents();

end
