function objects = carlaGetNearbyActorObjects(rangeM)
% carlaGetNearbyActorObjects - Phase 10: retrieves CARLA's own
% simulator-grounded ground-truth metadata (position, velocity, heading,
% extent, class) for every vehicle/pedestrian actor within rangeM of the
% ego vehicle.
%
% IMPORTANT: this is CARLA ground-truth actor state, NOT an RGB-image-
% based object detector. It is used here so the camera "object/class
% information" requirement (Phase 10 spec) is satisfied honestly, and it
% must never be presented as if a trained image detector produced it -
% see carla_adapter.py's get_nearby_actor_objects() docstring for the
% same disclosure.
%
% Input:
%   rangeM - scalar, search radius around the ego vehicle (meters).
% Output:
%   objects - struct:
%               .raw         Nx13 double, columns:
%                             [actorId, classCode, x, y, z, vx, vy, vz,
%                              yawDeg, extentX, extentY, extentZ,
%                              distanceM] - all in CARLA's own frame
%                             (left-handed, meters/degrees) - NOT yet
%                             transformed into the project frame; see
%                             carlaCoordToProject.m.
%               .numObjects  N
%               .classNames  cell array of class-code -> name strings
%                             (index 1 = class code 0, etc.)
%               .frame       CARLA simulation frame number this query's
%                             world snapshot was taken on
%               .timestamp   CARLA simulation timestamp (seconds) of
%                             that snapshot

session = getCarlaSession();
objects = session.getNearbyActorObjects(rangeM);

end
