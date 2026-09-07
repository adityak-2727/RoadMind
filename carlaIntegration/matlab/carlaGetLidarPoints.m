function points = carlaGetLidarPoints()
% carlaGetLidarPoints - Phase 10: retrieves the most recently received
% LiDAR sweep. Requires carlaAttachLidar() first.
%
% Output:
%   points - [] if no sweep has arrived yet. Otherwise a struct:
%              .xyzi       Nx4 double, [x, y, z, intensity] per point,
%                          in the LiDAR sensor's own local CARLA frame
%                          (left-handed, meters) - NOT yet transformed
%                          into the project frame; see
%                          carlaCoordToProject.m for that conversion.
%              .numPoints  N
%              .frame      CARLA simulation frame number of the sweep
%              .timestamp  CARLA simulation timestamp (seconds)

session = getCarlaSession();
points = session.getLidarPoints();

end
