function sensorId = carlaAttachLidar(lidarCfg)
% carlaAttachLidar - Phase 10: spawns and attaches a ray-cast LiDAR
% sensor to the already-spawned ego vehicle. Requires
% carlaSpawnEgoVehicle() first.
%
% Input:
%   lidarCfg - struct matching config/carlaConfig.m's cfg.lidar schema
%              (channels, range, pointsPerSecond, rotationFrequency,
%              upperFov, lowerFov, mountX/Y/Z, mountPitch/Yaw/Roll).
% Output:
%   sensorId - CARLA actor id of the spawned LiDAR sensor.

session = getCarlaSession();
sensorId = session.attachLidar(lidarCfg);

end
