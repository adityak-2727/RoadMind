function sensorId = carlaAttachRadar(radarCfg)
% carlaAttachRadar - Phase 10: spawns and attaches a radar sensor to the
% already-spawned ego vehicle. Requires carlaSpawnEgoVehicle() first.
%
% Input:
%   radarCfg - struct matching config/carlaConfig.m's cfg.radar schema
%              (horizontalFov, verticalFov, range, pointsPerSecond,
%              mountX/Y/Z, mountPitch/Yaw/Roll).
% Output:
%   sensorId - CARLA actor id of the spawned radar sensor.

session = getCarlaSession();
sensorId = session.attachRadar(radarCfg);

end
