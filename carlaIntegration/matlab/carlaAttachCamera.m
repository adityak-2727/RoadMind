function sensorId = carlaAttachCamera(camCfg)
% carlaAttachCamera - Phase 10: spawns and attaches an RGB camera sensor
% to the already-spawned ego vehicle. Requires carlaSpawnEgoVehicle()
% first.
%
% Input:
%   camCfg - struct matching config/carlaConfig.m's cfg.camera schema
%            (width, height, fov, mountX/Y/Z, mountPitch/Yaw/Roll).
% Output:
%   sensorId - CARLA actor id of the spawned camera sensor.

session = getCarlaSession();
sensorId = session.attachCamera(camCfg);

end
