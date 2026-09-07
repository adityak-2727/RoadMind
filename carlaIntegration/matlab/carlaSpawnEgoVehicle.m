function actorId = carlaSpawnEgoVehicle(cfg)
% carlaSpawnEgoVehicle - Task 5 interface: spawns one deterministic ego
% vehicle (config/carlaConfig.m's egoBlueprint/spawnPointIndex) and
% returns its CARLA actor id. Requires carlaConnect() to have succeeded
% first.
%
% Input:
%   cfg - optional, struct from config/carlaConfig.m; defaults to
%         carlaConfig() if omitted.
% Output:
%   actorId - the spawned vehicle's CARLA actor id (double)

if nargin < 1 || isempty(cfg)
    cfg = carlaConfig();
end

session = getCarlaSession();
actorId = session.spawnEgoVehicle(cfg);

end
