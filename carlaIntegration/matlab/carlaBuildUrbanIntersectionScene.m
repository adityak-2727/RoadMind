function sceneState = carlaBuildUrbanIntersectionScene(cfg)
% carlaBuildUrbanIntersectionScene - EMERGENCY DEMO BRIDGE: spawns the
% ego and the four scenarios/urbanIntersection.m traffic/pedestrian
% agents into CARLA, reusing the exact same pattern as
% carlaBuildIndianHeroScene.m (unmodified - that file is not touched).
%
% Input:
%   cfg - optional, from config/carlaUrbanIntersectionConfig.m
% Output:
%   sceneState - .egoId, .trafficActorIds (parallel to cfg.trafficActors),
%                .pedestrianIds (parallel to cfg.pedestrians), .failedSpawns

if nargin < 1 || isempty(cfg)
    cfg = carlaUrbanIntersectionConfig();
end

carlaCfg = carlaConfig();
carlaCfg.mapName = cfg.mapName;
carlaConnect(carlaCfg);

mapReloaded = carlaLoadMap(cfg.mapName);
if mapReloaded
    fprintf('[carlaBuildUrbanIntersectionScene] Loaded map %s (was not already active).\n', cfg.mapName);
    pause(2.0);
end

sceneState = struct();
sceneState.failedSpawns = repmat(struct('kind', "", 'index', 0, 'blueprint', ""), 0, 0);

sceneState.egoId = carlaSpawnEgoVehicleAtTransform(cfg.egoBlueprint, ...
    cfg.egoApproach.x, cfg.egoApproach.y, cfg.egoApproach.z, cfg.egoApproach.yawDeg);
fprintf('[carlaBuildUrbanIntersectionScene] Ego spawned, id=%d\n', sceneState.egoId);

sceneState.trafficActorIds = nan(1, numel(cfg.trafficActors));
for i = 1:numel(cfg.trafficActors)
    a = cfg.trafficActors(i);
    id = carlaSpawnActorAtTransform(a.blueprint, a.x, a.y, a.z, a.yawDeg);
    if isempty(id)
        sceneState.failedSpawns(end+1) = struct('kind', "traffic", 'index', i, 'blueprint', string(a.blueprint)); %#ok<AGROW>
        fprintf('[carlaBuildUrbanIntersectionScene] WARNING: traffic actor %d (%s, matlab agent %d) FAILED to spawn\n', ...
            i, a.blueprint, a.matlabId);
    else
        sceneState.trafficActorIds(i) = id;
    end
end

sceneState.pedestrianIds = nan(1, numel(cfg.pedestrians));
for i = 1:numel(cfg.pedestrians)
    p = cfg.pedestrians(i);
    id = carlaSpawnActorAtTransform('walker.pedestrian.0001', p.x, p.y, p.z, p.yawDeg);
    if isempty(id)
        sceneState.failedSpawns(end+1) = struct('kind', "pedestrian", 'index', i, 'blueprint', "walker.pedestrian.0001"); %#ok<AGROW>
        fprintf('[carlaBuildUrbanIntersectionScene] WARNING: pedestrian %d FAILED to spawn\n', i);
    else
        sceneState.pedestrianIds(i) = id;
    end
end

sceneState.numTrafficLightsFrozen = carlaFreezeTrafficLights( ...
    cfg.junctionCenter(1), cfg.junctionCenter(2), 80.0);

fprintf('[carlaBuildUrbanIntersectionScene] Scene built: %d/%d traffic, %d/%d pedestrians, %d traffic lights frozen, %d failed spawns\n', ...
    sum(~isnan(sceneState.trafficActorIds)), numel(cfg.trafficActors), ...
    sum(~isnan(sceneState.pedestrianIds)), numel(cfg.pedestrians), ...
    sceneState.numTrafficLightsFrozen, numel(sceneState.failedSpawns));

pause(3.0); % let spawn-overlap physics settle before driving (Phase 14 lesson, same as carlaBuildIndianHeroScene.m)

end
