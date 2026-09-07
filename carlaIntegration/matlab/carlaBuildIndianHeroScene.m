function sceneState = carlaBuildIndianHeroScene(cfg)
% carlaBuildIndianHeroScene - Phase 11.5: builds the Indian urban hero
% environment from config/carlaIndianSceneConfig.m. Connects to CARLA,
% loads the hero map, spawns the ego at its approach point, spawns every
% traffic/parked/pedestrian actor and every visual prop from the config,
% and freezes traffic-light infrastructure to non-controlling.
%
% Does NOT attach sensors and does NOT drive the ego - this is
% environment staging only, matching the explicit Phase 11.5 scope
% ("environment-only validation" before connecting the autonomy stack).
% Callers that want the full sensor/perception/fusion/tracking/
% prediction/decision/planning/control chain running against this scene
% should attach sensors (carlaAttachCamera.m/carlaAttachLidar.m/
% carlaAttachRadar.m, all unmodified) and drive it via
% carlaClosedLoopInit.m/carlaClosedLoopStep.m (unmodified) exactly as
% Phase 12 already does against the old environment - the whole point of
% this phase is that the autonomy stack does not change.
%
% GROUND TRUTH USE: every position in the config is used here only for
% SCENE STAGING (where to spawn each actor) - this is explicitly
% permitted ("Ground truth may be used for spawning actors, setting
% initial positions..."). It is never fed into perception/prediction/
% planning; those still only ever see real camera/LiDAR/radar
% observations, exactly as every prior phase has enforced.
%
% Input:
%   cfg - optional, struct from config/carlaIndianSceneConfig.m;
%         defaults to carlaIndianSceneConfig() if omitted.
% Output:
%   sceneState - struct: .egoId, .trafficActorIds (parallel to
%                cfg.trafficActors), .parkedVehicleIds, .pedestrianIds,
%                .roadDefectPropIds, .clutterPropIds, .numTrafficLightsFrozen,
%                .failedSpawns (cell array of {kind, index, blueprint}
%                for any spawn that returned [] - collisions do happen
%                with this many actors, and are reported rather than
%                silently ignored)

if nargin < 1 || isempty(cfg)
    cfg = carlaIndianSceneConfig();
end

carlaCfg = carlaConfig();
carlaCfg.mapName = cfg.mapName;
carlaConnect(carlaCfg);

% FIX (found during the Phase 11.6 audit): carlaConnect()/connect() never
% loads a specific map on its own - see carlaLoadMap.m's header. Without
% this, a scene built against a freshly-launched server (which defaults
% to Town10HD_Opt) would silently spawn this scene's Town03-specific
% coordinates into the wrong map's geometry.
mapReloaded = carlaLoadMap(cfg.mapName);
if mapReloaded
    fprintf('[carlaBuildIndianHeroScene] Loaded map %s (was not already active).\n', cfg.mapName);
    pause(2.0); % let the new world settle before spawning into it
end

sceneState = struct();
sceneState.failedSpawns = repmat(struct('kind', "", 'index', 0, 'blueprint', ""), 0, 0);

sceneState.egoId = carlaSpawnEgoVehicleAtTransform(cfg.egoBlueprint, ...
    cfg.egoApproach.x, cfg.egoApproach.y, cfg.egoApproach.z, cfg.egoApproach.yawDeg);
fprintf('[carlaBuildIndianHeroScene] Ego spawned, id=%d, approach=%s\n', sceneState.egoId, cfg.egoApproach.approach);

sceneState.trafficActorIds = nan(1, numel(cfg.trafficActors));
for i = 1:numel(cfg.trafficActors)
    a = cfg.trafficActors(i);
    id = carlaSpawnActorAtTransform(a.blueprint, a.x, a.y, a.z, a.yawDeg);
    if isempty(id)
        sceneState.failedSpawns(end+1) = struct('kind', "traffic", 'index', i, 'blueprint', string(a.blueprint)); %#ok<AGROW>
        fprintf('[carlaBuildIndianHeroScene] WARNING: traffic actor %d (%s, approach %s) FAILED to spawn (collision at that point)\n', i, a.blueprint, a.approach);
    else
        sceneState.trafficActorIds(i) = id;
    end
end

sceneState.parkedVehicleIds = nan(1, numel(cfg.parkedVehicles));
for i = 1:numel(cfg.parkedVehicles)
    p = cfg.parkedVehicles(i);
    id = carlaSpawnActorAtTransform(p.blueprint, p.x, p.y, p.z, p.yawDeg);
    if isempty(id)
        sceneState.failedSpawns(end+1) = struct('kind', "parked", 'index', i, 'blueprint', string(p.blueprint)); %#ok<AGROW>
        fprintf('[carlaBuildIndianHeroScene] WARNING: parked vehicle %d (%s) FAILED to spawn\n', i, p.blueprint);
    else
        sceneState.parkedVehicleIds(i) = id;
    end
end

sceneState.pedestrianIds = nan(1, numel(cfg.pedestrians));
for i = 1:numel(cfg.pedestrians)
    ped = cfg.pedestrians(i);
    id = carlaSpawnActorAtTransform('walker.pedestrian.0001', ped.x, ped.y, ped.z, ped.yawDeg);
    if isempty(id)
        sceneState.failedSpawns(end+1) = struct('kind', "pedestrian", 'index', i, 'blueprint', "walker.pedestrian.0001"); %#ok<AGROW>
        fprintf('[carlaBuildIndianHeroScene] WARNING: pedestrian %d FAILED to spawn\n', i);
    else
        sceneState.pedestrianIds(i) = id;
    end
end

sceneState.roadDefectPropIds = nan(1, numel(cfg.roadDefectProps));
for i = 1:numel(cfg.roadDefectProps)
    d = cfg.roadDefectProps(i);
    id = carlaSpawnActorAtTransform(d.blueprint, d.x, d.y, d.z, d.yawDeg);
    if isempty(id)
        sceneState.failedSpawns(end+1) = struct('kind', "roadDefect", 'index', i, 'blueprint', string(d.blueprint)); %#ok<AGROW>
    else
        sceneState.roadDefectPropIds(i) = id;
    end
end

sceneState.clutterPropIds = nan(1, numel(cfg.clutterProps));
for i = 1:numel(cfg.clutterProps)
    c = cfg.clutterProps(i);
    id = carlaSpawnActorAtTransform(c.blueprint, c.x, c.y, c.z, c.yawDeg);
    if isempty(id)
        sceneState.failedSpawns(end+1) = struct('kind', "clutter", 'index', i, 'blueprint', string(c.blueprint)); %#ok<AGROW>
    else
        sceneState.clutterPropIds(i) = id;
    end
end

sceneState.numTrafficLightsFrozen = carlaFreezeTrafficLights( ...
    cfg.junctionCenter(1), cfg.junctionCenter(2), cfg.trafficLightFreezeRangeM);

fprintf(['[carlaBuildIndianHeroScene] Scene built: %d/%d traffic, %d/%d parked, %d/%d pedestrians, ' ...
    '%d/%d road-defect props, %d/%d clutter props, %d traffic lights frozen, %d failed spawns\n'], ...
    sum(~isnan(sceneState.trafficActorIds)), numel(cfg.trafficActors), ...
    sum(~isnan(sceneState.parkedVehicleIds)), numel(cfg.parkedVehicles), ...
    sum(~isnan(sceneState.pedestrianIds)), numel(cfg.pedestrians), ...
    sum(~isnan(sceneState.roadDefectPropIds)), numel(cfg.roadDefectProps), ...
    sum(~isnan(sceneState.clutterPropIds)), numel(cfg.clutterProps), ...
    sceneState.numTrafficLightsFrozen, numel(sceneState.failedSpawns));

end
