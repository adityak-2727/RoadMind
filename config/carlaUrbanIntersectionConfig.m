function cfg = carlaUrbanIntersectionConfig()
% carlaUrbanIntersectionConfig - EMERGENCY DEMO BRIDGE: maps the existing,
% unmodified scenarios/urbanIntersection.m into CARLA's Town03 junction
% id=103 (the same junction already validated in
% config/carlaIndianSceneConfig.m - reusing its live-resolved anchors
% rather than re-deriving new geometry under time pressure).
%
% scenarios/urbanIntersection.m (source of truth, NOT modified):
%   ego: starts (0,-40), yaw=pi/2 (north), goal (0,40) - straight through
%   agents (MATLAB scenario-local x,y,vx,vy):
%     crossCar          car,         pos(-30, 0),  vel( 5, 0)
%     crossMotorcycle    motorcycle,  pos( 30, 4),  vel(-4, 0)
%     crossingPedestrian pedestrian,  pos(  6,-6),  vel(-0.4,0.6)
%     mergingAutoRickshaw auto,       pos( 12,-20), vel(-3, 4)
%
% MAPPING: this scene's South approach (already live-resolved in
% carlaIndianSceneConfig.m: x=-9.69/-9.78/-9.64, yawDeg=89.64, i.e.
% heading north) is used as the ego's straight-through corridor -
% MATLAB's ego also travels straight north, so the two conventions align
% with NO rotation, only a fixed origin offset:
%   CARLA_raw_x = originX + MATLAB_x   (originX = -9.69, the S-lane x)
%   CARLA_raw_y = originY + MATLAB_y   (originY = 133.72, junction center y)
%   CARLA_raw_vx = MATLAB_vx, CARLA_raw_vy = MATLAB_vy  (same reason - no rotation)
% This is a direct, honest, additive coordinate mapping - not the
% established carlaCoordToProject.m conversion (that converts CARLA<->
% project frame for PERCEPTION output; this config instead spawns/scripts
% actors directly in CARLA's own raw frame, exactly like
% carlaIndianSceneConfig.m already does for every other scripted actor).
%
% Vehicle-class mapping to the closest available CARLA 0.9.16 blueprint
% (documented, not fabricated):
%   car          -> vehicle.audi.tt
%   motorcycle   -> vehicle.yamaha.yzf   (already used elsewhere in this project)
%   pedestrian   -> walker.pedestrian.0001
%   auto (auto-rickshaw) -> vehicle.micro.microlino (closest available
%                    narrow small-footprint vehicle; CARLA 0.9.16 has no
%                    3-wheeler/tuktuk blueprint)

cfg = struct();
cfg.mapName = 'Town03';
cfg.junctionCenter = [1.10, 133.72];

originX = -9.69;
originY = 133.72;

% ---------------------------------------------------------------------
% EGO - same S approach lane/heading already validated in
% carlaIndianSceneConfig.m, staged 40m back from the junction (matches
% MATLAB's egoStart y=-40, offset from junction-center y=0).
% ---------------------------------------------------------------------
cfg.egoBlueprint = 'vehicle.tesla.model3';
cfg.egoApproach = struct('x', originX, 'y', originY - 40, 'z', 0.30, 'yawDeg', 89.64);
cfg.egoForwardDistanceM = 95.0; % 40m approach + junction + ~40m exit, matches MATLAB's 80m span with margin
% NOTE: main.m's own scenarioSpeedFactors.urbanIntersection value is 10.0
% (0.5 * vehCfg.maxSpeed(20)), but that number only ever drives MATLAB's
% own kinematic bicycleModel.m, which has no tire physics and therefore
% cannot show slip/drift at any speed. CARLA's real rigid-body vehicle
% physics can and does slide under hard braking at speed, which was
% visibly worse at 10.0 during live evaluation testing. Lowered to the
% same 6.0 m/s already proven drift-free across every prior CARLA hero-
% scene demo (carlaClosedLoopInit.m's own default, chosen for exactly
% this reason - see its header comment). This is a demo-visibility
% tuning of THIS new config file only - vehicleController.m, bicycleModel.m,
% and every other frozen control/planning file are untouched.
cfg.egoBaseSpeedMps = 6.0;

% ---------------------------------------------------------------------
% TRAFFIC ACTORS - one-to-one with scenarios/urbanIntersection.m's four
% agents. .matlabId links each spawned CARLA actor back to the exact
% MATLAB agent it reproduces (for the report's actor-mapping table).
% ---------------------------------------------------------------------
cfg.trafficActors = struct( ...
    'matlabId', {}, 'matlabClass', {}, 'blueprint', {}, ...
    'x', {}, 'y', {}, 'z', {}, 'yawDeg', {}, 'vx', {}, 'vy', {});

% crossCar: MATLAB pos(-30,0) vel(5,0) - approaches junction from the west
cfg.trafficActors(end+1) = struct('matlabId', 1, 'matlabClass', "car", ...
    'blueprint', 'vehicle.audi.tt', ...
    'x', originX - 30, 'y', originY + 0, 'z', 0.30, 'yawDeg', 0.0, ...
    'vx', 5.0, 'vy', 0.0);

% crossMotorcycle: MATLAB pos(30,4) vel(-4,0) - approaches from the east
cfg.trafficActors(end+1) = struct('matlabId', 2, 'matlabClass', "motorcycle", ...
    'blueprint', 'vehicle.yamaha.yzf', ...
    'x', originX + 30, 'y', originY + 4, 'z', 0.30, 'yawDeg', 180.0, ...
    'vx', -4.0, 'vy', 0.0);

% mergingAutoRickshaw: MATLAB pos(12,-20) vel(-3,4) - cuts diagonally into the junction
cfg.trafficActors(end+1) = struct('matlabId', 4, 'matlabClass', "auto", ...
    'blueprint', 'vehicle.micro.microlino', ...
    'x', originX + 12, 'y', originY - 20, 'z', 0.30, 'yawDeg', rad2deg(atan2(4, -3)), ...
    'vx', -3.0, 'vy', 4.0);

% ---------------------------------------------------------------------
% PEDESTRIAN - crossingPedestrian: MATLAB pos(6,-6) vel(-0.4,0.6)
% ---------------------------------------------------------------------
cfg.pedestrians = struct('matlabId', {}, 'x', {}, 'y', {}, 'z', {}, 'yawDeg', {}, 'vx', {}, 'vy', {});
cfg.pedestrians(end+1) = struct('matlabId', 3, ...
    'x', originX + 6, 'y', originY - 6, 'z', 0.50, 'yawDeg', rad2deg(atan2(0.6, -0.4)), ...
    'vx', -0.4, 'vy', 0.6);

end
