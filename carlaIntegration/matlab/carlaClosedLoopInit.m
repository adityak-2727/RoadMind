function loopState = carlaClosedLoopInit(forwardDistanceMeters, baseSpeedMps, demoRangeMeters, customGlobalPath)
% carlaClosedLoopInit - Phase 12: builds the state bundle for
% carlaClosedLoopStep.m's tick-by-tick closed loop. Call once after
% carlaConnect/carlaSpawnEgoVehicle/carlaAttachCamera/carlaAttachLidar/
% carlaAttachRadar have already been called (this function does not
% connect to CARLA itself - it only reads the current ego state to seed
% a start pose).
%
% GOAL/PATH: this is a live CARLA demo, not one of the five named
% scenarios/*.m with an authored map - there is no predefined road
% network or goal here. A straight corridor `forwardDistanceMeters` ahead
% of the ego's CURRENT heading is built via globalPlanner.m (unmodified;
% called exactly as main.m calls it, with mapData=[] so it uses that
% function's own existing straight-line fallback - not a new code path).
% This is a deliberate, documented simplification for the demo, not a
% claim of real route planning.
%
% Inputs:
%   forwardDistanceMeters - optional, default 60. Length of the straight
%                            demo corridor.
%   baseSpeedMps           - optional, default 6.0 m/s (~21.6 km/h) - a
%                            deliberately modest cruise speed for a
%                            readable, deterministic jury demo (Phase 12
%                            spec: "prioritize correctness... deterministic
%                            demonstration" over speed).
%   customGlobalPath        - optional, Phase 13: an Nx2 waypoint array
%                             (e.g. from carlaGenerateIntersectionTurnPath.m)
%                             to use INSTEAD of the straight-line
%                             fallback above. When given, startPose/
%                             goalPose/globalPlanner() are skipped
%                             entirely and this path is used as-is -
%                             localPlanner.m (frozen) consumes it
%                             identically either way (arc-length
%                             parameterization, no straight-line
%                             assumption - confirmed by inspection during
%                             the Phase 11.5 audit), so this is a pure
%                             path-source substitution, not a change to
%                             any planning/decision logic.
%   demoRangeMeters         - optional, default 30. Overrides
%                             carlaPerceptionConfig()'s default
%                             maxSensorRangeMeters (60) for THIS demo
%                             only - a config-level bound, not a change to
%                             any frozen Phase 10/11 file. Necessary
%                             because Phase 10/11's documented LiDAR
%                             limitation (clustering cannot separate
%                             traffic actors from static world geometry -
%                             see docs/carla_integration.md) compounds
%                             tick over tick in a closed loop: live
%                             testing showed the tracked-agent count grow
%                             past 150 within 25 ticks at the 60m default,
%                             which is neither a readable jury demo nor
%                             good for tick latency. A tighter range still
%                             comfortably covers this demo's obstacles
%                             (spawned at 15-30m) while sharply reducing
%                             distant clutter.
% Output:
%   loopState - opaque struct, pass into carlaClosedLoopStep.m

if nargin < 1 || isempty(forwardDistanceMeters)
    forwardDistanceMeters = 60;
end
if nargin < 2 || isempty(baseSpeedMps)
    baseSpeedMps = 6.0;
end
if nargin < 3 || isempty(demoRangeMeters)
    demoRangeMeters = 30;
end

egoState0 = carlaGetEgoState();

if nargin >= 4 && ~isempty(customGlobalPath)
    globalPath = customGlobalPath;
    goalPose = [globalPath(end, 1), globalPath(end, 2), 0];
else
    startPose = [egoState0.x, egoState0.y, egoState0.yaw];
    goalPose  = [egoState0.x + forwardDistanceMeters * cos(egoState0.yaw), ...
                 egoState0.y + forwardDistanceMeters * sin(egoState0.yaw), 0];
    globalPath = globalPlanner(startPose, goalPose, []); % unmodified frozen function
end

perceptionCfg = carlaPerceptionConfig();
perceptionCfg.maxSensorRangeMeters = demoRangeMeters;

loopState = struct( ...
    'vehCfg',              vehicleConfig(), ...
    'planCfg',             plannerConfig(), ...
    'perceptionCfg',       perceptionCfg, ...
    'trackingCfg',         carlaTrackingConfig(), ...
    'globalPath',          globalPath, ...
    'goalPosition',        goalPose(1:2), ...
    'planHorizon',         4.0, ...
    'baseSpeed',           baseSpeedMps, ...
    'decisionSpeedFactors', struct( ...
        'cruise', 1.0, 'follow', 0.8, 'merge', 0.8, 'replan', 0.75, ...
        'avoid', 0.7, 'brake', 0.15, 'wait', 0.0, 'emergency_stop', 0.0), ...
    'decisionState',       "cruise", ...
    'lastTransitionTime',  -Inf, ...
    'minDwellTime',        1.5, ...
    'stateTransitionCount', 0, ...
    'previousCandidateIndex', [], ...
    'previousFusedAgents', [], ...
    'trackerState',        [], ...
    'lastSteeringAngle',   0, ...
    'lastTickTic',         tic, ...
    'firstTick',           true, ...
    'steerSign',           -1, ... % CARLA steer = steerSign * steeringAngle/maxSteerAngle - see carlaClosedLoopStep.m header for the live measurement that verified this sign
    'tickCount',            0 ...
);

end
