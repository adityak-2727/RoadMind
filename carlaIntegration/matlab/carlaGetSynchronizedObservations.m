function obs = carlaGetSynchronizedObservations(perceptionCfg)
% carlaGetSynchronizedObservations - Phase 11 time-synchronization layer.
% Polls all three CARLA sensors plus the CARLA-grounded actor metadata,
% then decides which of those observations legitimately belong to the same
% simulation instant before anything is fused.
%
% This exists because CARLA's sensors are genuinely asynchronous: each
% delivers frames on its own callback (~16 Hz each, measured), so a naive
% "read all three and fuse" silently mixes observations from different
% simulation instants. Measured spread across the three sensors is
% ~0.032s typical and up to ~0.10s - see config/carlaPerceptionConfig.m
% for the full measured derivation of syncToleranceSeconds.
%
% WHY THE REFERENCE IS A SENSOR TIMESTAMP, NOT THE WORLD SNAPSHOT:
% the CARLA-grounded actor metadata is an ON-DEMAND query - its
% timestamp reflects when MATLAB *asked*, not when an observation was
% made, so it is always "now". Using it as the reference instant
% systematically penalises the three real sensors by however long the
% MATLAB-side data transfer takes (measured: pulling a 640x480 RGB frame
% plus a LiDAR sweep across the py.* boundary costs ~0.1-0.4s), which in
% an early version marked ALL THREE genuine sensors stale on every tick
% and left fusion running on ground truth alone. The reference is
% therefore taken from the ASYNCHRONOUS SENSORS ONLY (camera/LiDAR/
% radar), which are the streams that actually carry an observation
% instant; the actor metadata is then gated against that same reference
% like any other stream. Actor metadata is also queried FIRST, before
% the expensive sensor transfers, so its timestamp lands at or before
% the sensor instants rather than drifting past them.
%
% SYNCHRONIZATION RULE:
%   1. The NEWEST timestamp among the asynchronous sensors (camera,
%      LiDAR, radar) becomes the reference instant for this tick.
%   2. Every stream within syncToleranceSeconds of that reference is
%      marked in-sync and is eligible for fusion.
%   3. A sensor outside the tolerance but within staleTimeoutSeconds is
%      marked STALE: it is reported in the metadata but EXCLUDED from
%      this tick's fusion, rather than being silently fused across
%      instants.
%   4. A sensor with no data at all, or older than staleTimeoutSeconds,
%      is marked MISSING. Stale data is never reused indefinitely.
%
% Nothing is silently dropped: obs.status records, per sensor, exactly
% which of those four outcomes occurred and the measured offset, so a
% caller (and the Phase 11 visualization) can always explain why a sensor
% did or did not contribute.
%
% Input:
%   perceptionCfg - optional, struct from config/carlaPerceptionConfig.m;
%                   defaults to carlaPerceptionConfig() if omitted.
% Output:
%   obs - struct:
%     .cameraFrame      raw struct from carlaGetCameraFrame(), or []
%     .lidarPoints      raw struct from carlaGetLidarPoints(), or []
%     .radarDetections  raw struct from carlaGetRadarDetections(), or []
%     .actorObjects     raw struct from carlaGetNearbyActorObjects(), or []
%                        (CARLA-grounded actor metadata - simulator ground
%                        truth, NOT an image-based detector; see
%                        carlaActorObjectsToAgents.m's honesty note)
%     .egoState         project-frame ego state (createEgoState schema)
%     .referenceTime    [s] the reference instant chosen above
%     .referenceFrame   CARLA frame number of the reference observation
%     .status           struct with one field per stream (camera, lidar,
%                       radar, actors), each:
%                         .state    "in_sync" | "stale" | "missing"
%                         .offset   [s] measured |t - referenceTime|, or []
%                         .frame    that stream's CARLA frame number, or []
%                         .usable   logical, true only for "in_sync"
%     .maxOffset        [s] measured max offset among usable streams
%     .anyUsable        logical, false when nothing is usable this tick

if nargin < 1 || isempty(perceptionCfg)
    perceptionCfg = carlaPerceptionConfig();
end

% --- Poll every stream. Each getter already returns [] rather than
% erroring when its sensor was never attached or has produced nothing yet
% (Phase 10 contract), EXCEPT that a getter called before its sensor is
% attached raises - so each is guarded independently. A sensor that is
% simply absent must degrade to "missing", never crash the pipeline.
%
% Order matters (see header): the cheap on-demand actor query goes FIRST
% so its "now" timestamp does not drift past the sensor instants while
% the expensive image/point-cloud transfers happen.
acquisitionTic  = tic;
actorObjects    = tryGet(@() carlaGetNearbyActorObjects(perceptionCfg.maxSensorRangeMeters));
egoState        = tryGet(@carlaGetEgoState);
radarDetections = tryGet(@carlaGetRadarDetections);
lidarPoints     = tryGet(@carlaGetLidarPoints);
cameraFrame     = tryGet(@carlaGetCameraFrame);
acquisitionElapsed = toc(acquisitionTic);

streams = { ...
    'camera', cameraFrame; ...
    'lidar',  lidarPoints; ...
    'radar',  radarDetections; ...
    'actors', actorObjects};

% --- Reference instant = newest ASYNCHRONOUS SENSOR timestamp. The
% actor-metadata stream is deliberately excluded from defining the
% reference (see header) - it is gated against the reference, not the
% source of it.
timestamps = [];
for i = 1:3 % camera, lidar, radar only
    s = streams{i, 2};
    if ~isempty(s)
        timestamps(end + 1) = s.timestamp; %#ok<AGROW>
    end
end

% Fallback: if no real sensor is attached at all but actor metadata is
% available, that stream defines the instant instead - otherwise a
% ground-truth-only configuration could never produce output.
if isempty(timestamps) && ~isempty(actorObjects)
    timestamps = actorObjects.timestamp;
end

obs = struct();
obs.cameraFrame     = cameraFrame;
obs.lidarPoints     = lidarPoints;
obs.radarDetections = radarDetections;
obs.actorObjects    = actorObjects;
obs.egoState        = egoState;
obs.status          = struct();

if isempty(timestamps)
    % Nothing at all arrived - a clean empty result, not an error.
    for i = 1:size(streams, 1)
        obs.status.(streams{i, 1}) = struct('state', "missing", 'offset', [], 'frame', [], 'usable', false);
    end
    obs.referenceTime  = [];
    obs.referenceFrame = [];
    obs.maxOffset      = 0;
    obs.anyUsable      = false;
    obs.acquisitionElapsed = acquisitionElapsed;
    return;
end

referenceTime = max(timestamps);

referenceFrame = [];
usableOffsets = [];
for i = 1:size(streams, 1)
    name = streams{i, 1};
    s = streams{i, 2};

    if isempty(s)
        obs.status.(name) = struct('state', "missing", 'offset', [], 'frame', [], 'usable', false);
        continue;
    end

    % The actor-metadata stream is gated with its own, looser tolerance -
    % it is an on-demand query whose offset is dominated by MATLAB<->Python
    % transfer cost rather than by CARLA, and it is motion-compensated to
    % the reference instant downstream. See carlaPerceptionConfig.m for the
    % full justification of why these two tolerances differ.
    if strcmp(name, 'actors')
        tolerance = perceptionCfg.actorQueryToleranceSeconds;
    else
        tolerance = perceptionCfg.syncToleranceSeconds;
    end

    offset = abs(s.timestamp - referenceTime);
    if offset <= tolerance
        state = "in_sync";
        usable = true;
        usableOffsets(end + 1) = offset; %#ok<AGROW>
        if offset == 0
            referenceFrame = s.frame; % the stream that defined the reference instant
        end
    elseif offset <= perceptionCfg.staleTimeoutSeconds
        state = "stale";   % present but too far off to be the same instant
        usable = false;
    else
        state = "missing"; % older than the stale timeout - never reused
        usable = false;
    end

    obs.status.(name) = struct('state', state, 'offset', offset, 'frame', s.frame, 'usable', usable);
end

obs.referenceTime  = referenceTime;
obs.referenceFrame = referenceFrame;
obs.maxOffset      = maxOrZero(usableOffsets);
obs.anyUsable      = ~isempty(usableOffsets);
obs.acquisitionElapsed = acquisitionElapsed; % [s] wall-clock cost of pulling all streams across the py.* boundary

end

function value = tryGet(getterFn)
% Calls a Phase 10 getter, converting "sensor not attached" into [] rather
% than letting it abort the whole perception tick. A genuinely absent
% sensor is a supported Phase 11 configuration (camera-only, LiDAR-only,
% etc.), not an error condition.
try
    value = getterFn();
catch
    value = [];
end
end

function m = maxOrZero(v)
if isempty(v)
    m = 0;
else
    m = max(v);
end
end
