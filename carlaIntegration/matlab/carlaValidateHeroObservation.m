function carlaValidateHeroObservation(obs, cfg)
% Demo-only validation, before planning/actuation; Inf TTC is legitimate.
assert(~isempty(obs.egoState), 'Phase15:missingEgo', 'Missing live ego state.');
e = obs.egoState;
assert(all(isfinite([e.x e.y e.yaw e.velocity e.timestamp])), ...
    'Phase15:invalidEgo', 'Non-finite ego state.');
names = {'cameraFrame','lidarPoints','radarDetections','actorObjects'};
for i = 1:numel(names)
    stream = obs.(names{i});
    assert(~isempty(stream), 'Phase15:missingSensor', 'Required stream %s is missing.', names{i});
    assert(isfinite(stream.timestamp) && abs(e.timestamp-stream.timestamp) <= cfg.staleTimeoutSeconds, ...
        'Phase15:staleSensor', 'Required stream %s is stale against live ego time.', names{i});
end
assert(all(isfinite(obs.lidarPoints.xyzi), 'all') && ...
    all(isfinite(obs.radarDetections.raw), 'all') && ...
    all(isfinite(obs.actorObjects.raw), 'all'), ...
    'Phase15:invalidSensor', 'Non-finite sensor data.');
end
