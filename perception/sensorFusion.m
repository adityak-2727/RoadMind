function fusedAgents = sensorFusion(cameraAgents, lidarAgents, radarAgents)
% sensorFusion - associates detections across modalities by spatial nearest-
% neighbor (within a fixed gating distance) and merges them per each
% sensor's simulated strength: camera contributes class, lidar contributes
% position, radar contributes velocity. Detections seen by only one sensor
% are still included using whatever fields that sensor provided, rather
% than being dropped.
%
% Inputs:
%   cameraAgents, lidarAgents, radarAgents - struct arrays from each modality
% Output:
%   fusedAgents - struct array of merged agents, agent.source = "fused"

GATING_DIST = 2.5; % [m] max distance to consider two detections the same object

usedLidar = false(1, numel(lidarAgents));
usedRadar = false(1, numel(radarAgents));
fusedAgents = repmat(createAgent(), 0, 0);

for i = 1:numel(cameraAgents)
    cam = cameraAgents(i);

    lidarIdx = nearestUnused(cam.position, lidarAgents, usedLidar, GATING_DIST);
    radarIdx = nearestUnused(cam.position, radarAgents, usedRadar, GATING_DIST);

    fused = createAgent();
    fused.class = cam.class; % camera -> class

    if ~isempty(lidarIdx)
        fused.position = lidarAgents(lidarIdx).position; % lidar -> position
        usedLidar(lidarIdx) = true;
    else
        fused.position = cam.position;
    end

    if ~isempty(radarIdx)
        fused.velocity = radarAgents(radarIdx).velocity; % radar -> velocity
        usedRadar(radarIdx) = true;
    else
        fused.velocity = cam.velocity;
    end

    fused.heading = atan2(fused.velocity(2), fused.velocity(1));
    fused.confidence = cam.confidence;
    fused.source = "fused";
    fused.timestamp = cam.timestamp;

    fusedAgents(end + 1) = fused; %#ok<AGROW>
end

% Detections only lidar or only radar saw are still real objects - include
% them rather than silently dropping information a single-sensor view had.
for i = 1:numel(lidarAgents)
    if ~usedLidar(i)
        a = lidarAgents(i);
        a.source = "fused";
        fusedAgents(end + 1) = a; %#ok<AGROW>
    end
end
for i = 1:numel(radarAgents)
    if ~usedRadar(i)
        a = radarAgents(i);
        a.source = "fused";
        fusedAgents(end + 1) = a; %#ok<AGROW>
    end
end

end

function idx = nearestUnused(position, candidates, used, gatingDist)
% Returns the index of the closest not-yet-claimed candidate within
% gatingDist, or [] if none qualifies.
idx = [];
bestDist = gatingDist;
for k = 1:numel(candidates)
    if used(k)
        continue;
    end
    d = norm(candidates(k).position - position);
    if d < bestDist
        bestDist = d;
        idx = k;
    end
end
end
