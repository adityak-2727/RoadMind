function agents = carlaLidarPointsToAgents(lidarPoints)
% carlaLidarPointsToAgents - Phase 10: converts a real CARLA LiDAR sweep
% (carlaGetLidarPoints.m's output) into config/createAgent.m-schema
% agents, via minimum-necessary ground filtering + distance-based
% clustering. This is sensor-INTERFACE-level object extraction (Phase 10
% scope), not a tracker, not a classifier, and not sensor fusion (Phase
% 11) - deliberately simple, per the explicit "minimum necessary...not
% an over-engineered stack" instruction.
%
% Pipeline:
%   1. Downsample (deterministic stride) if the sweep exceeds
%      maxPointsForClustering - a reliability safeguard against a
%      pathologically slow clustering pass, not an accuracy feature.
%   2. Ground filter: drop points at/below groundZThreshold in the
%      sensor's own local frame (the LiDAR is mounted ~2.2m above the
%      road - see config/carlaConfig.m's cfg.lidar.mountZ - so points
%      near the sensor's local z=-2.2 are the road surface).
%   3. Axis convert: apply carlaCoordToProject's x,-y mirror to the
%      surviving points (sensor-local CARLA frame -> project frame).
%   4. Cluster: greedy single-link distance clustering (each point joins
%      the nearest existing cluster centroid within clusterRadius, else
%      starts a new cluster) - a minimum-necessary "one cluster per
%      obstacle" grouping, not a full segmentation stack.
%   5. Emit one createAgent() per cluster, at its centroid.
%
% KNOWN LIMITATION (documented, not fixed here - out of Phase 10 scope):
% the ground filter only removes points near the road surface; it does
% NOT distinguish traffic actors from static world geometry (buildings,
% poles, curbs, foliage), so clusters may include non-traffic obstacles.
% A real obstacle/background classifier is deferred to a later phase.
%
% class is always "unknown" (never invented - raw geometry alone gives
% no semantic class, matching perception/lidarDetection.m's synthetic
% counterpart's identical documented limitation). velocity is always
% [0, 0] (not observable from a single sweep, same limitation).
%
% Input:
%   lidarPoints - struct from carlaGetLidarPoints.m (.xyzi Nx4, sensor-
%                 local CARLA frame, .numPoints, .timestamp), or [] (no
%                 sweep yet -> returns an empty agent array). Each
%                 emitted agent is stamped with lidarPoints.timestamp -
%                 the sweep's own CARLA capture time - not a caller-
%                 supplied value, so an agent's timestamp always
%                 traces back to the exact sweep it came from.
% Output:
%   agents - struct array of createAgent()-schema agents, agent.source =
%            "carla_lidar" (distinguishable from the synthetic "lidar"
%            source used by perception/lidarDetection.m elsewhere in the
%            project).

agents = repmat(createAgent(), 0, 0);
if isempty(lidarPoints) || lidarPoints.numPoints == 0
    return;
end

groundZThreshold = -1.5;      % [m] sensor-local z (mount height ~2.2m)
clusterRadius = 1.5;          % [m] greedy single-link clustering radius
maxPointsForClustering = 3000; % reliability cap, deterministic downsample above this

xyzi = lidarPoints.xyzi;
if size(xyzi, 1) > maxPointsForClustering
    stride = ceil(size(xyzi, 1) / maxPointsForClustering);
    xyzi = xyzi(1:stride:end, :);
end

nonGround = xyzi(xyzi(:, 3) > groundZThreshold, :);
if isempty(nonGround)
    return;
end

[xProj, yProj] = carlaCoordToProject(nonGround(:, 1), nonGround(:, 2));
points = [xProj, yProj];

clusterCentroids = zeros(0, 2);
clusterCounts = zeros(0, 1);

for i = 1:size(points, 1)
    p = points(i, :);
    assigned = 0;
    for c = 1:size(clusterCentroids, 1)
        if norm(p - clusterCentroids(c, :)) <= clusterRadius
            assigned = c;
            break;
        end
    end
    if assigned == 0
        clusterCentroids(end + 1, :) = p; %#ok<AGROW>
        clusterCounts(end + 1, 1) = 1; %#ok<AGROW>
    else
        n = clusterCounts(assigned);
        clusterCentroids(assigned, :) = (clusterCentroids(assigned, :) * n + p) / (n + 1);
        clusterCounts(assigned) = n + 1;
    end
end

for c = 1:size(clusterCentroids, 1)
    a = createAgent();
    a.id = c;
    a.class = "unknown";
    a.position = clusterCentroids(c, :);
    a.velocity = [0, 0];
    a.confidence = min(0.95, 0.4 + 0.05 * clusterCounts(c)); % more points -> mildly higher confidence, capped
    a.source = "carla_lidar";
    a.timestamp = lidarPoints.timestamp;
    agents(end + 1) = a; %#ok<AGROW>
end

end
