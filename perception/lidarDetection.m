function agents = lidarDetection(pointCloud, lidarParams, timestamp)
% lidarDetection - simulates lidar-based obstacle detection. `pointCloud`
% is not a real point cloud - it is the set of ground-truth agents already
% filtered to this sensor's range/FOV by the caller (main.m), which plays
% the role of "the lidar sweeping the scene"; this function plays the role
% of "clustering/segmenting that sweep into obstacles". See
% cameraDetection.m for why this ground-truth-as-synthetic-detection
% approach is used and how it must be labeled.
%
% Lidar's simulated strength is precise position (low noise); its simulated
% weakness is that raw geometry alone gives no semantic class without a
% separate classifier, and a single sweep gives no velocity either.
%
% Inputs:
%   pointCloud  - struct array of ground-truth agents in lidar range/FOV
%   lidarParams - struct from config/sensorConfig.m (cfg.lidar)
%   timestamp   - [s] current simulation time
% Output:
%   agents      - struct array of detected agents, agent.source = "lidar"

agents = repmat(createAgent(), 0, 0);

for i = 1:numel(pointCloud)
    if rand() > lidarParams.detectionProbability
        continue; % simulated missed detection
    end

    gt = pointCloud(i);
    detected = gt;
    detected.position = gt.position + randn(1, 2) * lidarParams.positionNoiseStd;
    detected.class = "unknown"; % geometry alone doesn't give semantic class
    detected.velocity = [0, 0]; % not observable from a single sweep
    detected.confidence = min(0.99, max(0.3, lidarParams.confidenceBase + randn() * 0.03));
    detected.source = "lidar";
    detected.timestamp = timestamp;

    agents(end + 1) = detected; %#ok<AGROW>
end

end
