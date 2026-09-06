function agents = radarDetection(radarData, radarParams, timestamp)
% radarDetection - simulates radar-based detection. `radarData` is not raw
% range/doppler returns - it is the set of ground-truth agents already
% filtered to this sensor's range/FOV by the caller (main.m), which plays
% the role of "the radar sweeping its beam"; this function plays the role
% of "extracting range/velocity tracks from that return". See
% cameraDetection.m for why this ground-truth-as-synthetic-detection
% approach is used and how it must be labeled.
%
% Radar's simulated strength is velocity (doppler is precise); its
% simulated weakness is poor angular/position resolution, and raw returns
% give no semantic class.
%
% Inputs:
%   radarData   - struct array of ground-truth agents in radar range/FOV
%   radarParams - struct from config/sensorConfig.m (cfg.radar)
%   timestamp   - [s] current simulation time
% Output:
%   agents      - struct array of detected agents, agent.source = "radar"

agents = repmat(createAgent(), 0, 0);

for i = 1:numel(radarData)
    if rand() > radarParams.detectionProbability
        continue; % simulated missed detection
    end

    gt = radarData(i);
    detected = gt;
    detected.position = gt.position + randn(1, 2) * radarParams.positionNoiseStd;
    detected.velocity = gt.velocity + randn(1, 2) * radarParams.velocityNoiseStd;
    detected.class = "unknown"; % raw returns give no semantic class
    detected.confidence = min(0.99, max(0.3, radarParams.confidenceBase + randn() * 0.05));
    detected.source = "radar";
    detected.timestamp = timestamp;

    agents(end + 1) = detected; %#ok<AGROW>
end

end
