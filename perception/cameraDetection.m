function agents = cameraDetection(image, cameraParams, timestamp)
% cameraDetection - simulates camera-based object detection. `image` is not
% a real camera frame - there is no rendered scene here - it is the set of
% ground-truth agents already filtered to this sensor's range/FOV (the
% caller, main.m, plays the role of "the camera capturing a frame"; this
% function plays the role of "the detector interpreting it"). This is the
% simulation-ground-truth-as-synthetic-detection approach docs/architecture.md
% and the project brief call for when no trained detector is available -
% it is not, and must never be presented as, real AI detection output.
%
% Camera's simulated strength is classification (kept exact); its simulated
% weakness is depth/position estimation (noisy) and it has no way to
% directly measure velocity from a single frame.
%
% Inputs:
%   image        - struct array of ground-truth agents in camera range/FOV
%   cameraParams - struct from config/sensorConfig.m (cfg.camera)
%   timestamp    - [s] current simulation time
% Output:
%   agents       - struct array of detected agents, agent.source = "camera"

agents = repmat(createAgent(), 0, 0);

for i = 1:numel(image)
    if rand() > cameraParams.detectionProbability
        continue; % simulated missed detection
    end

    gt = image(i);
    detected = gt;
    detected.position = gt.position + randn(1, 2) * cameraParams.positionNoiseStd;
    detected.velocity = [0, 0]; % not observable from a single camera frame
    % Confidence jitters around a baseline rather than decaying with range:
    % this function only receives already-range-filtered agents (main.m
    % does that filtering, since it has ego's position and this doesn't -
    % a real camera frame is already egocentric before any detector sees
    % it), so there is no ego-relative distance available here to decay against.
    detected.confidence = min(0.99, max(0.3, cameraParams.confidenceBase + randn() * 0.05));
    detected.source = "camera";
    detected.timestamp = timestamp;

    agents(end + 1) = detected; %#ok<AGROW>
end

end
