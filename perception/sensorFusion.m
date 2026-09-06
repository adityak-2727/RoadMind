function fusedAgents = sensorFusion(cameraAgents, lidarAgents, radarAgents)
% sensorFusion - stub: will associate and merge detections from all sensors
% into a single fused agent list (e.g. via IoU/Mahalanobis association +
% weighted averaging). Phase 0: no logic yet.
%
% Inputs:
%   cameraAgents, lidarAgents, radarAgents - struct arrays from each modality
% Output:
%   fusedAgents - struct array of merged agents, agent.source = "fused"

fusedAgents = [cameraAgents, lidarAgents, radarAgents]; % placeholder passthrough, no real association yet

end
