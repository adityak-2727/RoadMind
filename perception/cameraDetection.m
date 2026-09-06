function agents = cameraDetection(image, cameraParams, timestamp)
% cameraDetection - stub: will run camera-based object detection and return
% a list of agent structs (see config/createAgent.m). Phase 0: no logic yet.
%
% Inputs:
%   image        - camera frame (format TBD; placeholder for real sensor data)
%   cameraParams - intrinsics/extrinsics/model config (TBD)
%   timestamp    - [s] current simulation time
% Output:
%   agents       - struct array of detected agents, agent.source = "camera"

agents = repmat(createAgent(), 0, 0);

end
