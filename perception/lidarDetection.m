function agents = lidarDetection(pointCloud, lidarParams, timestamp)
% lidarDetection - stub: will cluster/segment a lidar point cloud into agent
% detections. Phase 0: no logic yet.
%
% Inputs:
%   pointCloud - lidar point cloud (format TBD; placeholder)
%   lidarParams - sensor mounting/range/resolution config (TBD)
%   timestamp   - [s] current simulation time
% Output:
%   agents      - struct array of detected agents, agent.source = "lidar"

agents = repmat(createAgent(), 0, 0);

end
