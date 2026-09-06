function trackedAgents = objectTracking(fusedAgents, trackedAgentsPrev, dt)
% objectTracking - stub: will assign persistent IDs and filter/predict agent
% state across frames (e.g. Kalman filter per track). Phase 0: no logic yet.
%
% Inputs:
%   fusedAgents       - struct array, this frame's fused detections
%   trackedAgentsPrev - struct array, previous frame's tracked agents (with ids)
%   dt                - [s] time since previous frame
% Output:
%   trackedAgents     - struct array with stable agent.id across frames

trackedAgents = fusedAgents; % placeholder passthrough, no id association yet

end
