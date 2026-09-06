function agents = radarDetection(radarData, radarParams, timestamp)
% radarDetection - stub: will convert raw radar returns (range/doppler) into
% agent detections, primarily useful for velocity estimation. Phase 0: no logic yet.
%
% Inputs:
%   radarData   - raw radar returns (format TBD; placeholder)
%   radarParams - sensor config (TBD)
%   timestamp   - [s] current simulation time
% Output:
%   agents      - struct array of detected agents, agent.source = "radar"

agents = repmat(createAgent(), 0, 0);

end
