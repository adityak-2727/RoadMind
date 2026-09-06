function latencyStats = calculateReplanningLatency(replanTimestamps)
% calculateReplanningLatency - stub: will compute latency statistics (mean,
% max, count) between successive replanning events during a run. Phase 0: no logic yet.
%
% Input:
%   replanTimestamps - vector of [s] timestamps at which replanning occurred
% Output:
%   latencyStats - struct with fields: meanLatency, maxLatency, count (placeholder zeros)

latencyStats = struct( ...
    'meanLatency', 0, ...
    'maxLatency',  0, ...
    'count',       0 ...
);

end
