function latencyStats = calculateReplanningLatency(replanLatencies)
% calculateReplanningLatency - computes mean/median/max/count over a set of
% already-measured replanning latencies, one value per replan event:
% latency = (timestamp the system actually produced a materially different
% plan) - (timestamp a replan trigger first became active). main.m measures
% each event's latency as it happens (rising edge of a
% plannerConfig.replanTriggers condition to the next step where
% adaptivePlanner's selected candidate index changes or the decision state
% escalates) and passes the collected list here.
%
% Renamed from the original Phase 0 draft's `replanTimestamps` - the
% project brief's own diagram (new_path_time - obstacle_detection_time)
% needs two timestamps per event to produce one latency value, so a single
% list of raw timestamps could not have been what this function consumes;
% it must already be per-event latency values.
%
% Input:
%   replanLatencies - vector of [s] latency values, one per replan event
% Output:
%   latencyStats - struct with fields: meanLatency, medianLatency, maxLatency, count

if isempty(replanLatencies)
    latencyStats = struct( ...
        'meanLatency',   0, ...
        'medianLatency', 0, ...
        'maxLatency',    0, ...
        'count',         0 ...
    );
    return;
end

latencyStats = struct( ...
    'meanLatency',   mean(replanLatencies), ...
    'medianLatency', median(replanLatencies), ...
    'maxLatency',    max(replanLatencies), ...
    'count',         numel(replanLatencies) ...
);

end
