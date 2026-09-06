function metricsReport = evaluateScenario(simLog, scenarioConfig)
% evaluateScenario - stub: will run all evaluation metrics (TTC, path
% smoothness, replanning latency, completion rate) over one scenario's log
% and assemble a single report struct. Phase 0: no logic yet.
%
% Inputs:
%   simLog         - struct/table of logged simulation data
%   scenarioConfig - scenario struct (from scenarios/*.m)
% Output:
%   metricsReport - struct aggregating all evaluation metrics (placeholders)

metricsReport = struct( ...
    'scenarioName',       "", ...
    'minTTC',             Inf, ...
    'pathSmoothness',     0, ...
    'replanningLatency',  struct('meanLatency', 0, 'maxLatency', 0, 'count', 0), ...
    'completionRate',     0 ...
);

end
