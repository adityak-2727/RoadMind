function metrics = runDemo(scenarioName, realTimePlot)
% runDemo - clean entry point for demonstrating the closed-loop system on
% one named scenario, without hand-editing main.m's scenario-selection
% block. Thin wrapper only: every simulation, perception, prediction,
% decision, planning (including the K1 predictable-unknown prediction and
% K2 feasibility-aware fallback), collision-checking, and control step
% lives in main.m, unchanged - this validates the scenario name, forwards
% to main.m, and prints a concise demo summary from the metrics main.m
% already computes and returns.
%
% Usage:
%   runDemo("highwayMerge")
%   runDemo("highwayMerge", true)   % explicit real-time plotting (default)
%   runDemo("highwayMerge", false)  % headless run, no plotting
%
% Inputs:
%   scenarioName - required. One of: "villageRoad", "urbanIntersection",
%                  "highwayMerge", "marketArea", "cattleCrossing". Any
%                  other value raises a clear error listing the supported
%                  names - never silently falls back to a default scenario.
%   realTimePlot  - optional, default true (matches main.m's own default,
%                  which matches this project's prior hardcoded behavior).
% Output:
%   metrics - the struct main.m returns: scenarioName, goalReached,
%             completionTime, geometricCollision, minClearance, minTTC,
%             fallbackCount, pathSmoothness.

SUPPORTED_SCENARIOS = ["villageRoad", "urbanIntersection", "highwayMerge", "marketArea", "cattleCrossing"];

if nargin < 1 || isempty(scenarioName)
    error('runDemo:missingScenario', ...
        'runDemo requires a scenario name. Supported scenarios: %s.', strjoin(SUPPORTED_SCENARIOS, ', '));
end

scenarioName = string(scenarioName);
if ~any(scenarioName == SUPPORTED_SCENARIOS)
    error('runDemo:invalidScenario', ...
        '"%s" is not a supported scenario. Supported scenarios: %s.', scenarioName, strjoin(SUPPORTED_SCENARIOS, ', '));
end

if nargin < 2 || isempty(realTimePlot)
    realTimePlot = true;
end

metrics = main(scenarioName, realTimePlot);

fprintf('\n=== Demo summary: %s ===\n', metrics.scenarioName);
if metrics.goalReached
    goalStr = 'YES';
else
    goalStr = 'NO';
end
fprintf('Goal reached:                    %s\n', goalStr);
fprintf('Completion time:                 %.1f s\n', metrics.completionTime);
fprintf('Geometric collision:             %d\n', metrics.geometricCollision);
fprintf('Minimum ground-truth clearance:  %.2f m\n', metrics.minClearance);
fprintf('Minimum planner TTC:             %.2f s\n', metrics.minTTC);
fprintf('Fallback count:                  %d\n', metrics.fallbackCount);
fprintf('Path smoothness:                 %.2f rad\n', metrics.pathSmoothness);
fprintf(['(planner TTC is the planner''s own predicted-risk assessment, not proof of a collision; ' ...
    'fallback count is a risk-flag tick count, not a collision count; both are separate from the ' ...
    'geometric-collision flag above, which is measured against actual ground-truth agent positions.)\n']);

end
