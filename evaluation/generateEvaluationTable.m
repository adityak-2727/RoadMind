function results = generateEvaluationTable()
% generateEvaluationTable - Phase 8 deliverable: reads the real, already-
% generated per-scenario logs (results/logs/<scenario>_summary.txt, the
% same file main.m/demo/runDemo.m already write) and assembles a
% consolidated cross-scenario evaluation table, saved to
% results/tables/evaluation_results.csv and
% results/tables/evaluation_summary.txt.
%
% This function does NOT run any simulation itself and does NOT touch any
% planning/prediction/decision/control file - it only parses text that
% main.m already wrote from real runs. Every field it extracts (goal
% reached, geometric collision, minimum clearance, path smoothness,
% replanning latency mean/count) is read verbatim from the existing
% summary.txt format (see main.m's summaryFile fprintf block) - no metric
% is invented, recomputed, or renamed; this only reformats what already
% exists into one consolidated table.
%
% Before calling this, generate/refresh the five logs with, e.g.:
%   runDemo("villageRoad", false); runDemo("urbanIntersection", false);
%   runDemo("highwayMerge", false); runDemo("marketArea", false);
%   runDemo("cattleCrossing", false);
%
% Output:
%   results - struct array, one element per scenario, with the same
%             fields written to the CSV (see below) - returned so a
%             caller can inspect what was written without re-parsing the
%             CSV.

SCENARIOS = ["villageRoad", "urbanIntersection", "highwayMerge", "marketArea", "cattleCrossing"];

logDir = fullfile(pwd, 'results', 'logs');
tableDir = fullfile(pwd, 'results', 'tables');
if ~exist(tableDir, 'dir')
    mkdir(tableDir);
end

results = struct('scenario', {}, 'completed', {}, 'collision', {}, 'replans', {}, ...
    'avgReplanLatency', {}, 'minClearance', {}, 'pathSmoothness', {});

for i = 1:numel(SCENARIOS)
    name = SCENARIOS(i);
    summaryPath = fullfile(logDir, sprintf('%s_summary.txt', name));
    if ~isfile(summaryPath)
        error('generateEvaluationTable:missingLog', ...
            ['No summary log found for scenario "%s" at %s. Run it first, e.g. ' ...
             'runDemo("%s", false), before generating the evaluation table.'], name, summaryPath, name);
    end

    text = fileread(summaryPath);
    results(i).scenario         = char(name);
    results(i).completed        = parseField(text, 'Goal reached:\s*(\d+)', name, summaryPath) ~= 0;
    results(i).collision        = parseField(text, 'Geometric collision occurred:\s*(\d+)', name, summaryPath);
    results(i).minClearance     = parseField(text, 'Minimum clearance to any ground-truth agent:\s*([\d.]+)\s*m', name, summaryPath);
    results(i).pathSmoothness   = parseField(text, 'Path smoothness \(sum \|delta curvature\|\):\s*([\d.]+)\s*rad', name, summaryPath);

    replanTokens = regexp(text, 'Replanning latency:\s*mean=([\d.]+)s\s*median=[\d.]+s\s*max=[\d.]+s\s*count=(\d+)', 'tokens', 'once');
    if isempty(replanTokens)
        error('generateEvaluationTable:parseError', ...
            'Could not parse the "Replanning latency" line for scenario "%s" in %s.', name, summaryPath);
    end
    results(i).avgReplanLatency = str2double(replanTokens{1});
    results(i).replans           = str2double(replanTokens{2});
end

%% Write results/tables/evaluation_results.csv
csvPath = fullfile(tableDir, 'evaluation_results.csv');
csvFile = fopen(csvPath, 'w');
fprintf(csvFile, 'Scenario,Completed,Collision,Replans,AverageReplanningLatency,MinimumClearance,PathSmoothness\n');
for i = 1:numel(results)
    r = results(i);
    if r.completed
        completedStr = 'true';
    else
        completedStr = 'false';
    end
    fprintf(csvFile, '%s,%s,%d,%d,%.2f,%.2f,%.2f\n', ...
        r.scenario, completedStr, r.collision, r.replans, r.avgReplanLatency, r.minClearance, r.pathSmoothness);
end
fclose(csvFile);

%% Compute completion rate / collision rate programmatically (not hardcoded)
numScenarios = numel(results);
numCompleted = sum([results.completed]);
numCollisions = sum([results.collision] ~= 0);
completionRate = 100 * numCompleted / numScenarios;
collisionRate = 100 * numCollisions / numScenarios;

clearances = [results.minClearance];
smoothness = [results.pathSmoothness];

%% Write results/tables/evaluation_summary.txt
summaryPath = fullfile(tableDir, 'evaluation_summary.txt');
summaryFile = fopen(summaryPath, 'w');
fprintf(summaryFile, 'Phase 8 Evaluation Summary\n');
fprintf(summaryFile, 'Generated at: %s\n', datestr(now)); %#ok<TNOW1,DATST>
fprintf(summaryFile, 'Source: results/logs/<scenario>_summary.txt (real simulation runs, parsed verbatim)\n\n');
fprintf(summaryFile, 'Total scenarios: %d\n', numScenarios);
fprintf(summaryFile, 'Successful scenarios (goal reached): %d\n', numCompleted);
fprintf(summaryFile, 'Completion rate: %d/%d = %.0f%%\n', numCompleted, numScenarios, completionRate);
fprintf(summaryFile, 'Scenarios with geometric collision: %d\n', numCollisions);
fprintf(summaryFile, 'Collision rate: %d/%d = %.0f%%\n', numCollisions, numScenarios, collisionRate);
fprintf(summaryFile, '\n');
fprintf(summaryFile, 'Minimum ground-truth clearance across scenarios: mean=%.2fm min=%.2fm max=%.2fm\n', ...
    mean(clearances), min(clearances), max(clearances));
fprintf(summaryFile, 'Path smoothness across scenarios: mean=%.2frad min=%.2frad max=%.2frad\n', ...
    mean(smoothness), min(smoothness), max(smoothness));
fprintf(summaryFile, '\n');
fprintf(summaryFile, ['REPLANNING LATENCY LIMITATION: this simulation replans synchronously every\n' ...
    'simulation tick (no artificial per-tick computation delay), so the\n' ...
    '"AverageReplanningLatency"/"Replans" values above should NOT be interpreted\n' ...
    'as real-world perception-to-response computational latency. They measure\n' ...
    'how long a trigger condition (TTC dropping below the warning threshold)\n' ...
    'stays active before the planner''s selected candidate or the decision state\n' ...
    'materially changes - a real, but different, quantity. Kept exactly as\n' ...
    'currently implemented in main.m; not redesigned in this task.\n']);
fprintf(summaryFile, '\n');
fprintf(summaryFile, ['OPTIONAL METRICS NOT IMPLEMENTED: travel efficiency (actual/ideal path\n' ...
    'length ratio) and comfort metrics beyond peak steering/accel/braking\n' ...
    '(steering rate statistics, jerk) are not currently computed anywhere in\n' ...
    'this project. Both are optional per the evaluation spec and were not\n' ...
    'implemented in this task.\n']);
fclose(summaryFile);

fprintf('Evaluation table written to %s\n', csvPath);
fprintf('Evaluation summary written to %s\n', summaryPath);

end

function value = parseField(text, pattern, scenarioName, filePath)
tokens = regexp(text, pattern, 'tokens', 'once');
if isempty(tokens)
    error('generateEvaluationTable:parseError', ...
        'Could not parse pattern "%s" for scenario "%s" in %s.', pattern, scenarioName, filePath);
end
value = str2double(tokens{1});
end
