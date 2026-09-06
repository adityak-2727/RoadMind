% main.m - Phase 1 closed-loop demo for SIH 26037.
% Wires vehicle model + control + a fixed reference path (villageRoad
% centerline) through the real loop, proving the Phase 0 scaffolding holds
% together end-to-end in MATLAB. Perception, prediction, decision, and the
% adaptive/cost-based planner are still stubs (Phase 2+); this loop drives
% the ego vehicle along a hand-fed reference path only.
% See docs/architecture.md for the full pipeline signatures and diagram.

clear; clc;

addpath(genpath(pwd));

%% Load configs
simCfg = simulationConfig();
vehCfg = vehicleConfig();
planCfg = plannerConfig(); %#ok<NASGU> % not consumed until the adaptive planner lands (Phase 2+)

%% Select scenario
scenario = villageRoad();
% scenario = urbanIntersection();
% scenario = highwayMerge();
% scenario = marketArea();
% scenario = cattleCrossing();

%% Initialize state
egoState = scenario.egoStart;
referencePath = scenario.map.centerline; % fixed reference path (Phase 1: no global/local planner yet)
goalTolerance = 2.0; % [m]

simCfg.numSteps = 300;
simCfg.realTimePlot = true;

if simCfg.realTimePlot
    fig = figure('Name', 'Phase 1 closed-loop demo');
    ax = axes(fig);
end

egoHistory = zeros(simCfg.numSteps, 2);

%% Main simulation loop
steps = simCfg.numSteps;
for t = 1:steps

    % --- 1. Perception ---   (Phase 2+: stub)
    % --- 2. Tracking ---     (Phase 2+: stub)
    % --- 3. Prediction ---   (Phase 2+: stub)
    % --- 4. Decision ---     (Phase 2+: stub)
    % --- 5. Planning ---     (Phase 2+: stub; referencePath stands in for smoothPath)

    % --- 6. Control ---
    controlCommand = vehicleController(egoState, referencePath, vehCfg);

    % --- 7. Vehicle model (closes the loop back to perception at t+1) ---
    egoState = bicycleModel(egoState, controlCommand, vehCfg, simCfg.dt);

    egoHistory(t, :) = [egoState.x, egoState.y];

    % --- 8. Visualization ---
    if simCfg.realTimePlot
        plotPlannedPath(referencePath, egoState, ax);
        drawnow limitrate;
    end

    if norm([egoState.x, egoState.y] - scenario.egoGoal) < goalTolerance
        egoHistory = egoHistory(1:t, :);
        fprintf('Reached goal at t = %.1f s (step %d)\n', egoState.timestamp, t);
        break;
    end

end

if simCfg.realTimePlot
    plotPlannedPath(referencePath, egoState, ax);
    hold(ax, 'on');
    plot(ax, egoHistory(:, 1), egoHistory(:, 2), 'g-', 'LineWidth', 1.5);
    hold(ax, 'off');
end

%% Evaluation (placeholder, Phase 2+)
% simLog = struct();
% metricsReport = evaluateScenario(simLog, scenario);
% disp(metricsReport);
