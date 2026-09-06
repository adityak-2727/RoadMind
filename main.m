% main.m - Phase 4 closed-loop demo for SIH 26037.
% Wires prediction + decision (behavior state machine) + local/adaptive
% planning + collision checking + control + vehicle model through the real
% loop: the ego vehicle now reacts to agents by both steering around them
% (adaptivePlanner) and slowing/stopping for them (behaviorDecision +
% decisionStateMachine modulating vehicleController's target speed) instead
% of only ever cruising at one fixed speed. Camera/LiDAR/radar perception
% is still a stub - scenario.agents are used directly as ground-truth
% "tracked" agents in the meantime, matching the ground-truth-vs-perception
% distinction docs/architecture.md calls out.
% See docs/architecture.md for the full pipeline signatures and diagram.

clear; clc;

addpath(genpath(pwd));

%% Load configs
simCfg = simulationConfig();
vehCfg = vehicleConfig();
planCfg = plannerConfig();

%% Select scenario
scenario = villageRoad();
% scenario = urbanIntersection();
% scenario = highwayMerge();
% scenario = marketArea();
% scenario = cattleCrossing();

%% Initialize state
egoState = scenario.egoStart;
groundTruthAgents = scenario.agents; % perception stand-in (real sensing is Phase 5+)
decisionState = "cruise";
goalTolerance = 2.0; % [m]

planHorizon = 4.0; % [s] must match localPlanner's PLANNING_HORIZON/PLANNING_DT convention

startPose = [egoState.x, egoState.y, egoState.yaw];
goalPose = [scenario.egoGoal(1), scenario.egoGoal(2), 0];
globalPath = globalPlanner(startPose, goalPose, scenario.map);

% Context-appropriate base cruise speed as a fraction of vehCfg.maxSpeed -
% e.g. a dense market corridor should never cruise as fast as a highway,
% independent of any moment-to-moment risk the decision layer reacts to.
scenarioSpeedFactors = struct( ...
    'villageRoad',       0.5, ...
    'urbanIntersection', 0.5, ...
    'highwayMerge',      0.65, ...
    'marketArea',        0.3, ...
    'cattleCrossing',    0.5 ...
);
if isfield(scenarioSpeedFactors, scenario.name)
    baseSpeed = scenarioSpeedFactors.(scenario.name) * vehCfg.maxSpeed;
else
    baseSpeed = 0.5 * vehCfg.maxSpeed;
end

% Decision-state speed multiplier on top of baseSpeed.
decisionSpeedFactors = struct('cruise', 1.0, 'yield', 0.5, 'emergency_stop', 0.0);

simCfg.numSteps = 500;
simCfg.realTimePlot = true;

if simCfg.realTimePlot
    fig = figure('Name', 'Phase 4 closed-loop demo');
    ax = axes(fig);
end

egoHistory = zeros(simCfg.numSteps, 2);
minTTCObserved = Inf;
anyUnavoidableCollision = false;

%% Main simulation loop
steps = simCfg.numSteps;
for t = 1:steps

    % --- 1. Perception ---   (Phase 5+: stub; groundTruthAgents stands in)
    % --- 2. Tracking ---     (Phase 5+: stub)
    trackedAgents = groundTruthAgents;

    % --- 3. Prediction ---
    predictedTrajectories = trajectoryPrediction(trackedAgents, planHorizon, simCfg.dt);

    % --- 4. Decision ---
    behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, planCfg, decisionState);
    prevDecisionState = decisionState;
    decisionState = decisionStateMachine(decisionState, behaviorCommand, struct());
    if decisionState ~= prevDecisionState
        fprintf('  [t=%.1fs] decision state: %s -> %s\n', egoState.timestamp, prevDecisionState, decisionState);
    end
    targetSpeed = baseSpeed * decisionSpeedFactors.(decisionState);

    % --- 5. Planning ---
    candidateTrajectories = localPlanner(egoState, globalPath, predictedTrajectories, planCfg);
    selectedTrajectory = adaptivePlanner(egoState, candidateTrajectories, predictedTrajectories, vehCfg, planCfg, scenario.name);
    [isColliding, minTTC] = collisionCheck(selectedTrajectory, predictedTrajectories, vehCfg);
    smoothPath = pathSmoothing(selectedTrajectory, struct());

    minTTCObserved = min(minTTCObserved, minTTC);
    if isColliding
        anyUnavoidableCollision = true;
    end

    % --- 6. Control ---
    controlCommand = vehicleController(egoState, smoothPath, vehCfg, targetSpeed);

    % --- 7. Vehicle model (closes the loop back to perception at t+1) ---
    egoState = bicycleModel(egoState, controlCommand, vehCfg, simCfg.dt);

    egoHistory(t, :) = [egoState.x, egoState.y];

    % --- Ground-truth agent motion (Phase 4+ perception would observe this) ---
    for a = 1:numel(groundTruthAgents)
        groundTruthAgents(a).position = groundTruthAgents(a).position + groundTruthAgents(a).velocity * simCfg.dt;
        groundTruthAgents(a).timestamp = egoState.timestamp;
    end

    % --- 8. Visualization ---
    if simCfg.realTimePlot
        cla(ax);
        hold(ax, 'on');
        plotPredictedTrajectories(predictedTrajectories, ax);
        plotDetectedObjects(trackedAgents, egoState, ax);
        plotPlannedPath(smoothPath, egoState, ax);
        axis(ax, 'equal');
        grid(ax, 'on');
        xlabel(ax, 'x [m]');
        ylabel(ax, 'y [m]');
        hold(ax, 'off');
        drawnow limitrate;
    end

    if norm([egoState.x, egoState.y] - scenario.egoGoal) < goalTolerance
        egoHistory = egoHistory(1:t, :);
        fprintf('Reached goal at t = %.1f s (step %d)\n', egoState.timestamp, t);
        break;
    end

end

fprintf('Minimum TTC observed across the run: %.2f s\n', minTTCObserved);
if anyUnavoidableCollision
    fprintf('WARNING: at least one step had no safe candidate trajectory (emergency fallback used).\n');
end

if simCfg.realTimePlot
    hold(ax, 'on');
    plot(ax, egoHistory(:, 1), egoHistory(:, 2), 'g-', 'LineWidth', 1.5);
    hold(ax, 'off');
end

%% Evaluation (placeholder, Phase 5+)
% simLog = struct();
% metricsReport = evaluateScenario(simLog, scenario);
% disp(metricsReport);
