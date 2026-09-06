% main.m - Phase 5 closed-loop demo for SIH 26037.
% Wires the full pipeline through the real loop: synthetic camera/lidar/
% radar detection + fusion + tracking (perception/) feeds prediction,
% decision (behavior state machine), local/adaptive planning, collision
% checking, control, and the vehicle model. The vehicle reacts to agents by
% steering around them (adaptivePlanner) and slowing/stopping for them
% (behaviorDecision + decisionStateMachine modulating vehicleController's
% target speed).
%
% There is no real camera/lidar/radar simulator here, so perception/*.m
% take ground-truth agents (already filtered to each sensor's range/FOV by
% filterAgentsForSensor below) as their synthetic "raw" input and degrade
% them to mimic that sensor's real characteristics - this is explicitly
% simulated detection, never real AI output; see perception/cameraDetection.m
% for the full rationale. rng(simCfg.randomSeed) makes every run
% reproducible despite the sensor noise/missed-detection randomness.
% See docs/architecture.md for the full pipeline signatures and diagram.

clear; clc;

addpath(genpath(pwd));

%% Load configs
simCfg = simulationConfig();
vehCfg = vehicleConfig();
planCfg = plannerConfig();
sensorCfg = sensorConfig();

rng(simCfg.randomSeed);

%% Select scenario
%scenario = villageRoad();
% scenario = urbanIntersection();
 scenario = highwayMerge();
% scenario = marketArea();
% scenario = cattleCrossing();

%% Initialize state
egoState = scenario.egoStart;
groundTruthAgents = scenario.agents; % world truth; perception/*.m only ever sees a degraded view of this
trackedAgentsPrev = repmat(createAgent(), 0, 0);
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

% De-escalation dwell time: calculateTTC depends on egoState.velocity, and
% the decision state sets that same velocity (0 for emergency_stop, half
% cruise for yield) - braking hard enough can itself make the next
% instant's TTC estimate look briefly safe, which without a minimum dwell
% time flips the state straight back and re-accelerates, dropping TTC
% again, oscillating every ~0.5-0.9s. Escalating to more caution stays
% immediate (safety-first); only relaxing to a less cautious state is
% rate-limited by this.
MIN_DWELL_TIME = 1.5; % [s]
lastTransitionTime = -Inf;
severityOrder = ["cruise", "yield", "emergency_stop"];

%% Main simulation loop
steps = simCfg.numSteps;
for t = 1:steps

    % --- 1. Perception ---
    cameraView = filterAgentsForSensor(groundTruthAgents, egoState, sensorCfg.camera.maxRange, sensorCfg.camera.fov);
    lidarView  = filterAgentsForSensor(groundTruthAgents, egoState, sensorCfg.lidar.maxRange, sensorCfg.lidar.fov);
    radarView  = filterAgentsForSensor(groundTruthAgents, egoState, sensorCfg.radar.maxRange, sensorCfg.radar.fov);

    cameraAgents = cameraDetection(cameraView, sensorCfg.camera, egoState.timestamp);
    lidarAgents  = lidarDetection(lidarView, sensorCfg.lidar, egoState.timestamp);
    radarAgents  = radarDetection(radarView, sensorCfg.radar, egoState.timestamp);
    fusedAgents  = sensorFusion(cameraAgents, lidarAgents, radarAgents);

    % --- 2. Tracking ---
    trackedAgents = objectTracking(fusedAgents, trackedAgentsPrev, simCfg.dt);
    trackedAgentsPrev = trackedAgents;

    % --- 3. Prediction ---
    predictedTrajectories = trajectoryPrediction(trackedAgents, planHorizon, simCfg.dt);

    % --- 4. Decision ---
    behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, planCfg, decisionState);
    proposedState = decisionStateMachine(decisionState, behaviorCommand, struct());
    isEscalation = find(severityOrder == proposedState, 1) > find(severityOrder == decisionState, 1);
    dwellSatisfied = (egoState.timestamp - lastTransitionTime) >= MIN_DWELL_TIME;
    if proposedState ~= decisionState && (isEscalation || dwellSatisfied)
        fprintf('  [t=%.1fs] decision state: %s -> %s\n', egoState.timestamp, decisionState, proposedState);
        decisionState = proposedState;
        lastTransitionTime = egoState.timestamp;
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

    % --- Ground-truth agent motion (perception above only observes a degraded view of this) ---
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

%% Evaluation (placeholder, Phase 6+)
% simLog = struct();
% metricsReport = evaluateScenario(simLog, scenario);
% disp(metricsReport);

function inRangeAgents = filterAgentsForSensor(agents, egoState, maxRange, fov)
% Selects the ground-truth agents within maxRange and within the fov cone
% centered on egoState.yaw - i.e. simulates "capturing" one sensor's raw
% frame from the ego's current pose, before perception/*.m degrades it.
inRangeAgents = repmat(createAgent(), 0, 0);
egoPos = [egoState.x, egoState.y];
for i = 1:numel(agents)
    relVec = agents(i).position - egoPos;
    dist = norm(relVec);
    if dist > maxRange
        continue;
    end
    bearing = atan2(relVec(2), relVec(1)) - egoState.yaw;
    bearing = atan2(sin(bearing), cos(bearing));
    if abs(bearing) <= fov / 2
        inRangeAgents(end + 1) = agents(i); %#ok<AGROW>
    end
end
end
