function metrics = main(scenarioNameOverride, realTimePlotOverride)
% main - full closed-loop demo for SIH 26037.
% Wires the full pipeline through the real loop: synthetic camera/lidar/
% radar detection + fusion + tracking (perception/) feeds prediction,
% decision (behavior state machine), the six-weight adaptive local planner,
% collision checking, control, and the vehicle model. The vehicle reacts to
% agents by steering around them (adaptivePlanner) and slowing/stopping for
% them (behaviorDecision + decisionStateMachine modulating
% vehicleController's target speed). Also instruments the project brief's
% "Replanning Trigger"/"Replanning Latency" concept: this loop replans every
% tick already, so latency here measures how long from a trigger condition
% first becoming active until the system produces a materially different
% plan - not zero by construction, since the decision dwell time and the
% planner's consistency cost both deliberately delay a visible response.
%
% There is no real camera/lidar/radar simulator here, so perception/*.m
% take ground-truth agents (already filtered to each sensor's range/FOV by
% filterAgentsForSensor below) as their synthetic "raw" input and degrade
% them to mimic that sensor's real characteristics - this is explicitly
% simulated detection, never real AI output; see perception/cameraDetection.m
% for the full rationale. rng(simCfg.randomSeed) makes every run
% reproducible despite the sensor noise/missed-detection randomness.
% See docs/architecture.md for the full pipeline signatures and diagram.
%
% Converted from a script to a function so a scenario can be selected
% programmatically (demo/runDemo.m) instead of only by hand-editing the
% scenario-selection block below; the defaults below reproduce exactly
% what this file used to hardcode, so a bare `main` (no arguments) behaves
% identically to every prior manual run. Nothing about the simulation,
% planning, prediction, decision, or control logic changed.
%
% Inputs (both optional):
%   scenarioNameOverride - one of "villageRoad", "urbanIntersection",
%                          "highwayMerge", "marketArea", "cattleCrossing".
%                          Defaults to "highwayMerge" (the prior hardcoded
%                          selection).
%   realTimePlotOverride - true/false. Defaults to true (the prior
%                          hardcoded simCfg.realTimePlot value).
% Output:
%   metrics - struct with fields: scenarioName, goalReached,
%             completionTime, geometricCollision, minClearance, minTTC,
%             fallbackCount, pathSmoothness - the same values already
%             printed/logged below, also returned so a caller (e.g.
%             demo/runDemo.m) doesn't have to re-parse the log files.

if nargin < 1 || isempty(scenarioNameOverride)
    scenarioNameOverride = "highwayMerge";
end
if nargin < 2 || isempty(realTimePlotOverride)
    realTimePlotOverride = true;
end

clc;

addpath(genpath(pwd));

%% Load configs
simCfg = simulationConfig();
vehCfg = vehicleConfig();
planCfg = plannerConfig();
sensorCfg = sensorConfig();

rng(simCfg.randomSeed);

%% Select scenario
scenario = selectScenarioByName(scenarioNameOverride);

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

% Decision-state speed multiplier on top of baseSpeed, one per state in
% decision/behaviorSeverity.m's set. "wait" and "emergency_stop" are full
% stops; "brake" still creeps rather than fully stopping, so the ego can
% clear a static hazard instead of holding against a condition that would
% never change while stopped.
% Kept monotonic with decision/behaviorSeverity.m's ranks - a more
% restrictive state must never drive faster than a less restrictive one,
% because decisionStateMachine relaxes one rank at a time and those ranks
% double as the vehicle's speed ramp back up after a hazard.
% These are fractions of the scenario's base cruise speed, so they stack on
% top of the per-scenario context factor above. Dropping "avoid" much below
% ~0.7 measurably backfires on highwayMerge: at 0.45 the ego crawls at
% 4.9 m/s beside a 3 m/s truck, turning a brisk overtake into a prolonged
% close-range crawl that ended 1.62m from it, versus 2.78m at 0.7.
decisionSpeedFactors = struct( ...
    'cruise',         1.0, ...
    'follow',         0.8, ...
    'merge',          0.8, ...
    'replan',         0.75, ...
    'avoid',          0.7, ...
    'brake',          0.15, ...
    'wait',           0.0, ...
    'emergency_stop', 0.0 ...
);

simCfg.numSteps = 500;
simCfg.realTimePlot = realTimePlotOverride;

% Slows the animation for manual review: each 0.1s simulation step gets an
% extra real-world pause on top of however long drawing the frame took, so
% a full run takes roughly numSteps * PLAYBACK_SLOWDOWN seconds to watch
% instead of finishing in a couple of seconds. Set to 0 to run at whatever
% speed MATLAB can draw (e.g. for headless/batch runs).
PLAYBACK_SLOWDOWN = 0.3; % [s] extra pause per frame

if simCfg.realTimePlot
    fig = figure('Name', 'Closed-loop demo');
    ax = axes(fig);
end

% Per-step + summary logs to results/logs/, gated by simCfg.logToFile, so
% every run's numbers can be reviewed after the fact, not just watched live.
if simCfg.logToFile
    logDir = fullfile(pwd, 'results', 'logs');
    if ~exist(logDir, 'dir')
        mkdir(logDir);
    end
    stepLogPath = fullfile(logDir, sprintf('%s_steps.csv', scenario.name));
    stepLogFile = fopen(stepLogPath, 'w');
    fprintf(stepLogFile, ['step,timestamp,ego_x,ego_y,ego_yaw,ego_velocity,decision_state,target_speed,' ...
        'num_tracked_agents,selected_candidate_index,steering_command_deg,throttle,brake,' ...
        'min_ttc,min_clearance,is_colliding\n']);
end

egoHistory = zeros(simCfg.numSteps, 2);
goalReached = false;
minTTCObserved = Inf;
minClearanceObserved = Inf; % measured against ground-truth agent positions, not predictions
anyUnavoidableCollision = false; % "no safe candidate" flag from adaptivePlanner's fallback, NOT a geometric collision
fallbackCount = 0; % per-tick count of the same "no safe candidate" event anyUnavoidableCollision flags
anyGeometricCollision = false;   % actual ego-to-ground-truth-agent distance below contact distance
CONTACT_DISTANCE = 1.1; % [m] matches collisionCheck.m's egoRadius; a real geometric collision, not a risk flag
speedTrackingErrors = zeros(simCfg.numSteps, 1);
maxAbsSteeringCmd = 0;
maxAccelCmd = 0;
maxBrakeCmd = 0;
stateTransitionCount = 0;

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
previousCandidateIndex = [];

% Replan-trigger/latency instrumentation (project brief Hour 15-20,
% "Replanning Trigger"/"Replanning Latency"): the planner already runs
% every tick (continuous replanning), so "latency" here measures something
% real - how long from a trigger condition first becoming active until the
% system actually produces a *materially different* plan (selected
% candidate index changes, or decision state escalates) - not zero by
% construction, since MIN_DWELL_TIME and the consistency cost in
% adaptivePlanner both deliberately delay a visible response for stability.
previousTrackedIds = [];
previousPathMinTTC = Inf;
pendingTriggerTime = [];
replanLatencies = [];

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
    fusedAgents  = sensorFusion(cameraAgents, lidarAgents, radarAgents, false, trackedAgentsPrev);

    % --- 2. Tracking ---
    trackedAgents = objectTracking(fusedAgents, trackedAgentsPrev, simCfg.dt);
    trackedAgentsPrev = trackedAgents;

    currentTrackedIds = [trackedAgents.id];
    % Neither onNewObstacle nor onTrackLost is used for latency measurement
    % below, despite both being computable from ID churn - IDs turned out
    % to churn almost every frame even with only 1-2 real agents nearby
    % (sensor position noise intermittently fails sensorFusion's/
    % objectTracking's nearest-neighbor gating for the *same* physical
    % object, minting a fresh ID). That's a real perception-layer accuracy
    % issue worth fixing separately, but it makes ID-based "is this a new
    % obstacle" detection useless as a trigger signal right now - it would
    % fire almost every step. TTC is a continuous numeric signal and isn't
    % affected by this, so it's the only trigger used below. Both flags
    % stay in plannerConfig.replanTriggers for when tracking is more stable.

    % --- 3. Prediction ---
    predictedTrajectories = trajectoryPrediction(trackedAgents, egoState, planHorizon, simCfg.dt);

    % --- 4. Decision ---
    behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, planCfg, decisionState);
    proposedState = decisionStateMachine(decisionState, behaviorCommand, struct());
    isEscalation = behaviorSeverity(proposedState) > behaviorSeverity(decisionState);
    dwellSatisfied = (egoState.timestamp - lastTransitionTime) >= MIN_DWELL_TIME;
    if proposedState ~= decisionState && (isEscalation || dwellSatisfied)
        fprintf('  [t=%.1fs] decision state: %s -> %s\n', egoState.timestamp, decisionState, proposedState);
        decisionState = proposedState;
        lastTransitionTime = egoState.timestamp;
        stateTransitionCount = stateTransitionCount + 1;
    end
    targetSpeed = baseSpeed * decisionSpeedFactors.(decisionState);

    % --- 5. Planning ---
    candidateTrajectories = localPlanner(egoState, globalPath, predictedTrajectories, planCfg);
    priorCandidateIndex = previousCandidateIndex;
    [selectedTrajectory, previousCandidateIndex] = adaptivePlanner(egoState, candidateTrajectories, predictedTrajectories, vehCfg, planCfg, scenario.name, priorCandidateIndex);
    [isColliding, minTTC] = collisionCheck(selectedTrajectory, predictedTrajectories, vehCfg);
    smoothPath = pathSmoothing(selectedTrajectory, struct());

    minTTCObserved = min(minTTCObserved, minTTC);
    if isColliding
        anyUnavoidableCollision = true;
        fallbackCount = fallbackCount + 1;
    end

    ttcTrigger = planCfg.replanTriggers.onTTCBelowWarning && ...
                 minTTC < planCfg.ttcThresholds.warning && previousPathMinTTC >= planCfg.ttcThresholds.warning;

    if ttcTrigger && isempty(pendingTriggerTime)
        pendingTriggerTime = egoState.timestamp;
    end

    candidateChanged = ~isequal(previousCandidateIndex, priorCandidateIndex);
    if ~isempty(pendingTriggerTime) && (candidateChanged || isEscalation)
        replanLatencies(end + 1) = egoState.timestamp - pendingTriggerTime; %#ok<AGROW>
        pendingTriggerTime = [];
    end

    previousTrackedIds = currentTrackedIds;
    previousPathMinTTC = minTTC;

    % --- 6. Control ---
    controlCommand = vehicleController(egoState, smoothPath, vehCfg, targetSpeed, simCfg.dt);

    speedTrackingErrors(t) = abs(egoState.velocity - targetSpeed);
    maxAbsSteeringCmd = max(maxAbsSteeringCmd, abs(controlCommand.steeringAngle));
    maxAccelCmd = max(maxAccelCmd, controlCommand.throttle * vehCfg.maxAccel);
    maxBrakeCmd = max(maxBrakeCmd, controlCommand.brake * vehCfg.maxBraking);

    % --- 7. Vehicle model (closes the loop back to perception at t+1) ---
    egoState = bicycleModel(egoState, controlCommand, vehCfg, simCfg.dt);

    egoHistory(t, :) = [egoState.x, egoState.y];

    % --- Ground-truth agent motion (perception above only observes a degraded view of this) ---
    stepMinClearance = Inf;
    for a = 1:numel(groundTruthAgents)
        groundTruthAgents(a).position = groundTruthAgents(a).position + groundTruthAgents(a).velocity * simCfg.dt;
        groundTruthAgents(a).timestamp = egoState.timestamp;
        d = norm([egoState.x, egoState.y] - groundTruthAgents(a).position);
        stepMinClearance = min(stepMinClearance, d);
        if d < CONTACT_DISTANCE
            anyGeometricCollision = true;
        end
    end
    minClearanceObserved = min(minClearanceObserved, stepMinClearance);

    % --- Logging ---
    if simCfg.logToFile
        fprintf(stepLogFile, '%d,%.2f,%.2f,%.2f,%.3f,%.2f,%s,%.2f,%d,%d,%.2f,%.3f,%.3f,%.2f,%.2f,%d\n', ...
            t, egoState.timestamp, egoState.x, egoState.y, egoState.yaw, egoState.velocity, ...
            decisionState, targetSpeed, numel(trackedAgents), previousCandidateIndex, ...
            rad2deg(controlCommand.steeringAngle), controlCommand.throttle, controlCommand.brake, ...
            minTTC, stepMinClearance, isColliding);
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
        title(ax, sprintf('%s | t=%.1fs | speed=%.1f m/s (target %.1f) | state=%s | minTTC=%.1fs | tracked=%d', ...
            scenario.name, egoState.timestamp, egoState.velocity, targetSpeed, decisionState, minTTC, numel(trackedAgents)), ...
            'Interpreter', 'none');
        hold(ax, 'off');
        drawnow limitrate;
        if PLAYBACK_SLOWDOWN > 0
            pause(PLAYBACK_SLOWDOWN);
        end
    end

    if norm([egoState.x, egoState.y] - scenario.egoGoal) < goalTolerance
        egoHistory = egoHistory(1:t, :);
        speedTrackingErrors = speedTrackingErrors(1:t);
        goalReached = true;
        fprintf('Reached goal at t = %.1f s (step %d)\n', egoState.timestamp, t);
        break;
    end

end

if ~goalReached
    fprintf('DID NOT reach goal within %d steps. Final position: (%.2f, %.2f), goal: (%.2f, %.2f)\n', ...
        simCfg.numSteps, egoState.x, egoState.y, scenario.egoGoal(1), scenario.egoGoal(2));
end

fprintf('Minimum TTC observed across the run: %.2f s\n', minTTCObserved);
fprintf('Minimum clearance to any ground-truth agent: %.2f m (contact distance %.2f m)\n', minClearanceObserved, CONTACT_DISTANCE);
if anyGeometricCollision
    fprintf('WARNING: a geometric collision occurred (clearance dropped below %.2fm).\n', CONTACT_DISTANCE);
end
if anyUnavoidableCollision
    fprintf('NOTE: at least one step had no safe candidate trajectory (adaptivePlanner fallback used) - this is a risk flag, not necessarily a geometric collision; see minimum clearance above.\n');
end

pathSmoothness = calculatePathSmoothness(egoHistory);
fprintf('Path smoothness (sum |delta curvature| over the driven path): %.2f rad\n', pathSmoothness);

latencyStats = calculateReplanningLatency(replanLatencies);
fprintf('Replanning latency: mean=%.2fs median=%.2fs max=%.2fs count=%d\n', ...
    latencyStats.meanLatency, latencyStats.medianLatency, latencyStats.maxLatency, latencyStats.count);

fprintf('State transitions: %d\n', stateTransitionCount);
fprintf('Fallback count (no-safe-candidate ticks): %d\n', fallbackCount);
fprintf('Speed tracking error: mean=%.2f m/s max=%.2f m/s\n', mean(speedTrackingErrors), max(speedTrackingErrors));
fprintf('Max steering command: %.1f deg (limit %.1f deg)\n', rad2deg(maxAbsSteeringCmd), rad2deg(vehCfg.maxSteerAngle));
fprintf('Max acceleration command: %.2f m/s^2 (limit %.2f)\n', maxAccelCmd, vehCfg.maxAccel);
fprintf('Max braking command: %.2f m/s^2 (limit %.2f)\n', maxBrakeCmd, vehCfg.maxBraking);

if simCfg.logToFile
    fclose(stepLogFile);
    summaryPath = fullfile(logDir, sprintf('%s_summary.txt', scenario.name));
    summaryFile = fopen(summaryPath, 'w');
    fprintf(summaryFile, 'Scenario: %s\n', scenario.name);
    fprintf(summaryFile, 'Run at: %s\n', datestr(now)); %#ok<TNOW1,DATST>
    fprintf(summaryFile, 'Goal reached: %d\n', goalReached);
    fprintf(summaryFile, 'Final ego state: x=%.2f y=%.2f timestamp=%.1fs\n', egoState.x, egoState.y, egoState.timestamp);
    fprintf(summaryFile, 'Goal: [%.2f, %.2f], tolerance=%.1fm\n', scenario.egoGoal(1), scenario.egoGoal(2), goalTolerance);
    fprintf(summaryFile, 'Minimum TTC observed: %.2f s\n', minTTCObserved);
    fprintf(summaryFile, 'Minimum clearance to any ground-truth agent: %.2f m (contact distance %.2f m)\n', minClearanceObserved, CONTACT_DISTANCE);
    fprintf(summaryFile, 'Geometric collision occurred: %d\n', anyGeometricCollision);
    fprintf(summaryFile, 'No-safe-candidate fallback used at least once (risk flag, not necessarily collision): %d\n', anyUnavoidableCollision);
    fprintf(summaryFile, 'Fallback count (no-safe-candidate ticks): %d\n', fallbackCount);
    fprintf(summaryFile, 'Path smoothness (sum |delta curvature|): %.2f rad\n', pathSmoothness);
    fprintf(summaryFile, 'Replanning latency: mean=%.2fs median=%.2fs max=%.2fs count=%d\n', ...
        latencyStats.meanLatency, latencyStats.medianLatency, latencyStats.maxLatency, latencyStats.count);
    fprintf(summaryFile, 'State transitions: %d\n', stateTransitionCount);
    fprintf(summaryFile, 'Speed tracking error: mean=%.2f m/s max=%.2f m/s\n', mean(speedTrackingErrors), max(speedTrackingErrors));
    fprintf(summaryFile, 'Max steering command: %.1f deg (limit %.1f deg)\n', rad2deg(maxAbsSteeringCmd), rad2deg(vehCfg.maxSteerAngle));
    fprintf(summaryFile, 'Max acceleration command: %.2f m/s^2 (limit %.2f)\n', maxAccelCmd, vehCfg.maxAccel);
    fprintf(summaryFile, 'Max braking command: %.2f m/s^2 (limit %.2f)\n', maxBrakeCmd, vehCfg.maxBraking);
    fprintf(summaryFile, 'Per-step data: %s\n', stepLogPath);
    fclose(summaryFile);
    fprintf('Logs written to %s and %s\n', stepLogPath, summaryPath);
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

metrics = struct( ...
    'scenarioName',       char(scenario.name), ...
    'goalReached',        goalReached, ...
    'completionTime',     egoState.timestamp, ...
    'geometricCollision', anyGeometricCollision, ...
    'minClearance',       minClearanceObserved, ...
    'minTTC',             minTTCObserved, ...
    'fallbackCount',      fallbackCount, ...
    'pathSmoothness',     pathSmoothness ...
);

end

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

function scenario = selectScenarioByName(name)
% selectScenarioByName - maps a scenario name string to the corresponding
% existing scenarios/*.m function call. Defines no scenario content itself
% - purely a name -> function dispatch so a scenario can be selected
% programmatically (this file's scenarioNameOverride, or demo/runDemo.m)
% instead of only by hand-editing a comment block. Rejects an unrecognized
% name with a clear error rather than silently defaulting to any one
% scenario.
name = string(name);
switch name
    case "villageRoad"
        scenario = villageRoad();
    case "urbanIntersection"
        scenario = urbanIntersection();
    case "highwayMerge"
        scenario = highwayMerge();
    case "marketArea"
        scenario = marketArea();
    case "cattleCrossing"
        scenario = cattleCrossing();
    otherwise
        error('main:invalidScenario', ...
            '"%s" is not a supported scenario. Supported scenarios: villageRoad, urbanIntersection, highwayMerge, marketArea, cattleCrossing.', name);
end
end
