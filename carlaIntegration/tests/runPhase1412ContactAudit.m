function out = runPhase1412ContactAudit(runLabel, maxTicks)
% runPhase1412ContactAudit - Phase 14.12 baseline forensic harness.
% Runs the unmodified hero-scene traffic behavior while recording bounded,
% per-tick actor diagnostics and incremental CARLA collision events. It does
% not enable contact recovery or otherwise alter control/traffic behavior.

if nargin < 1 || isempty(runLabel)
    runLabel = "baseline";
end
if nargin < 2 || isempty(maxTicks)
    maxTicks = 1000;
end

sceneCfg = carlaIndianSceneConfig();
carlaCfg = carlaConfig();
currentPyEnv = pyenv;
if currentPyEnv.Status == "NotLoaded"
    pyenv('Version', carlaCfg.pythonExecutable);
end
carlaConnect(carlaCfg);
cleanupObj = onCleanup(@() carlaDisconnect()); %#ok<NASGU>
carlaLoadMap(sceneCfg.mapName);
sceneState = carlaBuildIndianHeroScene(sceneCfg);
carlaAttachCamera(carlaCfg.camera);
carlaAttachLidar(carlaCfg.lidar);
carlaAttachRadar(carlaCfg.radar);
carlaAttachCollisionSensor();
pause(2.0);

ego0 = carlaGetEgoState();
[globalPath, ~] = carlaGenerateIntersectionTurnPath( ...
    [ego0.x, ego0.y], ego0.yaw, deg2rad(89.64), 12.0, 45.0, 25.0);
loopState = carlaClosedLoopInit(60, 4.0, 25, globalPath);
trafficState = [];
lastCollisionIndex = 0;
pendingCollisionActorIds = [];

telemetry = repmat(emptyTelemetryRow(), 1, 0);
trafficRows = repmat(emptyTrafficRow(), 1, 0);
collisionRows = repmat(emptyCollisionRow(), 1, 0);
goalReached = false;

for tick = 1:maxTicks
    egoBefore = carlaGetEgoState();
    trafficState = carlaIndianSceneTrafficStep( ...
        sceneState, sceneCfg, trafficState, [egoBefore.x, egoBefore.y], pendingCollisionActorIds);

    for i = 1:numel(trafficState.diagnostics)
        d = trafficState.diagnostics(i);
        trafficRows(end + 1) = struct( ...
            'tick', tick, 'actorId', d.actorId, 'blueprint', d.blueprint, ...
            'intent', d.intent, 'distanceToEgo', d.distanceToEgo, ...
            'originalTargetVx', d.originalTargetVelocity(1), ...
            'originalTargetVy', d.originalTargetVelocity(2), ...
            'appliedTargetVx', d.appliedTargetVelocity(1), ...
            'appliedTargetVy', d.appliedTargetVelocity(2), ...
            'actualVx', d.actualVelocity(1), 'actualVy', d.actualVelocity(2), ...
            'clampActive', d.clampActive, 'newCollisionEvent', d.newCollisionEvent, ...
            'recoveryState', d.recoveryState, 'recoveryTicks', d.recoveryTicks, ...
            'recoveryActivationCount', d.recoveryActivationCount, ...
            'recoveryTimeoutCount', d.recoveryTimeoutCount); %#ok<AGROW>
    end

    [loopState, report] = carlaClosedLoopStep(loopState);
    if report.skipped
        continue;
    end

    collisionCount = carlaGetCollisionCount();
    pendingCollisionActorIds = [];
    if collisionCount > lastCollisionIndex
        newEvents = carlaGetNewCollisionEvents(lastCollisionIndex);
        pendingCollisionActorIds = [newEvents.otherActorId];
        for i = 1:numel(newEvents)
            e = newEvents(i);
            collisionRows(end + 1) = struct( ...
                'tick', tick, 'timestamp', e.timestamp, 'actorId', e.otherActorId, ...
                'actorType', e.otherActorType, 'impulseMagnitude', e.impulseMagnitude, ...
                'egoX', e.egoX, 'egoY', e.egoY, 'egoYawDeg', e.egoYawDeg); %#ok<AGROW>
        end
        lastCollisionIndex = collisionCount;
    end

    telemetry(end + 1) = struct( ...
        'tick', tick, 'egoSpeed', report.egoState.velocity, ...
        'throttle', report.controlCommand.throttle, 'brake', report.controlCommand.brake, ...
        'steerDeg', rad2deg(report.controlCommand.steeringAngle), ...
        'decisionState', string(report.decisionState), ...
        'selectedCandidate', report.selectedCandidateIndex, ...
        'goalDistance', report.goalDistance, 'headingErrorDeg', headingErrorDeg(report), ...
        'collisionCount', collisionCount, 'newCollisionCount', numel(pendingCollisionActorIds)); %#ok<AGROW>

    if report.goalDistance < 3.0
        carlaApplyControl(0, 0, 1);
        goalReached = true;
        break;
    end
end

out = struct('runLabel', string(runLabel), 'goalReached', goalReached, ...
    'telemetry', telemetry, 'traffic', trafficRows, 'collisions', collisionRows, ...
    'totalCollisionCount', lastCollisionIndex);
writeAuditFiles(out);
end

function row = emptyTelemetryRow()
row = struct('tick', 0, 'egoSpeed', 0, 'throttle', 0, 'brake', 0, ...
    'steerDeg', 0, 'decisionState', "", 'selectedCandidate', 0, ...
    'goalDistance', 0, 'headingErrorDeg', 0, 'collisionCount', 0, ...
    'newCollisionCount', 0);
end

function row = emptyTrafficRow()
row = struct('tick', 0, 'actorId', 0, 'blueprint', "", 'intent', "", ...
    'distanceToEgo', NaN, 'originalTargetVx', NaN, 'originalTargetVy', NaN, ...
    'appliedTargetVx', NaN, 'appliedTargetVy', NaN, 'actualVx', NaN, ...
    'actualVy', NaN, 'clampActive', false, 'newCollisionEvent', false, ...
    'recoveryState', "normal", 'recoveryTicks', 0, ...
    'recoveryActivationCount', 0, 'recoveryTimeoutCount', 0);
end

function row = emptyCollisionRow()
row = struct('tick', 0, 'timestamp', 0, 'actorId', 0, 'actorType', "", ...
    'impulseMagnitude', 0, 'egoX', 0, 'egoY', 0, 'egoYawDeg', 0);
end

function value = headingErrorDeg(report)
path = report.smoothPath;
if isempty(path) || size(path, 1) < 2
    value = NaN;
    return;
end
[~, index] = min(vecnorm(path - [report.egoState.x, report.egoState.y], 2, 2));
index = min(index, size(path, 1) - 1);
pathYaw = atan2(path(index + 1, 2) - path(index, 2), path(index + 1, 1) - path(index, 1));
value = rad2deg(atan2(sin(pathYaw - report.egoState.yaw), cos(pathYaw - report.egoState.yaw)));
end

function writeAuditFiles(out)
resultDir = fullfile('results', 'phase14_12');
if ~isfolder(resultDir)
    mkdir(resultDir);
end
tag = char(out.runLabel);
writetable(struct2table(out.telemetry), fullfile(resultDir, "phase1412_" + tag + "_telemetry.csv"));
writetable(struct2table(out.traffic), fullfile(resultDir, "phase1412_" + tag + "_traffic.csv"));
writetable(struct2table(out.collisions), fullfile(resultDir, "phase1412_" + tag + "_collisions.csv"));
end
