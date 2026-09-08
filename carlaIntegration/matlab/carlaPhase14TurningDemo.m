function results = carlaPhase14TurningDemo(savePngDir)
% carlaPhase14TurningDemo - Phase 14: demonstrates the complete, real,
% unmodified pipeline PHYSICALLY driving the CARLA ego vehicle through
% APPROACH -> OBSERVE -> DECIDE -> PLAN -> TURN -> EXIT, via
% carlaClosedLoopStep.m (Phase 12/13, extended for Phase 14 with a
% coordinate-frame runtime assertion and a NaN/Inf control-command
% failsafe - see its own header) driving carlaApplyControl every tick -
% never CARLA autopilot, never a scripted/prerecorded ego trajectory,
% never teleportation.
%
% ApproachLengthM=45.0 (not Phase 13 Demo H's 5.0) is the LIVE-MEASURED
% distance (via CARLA's own waypoint API, walking wp.next() from the ego
% spawn point until reaching junction id=103) from the hero scene's ego
% staging point to the real junction entry - see docs/carla_integration.md's
% Phase 14 section for the measurement. Using the shorter Phase 13 value
% here would make the vehicle arc through the approach road, not the
% actual visual intersection.
%
% A real CARLA collision sensor (carlaAttachCollisionSensor.m /
% carlaGetCollisionEvents.m, new this phase) is attached in every demo,
% because collisionCheck.m's own predicted/geometric TTC flag is NOT
% sufficient on its own to claim the real vehicle never made physical
% contact - see the "Real collision ground-truth" note in each demo's
% printed summary.
%
% Demos:
%   A - clear-ish intersection approach (no staged actor; the hero
%       scene's own static roadside clutter/parked vehicles still
%       legitimately trigger frequent avoid/brake reactions in this
%       dense, unstructured scene - reported honestly, not smoothed over)
%   B - crossing vehicle (staged, guaranteed conflict)
%   C - pedestrian crossing conflict (staged, VRU-sensitive)
%   D - dense mixed hero-scene traffic, no staged actor, full length run
%   E - the genuine turn (primary Phase 14 demonstration): full
%       APPROACH -> TURN -> EXIT via the plain MATLAB closed-loop caller
%   F - the same full closed loop, but hosted inside a running Simulink
%       model (carlaIntegration/simulink/carlaClosedLoopPipeline.slx),
%       satisfying "MATLAB/Simulink genuinely drives the CARLA ego"
%       literally, not just via the underlying MATLAB functions alone.
%
% Output:
%   results - struct with one field per demo (A..F).

if nargin < 1
    savePngDir = 'C:\Users\ADITYA\sih-autonomous-india\results\figures\';
end

currentPyEnv = pyenv;
if currentPyEnv.Status == "NotLoaded"
    pyenv('Version', 'C:\Users\ADITYA\carla_venv\Scripts\python.exe');
end

results = struct();

fprintf('\n================ DEMO A: clear-ish intersection approach ================\n');
results.A = runStraightDemo('phase14_evidenceA_clear', 80, [], savePngDir);

fprintf('\n================ DEMO B: crossing vehicle ================\n');
results.B = runStraightDemo('phase14_evidenceB_crossing', 70, ...
    struct('bp','vehicle.audi.tt','fwd',12,'right',6,'upM',1.5,'velFwd',0.0,'velRight',-1.0), savePngDir);

fprintf('\n================ DEMO C: pedestrian conflict ================\n');
results.C = runStraightDemo('phase14_evidenceC_pedestrian', 70, ...
    struct('bp','walker.pedestrian.0001','fwd',12,'right',-4,'upM',0.5,'velFwd',0.0,'velRight',0.7), savePngDir);

fprintf('\n================ DEMO D: dense mixed hero-scene traffic ================\n');
results.D = runStraightDemo('phase14_evidenceD_dense', 150, [], savePngDir);

fprintf('\n================ DEMO E: genuine approach->turn->exit (plain MATLAB loop) ================\n');
results.E = runTurnDemo('phase14_evidenceE_turn', savePngDir);

fprintf('\n================ DEMO F: full closed loop hosted in Simulink ================\n');
results.F = runSimulinkDemo(savePngDir);

fprintf('\nDONE_PHASE14_DEMO\n');
end

% =======================================================================
function out = runStraightDemo(tag, numTicks, extraActor, savePngDir)
sceneCfg = carlaIndianSceneConfig();
carlaCfg = carlaConfig();
carlaConnect(carlaCfg);
cleanupObj = onCleanup(@() carlaDisconnect()); %#ok<NASGU>
carlaLoadMap(sceneCfg.mapName);

sceneState = carlaBuildIndianHeroScene(sceneCfg);
carlaAttachCamera(carlaCfg.camera);
carlaAttachLidar(carlaCfg.lidar);
carlaAttachRadar(carlaCfg.radar);
carlaAttachCollisionSensor();
pause(2.0);

extraActorId = [];
if ~isempty(extraActor)
    if startsWith(extraActor.bp, 'walker.')
        yawOffsetDeg = 0.0;
    else
        yawOffsetDeg = atan2d(extraActor.velRight, extraActor.velFwd);
    end
    extraActorId = carlaSpawnActorRelativeToEgo(extraActor.bp, extraActor.fwd, extraActor.right, extraActor.upM, yawOffsetDeg);
    if ~isempty(extraActorId)
        carlaSetActorVelocityRelativeToEgo(extraActorId, extraActor.velFwd, extraActor.velRight, 0.0);
    else
        fprintf('[%s] WARNING: extra conflict actor spawn FAILED - this demo has NO staged conflict actor.\n', tag);
    end
end

trafficState = [];
loopState = carlaClosedLoopInit(60, 4.0, 25);

decisionCounts = struct('cruise',0,'follow',0,'merge',0,'replan',0,'avoid',0,'brake',0,'wait',0,'emergency_stop',0);
minTTCObserved = Inf; minClearanceObserved = Inf; anyGeometricColliding = false;
speeds = []; steerDegs = []; throttles = []; brakes = []; planningTimes = [];
lastReport = [];

fig = figure('Visible','off','Color','w','Position',[0 0 1100 850]);

for k = 1:numTicks
    if ~isempty(extraActorId)
        carlaSetActorVelocityRelativeToEgo(extraActorId, extraActor.velFwd, extraActor.velRight, 0.0);
    end
    egoNow = carlaGetEgoState();
    trafficState = carlaIndianSceneTrafficStep(sceneState, sceneCfg, trafficState, [egoNow.x, egoNow.y]);
    [loopState, report] = carlaClosedLoopStep(loopState);
    if report.skipped
        continue;
    end
    lastReport = report;

    ds = char(report.decisionState);
    if isfield(decisionCounts, ds)
        decisionCounts.(ds) = decisionCounts.(ds) + 1;
    end
    minTTCObserved = min(minTTCObserved, report.minTTC);
    anyGeometricColliding = anyGeometricColliding || report.isColliding;
    speeds(end+1) = report.egoState.velocity; %#ok<AGROW>
    steerDegs(end+1) = rad2deg(report.controlCommand.steeringAngle); %#ok<AGROW>
    throttles(end+1) = report.controlCommand.throttle; %#ok<AGROW>
    brakes(end+1) = report.controlCommand.brake; %#ok<AGROW>
    planningTimes(end+1) = report.planningElapsedS; %#ok<AGROW>

    for i = 1:numel(report.trackedAgents)
        d = norm(report.trackedAgents(i).position - [report.egoState.x, report.egoState.y]);
        minClearanceObserved = min(minClearanceObserved, d);
    end
end

realCollisions = carlaGetCollisionEvents();

if ~isempty(lastReport)
    ego = lastReport.egoState;
    toEgoView = @(xy) [ ...
        cos(-ego.yaw) * (xy(:,1)-ego.x) - sin(-ego.yaw) * (xy(:,2)-ego.y), ...
        sin(-ego.yaw) * (xy(:,1)-ego.x) + cos(-ego.yaw) * (xy(:,2)-ego.y)];
    clf(fig); hold on;
    scatter(0,0,140,'k','^','filled');
    for i = 1:numel(lastReport.trackedAgents)
        ta = lastReport.trackedAgents(i);
        pv = toEgoView(ta.position);
        scatter(pv(1), pv(2), 70, 'b', 'filled');
        text(pv(1)+0.4, pv(2)+0.4, sprintf('id%d %s', ta.id, ta.class), 'FontSize', 7);
    end
    for c = 1:numel(lastReport.candidateTrajectories)
        cv = toEgoView(lastReport.candidateTrajectories{c});
        plot(cv(:,1), cv(:,2), 'Color', [0.85 0.85 0.85]);
    end
    sv = toEgoView(lastReport.selectedTrajectory);
    plot(sv(:,1), sv(:,2), 'g-', 'LineWidth', 2.5);
    title(sprintf('Phase 14 %s: decision=%s minTTC=%.2f realCollisions=%d', ...
        strrep(tag,'_',' '), lastReport.decisionState, lastReport.minTTC, numel(realCollisions)), 'Interpreter', 'none');
    xlabel('x [m] ego-relative (display only)'); ylabel('y [m] ego-relative (display only)'); axis equal; grid on;
    exportgraphics(fig, [savePngDir tag '.png']);
end

out = struct('decisionCounts', decisionCounts, 'minTTCObserved', minTTCObserved, ...
    'minClearanceObserved', minClearanceObserved, 'anyGeometricColliding', anyGeometricColliding, ...
    'realCollisionCount', numel(realCollisions), 'realCollisions', realCollisions, ...
    'maxSpeed', max(speeds), 'meanSpeed', mean(speeds), 'maxSteerDeg', max(abs(steerDegs)), ...
    'maxThrottle', max(throttles), 'maxBrake', max(brakes), ...
    'meanPlanningTimeS', mean(planningTimes), 'p90PlanningTimeS', prctile(planningTimes,90), ...
    'extraActorId', extraActorId);

fprintf('[%s] decisions: %s\n', tag, strjoin(cellfun(@(f) sprintf('%s=%d', f, decisionCounts.(f)), fieldnames(decisionCounts), 'UniformOutput', false), ' '));
fprintf('[%s] minTTC=%.2f minClearance=%.2f geometricCollisionFlag=%d REAL collisions=%d maxSpeed=%.2f maxSteer=%.1fdeg\n', ...
    tag, minTTCObserved, minClearanceObserved, anyGeometricColliding, numel(realCollisions), out.maxSpeed, out.maxSteerDeg);
end

% =======================================================================
function out = runTurnDemo(tag, savePngDir)
sceneCfg = carlaIndianSceneConfig();
carlaCfg = carlaConfig();
carlaConnect(carlaCfg);
cleanupObj = onCleanup(@() carlaDisconnect()); %#ok<NASGU>
carlaLoadMap(sceneCfg.mapName);

sceneState = carlaBuildIndianHeroScene(sceneCfg);
carlaAttachCamera(carlaCfg.camera);
carlaAttachLidar(carlaCfg.lidar);
carlaAttachRadar(carlaCfg.radar);
carlaAttachCollisionSensor();
pause(2.0);

egoState0 = carlaGetEgoState();
[globalPath, turnInfo] = carlaGenerateIntersectionTurnPath( ...
    [egoState0.x, egoState0.y], egoState0.yaw, deg2rad(89.64), 12.0, 45.0, 25.0);
vehCfg = vehicleConfig();
feasibility = carlaCheckTurnFeasibility(globalPath, turnInfo, vehCfg);
fprintf('[E] turn feasibility: length=%.1fm minRadius=%.2fm isFeasible=%d\n', ...
    feasibility.pathLengthM, feasibility.minRadiusM, feasibility.isFeasible);

trafficState = [];
loopState = carlaClosedLoopInit(60, 4.0, 25, globalPath);

GOAL_TOL = 3.0;
MAX_TICKS = 900;
xs = []; ys = []; yaws = []; speeds = []; steerDegs = []; distsToGoal = [];
lastReport = [];
goalReachedTick = [];

for k = 1:MAX_TICKS
    egoNow = carlaGetEgoState();
    trafficState = carlaIndianSceneTrafficStep(sceneState, sceneCfg, trafficState, [egoNow.x, egoNow.y]);
    [loopState, report] = carlaClosedLoopStep(loopState);
    if report.skipped
        continue;
    end
    lastReport = report;
    xs(end+1) = report.egoState.x; %#ok<AGROW>
    ys(end+1) = report.egoState.y; %#ok<AGROW>
    yaws(end+1) = report.egoState.yaw; %#ok<AGROW>
    speeds(end+1) = report.egoState.velocity; %#ok<AGROW>
    steerDegs(end+1) = rad2deg(report.controlCommand.steeringAngle); %#ok<AGROW>
    distsToGoal(end+1) = report.goalDistance; %#ok<AGROW>

    if report.goalDistance < GOAL_TOL
        carlaApplyControl(0, 0, 1); % final controlled stop
        goalReachedTick = k;
        break;
    end
end

realCollisions = carlaGetCollisionEvents();

if ~isempty(lastReport)
    fig = figure('Visible','off','Color','w','Position',[0 0 1200 900]);
    subplot(2,1,1); hold on;
    plot(globalPath(:,1), globalPath(:,2), 'k:', 'LineWidth', 1);
    plot(xs, ys, 'g-', 'LineWidth', 2);
    scatter(xs(1), ys(1), 100, 'b', 'filled');
    scatter(xs(end), ys(end), 100, 'r', 'filled');
    legend('planned curved path', 'actual driven trajectory', 'start', 'end', 'Location', 'best');
    title(sprintf('Phase 14 Demo E: genuine turn (%s, radius=%.1fm) - actual vs planned', turnInfo.turnDirection, turnInfo.arcRadius));
    xlabel('x [m] world'); ylabel('y [m] world'); axis equal; grid on;

    subplot(2,1,2); hold on;
    plot((0:numel(yaws)-1)*0.1, rad2deg(unwrap(yaws)), 'b-', 'LineWidth', 1.5);
    xlabel('time [s]'); ylabel('ego heading [deg]'); grid on;
    title('Heading over time (continuous change during the turn)');

    exportgraphics(fig, [savePngDir tag '.png']);
end

out = struct('turnInfo', turnInfo, 'feasibility', feasibility, ...
    'goalReached', ~isempty(goalReachedTick), 'goalReachedTick', goalReachedTick, ...
    'finalDistToGoal', distsToGoal(end), 'totalHeadingChangeDeg', rad2deg(yaws(end)-yaws(1)), ...
    'pathLengthTraveledM', sum(hypot(diff(xs), diff(ys))), 'maxSpeed', max(speeds), ...
    'maxSteerDeg', max(abs(steerDegs)), 'realCollisionCount', numel(realCollisions), ...
    'numTicks', numel(xs));

fprintf('[E] goalReached=%d finalDistToGoal=%.2f headingChange=%.1fdeg pathTraveled=%.1fm maxSpeed=%.2f REAL collisions=%d ticks=%d\n', ...
    out.goalReached, out.finalDistToGoal, out.totalHeadingChangeDeg, out.pathLengthTraveledM, out.maxSpeed, out.realCollisionCount, out.numTicks);
end

% =======================================================================
function out = runSimulinkDemo(savePngDir)
buildCarlaClosedLoopModel();
load_system('carlaClosedLoopPipeline');
tic;
simOut = sim('carlaClosedLoopPipeline', 'StopTime', '120');
wallClockS = toc;
close_system('carlaClosedLoopPipeline', 0);

x = squeeze(simOut.carlaEgoX);
y = squeeze(simOut.carlaEgoY);
yawv = squeeze(simOut.carlaEgoYaw);
steer = squeeze(simOut.carlaSteeringDeg);
vel = squeeze(simOut.carlaEgoVelocity);
distToGoal = squeeze(simOut.carlaDistToGoal);
goalReached = squeeze(simOut.carlaGoalReached);
realCollisionCount = squeeze(simOut.carlaRealCollisionCount);

fig = figure('Visible','off','Color','w','Position',[0 0 1000 800]);
plot(x, y, 'g-', 'LineWidth', 2); hold on;
scatter(x(1), y(1), 100, 'b', 'filled');
scatter(x(end), y(end), 100, 'r', 'filled');
title(sprintf('Phase 14 Demo F: full closed loop hosted in Simulink - goalReached=%d', goalReached(end)));
xlabel('x [m] world'); ylabel('y [m] world'); axis equal; grid on;
exportgraphics(fig, [savePngDir 'phase14_evidenceF_simulink.png']);

out = struct('goalReached', goalReached(end) > 0, 'finalDistToGoal', distToGoal(end), ...
    'totalHeadingChangeDeg', rad2deg(yawv(end)-yawv(1)), 'maxSpeed', max(vel), ...
    'maxSteerDeg', max(abs(steer)), 'realCollisionCount', realCollisionCount(end), ...
    'numTicks', numel(x), 'wallClockS', wallClockS);

fprintf('[F] goalReached=%d finalDistToGoal=%.2f headingChange=%.1fdeg maxSpeed=%.2f REAL collisions=%d ticks=%d wallClock=%.1fs\n', ...
    out.goalReached, out.finalDistToGoal, out.totalHeadingChangeDeg, out.maxSpeed, out.realCollisionCount, out.numTicks, out.wallClockS);
end
