function results = carlaPlanningDecisionDemo(savePngDir)
% carlaPlanningDecisionDemo - Phase 13: validates the COMPLETE, real,
% unmodified pipeline
%   perception -> fusion -> tracking -> prediction -> decision ->
%   adaptivePlanner (K2) -> collisionCheck -> controller-ready output
% against the Indian hero scene, via carlaClosedLoopStep.m (Phase 12,
% extended additively for Phase 13's planning-diagnostics fields - see
% its own header) - never CARLA autopilot, never a scripted ego path.
%
% Demos A-G use the full hero scene (carlaBuildIndianHeroScene.m,
% unmodified) plus, where a specific conflict needs to be GUARANTEED
% rather than left to chance, one additional controlled actor spawned
% relative to the ego (the same carlaSpawnActorRelativeToEgo.m pattern
% Phase 12 used) - never fabricating a decision label, only staging a
% real actor for the real pipeline to react to.
%
% Demo H uses carlaGenerateIntersectionTurnPath.m's curved approach ->
% turn -> exit path (Phase 13, new) as the globalPath instead of the
% straight-line demo corridor - proving the curved path is genuinely
% consumed by the same frozen localPlanner/adaptivePlanner/collisionCheck
% chain, WITHOUT physically completing the turn (that is Phase 14's job -
% this demo runs enough ticks to show the planner engaging with the
% curved path's geometry, not a full traversal).
%
% Output:
%   results - struct with one field per demo (A..H), each holding the
%             measured metrics used in the Phase 13 report.

if nargin < 1
    savePngDir = 'C:\Users\ADITYA\sih-autonomous-india\results\figures\';
end

currentPyEnv = pyenv;
if currentPyEnv.Status == "NotLoaded"
    pyenv('Version', 'C:\Users\ADITYA\carla_venv\Scripts\python.exe');
end

results = struct();

% Spawn distances tightened close (10-14m) and tick counts raised (60):
% found live during Phase 13 development that the ego, starting from
% rest and accelerating at vehicleConfig.maxAccel, had not yet closed
% the distance to a 22-30m actor within 40 ticks (~4-6s) - every demo
% showed 15/15 candidates feasible the whole run (i.e. no interaction had
% begun yet), not because the decision/planning layer failed to react.
fprintf('\n================ DEMO A: normal approach ================\n');
results.A = runSceneDemo('evidenceA_normal', 60, [], savePngDir);

fprintf('\n================ DEMO B: parked vehicle obstruction ================\n');
results.B = runSceneDemo('evidenceB_parked', 70, [], savePngDir);

% Each conflict actor's velocity is expressed EGO-RELATIVE (velFwd,
% velRight m/s along the ego's own forward/right axes, closing toward the
% ego's ~2.5m-half-width planning corridor) and applied every tick via
% carlaSetActorVelocityRelativeToEgo.m (Phase 13, new - see its header).
% For every VEHICLE-class actor (C/E/F/G - not D's pedestrian, which uses
% CARLA WalkerControl and is unaffected), yawOffsetDeg spawns the actor
% ALREADY FACING its intended (velFwd, velRight) direction:
% yawOffsetDeg = atan2d(velRight, velFwd). This was found to be required
% by a live test during Phase 13 development, not a design preference -
% CARLA's wheeled-vehicle tire-friction model resists any commanded
% velocity that is not aligned with the actor's own heading (a bicycle
% spawned facing along the road and given a purely-lateral target
% velocity had that velocity decay from 0.60 m/s to ~0.04 m/s within 20
% ticks, confirmed by isolated measurement of the actor's own raw CARLA
% velocity - not a perception/tracking artifact). Spawning the actor
% facing the direction it is meant to travel eliminates this: the same
% velocity request, applied to an actor already oriented that way, was
% measured to hold at a constant -0.60 m/s (no decay) over 15 consecutive
% ticks. up_m raised to 1.5 for all conflict actors (Demo C's original
% up_m=0.5 was found to collide with the road/prop geometry at that exact
% point and silently fail to spawn - 1.5 was confirmed clear in a sweep
% covering every demo's spawn point).
fprintf('\n================ DEMO C: crossing vehicle ================\n');
results.C = runSceneDemo('evidenceC_crossing', 60, struct('bp','vehicle.audi.tt', ...
    'fwd',12,'right',6,'upM',1.5,'velFwd',0.0,'velRight',-1.0), savePngDir);

fprintf('\n================ DEMO D: pedestrian conflict ================\n');
results.D = runSceneDemo('evidenceD_pedestrian', 60, struct('bp','walker.pedestrian.0001', ...
    'fwd',12,'right',-4,'upM',0.5,'velFwd',0.0,'velRight',0.7), savePngDir);

fprintf('\n================ DEMO E: bicycle / motorcycle ================\n');
results.E = runSceneDemo('evidenceE_bicycle_motorcycle', 60, struct('bp','vehicle.bh.crossbike', ...
    'fwd',12,'right',3,'upM',1.5,'velFwd',0.0,'velRight',-0.6), savePngDir);

fprintf('\n================ DEMO F: informal merge ================\n');
results.F = runSceneDemo('evidenceF_merge', 60, struct('bp','vehicle.audi.tt', ...
    'fwd',14,'right',-8,'upM',1.5,'velFwd',1.0,'velRight',1.2), savePngDir);

fprintf('\n================ DEMO G: irregular / unknown ================\n');
results.G = runSceneDemo('evidenceG_irregular', 60, struct('bp','vehicle.vespa.zx125', ...
    'fwd',13,'right',5,'upM',1.5,'velFwd',-0.3,'velRight',-0.9), savePngDir);

fprintf('\n================ DEMO H: full hero intersection, curved turn path ================\n');
results.H = runTurnPathDemo(savePngDir);

fprintf('\nDONE_PHASE13_DEMO\n');
end

% =======================================================================
function out = runSceneDemo(tag, numTicks, extraActor, savePngDir)
sceneCfg = carlaIndianSceneConfig();
carlaCfg = carlaConfig();
carlaConnect(carlaCfg);
cleanupObj = onCleanup(@() carlaDisconnect()); %#ok<NASGU>
carlaLoadMap(sceneCfg.mapName);

sceneState = carlaBuildIndianHeroScene(sceneCfg);
carlaAttachCamera(carlaCfg.camera);
carlaAttachLidar(carlaCfg.lidar);
carlaAttachRadar(carlaCfg.radar);
pause(2.0);

extraActorId = [];
if ~isempty(extraActor)
    % VEHICLE-class actors are spawned already facing (velFwd, velRight)
    % so CARLA's tire-friction model doesn't fight the commanded velocity
    % - see the header comment above for the live measurement that found
    % this necessary. Pedestrians (walker.pedestrian.*) ignore spawn yaw
    % for motion purposes (WalkerControl uses direction+speed directly),
    % so 0 is passed for them without effect.
    if startsWith(extraActor.bp, 'walker.')
        yawOffsetDeg = 0.0;
    else
        yawOffsetDeg = atan2d(extraActor.velRight, extraActor.velFwd);
    end
    extraActorId = carlaSpawnActorRelativeToEgo(extraActor.bp, extraActor.fwd, extraActor.right, extraActor.upM, yawOffsetDeg);
    if ~isempty(extraActorId)
        carlaSetActorVelocityRelativeToEgo(extraActorId, extraActor.velFwd, extraActor.velRight, 0.0);
    else
        fprintf('[%s] WARNING: extra conflict actor spawn FAILED (collision at that point) - this demo has NO staged conflict actor.\n', tag);
    end
end

trafficState = [];
loopState = carlaClosedLoopInit(60, 4.0, 25); % modest speed, bounded range - see carlaClosedLoopInit.m's own evidence-based range note

decisionCounts = struct('cruise',0,'follow',0,'merge',0,'replan',0,'avoid',0,'brake',0,'wait',0,'emergency_stop',0);
minTTCObserved = Inf; minClearanceObserved = Inf; anyColliding = false;
totalCandidatesHist = []; feasibleHist = []; collisionRejHist = []; ttcRejHist = []; fallbackCount = 0;
candidateChanges = 0;
planningTimes = [];
lastReport = [];

fig = figure('Visible','off','Color','w','Position',[0 0 1100 850]);

for k = 1:numTicks
    if ~isempty(extraActorId)
        carlaSetActorVelocityRelativeToEgo(extraActorId, extraActor.velFwd, extraActor.velRight, 0.0);
    end
    trafficState = carlaIndianSceneTrafficStep(sceneState, sceneCfg, trafficState);
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
    anyColliding = anyColliding || report.isColliding;
    totalCandidatesHist(end+1) = report.totalCandidates; %#ok<AGROW>
    feasibleHist(end+1) = report.feasibleCandidateCount; %#ok<AGROW>
    collisionRejHist(end+1) = report.collisionRejectedCount; %#ok<AGROW>
    ttcRejHist(end+1) = report.ttcRejectedCount; %#ok<AGROW>
    if report.usedFallback; fallbackCount = fallbackCount + 1; end
    if report.candidateChanged; candidateChanges = candidateChanges + 1; end
    planningTimes(end+1) = report.planningElapsedS; %#ok<AGROW>

    for i = 1:numel(report.trackedAgents)
        % Phase 13: trackedAgents.position is GLOBAL frame (see
        % carlaFusedAgentsToGlobal.m) - distance to ego requires
        % subtracting the ego's own global position, not norm() alone.
        d = norm(report.trackedAgents(i).position - [report.egoState.x, report.egoState.y]);
        minClearanceObserved = min(minClearanceObserved, d);
    end
end

if ~isempty(lastReport)
    % Phase 13: trackedAgents/candidateTrajectories/selectedTrajectory are
    % all GLOBAL frame now (carlaFusedAgentsToGlobal.m fix - see
    % carlaClosedLoopStep.m). Transform into an ego-centered, ego-heading-
    % aligned view HERE, for display only - this does not feed back into
    % any decision/planning computation above.
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
    title(sprintf('Phase 13 %s: decision=%s minTTC=%.2f candidates=%d(feasible=%d)', ...
        strrep(tag,'_',' '), lastReport.decisionState, lastReport.minTTC, lastReport.totalCandidates, lastReport.feasibleCandidateCount), 'Interpreter', 'none');
    xlabel('x [m] ego-relative (display only)'); ylabel('y [m] ego-relative (display only)'); axis equal; grid on;
    exportgraphics(fig, [savePngDir 'phase13_' tag '.png']);
end

out = struct('decisionCounts', decisionCounts, 'minTTCObserved', minTTCObserved, ...
    'minClearanceObserved', minClearanceObserved, 'anyColliding', anyColliding, ...
    'meanCandidates', mean(totalCandidatesHist), 'meanFeasible', mean(feasibleHist), ...
    'meanCollisionRejected', mean(collisionRejHist), 'meanTTCRejected', mean(ttcRejHist), ...
    'fallbackCount', fallbackCount, 'candidateChanges', candidateChanges, ...
    'meanPlanningTimeS', mean(planningTimes), 'p90PlanningTimeS', prctile(planningTimes,90), 'maxPlanningTimeS', max(planningTimes), ...
    'extraActorId', extraActorId);

fprintf('[%s] decisions: %s\n', tag, strjoin(cellfun(@(f) sprintf('%s=%d', f, decisionCounts.(f)), fieldnames(decisionCounts), 'UniformOutput', false), ' '));
fprintf('[%s] minTTC=%.2f minClearance=%.2f anyColliding=%d meanCandidates=%.1f meanFeasible=%.1f fallbackCount=%d candidateChanges=%d meanPlanningTime=%.4fs\n', ...
    tag, minTTCObserved, minClearanceObserved, anyColliding, out.meanCandidates, out.meanFeasible, fallbackCount, candidateChanges, out.meanPlanningTimeS);
end

% =======================================================================
function out = runTurnPathDemo(savePngDir)
sceneCfg = carlaIndianSceneConfig();
carlaCfg = carlaConfig();
carlaConnect(carlaCfg);
cleanupObj = onCleanup(@() carlaDisconnect()); %#ok<NASGU>
carlaLoadMap(sceneCfg.mapName);

sceneState = carlaBuildIndianHeroScene(sceneCfg);
carlaAttachCamera(carlaCfg.camera);
carlaAttachLidar(carlaCfg.lidar);
carlaAttachRadar(carlaCfg.radar);
pause(2.0);

egoState0 = carlaGetEgoState();
% West approach -> North exit, LEFT turn, matching the hero scene's real
% junction geometry (Phase 11.5's live-resolved waypoints).
[globalPath, turnInfo] = carlaGenerateIntersectionTurnPath( ...
    [egoState0.x, egoState0.y], egoState0.yaw, deg2rad(89.64), 12.0, 5.0, 25.0);
vehCfg = vehicleConfig();
feasibility = carlaCheckTurnFeasibility(globalPath, turnInfo, vehCfg);
fprintf('[H] turn feasibility: length=%.1fm minRadius=%.2fm maxFeasibleCurvature-derived minRadius=%.2fm isFeasible=%d\n', ...
    feasibility.pathLengthM, feasibility.minRadiusM, 1/feasibility.maxFeasibleCurvature, feasibility.isFeasible);

trafficState = [];
loopState = carlaClosedLoopInit(60, 4.0, 25, globalPath);

decisionsSeen = strings(1,0);
totalCandidatesHist = []; feasibleHist = [];
lastReport = [];
NUM_TICKS = 40;
for k = 1:NUM_TICKS
    trafficState = carlaIndianSceneTrafficStep(sceneState, sceneCfg, trafficState);
    [loopState, report] = carlaClosedLoopStep(loopState);
    if report.skipped; continue; end
    lastReport = report;
    decisionsSeen(end+1) = report.decisionState; %#ok<AGROW>
    totalCandidatesHist(end+1) = report.totalCandidates; %#ok<AGROW>
    feasibleHist(end+1) = report.feasibleCandidateCount; %#ok<AGROW>
end

if ~isempty(lastReport)
    % Phase 13: same ego-centered display transform as runSceneDemo - see
    % its comment for why (trackedAgents/candidateTrajectories/
    % selectedTrajectory/globalPath are all GLOBAL frame now).
    ego = lastReport.egoState;
    toEgoView = @(xy) [ ...
        cos(-ego.yaw) * (xy(:,1)-ego.x) - sin(-ego.yaw) * (xy(:,2)-ego.y), ...
        sin(-ego.yaw) * (xy(:,1)-ego.x) + cos(-ego.yaw) * (xy(:,2)-ego.y)];

    fig = figure('Visible','off','Color','w','Position',[0 0 1100 850]);
    hold on;
    gv = toEgoView(globalPath);
    plot(gv(:,1), gv(:,2), 'k:', 'LineWidth', 1);
    scatter(0,0,140,'k','^','filled');
    for i = 1:numel(lastReport.trackedAgents)
        ta = lastReport.trackedAgents(i);
        pv = toEgoView(ta.position);
        scatter(pv(1), pv(2), 70, 'b', 'filled');
    end
    for c = 1:numel(lastReport.candidateTrajectories)
        cv = toEgoView(lastReport.candidateTrajectories{c});
        plot(cv(:,1), cv(:,2), 'Color', [0.85 0.85 0.85]);
    end
    sv = toEgoView(lastReport.selectedTrajectory);
    plot(sv(:,1), sv(:,2), 'g-', 'LineWidth', 2.5);
    title(sprintf('Phase 13 H: curved intersection path (%s turn, radius=%.1fm, feasible=%d), tick %d/%d', ...
        turnInfo.turnDirection, turnInfo.arcRadius, feasibility.isFeasible, k, NUM_TICKS));
    xlabel('x [m] ego-relative'); ylabel('y [m] ego-relative'); axis equal; grid on;
    exportgraphics(fig, [savePngDir 'phase13_evidenceH_curved_turn.png']);
    fprintf('[H] saved evidence. decisions seen: %s\n', strjoin(unique(decisionsSeen), ', '));
end

out = struct('turnInfo', turnInfo, 'feasibility', feasibility, 'decisionsSeen', unique(decisionsSeen), ...
    'meanCandidates', mean(totalCandidatesHist), 'meanFeasible', mean(feasibleHist));
end
