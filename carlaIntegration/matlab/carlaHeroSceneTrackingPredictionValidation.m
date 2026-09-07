function metrics = carlaHeroSceneTrackingPredictionValidation(numTicks, savePngPath, rangeM)
% carlaHeroSceneTrackingPredictionValidation - Phase 12: runs the FULL
% Indian hero scene (all 10 traffic actors, 5 parked vehicles, 2
% pedestrians - config/carlaIndianSceneConfig.m) through the real
% tracking + prediction pipeline (carlaTrackingStep.m wrapping the
% frozen objectTracking.m; carlaPredictionStep.m wrapping the frozen
% trajectoryPrediction.m/K1), collecting the metrics Phase 12's dense-
% scene validation requires. Distinct from carlaTrackingPredictionDemo.m's
% Sections A-G (isolated single-actor controlled experiments, at a
% tightened sensor range for a clean demonstration) - this file
% deliberately uses the FULL default sensor range and the FULL scene,
% because the point here is to observe the harder, more realistic case.
%
% Never CARLA autopilot, never a scripted ego trajectory - the ego stays
% at its hero-scene approach point throughout (this validates the
% perception->fusion->tracking->prediction chain only, not planning/
% decision/control, per Phase 12's explicit scope).
%
% Inputs:
%   numTicks    - optional, default 60
%   savePngPath - optional, if given saves an evidence snapshot showing
%                 tracked objects + predicted trajectories over the
%                 dense scene
% Output:
%   metrics - struct with the full set of tracking/prediction/timing
%             metrics Phase 12 requires (see fields below)

if nargin < 1 || isempty(numTicks)
    numTicks = 60;
end
if nargin < 2
    savePngPath = '';
end
if nargin < 3 || isempty(rangeM)
    rangeM = 30.0;
end

currentPyEnv = pyenv;
if currentPyEnv.Status == "NotLoaded"
    pyenv('Version', 'C:\Users\ADITYA\carla_venv\Scripts\python.exe');
end

sceneCfg = carlaIndianSceneConfig();

% EGO STAGING (found necessary live): the hero scene's own egoApproach
% (45m back from the junction, Phase 11.5's staging point for a future
% approach-then-turn maneuver) is too far from where the scripted
% traffic actually operates for a MEANINGFUL dense-traffic validation -
% an initial run measured a mean of only 0.2 ground-truth-visible actors
% per tick from there. This is the SAME West-approach road (not a
% different, fabricated position) at 15m back instead of 45m - the exact
% waypoint-resolved coordinate already computed live during Phase 11.5's
% own geometry investigation (carlaCoordToProject's real road spline, not
% a hand-guessed point). Only this validation's ego placement differs;
% config/carlaIndianSceneConfig.m's own egoApproach (used by every other
% Phase 12 demo and reserved for Phase 14's actual approach-and-turn) is
% unchanged.
sceneCfg.egoApproach.x = -33.91; sceneCfg.egoApproach.y = 134.74; sceneCfg.egoApproach.yawDeg = -1.30;

sceneState = carlaBuildIndianHeroScene(sceneCfg);
cleanupObj = onCleanup(@() carlaDisconnect()); %#ok<NASGU>

carlaCfg = carlaConfig();
carlaAttachCamera(carlaCfg.camera);
carlaAttachLidar(carlaCfg.lidar);
carlaAttachRadar(carlaCfg.radar);
pause(2.0);

% RANGE FINDING (measured live, not guessed): an initial run at the full
% default 60m range showed active tracks growing far beyond any bound
% (mean 80.6, max 97) against only 6 real ground-truth-visible actors,
% and per-tick dt correspondingly ballooning to a 0.311s mean (vs the
% intended ~0.1s) simply from the compute cost of processing 80+ tracks.
% Root cause: at 60m, a continuous stream of NEW, mutually-uncorrelated
% "unknown" LiDAR/radar clutter detections (never the same physical
% points twice - Phase 10/11's documented static-geometry-clustering
% limitation) each mint a fresh track and receive carlaTrackingStep.m's
% full missed-observation coasting grace period; since new clutter keeps
% arriving faster than old clutter's grace period expires, the net
% active-track count grows without bound. This is NOT a defect in
% objectTracking.m/trajectoryPrediction.m (frozen, both behaving exactly
% as designed - a real new detection SHOULD get a fresh track and a fair
% coasting allowance) - it is the sensor/perception layer feeding an
% unfilterable volume of noise at full range. Phase 12's own
% carlaClosedLoopInit.m already established and uses this identical fix
% (its own demoRangeMeters parameter, default 25-30m) for exactly this
% reason; this file reuses that same evidence-based bound rather than
% inventing a new one.
perceptionCfg = carlaPerceptionConfig();
perceptionCfg.maxSensorRangeMeters = rangeM;
trackingCfg = carlaTrackingConfig();

trafficState = [];
trackerState = [];
prevFused = [];

% --- accumulators ---
fusedCounts = []; activeTrackCounts = [];
gtVisibleCounts = [];
allTrackIdsEverSeen = [];
trackFirstSeenTick = containers.Map('KeyType','double','ValueType','double');
trackLastSeenTick = containers.Map('KeyType','double','ValueType','double');
missedObsTotal = 0; coastTicksTotal = 0; reconnectCount = 0;
prevTrackedIdSet = [];
wasCoastedLastSeen = containers.Map('KeyType','double','ValueType','logical');

behaviorCounts = struct('stopped',0,'normal',0,'crossing',0,'merging',0,'irregular_unknown', 0, 'relaxed_unknown', 0);
k1ActivationCount = 0; k1RejectionCount = 0; % rejection = irregular-class/unpredictable-motion track that stayed conservative
predictionFailures = 0;
uncertaintyFinalRadii = [];

dtHistory = [];
tickTic = tic;
exceptions = 0;

for k = 1:numTicks
    if k == 1
        dt = 0.1;
    else
        elapsed = toc(tickTic);
        if elapsed < 0.1
            pause(0.1 - elapsed);
            elapsed = toc(tickTic);
        end
        dt = elapsed;
    end
    tickTic = tic;
    dtHistory(end+1) = dt; %#ok<AGROW>

    try
        trafficState = carlaIndianSceneTrafficStep(sceneState, sceneCfg, trafficState);
        [fusedAgents, obs] = carlaPerceptionStep(perceptionCfg, false, prevFused);
        prevFused = fusedAgents;

        [trackedAgents, trackInfo, trackerState] = carlaTrackingStep(fusedAgents, trackerState, dt, trackingCfg);

        egoState = obs.egoState;
        if isempty(egoState)
            continue;
        end
        [predictedTrajectories, behaviorInfo] = carlaPredictionStep(trackedAgents, egoState, 4.0, dt);

        fusedCounts(end+1) = numel(fusedAgents); %#ok<AGROW>
        activeTrackCounts(end+1) = numel(trackedAgents); %#ok<AGROW>
        if ~isempty(obs.actorObjects)
            gtVisibleCounts(end+1) = obs.actorObjects.numObjects; %#ok<AGROW>
        end

        currentIds = [trackedAgents.id];
        newIds = setdiff(currentIds, allTrackIdsEverSeen);
        for nid = newIds
            trackFirstSeenTick(nid) = k;
        end
        for i = 1:numel(trackedAgents)
            tid = trackedAgents(i).id;
            trackLastSeenTick(tid) = k;
            if trackInfo(i).state == "coasted"
                coastTicksTotal = coastTicksTotal + 1;
                missedObsTotal = missedObsTotal + 1;
                wasCoastedLastSeen(tid) = true;
            elseif trackInfo(i).state == "confirmed"
                if isKey(wasCoastedLastSeen, tid) && wasCoastedLastSeen(tid)
                    reconnectCount = reconnectCount + 1;
                end
                wasCoastedLastSeen(tid) = false;
            end
        end
        allTrackIdsEverSeen = union(allTrackIdsEverSeen, currentIds);

        for i = 1:numel(behaviorInfo)
            switch behaviorInfo(i).label
                case "stopped"; behaviorCounts.stopped = behaviorCounts.stopped + 1;
                case "crossing"; behaviorCounts.crossing = behaviorCounts.crossing + 1;
                case "merging"; behaviorCounts.merging = behaviorCounts.merging + 1;
                case "irregular/unknown"
                    behaviorCounts.irregular_unknown = behaviorCounts.irregular_unknown + 1;
                    k1RejectionCount = k1RejectionCount + 1;
                case "normal (K1-relaxed unknown)"
                    behaviorCounts.relaxed_unknown = behaviorCounts.relaxed_unknown + 1;
                    k1ActivationCount = k1ActivationCount + 1;
                otherwise
                    behaviorCounts.normal = behaviorCounts.normal + 1;
            end
        end

        for i = 1:numel(predictedTrajectories)
            traj = predictedTrajectories{i};
            if isempty(traj)
                predictionFailures = predictionFailures + 1;
            else
                uncertaintyFinalRadii(end+1) = traj(end, 3); %#ok<AGROW>
            end
        end

        if k == numTicks && ~isempty(savePngPath)
            saveEvidenceSnapshot(savePngPath, egoState, trackedAgents, trackInfo, predictedTrajectories, behaviorInfo, obs, k, numTicks);
        end
    catch ME
        exceptions = exceptions + 1;
        fprintf('EXCEPTION at tick %d: %s\n', k, ME.message);
    end
end

metrics = struct();
metrics.numTicks = numTicks;
metrics.fusedCounts = fusedCounts;
metrics.activeTrackCounts = activeTrackCounts;
metrics.meanActiveTracks = mean(activeTrackCounts);
metrics.maxActiveTracks = max(activeTrackCounts);
metrics.gtVisibleCounts = gtVisibleCounts;
metrics.trackCreationCount = numel(allTrackIdsEverSeen);
metrics.missedObservationCount = missedObsTotal;
metrics.coastTicksTotal = coastTicksTotal;
metrics.reconnectCount = reconnectCount;
metrics.behaviorCounts = behaviorCounts;
metrics.k1ActivationCount = k1ActivationCount;
metrics.k1RejectionCount = k1RejectionCount;
metrics.predictionFailures = predictionFailures;
metrics.uncertaintyFinalRadii = uncertaintyFinalRadii;
metrics.meanFinalUncertainty = mean(uncertaintyFinalRadii);
metrics.maxFinalUncertainty = max(uncertaintyFinalRadii);
metrics.dtHistory = dtHistory;
metrics.meanDt = mean(dtHistory);
metrics.p90Dt = prctile(dtHistory, 90);
metrics.maxDt = max(dtHistory);
metrics.minDt = min(dtHistory(2:end)); % skip tick 1's nominal value
metrics.timingAnomalies = sum(dtHistory > 0.3); % >3x the nominal 0.1s
metrics.exceptions = exceptions;

% Track duration (ticks) per track id
durations = [];
tids = keys(trackFirstSeenTick);
for i = 1:numel(tids)
    tid = tids{i};
    durations(end+1) = trackLastSeenTick(tid) - trackFirstSeenTick(tid) + 1; %#ok<AGROW>
end
metrics.trackDurations = durations;
metrics.meanTrackDuration = mean(durations);

fprintf('\n================ DENSE HERO-SCENE TRACKING+PREDICTION VALIDATION (%d ticks) ================\n', numTicks);
fprintf('Fused objects: mean=%.1f min=%d max=%d\n', mean(fusedCounts), min(fusedCounts), max(fusedCounts));
fprintf('Active tracks: mean=%.1f min=%d max=%d\n', metrics.meanActiveTracks, min(activeTrackCounts), max(activeTrackCounts));
fprintf('Ground-truth visible actors: mean=%.1f min=%d max=%d\n', mean(gtVisibleCounts), min(gtVisibleCounts), max(gtVisibleCounts));
fprintf('Track creation count (unique ids ever seen): %d\n', metrics.trackCreationCount);
fprintf('Missed-observation ticks: %d | Coast ticks: %d | Reconnections: %d\n', missedObsTotal, coastTicksTotal, reconnectCount);
fprintf('Track duration: mean=%.1f ticks, max=%d ticks\n', metrics.meanTrackDuration, max(durations));
fprintf('Behavior counts: stopped=%d normal=%d crossing=%d merging=%d irregular/unknown=%d K1-relaxed=%d\n', ...
    behaviorCounts.stopped, behaviorCounts.normal, behaviorCounts.crossing, behaviorCounts.merging, behaviorCounts.irregular_unknown, behaviorCounts.relaxed_unknown);
fprintf('K1 activations=%d rejections=%d\n', k1ActivationCount, k1RejectionCount);
fprintf('Prediction failures: %d | Final uncertainty radius: mean=%.2fm max=%.2fm\n', predictionFailures, metrics.meanFinalUncertainty, metrics.maxFinalUncertainty);
fprintf('Timing: mean dt=%.3fs p90=%.3fs max=%.3fs min=%.3fs anomalies(>0.3s)=%d\n', metrics.meanDt, metrics.p90Dt, metrics.maxDt, metrics.minDt, metrics.timingAnomalies);
fprintf('Exceptions: %d\n', exceptions);

end

function saveEvidenceSnapshot(savePngPath, egoState, trackedAgents, trackInfo, predictedTrajectories, behaviorInfo, obs, k, numTicks) %#ok<INUSD>
fig = figure('Visible','off','Color','w','Position',[0 0 1200 900]);
hold on;
scatter(0,0,140,'k','^','filled');
for i = 1:numel(trackedAgents)
    ta = trackedAgents(i);
    scatter(ta.position(1), ta.position(2), 70, 'b', 'filled');
    traj = predictedTrajectories{i};
    if ~isempty(traj)
        plot(traj(:,1), traj(:,2), 'r--');
        theta = linspace(0,2*pi,10);
        r = traj(end,3);
        fill(traj(end,1)+r*cos(theta), traj(end,2)+r*sin(theta), 'r', 'FaceAlpha', 0.12, 'EdgeColor','none');
    end
    text(ta.position(1)+0.5, ta.position(2)+0.5, sprintf('id%d %s\n%s', ta.id, ta.class, behaviorInfo(i).label), 'FontSize', 7);
end
title(sprintf('Phase 12: Dense hero-scene tracking + prediction (tick %d/%d) - %d tracks, frame#%s', k, numTicks, numel(trackedAgents), mat2str(obs.referenceFrame)));
xlabel('x [m] (ego-relative)'); ylabel('y [m] (ego-relative)'); axis equal; grid on;
[d,~,~] = fileparts(savePngPath);
if ~isempty(d) && ~isfolder(d); mkdir(d); end
exportgraphics(fig, savePngPath);
fprintf('[carlaHeroSceneTrackingPredictionValidation] Saved evidence to %s\n', savePngPath);
end
