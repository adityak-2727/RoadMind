function out = runCarlaIndianHeroDemo(runLabel, maxTicks)
% One evaluator-facing urban turn, preserving all failed attempts.
% addpath(genpath(pwd)); out = runCarlaIndianHeroDemo('jury',1000);
if nargin < 1; runLabel = 'jury'; end
if nargin < 2; maxTicks = 1000; end
validateattributes(maxTicks,{'numeric'},{'scalar','integer','positive'});
root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
tag = regexprep(char(runLabel),'[^a-zA-Z0-9_-]','_');
outDir = fullfile(root,'results','phase15', ...
    [char(datetime('now','Format','yyyyMMdd_HHmmss_SSS')) '_' tag]);
mkdir(outDir);
diary(fullfile(outDir,'console.txt')); diaryCleanup = onCleanup(@() diary('off')); %#ok<NASGU>
out = struct('runLabel',string(runLabel),'outputDirectory',string(outDir), ...
    'startupSuccess',false,'sceneBuildSuccess',false,'goalReached',false, ...
    'exception',"",'cleanupException',"",'orphanActorIds',NaN, ...
    'collisionCount',NaN,'simulinkTimingS',NaN);
reports = {}; traffic = {}; timings = []; driven = []; supervisor = []; fig = [];
events = []; scene = []; path = []; caught = []; snapshots = cell(1,3); stalledSince = [];
cleanupGuard = onCleanup(@emergencyCleanup); %#ok<NASGU>
try
    assert(~verLessThan('matlab','26.1'), 'Phase15:matlabVersion','MATLAB R2026a or later required.');
    cfg = carlaConfig(); sceneCfg = carlaIndianSceneConfig();
    sceneCfg.captureSpawnCollisions = true;
    env = pyenv;
    if env.Status == "NotLoaded"; pyenv('Version',cfg.pythonExecutable); end
    carlaConnect(cfg);
    carlaLoadMap('Town03');
    session = getCarlaSession(); supervisor = session.heroSupervisor();
    out.preflight = jsondecode(char(supervisor.preflight()));
    supervisor.set_demo_weather();
    out.startupSuccess = true;
    scene = carlaBuildIndianHeroScene(sceneCfg);
    supervisor.remember_owned();
    assert(isempty(scene.failedSpawns), 'Phase15:spawnFailure', ...
        'Scene has %d failed spawns; see scene manifest and console.',numel(scene.failedSpawns));
    assert(numel(scene.trafficActorIds)>=8 && numel(scene.trafficActorIds)<=10 && ...
        numel(scene.pedestrianIds)>=1 && scene.numTrafficLightsFrozen>0, ...
        'Phase15:incompleteScene','Required traffic, pedestrian or lights absent.');
    sensorIds = [carlaAttachCamera(cfg.camera), carlaAttachLidar(cfg.lidar), carlaAttachRadar(cfg.radar)];
    assert(all(isfinite(sensorIds)), 'Phase15:sensorSpawn','Required sensor failed to attach.');
    supervisor.remember_owned();
    carlaApplyControl(0,0,1); pause(2);
    carlaValidateHeroObservation(carlaGetSynchronizedObservations(),carlaPerceptionConfig());
    out.sceneBuildSuccess = true;
    overview = carlaCaptureSnapshot(-45,100,65,38,-48,1280,800,90);
    assert(~isempty(overview),'Phase15:overviewMissing','Overview camera produced no image.');
    imwrite(overview.image,fullfile(outDir,'phase15_hero_overview.png'));
    ego0 = carlaGetEgoState();
    [path,turnInfo] = carlaGenerateIntersectionTurnPath([ego0.x ego0.y],ego0.yaw,deg2rad(89.64),12,45,25);
    out.turnInfo = turnInfo;
    out.feasibility = carlaCheckTurnFeasibility(path,turnInfo,vehicleConfig());
    assert(out.feasibility.isFeasible,'Phase15:infeasibleRoute','Existing turn route is infeasible.');
    out.initialHeadingDeg = rad2deg(ego0.yaw); out.goalPosition = path(end,:);
    fig = carlaHeroDashboard([],[],path,[],0);
    loop = carlaClosedLoopInit(60,4,25,path); loop.strictDemo = true;
    manifest = sceneCfg.trafficActors;
    for i = 1:numel(manifest); manifest(i).id = scene.trafficActorIds(i); end
    supervisor.configure_traffic(jsonencode(manifest));
    stageSaved = false(1,3);
    for k = 1:maxTicks
        tickTic = tic;
        traffic{k} = jsondecode(char(supervisor.traffic_step())); %#ok<AGROW>
        for i = 1:numel(scene.pedestrianIds)
            v = sceneCfg.pedestrians(i).velocityCarla;
            carlaSetActorTargetVelocity(scene.pedestrianIds(i),v(1),v(2),0);
        end
        pipelineTic = tic;
        [loop,r] = carlaClosedLoopStep(loop);
        matlabElapsed = toc(pipelineTic);
        assert(~r.skipped,'Phase15:skipped','Closed-loop observation was skipped.');
        driven(end+1,:) = [r.egoState.x r.egoState.y r.egoState.yaw r.egoState.velocity]; %#ok<AGROW>
        count = carlaGetCollisionCount();
        r.physicalCollisionCount = count;
        stage = 1 + (abs(rad2deg(r.egoState.yaw-ego0.yaw))>20) + ...
            (abs(rad2deg(r.egoState.yaw-ego0.yaw))>70);
        if mod(k,10)==1 || ~stageSaved(stage) || r.goalDistance<3
            fig = carlaHeroDashboard(fig,r,path,driven,count);
            if ~stageSaved(stage)
                names = {'phase15_ego_approach','phase15_turn','phase15_after_turn'};
                snapshots{stage} = struct('report',r,'driven',driven,'count',count,'name',names{stage});
                stageSaved(stage) = true;
            end
        end
        % Retain every genuine reasoning output, omit bulky raw RGB/point arrays.
        stored = r;
        stored.obs = rmfield(stored.obs,{'cameraFrame','lidarPoints','radarDetections','actorObjects'});
        reports{end+1} = stored; %#ok<AGROW>
        timings(end+1,:) = [r.dt matlabElapsed r.obs.acquisitionElapsed r.obs.maxOffset toc(tickTic)]; %#ok<AGROW>
        if mod(k,50)==0
            fprintf('tick=%d goal=%.2fm speed=%.2f heading=%.1f collisions=%d\n', ...
                k,r.goalDistance,r.egoState.velocity,rad2deg(r.egoState.yaw),count);
            save(fullfile(outDir,'checkpoint.mat'),'out','reports','traffic','timings','driven','scene','path');
        end
        if r.goalDistance<3; out.goalReached = true; break; end
        if count>0 && r.egoState.velocity<0.1
            if isempty(stalledSince); stalledSince = r.egoState.timestamp; end
            assert(r.egoState.timestamp-stalledSince<10,'Phase15:contactDeadlock', ...
                'Physical contacts and speed below 0.1 m/s persisted for 10 s; demonstration stopped.');
        else
            stalledSince = [];
        end
    end
    carlaApplyControl(0,0,1); pause(0.5);
    events = carlaGetCollisionEvents();
catch err
    caught = err; out.exception = string(getReport(err,'extended','hyperlinks','off'));
    fprintf(2,'%s\n',out.exception);
end
% Preserve evidence BEFORE releasing resources, including on failure.
try
    if getCarlaSession().Connected
        carlaApplyControl(0,0,1); pause(0.3);
        events = carlaGetCollisionEvents();
    end
catch err
    out.collisionReadException = string(err.message);
end
if ~isempty(supervisor)
    try; supervisor.remember_owned(); catch err; out.cleanupException = string(err.message); end
end
save(fullfile(outDir,'run.mat'),'out','reports','traffic','timings','driven','events','scene','path');
try
    carlaDisconnect();
    if ~isempty(supervisor)
        out.orphanActorIds = jsondecode(char(supervisor.survivors()));
        supervisor.restore_environment();
    end
catch err
    out.cleanupException = string(err.message);
end
if ~isempty(driven)
    out.finalSpeed = driven(end,4); out.finalHeadingDeg = rad2deg(driven(end,3));
    yaw = unwrap(driven(:,3)); out.headingChangeDeg = rad2deg(yaw(end)-yaw(1));
    out.pathLengthM = sum(hypot(diff(driven(:,1)),diff(driven(:,2))));
    out.turnSuccess = abs(out.headingChangeDeg)>70 && out.goalReached;
    out.finalGoalDistance = reports{end}.goalDistance;
    out.maxSpeed = max(driven(:,4));
    out.meanDt = mean(timings(2:end,1)); out.p90Dt = prctile(timings(2:end,1),90);
    out.maxDt = max(timings(2:end,1)); out.meanMatlabS = mean(timings(:,2));
    out.meanAcquisitionS = mean(timings(:,3)); out.maxSyncOffsetS = max(timings(:,4));
    writematrix(timings,fullfile(outDir,'timings_dt_matlab_acquisition_sync_total.csv'));
end
if ~isempty(events)
    out.collisionCount = numel(events);
    out.collisionActorIds = unique([events.otherActorId]);
    out.collisionActorTypes = unique(string({events.otherActorType}));
    out.firstCollisionTimestamp = min([events.timestamp]);
    firstTick = find(cellfun(@(q) q.physicalCollisionCount>0,reports),1);
    if ~isempty(firstTick)
        out.firstCollisionObservedTick = firstTick;
        out.egoSpeedAtFirstObservedCollision = reports{firstTick}.egoState.velocity;
    end
    writetable(struct2table(events),fullfile(outDir,'collision_events.csv'));
elseif out.sceneBuildSuccess && ~isfield(out,'collisionReadException')
    out.collisionCount = 0;
end
% Contact-event span is an estimate, not continuous physical contact duration.
out.maxContactEventSpanS = 0;
if ~isempty(events)
    for id = unique([events.otherActorId])
        times = sort([events([events.otherActorId]==id).timestamp]);
        cuts = [1 find(diff(times)>0.5)+1 numel(times)+1];
        for j = 1:numel(cuts)-1
            out.maxContactEventSpanS = max(out.maxContactEventSpanS,times(cuts(j+1)-1)-times(cuts(j)));
        end
    end
end
save(fullfile(outDir,'run.mat'),'out','reports','traffic','timings','driven','events','scene','path');
% Rasterization happens only after safe stop/cleanup, never between controls.
if exist('r','var') && ~r.skipped && ~isempty(driven)
    try
        fig = carlaHeroDashboard(fig,r,path,driven,numel(events));
        exportgraphics(fig,fullfile(outDir,'phase15_final.png'));
    catch err
        out.evidenceException = string(err.message);
    end
end
for i = 1:numel(snapshots)
    if isempty(snapshots{i}); continue; end
    s = snapshots{i};
    try
        fig = carlaHeroDashboard(fig,s.report,path,s.driven,s.count);
        exportgraphics(fig,fullfile(outDir,[s.name '.png']));
        imwrite(s.report.obs.cameraFrame.image,fullfile(outDir,[s.name '_rgb.png']));
    catch err
        out.evidenceException = string(err.message);
    end
end
fid = fopen(fullfile(outDir,'summary.json'),'w');
fprintf(fid,'%s',jsonencode(out,PrettyPrint=true)); fclose(fid);
save(fullfile(outDir,'run.mat'),'out','reports','traffic','timings','driven','events','scene','path');
fprintf('Phase 15 evidence: %s\n',outDir);
if ~isempty(caught); rethrow(caught); end
assert(isempty(out.cleanupException) && isempty(out.orphanActorIds), ...
    'Phase15:cleanup','Cleanup failed or orphan verification unavailable; inspect summary.');

    function emergencyCleanup()
        try
            if getCarlaSession().Connected; carlaApplyControl(0,0,1); end
        catch cleanupErr
            warning('Phase15:stopFailed','%s',cleanupErr.message);
        end
        try; carlaDisconnect(); catch cleanupErr; warning('Phase15:disconnectFailed','%s',cleanupErr.message); end
    end
end
