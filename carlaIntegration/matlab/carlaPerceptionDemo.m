function carlaPerceptionDemo(numFrames, savePngPath)
% carlaPerceptionDemo - Phase 11 evidence/validation visualization.
% Connects to a live CARLA server, spawns the same 5-actor validation
% scene as Phase 10 plus two nearby actors (anti-false-merge case), and
% renders:
%   - left:  the live RGB camera image
%   - right: top-down view distinguishing camera(ground-truth)/LiDAR/
%            radar raw observations from the final FUSED agents, with
%            per-fused-object id/class/sources/confidence text
%
% Also prints "Fused objects: N / Camera observations: N / LiDAR
% objects: N / Radar detections: N" and sensor CONNECTED status, per the
% Phase 11 spec. Deliberately minimal, matching Phase 10's
% carlaSensorValidationDemo.m in style - not a polished dashboard.
%
% Requires a running CARLA server; sets pyenv to the known CARLA venv
% path if not already loaded.
%
% Inputs (both optional):
%   numFrames   - number of perception ticks to render (default 25)
%   savePngPath - if provided, saves the final frame

if nargin < 1 || isempty(numFrames)
    numFrames = 25;
end
if nargin < 2
    savePngPath = '';
end

currentPyEnv = pyenv;
if currentPyEnv.Status == "NotLoaded"
    pyenv('Version', 'C:\Users\ADITYA\carla_venv\Scripts\python.exe');
end

cfg = carlaConfig();
pcfg = carlaPerceptionConfig();
carlaConnect(cfg);
cleanupObj = onCleanup(@() carlaDisconnect()); %#ok<NASGU>

carlaSpawnEgoVehicle(cfg);
carlaAttachCamera(cfg.camera);
carlaAttachLidar(cfg.lidar);
carlaAttachRadar(cfg.radar);

sceneBlueprints = {'vehicle.audi.tt', 'vehicle.carlamotors.carlacola', ...
    'vehicle.harley-davidson.low_rider', 'walker.pedestrian.0001', 'vehicle.bh.crossbike'};
sceneOffsets = [10 -3; 16 3; 7 6; 5 -6; 13 -8];
for i = 1:numel(sceneBlueprints)
    carlaSpawnActorRelativeToEgo(sceneBlueprints{i}, sceneOffsets(i,1), sceneOffsets(i,2), 0.5, 0.0);
end
% Two-nearby-objects anti-false-merge demonstration.
carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 24, -2, 0.5, 0.0);
carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 24, 2, 0.5, 0.0);

pause(2.5);

fig = figure('Name', 'Phase 11 - CARLA Perception + Sensor Fusion', 'Color', 'w', 'Position', [80 80 1300 700]);
axCam = subplot(1, 2, 1); title(axCam, 'Camera'); axis(axCam, 'off');
axTop = subplot(1, 2, 2); title(axTop, 'Top-down: raw observations + fused agents');
axis(axTop, 'equal'); xlabel(axTop, 'x [m] (ego-relative, forward)'); ylabel(axTop, 'y [m] (ego-relative, left)');
hold(axTop, 'on');
xlim(axTop, [-10, 60]); ylim(axTop, [-30, 30]);

camImgHandle = [];
prevFused = [];

for k = 1:numFrames
    [fused, obs] = carlaPerceptionStep(pcfg, false, prevFused);
    prevFused = fused;

    if ~isempty(obs.cameraFrame)
        if isempty(camImgHandle) || ~isvalid(camImgHandle)
            camImgHandle = imshow(obs.cameraFrame.image, 'Parent', axCam);
        else
            set(camImgHandle, 'CData', obs.cameraFrame.image);
        end
    end

    cla(axTop);
    hold(axTop, 'on');
    scatter(axTop, 0, 0, 90, 'k', '^', 'filled', 'DisplayName', 'ego');

    nCamera = 0; nLidar = 0; nRadar = 0;
    if obs.status.actors.usable && ~isempty(obs.egoState)
        gt = carlaActorObjectsToAgents(obs.actorObjects);
        gt = localWorldToEgo(gt, obs.egoState);
        nCamera = numel(gt);
        if ~isempty(gt)
            p = vertcat(gt.position);
            scatter(axTop, p(:,1), p(:,2), 40, [0.6 0.6 0.6], 'o', 'DisplayName', 'ground-truth (camera slot)');
        end
    end
    if obs.status.lidar.usable
        la = carlaLidarPointsToAgents(obs.lidarPoints);
        la = localShift(la, cfg.lidar);
        nLidar = numel(la);
        if ~isempty(la)
            p = vertcat(la.position);
            scatter(axTop, p(:,1), p(:,2), 15, [0.2 0.4 0.8], '.', 'DisplayName', 'LiDAR');
        end
    end
    if obs.status.radar.usable
        ra = carlaRadarToAgents(obs.radarDetections);
        ra = localShift(ra, cfg.radar);
        nRadar = numel(ra);
        if ~isempty(ra)
            p = vertcat(ra.position);
            scatter(axTop, p(:,1), p(:,2), 25, [0.8 0.3 0.1], 'x', 'DisplayName', 'radar');
        end
    end

    for i = 1:numel(fused)
        a = fused(i);
        scatter(axTop, a.position(1), a.position(2), 80, 'g', 'o', 'LineWidth', 1.5, 'DisplayName', 'fused');
        label = sprintf('id%d %s\n%.2f', a.id, a.class, a.confidence);
        text(axTop, a.position(1) + 0.5, a.position(2) + 0.5, label, 'FontSize', 7, 'Color', [0 0.4 0]);
    end

    title(axTop, sprintf('Fused objects: %d | Camera(GT): %d | LiDAR: %d | Radar: %d', numel(fused), nCamera, nLidar, nRadar));

    statusStr = sprintf('Camera: %s   LiDAR: %s   Radar: %s   |   frame=%s syncOffset=%.3fs   |   tick %d/%d', ...
        connStr(obs.status.camera.usable), connStr(obs.status.lidar.usable), connStr(obs.status.radar.usable), ...
        mat2str(obs.referenceFrame), obs.maxOffset, k, numFrames);
    xlabel(axTop, statusStr);

    drawnow;
    pause(0.15);
end

if ~isempty(savePngPath)
    [saveDir, ~, ~] = fileparts(savePngPath);
    if ~isempty(saveDir) && ~isfolder(saveDir)
        mkdir(saveDir);
    end
    exportgraphics(fig, savePngPath);
    fprintf('[carlaPerceptionDemo] Saved snapshot to %s\n', savePngPath);
end

end

function s = connStr(usable)
if usable
    s = 'CONNECTED';
else
    s = 'stale/missing';
end
end

function agents = localWorldToEgo(agents, egoState)
if isempty(agents); return; end
c = cos(egoState.yaw); s = sin(egoState.yaw);
for i = 1:numel(agents)
    d = agents(i).position - [egoState.x, egoState.y];
    agents(i).position = [c*d(1)+s*d(2), -s*d(1)+c*d(2)];
end
end

function agents = localShift(agents, sensorCfg)
if isempty(agents); return; end
dx = sensorCfg.mountX; dy = -sensorCfg.mountY;
for i = 1:numel(agents)
    agents(i).position = agents(i).position + [dx, dy];
end
end
