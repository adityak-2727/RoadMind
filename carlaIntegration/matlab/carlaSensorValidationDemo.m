function carlaSensorValidationDemo(numFrames, savePngPath)
% carlaSensorValidationDemo - Phase 10 evidence/validation visualization.
% Connects to a live CARLA server, spawns the ego vehicle plus a small
% multi-actor validation scene (car/truck/motorcycle/pedestrian/bicycle -
% see the blueprint choices below), attaches camera+LiDAR+radar, and
% renders:
%   - top-left:    the live RGB camera image
%   - top-right:   LiDAR points, top-down, project frame (ego-centered)
%   - bottom-left: radar detections, top-down, project frame
%   - bottom-right: connection/status text (CONNECTED per sensor,
%                   current frame #, timestamp)
%
% Deliberately minimal - a functional validation view, not a polished
% dashboard (Phase 10 explicitly does not require visual polish). Not a
% real-time / high-FPS renderer like visualization/RealtimeRenderer.m -
% this is a slow, deliberate loop purely for sensor-interface evidence.
%
% Requires a running CARLA server and pyenv already pointed at the CARLA
% venv's python.exe (see config/carlaConfig.m's header) OR will set it to
% the known default path itself if pyenv is still unset.
%
% Inputs (both optional):
%   numFrames   - number of sim iterations to render (default 40)
%   savePngPath - if provided, saves a snapshot of the final frame to
%                 this path (e.g. results/figures/phase10_sensor_validation.png)

if nargin < 1 || isempty(numFrames)
    numFrames = 40;
end
if nargin < 2
    savePngPath = '';
end

currentPyEnv = pyenv;
if currentPyEnv.Status == "NotLoaded"
    pyenv('Version', 'C:\Users\ADITYA\carla_venv\Scripts\python.exe');
end

cfg = carlaConfig();
carlaConnect(cfg);
cleanupObj = onCleanup(@() carlaDisconnect()); %#ok<NASGU>

carlaSpawnEgoVehicle(cfg);
carlaAttachCamera(cfg.camera);
carlaAttachLidar(cfg.lidar);
carlaAttachRadar(cfg.radar);

sceneBlueprints = {'vehicle.audi.tt', 'vehicle.carlamotors.carlacola', ...
    'vehicle.harley-davidson.low_rider', 'walker.pedestrian.0001', 'vehicle.bh.crossbike'};
sceneOffsets = [8 -3; 12 3; 6 6; 5 -6; 10 -8]; % [forward, right] meters
for i = 1:numel(sceneBlueprints)
    carlaSpawnActorRelativeToEgo(sceneBlueprints{i}, sceneOffsets(i,1), sceneOffsets(i,2), 0.5, 0.0);
end

fig = figure('Name', 'Phase 10 - CARLA Sensor Validation', 'Color', 'w', 'Position', [100 100 1000 700]);
axCam = subplot(2, 2, 1); title(axCam, 'Camera'); axis(axCam, 'off');
axLidar = subplot(2, 2, 2); title(axLidar, 'LiDAR (top-down, project frame)');
axRadar = subplot(2, 2, 3); title(axRadar, 'Radar (top-down, project frame)');
axStatus = subplot(2, 2, 4); axis(axStatus, 'off');

camImgHandle = [];
lidarScatterHandle = [];
radarScatterHandle = [];
statusTextHandle = text(axStatus, 0.05, 0.5, '', 'FontName', 'Consolas', 'FontSize', 10, 'VerticalAlignment', 'middle');

for k = 1:numFrames
    frame = carlaGetCameraFrame();
    points = carlaGetLidarPoints();
    radar = carlaGetRadarDetections();

    if ~isempty(frame)
        if isempty(camImgHandle) || ~isvalid(camImgHandle)
            camImgHandle = imshow(frame.image, 'Parent', axCam);
        else
            set(camImgHandle, 'CData', frame.image);
        end
    end

    if ~isempty(points)
        lidarAgents = carlaLidarPointsToAgents(points);
        if isempty(lidarAgents)
            xs = []; ys = [];
        else
            positions = vertcat(lidarAgents.position);
            xs = positions(:, 1); ys = positions(:, 2);
        end
        if isempty(lidarScatterHandle) || ~isvalid(lidarScatterHandle)
            lidarScatterHandle = scatter(axLidar, xs, ys, 20, 'filled', 'MarkerFaceColor', [0.2 0.4 0.8]);
            hold(axLidar, 'on'); scatter(axLidar, 0, 0, 60, 'r', '^', 'filled'); hold(axLidar, 'off');
            axis(axLidar, 'equal'); xlabel(axLidar, 'x [m]'); ylabel(axLidar, 'y [m]');
        else
            set(lidarScatterHandle, 'XData', xs, 'YData', ys);
        end
    end

    if ~isempty(radar) && radar.numDetections > 0
        radarAgents = carlaRadarToAgents(radar);
        positions = vertcat(radarAgents.position);
        xs = positions(:, 1); ys = positions(:, 2);
        if isempty(radarScatterHandle) || ~isvalid(radarScatterHandle)
            radarScatterHandle = scatter(axRadar, xs, ys, 30, 'filled', 'MarkerFaceColor', [0.8 0.3 0.1]);
            hold(axRadar, 'on'); scatter(axRadar, 0, 0, 60, 'r', '^', 'filled'); hold(axRadar, 'off');
            axis(axRadar, 'equal'); xlabel(axRadar, 'x [m]'); ylabel(axRadar, 'y [m]');
        elseif isvalid(radarScatterHandle)
            set(radarScatterHandle, 'XData', xs, 'YData', ys);
        end
    end

    camStatus = ternaryStr(~isempty(frame), 'CONNECTED', 'WAITING');
    lidarStatus = ternaryStr(~isempty(points), 'CONNECTED', 'WAITING');
    radarStatus = ternaryStr(~isempty(radar), 'CONNECTED', 'WAITING');
    statusStr = sprintf(['Camera: %s\nLiDAR:  %s\nRadar:  %s\n\n' ...
        'frame #%d\ncam t=%.2fs\n\niteration %d/%d'], ...
        camStatus, lidarStatus, radarStatus, ...
        ternaryVal(~isempty(frame), @() frame.frame, 0), ...
        ternaryVal(~isempty(frame), @() frame.timestamp, 0), k, numFrames);
    set(statusTextHandle, 'String', statusStr);

    drawnow;
    pause(0.15);
end

if ~isempty(savePngPath)
    [saveDir, ~, ~] = fileparts(savePngPath);
    if ~isempty(saveDir) && ~isfolder(saveDir)
        mkdir(saveDir);
    end
    exportgraphics(fig, savePngPath);
    fprintf('[carlaSensorValidationDemo] Saved snapshot to %s\n', savePngPath);
end

end

function s = ternaryStr(cond, a, b)
if cond
    s = a;
else
    s = b;
end
end

function v = ternaryVal(cond, fnA, valB)
if cond
    v = fnA();
else
    v = valB;
end
end
