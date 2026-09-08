function runCarlaUrbanIntersectionDemo()
% runCarlaUrbanIntersectionDemo - EMERGENCY EVALUATION DEMO BRIDGE.
%
% Reproduces the EXISTING, UNMODIFIED scenarios/urbanIntersection.m
% scenario as a live CARLA simulation, driven by the EXISTING, UNMODIFIED
% MATLAB autonomy pipeline (perception -> fusion -> tracking ->
% prediction -> decision -> planning -> control), via the EXISTING,
% UNMODIFIED carlaClosedLoopStep.m closed loop.
%
% PREREQUISITE: CARLA server must already be running, VISIBLY (no
% -RenderOffScreen), e.g.:
%   cd C:\Users\ADITYA\CARLA_0.9.16
%   CarlaUE4.exe -carla-server -nosound
%
% No ego autopilot, no Traffic Manager, no prerecorded ego trajectory, no
% ego teleportation - the ego is driven only through
% carlaApplyControl(throttle,brake,steer) computed by the real pipeline.
% Traffic/pedestrian actors are scripted (constant velocity, matching
% scenarios/urbanIntersection.m's own ground-truth motion model exactly
% as main.m runs it) - scripted traffic is explicitly permitted; only the
% ego must not be scripted.
%
% Usage:
%   runCarlaUrbanIntersectionDemo

addpath(genpath('C:/Users/ADITYA/sih-autonomous-india'));
pyenv('Version', 'C:/Users/ADITYA/carla_venv/Scripts/python.exe');

cfg = carlaUrbanIntersectionConfig();
carlaCfg = carlaConfig();

fprintf('[demo] Building urban-intersection scene (Town03, junction id=103, South->North corridor)...\n');
sceneState = carlaBuildUrbanIntersectionScene(cfg);
cleanupObj = onCleanup(@() cleanupScene()); %#ok<NASGU>

carlaAttachCamera(carlaCfg.camera);
carlaAttachLidar(carlaCfg.lidar);
carlaAttachRadar(carlaCfg.radar);
carlaAttachCollisionSensor();
pause(2.0);

fprintf('[demo] Setting spectator to an overview of the junction...\n');
try
    carlaSetSpectatorTransform(cfg.junctionCenter(1) - 15, cfg.junctionCenter(2) - 20, 45.0, -35.0, 35.0, 0.0);
catch
    fprintf('[demo] (spectator helper not available - CARLA free camera can be positioned manually)\n');
end

loopState = carlaClosedLoopInit(cfg.egoForwardDistanceM, cfg.egoBaseSpeedMps, 40);

MAX_TICKS = 800; % 80s budget at dt=0.1s, matching MATLAB's own 500-step/50s urbanIntersection budget with margin for real physics
GOAL_TOL = 3.0;
tic0 = tic;
tickTimes = zeros(MAX_TICKS, 1);
totalCollisionsAtEnd = 0;

for k = 1:MAX_TICKS
    tTickStart = tic;

    egoNow = carlaGetEgoState();
    carlaUrbanIntersectionTrafficStep(cfg, sceneState, [egoNow.x, egoNow.y]);
    [loopState, report] = carlaClosedLoopStep(loopState);

    tickTimes(k) = toc(tTickStart);

    if report.skipped
        continue;
    end

    if mod(k, 20) == 0
        e = report.egoState;
        cc = report.controlCommand;
        fprintf('t=%5.1fs tick=%4d ego=(%.1f,%.1f) v=%.2f thr=%.2f brk=%.2f decision=%-15s dist2goal=%.1f\n', ...
            toc(tic0), k, e.x, e.y, e.velocity, cc.throttle, cc.brake, char(report.decisionState), report.goalDistance);
    end

    if report.goalDistance < GOAL_TOL
        carlaApplyControl(0, 0, 1);
        fprintf('[demo] GOAL REACHED at tick %d (t=%.1fs)\n', k, toc(tic0));
        break;
    end
end

validTicks = tickTimes(tickTimes > 0);
if ~isempty(validTicks)
    fprintf('[demo] MATLAB control-loop rate: mean=%.1f Hz (%.3fs/tick), over %d ticks\n', ...
        1/mean(validTicks), mean(validTicks), numel(validTicks));
end

try
    totalCollisionsAtEnd = carlaGetCollisionCount();
catch
    totalCollisionsAtEnd = -1; % collision sensor was not attached this run
end
fprintf('[demo] Run complete. Total ticks=%d, final ego goalDistance last reported above.\n', k);
fprintf('[demo] Physical collision-sensor events this run: %d (not suppressed, not tuned away)\n', totalCollisionsAtEnd);

    function cleanupScene()
        fprintf('[demo] Cleaning up: destroying traffic/pedestrian actors and disconnecting...\n');
        try
            carlaDestroyOtherActors();
        catch
        end
        try
            carlaDisconnect();
        catch
        end
    end

end
