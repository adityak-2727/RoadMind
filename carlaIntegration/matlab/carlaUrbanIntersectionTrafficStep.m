function carlaUrbanIntersectionTrafficStep(cfg, sceneState, egoPosXY)
% carlaUrbanIntersectionTrafficStep - EMERGENCY DEMO BRIDGE: one tick of
% traffic/pedestrian motion for the urban-intersection demo scene.
%
% Reproduces scenarios/urbanIntersection.m's own ground-truth motion
% model EXACTLY as main.m runs it (main.m line ~281:
%   groundTruthAgents(a).position = groundTruthAgents(a).position + groundTruthAgents(a).velocity * simCfg.dt
% ) - i.e. CONSTANT VELOCITY, no scripted turn/merge trigger, because
% scenarios/urbanIntersection.m itself defines none. Velocity is
% therefore just reapplied every tick via carlaSetActorTargetVelocity
% (CARLA decelerates an unmaintained target velocity, same fact every
% other scripted-traffic file in this project already accounts for).
%
% The only addition beyond pure constant-velocity is the SAME hard
% proximity hold already used (and already proven necessary) by
% carlaIndianSceneTrafficStep.m (unmodified, not called here) - if a
% scripted actor is within SAFETY_BUFFER_M of the ego it holds instead
% of driving through the ego's occupied space. This is a new, separate,
% minimal copy for THIS demo scene only - the frozen hero-scene file is
% untouched.
%
% Inputs:
%   cfg         - from config/carlaUrbanIntersectionConfig.m
%   sceneState  - from carlaBuildUrbanIntersectionScene.m (.trafficActorIds, .pedestrianIds)
%   egoPosXY    - optional [x,y], CARLA raw frame; when given, enables the
%                 proximity hold described above

SAFETY_BUFFER_M = 5.0;
if nargin < 3
    egoPosXY = [];
end

for i = 1:numel(cfg.trafficActors)
    a = cfg.trafficActors(i);
    id = sceneState.trafficActorIds(i);
    if isnan(id)
        continue;
    end
    vx = a.vx; vy = a.vy;
    if ~isempty(egoPosXY)
        state = carlaGetActorState(id);
        d = norm(egoPosXY - [state.location.x, state.location.y]);
        if d < SAFETY_BUFFER_M
            vx = 0; vy = 0;
        end
    end
    carlaSetActorTargetVelocity(id, vx, vy, 0.0);
end

for i = 1:numel(cfg.pedestrians)
    p = cfg.pedestrians(i);
    id = sceneState.pedestrianIds(i);
    if isnan(id)
        continue;
    end
    carlaSetActorTargetVelocity(id, p.vx, p.vy, 0.0);
end

end
