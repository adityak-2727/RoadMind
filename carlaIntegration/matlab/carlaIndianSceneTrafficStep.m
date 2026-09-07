function trafficState = carlaIndianSceneTrafficStep(sceneState, cfg, trafficState)
% carlaIndianSceneTrafficStep - Phase 11.5: one tick of scripted,
% NON-LANE-BASED traffic motion for the Indian hero scene. Never CARLA
% autopilot, never Traffic Manager - every actor's velocity is set
% directly via carlaSetActorTargetVelocity.m (Phase 10/12, unmodified),
% reapplied every tick (CARLA physics decelerates an unmaintained target
% velocity - the same fact discovered and fixed in Phase 12).
%
% Each traffic actor in config/carlaIndianSceneConfig.m carries a free-
% text "intent" label (never read by any frozen algorithm - purely a
% staging instruction for this file):
%   "straight_through"     constant velocity along its spawn heading
%   "slow_through"          same, at reduced speed (the "slow truck
%                           occupies part of the roadway" case)
%   "roadside_edge"         constant velocity, positioned near the road
%                           edge at spawn (see config's offset choice)
%   "turn_left"/"turn_right" constant velocity until within
%                           TURN_TRIGGER_DIST of the junction center,
%                           then the velocity direction is rotated +-85
%                           degrees ONCE - a genuine, physically-applied
%                           direction change, not a pre-baked animation
%   "informal_merge"        constant velocity until within
%                           TURN_TRIGGER_DIST, then rotated by a smaller
%                           +-40 degree angle - drifting obliquely toward
%                           the junction rather than a clean turn
%   "following_ego_lane"    constant velocity along its spawn heading
%                           (same road as the ego, following/overtaking
%                           interaction)
%
% Pedestrians move at their configured cfg.pedestrians(i).velocityCarla,
% reapplied every tick (WalkerControl path, via the same
% carlaSetActorTargetVelocity.m).
%
% Inputs:
%   sceneState   - from carlaBuildIndianHeroScene.m
%   cfg          - from config/carlaIndianSceneConfig.m
%   trafficState - [] on the first call; the previous call's returned
%                  trafficState on every call after
% Output:
%   trafficState - struct with .turned (logical array, one per traffic
%                  actor, whether its turn/merge transition has already
%                  been applied) - pass into the next call

TURN_TRIGGER_DIST = 12.0; % [m] from junction center - when a turning actor commits to its new direction
SPEED_MPS = struct('straight_through', 5.0, 'slow_through', 2.5, 'roadside_edge', 3.0, ...
    'turn_left', 4.0, 'turn_right', 4.0, 'informal_merge', 3.5, 'following_ego_lane', 5.5);
TURN_ANGLE_DEG = struct('turn_left', 85, 'turn_right', -85, 'informal_merge', 40);

if isempty(trafficState) || ~isstruct(trafficState)
    trafficState = struct('turned', false(1, numel(cfg.trafficActors)));
end

center = cfg.junctionCenter;

for i = 1:numel(cfg.trafficActors)
    id = sceneState.trafficActorIds(i);
    if isnan(id)
        continue; % this actor failed to spawn
    end
    a = cfg.trafficActors(i);
    intent = char(a.intent);
    speed = SPEED_MPS.(intent);

    baseHeadingRad = deg2rad(a.yawDeg);
    headingRad = baseHeadingRad;

    if isfield(TURN_ANGLE_DEG, intent)
        raw = carlaGetActorState(id);
        dist = hypot(raw.location.x - center(1), raw.location.y - center(2));
        if trafficState.turned(i)
            headingRad = baseHeadingRad + deg2rad(TURN_ANGLE_DEG.(intent));
        elseif dist <= TURN_TRIGGER_DIST
            trafficState.turned(i) = true;
            headingRad = baseHeadingRad + deg2rad(TURN_ANGLE_DEG.(intent));
        end
    end

    vx = speed * cos(headingRad);
    vy = speed * sin(headingRad);
    carlaSetActorTargetVelocity(id, vx, vy, 0.0);
end

for i = 1:numel(cfg.pedestrians)
    id = sceneState.pedestrianIds(i);
    if isnan(id)
        continue;
    end
    ped = cfg.pedestrians(i);
    carlaSetActorTargetVelocity(id, ped.velocityCarla(1), ped.velocityCarla(2), 0.0);
end

end
