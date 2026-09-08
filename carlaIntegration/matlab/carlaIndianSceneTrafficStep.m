function trafficState = carlaIndianSceneTrafficStep(sceneState, cfg, trafficState, egoPosXY, collisionActorIds)
% Phase 14 addition (optional 4th argument, backward compatible - every
% existing caller that omits it gets EXACTLY the prior behavior):
% egoPosXY, when given, is the ego's current [x, y] (this project's
% frame). See the proximity clamp block below for why this was added -
% it is a hard-contact-prevention clamp, not a redesign of any actor's
% scripted intent/heading/speed.
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
%   collisionActorIds - optional numeric actor IDs from collision events
%                  observed since the preceding traffic tick. This is
%                  diagnostic-only in Phase 14.12: it records which
%                  scripted actor contacted the ego but does not alter
%                  traffic behavior.
% Output:
%   trafficState - struct with .turned (logical array, one per traffic
%                  actor, whether its turn/merge transition has already
%                  been applied) - pass into the next call

TURN_TRIGGER_DIST = 12.0; % [m] from junction center - when a turning actor commits to its new direction
SPEED_MPS = struct('straight_through', 5.0, 'slow_through', 2.5, 'roadside_edge', 3.0, ...
    'turn_left', 4.0, 'turn_right', 4.0, 'informal_merge', 3.5, 'following_ego_lane', 5.5);
TURN_ANGLE_DEG = struct('turn_left', 85, 'turn_right', -85, 'informal_merge', 40);

% Phase 14 fix (found live, root-caused with a collision sensor - the
% first time any phase has checked for REAL physical contacts, not just
% the ego's own predicted/geometric collisionCheck.m flag): every
% scripted actor here is velocity-commanded with ZERO awareness of the
% ego's position - by design, since the ego is supposed to be the one
% doing the avoiding. That is fine while both keep moving, but when the
% ego correctly comes to a stop (e.g. emergency_stop for a genuine
% hazard), an actor whose scripted path continues straight toward the
% ego's now-stationary position has nothing stopping it from physically
% driving INTO the ego - measured live: a "following_ego_lane" actor
% (5.5 m/s, sharing the ego's own lane) rammed a correctly-stopped ego
% with 17 collision-sensor events in two ticks, one impulse peaking at
% 6836 (CARLA's own units) - a genuine physical contact, not a
% measurement artifact (confirmed by the ego's speed jumping from a
% commanded 0 m/s to 2.94 m/s with throttle still at 0.00). This is a
% property of the SCENE's traffic script, not a defect in the ego's own
% perception/tracking/prediction/decision/planning/collision-checking/
% control chain - none of those layers can prevent a DIFFERENT actor
% from driving into a stationary ego.
%
% Fix: a hard, deterministic proximity clamp, applied ONLY when egoPosXY
% is supplied - if a scripted actor is within SAFETY_BUFFER_M of the ego
% it holds (zero velocity) for that tick instead.
% (An earlier revision only held when the actor was CLOSING on the ego;
% that missed side-by-side/overtaking passes with near-zero longitudinal
% closing speed, so the test is now unconditional.) This changes nothing about any
% actor's intent, heading, per-intent speed, or turn-trigger geometry
% when the ego is not in its immediate path - it only ever prevents the
% specific failure mode of driving through the ego's own occupied space,
% exactly like a standard NPC-traffic "don't drive through the other
% car" safety net, not a redesign of the scripted scenario.
% Phase 14.5 NOTE - two attempted improvements to this clamp were tried
% and BOTH measurably regressed the result, so both were reverted and the
% Phase 14 value stands. Recorded here so they are not retried blindly:
%
%   (a) Body-aware clamp distance (actorHalfLength + egoHalfLength + 2m,
%       ~9.5m for the 10.3m-long bus instead of a flat 5m). Rationale was
%       sound - a flat 5m centre-to-centre test can never fire for a body
%       that long. Measured result: real collision events rose 199 -> 1048
%       on the same maneuver, because holding traffic further out turned
%       nearby actors into stationary roadblocks the ego then had to
%       squeeze past in an already-tight corridor.
%   (b) (a) PLUS a genuine brake + hand brake on the held actor
%       (set_actor_hold), since zeroing a TARGET velocity lets a 5.5 m/s
%       bus coast rather than stop. Measured result: 3399 events - far
%       worse. Forensics showed the ego WEDGED against a hand-braked
%       actor for hundreds of ticks while its own decision state read
%       "cruise" (a stationary obstacle at near-zero relative speed gives
%       an effectively infinite TTC, so nothing escalated) - i.e. the fix
%       created an immovable obstacle the ego could neither perceive as
%       urgent nor drive around.
%
% The flat 5.0m clamp below is therefore kept as the best measured
% configuration, NOT as a known-correct one - residual contacts remain and
% are reported honestly rather than tuned away.
SAFETY_BUFFER_M = 5.0;
RECOVERY_RELEASE_DISTANCE_M = 5.5; % 0.5 m hysteresis beyond the existing 5 m clamp
MAX_RECOVERY_TICKS = 50; % bounded ~5 s at the loop's nominal 0.1 s cadence
if nargin < 4
    egoPosXY = [];
end
if nargin < 5 || isempty(collisionActorIds)
    collisionActorIds = [];
end

if isempty(trafficState) || ~isstruct(trafficState)
    nTraffic = numel(cfg.trafficActors);
    trafficState = struct( ...
        'turned', false(1, nTraffic), ...
        'recoveryActive', false(1, nTraffic), ...
        'recoveryTimedOut', false(1, nTraffic), ...
        'recoveryTicks', zeros(1, nTraffic), ...
        'recoveryActivationCount', zeros(1, nTraffic), ...
        'recoveryTimeoutCount', zeros(1, nTraffic));
end
if ~isfield(trafficState, 'recoveryActive')
    nTraffic = numel(cfg.trafficActors);
    trafficState.recoveryActive = false(1, nTraffic);
    trafficState.recoveryTimedOut = false(1, nTraffic);
    trafficState.recoveryTicks = zeros(1, nTraffic);
    trafficState.recoveryActivationCount = zeros(1, nTraffic);
    trafficState.recoveryTimeoutCount = zeros(1, nTraffic);
end
trafficState.diagnostics = repmat(makeTrafficDiagnostic(), 1, numel(cfg.trafficActors));

center = cfg.junctionCenter;

for i = 1:numel(cfg.trafficActors)
    id = sceneState.trafficActorIds(i);
    if isnan(id)
        continue; % this actor failed to spawn
    end
    a = cfg.trafficActors(i);
    intent = char(a.intent);
    speed = SPEED_MPS.(intent);
    newCollisionEvent = any(collisionActorIds == id);
    if newCollisionEvent && ~trafficState.recoveryActive(i) && ~trafficState.recoveryTimedOut(i)
        trafficState.recoveryActive(i) = true;
        trafficState.recoveryTicks(i) = 0;
        trafficState.recoveryActivationCount(i) = trafficState.recoveryActivationCount(i) + 1;
    end

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

    originalVx = speed * cos(headingRad);
    originalVy = speed * sin(headingRad);
    vx = originalVx;
    vy = originalVy;
    actorRaw = [];
    distToEgo = NaN;
    clampActive = false;

    if ~isempty(egoPosXY)
        if isfield(TURN_ANGLE_DEG, intent)
            actorRaw = raw; % already queried above for the turn-trigger check
        else
            actorRaw = carlaGetActorState(id);
        end
        % actorRaw.location is CARLA's raw (left-handed) frame; egoPosXY
        % is this project's (right-handed) frame - must convert before
        % comparing, using the SAME verified conversion every other
        % CARLA-derived position in this codebase uses. Mixing the two
        % frames directly here would silently reproduce the exact class
        % of bug Phase 13 found and fixed for the perception pipeline.
        [actorProjX, actorProjY] = carlaCoordToProject(actorRaw.location.x, actorRaw.location.y);
        toEgo = egoPosXY - [actorProjX, actorProjY];
        distToEgo = norm(toEgo);

        clampDist = SAFETY_BUFFER_M; % see the Phase 14.5 note above for the two larger variants that were tried and regressed
        % Widened from a "hold only if closing" check (Phase 14, first
        % attempt) to an unconditional hold whenever within the buffer,
        % after live evidence showed it was insufficient: an offline
        % geometric check of "following_ego_lane" actors (explicitly
        % designed to share the ego's own lane - closely following/
        % overtaking, per this file's own docstring) against the Phase 14
        % turn path found one such actor's SPAWN POINT only 0.01m from
        % the planned route, and a live run with the closing-only clamp
        % still recorded 2179 real collision-sensor events over one
        % approach->turn->exit maneuver. A side-by-side or overtaking
        % pass can have near-zero longitudinal closing speed while still
        % being far too close laterally for two real vehicle bodies to
        % occupy - the closing-speed test cannot see that case. Holding
        % unconditionally whenever within the clamp distance is simpler and
        % safe: it costs the SAME scripted actor a few ticks of paused
        % motion only when it is already within a couple of vehicle
        % lengths of the ego, changing nothing else about its intent/
        % heading/speed/turn-trigger geometry.
        % CONTACT_RECOVERY is deliberately actor-specific: only a CARLA
        % collision event naming this actor can enter it. While active, the
        % original scripted velocity above is retained so CARLA physics can
        % separate the bodies naturally. No displacement, impulse, route
        % change, autopilot, or ego-control override is introduced.
        if trafficState.recoveryActive(i)
            trafficState.recoveryTicks(i) = trafficState.recoveryTicks(i) + 1;
            if distToEgo >= RECOVERY_RELEASE_DISTANCE_M
                trafficState.recoveryActive(i) = false;
                trafficState.recoveryTicks(i) = 0;
            elseif trafficState.recoveryTicks(i) > MAX_RECOVERY_TICKS
                trafficState.recoveryActive(i) = false;
                trafficState.recoveryTimedOut(i) = true;
                trafficState.recoveryTimeoutCount(i) = trafficState.recoveryTimeoutCount(i) + 1;
            end
        elseif trafficState.recoveryTimedOut(i) && distToEgo >= RECOVERY_RELEASE_DISTANCE_M
            % A timed-out actor may become eligible for a future, distinct
            % contact only after it has physically separated.
            trafficState.recoveryTimedOut(i) = false;
        end

        if distToEgo < clampDist && ~trafficState.recoveryActive(i)
            vx = 0; vy = 0; % hold - see the Phase 14/14.5 header blocks above
            clampActive = true;
        end
    end

    carlaSetActorTargetVelocity(id, vx, vy, 0.0);
    diag = makeTrafficDiagnostic();
    diag.actorId = id;
    diag.blueprint = string(a.blueprint);
    diag.intent = string(a.intent);
    diag.distanceToEgo = distToEgo;
    diag.originalTargetVelocity = [originalVx, originalVy];
    diag.appliedTargetVelocity = [vx, vy];
    diag.clampActive = clampActive;
    diag.newCollisionEvent = newCollisionEvent;
    if ~isempty(actorRaw)
        diag.actualVelocity = [actorRaw.velocity_mps.x, actorRaw.velocity_mps.y];
    end
    if trafficState.recoveryActive(i)
        diag.recoveryState = "contact_recovery";
    elseif trafficState.recoveryTimedOut(i)
        diag.recoveryState = "recovery_timeout";
    end
    diag.recoveryTicks = trafficState.recoveryTicks(i);
    diag.recoveryActivationCount = trafficState.recoveryActivationCount(i);
    diag.recoveryTimeoutCount = trafficState.recoveryTimeoutCount(i);
    trafficState.diagnostics(i) = diag;
end

for i = 1:numel(cfg.pedestrians)
    id = sceneState.pedestrianIds(i);
    if isnan(id)
        continue;
    end
    ped = cfg.pedestrians(i);
    carlaSetActorTargetVelocity(id, ped.velocityCarla(1), ped.velocityCarla(2), 0.0);
end

function diag = makeTrafficDiagnostic()
% Compact per-actor diagnostic record. It is deliberately derived from the
% state query already required for the proximity clamp; no callback or
% world-wide query is introduced by Phase 14.12 instrumentation.
diag = struct('actorId', NaN, 'blueprint', "", 'intent', "", ...
    'distanceToEgo', NaN, 'originalTargetVelocity', [NaN, NaN], ...
    'appliedTargetVelocity', [NaN, NaN], 'actualVelocity', [NaN, NaN], ...
    'clampActive', false, 'newCollisionEvent', false, ...
    'recoveryState', "normal", 'recoveryTicks', 0, ...
    'recoveryActivationCount', 0, 'recoveryTimeoutCount', 0);
end

end
