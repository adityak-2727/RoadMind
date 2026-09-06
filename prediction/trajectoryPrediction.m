function predictedTrajectories = trajectoryPrediction(trackedAgents, egoState, horizon, dt)
% trajectoryPrediction - dispatches each tracked agent to the appropriate
% motion model using both agent.class and its current behavior category
% (classifyBehavior.m): irregular-class agents (pedestrian/animal/pushcart/
% bicycle/unknown) always get the higher-uncertainty model, and so does any
% agent - regardless of class - currently classified "crossing" or
% "merging" relative to the ego's heading, since a car cutting across an
% intersection is exactly as hard to predict precisely as a pedestrian
% doing the same thing. A "stopped" agent always gets the lower-uncertainty
% model even if its class is normally treated as irregular (a parked cart
% isn't erratic just because carts can be).
%
% "unknown" stays in irregularClasses (a genuinely erratic or freshly-seen
% unknown-class object must still get the conservative model) - but an
% unknown-class track that has demonstrated several seconds of steady,
% consistent, non-crossing motion is a poor match for that conservative
% model (root-caused on highwayMerge: a correctly-tracked, dead-straight,
% constant-velocity truck loses its class in perception - see
% docs/architecture.md - and the resulting fast-growing irregular
% uncertainty radius made every candidate trajectory geometrically
% uncrossable, independent of controller tracking, confirmed by an offline
% zero-tracking-error counterfactual). isPredictableUnknown below (local
% function) checks for that evidence using a short rolling history buffer
% kept internally in this file (see "persistent trackHistory"); any track
% without enough consecutive, consistent history keeps the existing
% irregular model - this only ever relaxes conservatism, never adds to it,
% and never touches any class other than "unknown".
%
% Note: egoState was added versus the original Phase 0 draft signature -
% classifying crossing/merging needs a reference direction ("parallel to
% the road"), and egoState.yaw is the only one available without adding a
% road-geometry parameter. See docs/architecture.md's interface change log.
%
% Inputs:
%   trackedAgents - struct array of tracked agents (see config/createAgent.m)
%   egoState      - current ego state struct, for its yaw as the reference
%                   direction classifyBehavior measures crossing/merging against
%   horizon       - [s] prediction horizon
%   dt            - [s] prediction timestep
% Output:
%   predictedTrajectories - cell array, one Nx3 [x, y, uncertaintyRadius]
%                           array per agent, same order as trackedAgents

irregularClasses = ["pedestrian", "animal", "pushcart", "bicycle", "unknown"];

% --- Predictable-unknown thresholds -------------------------------------
% Derived from perception/objectTracking.m's own constant-velocity Kalman
% filter (F, Q, H, R built from PROCESS_NOISE=0.5, MEASUREMENT_NOISE=
% diag([0.3,0.3]), at simulationConfig.m's dt=0.1s) - not picked
% arbitrarily. Running that filter's covariance recursion forward from its
% own INITIAL_COVARIANCE (velocity variance 10 m^2/s^2) shows velocity
% variance falling to ~0.076 m^2/s^2 (std ~0.28 m/s) by frame 20 (2.0s),
% versus an asymptotic steady state of ~0.060 m^2/s^2 (std ~0.245 m/s) -
% i.e. by 2.0s of continuous tracking the filter's own velocity estimate
% has essentially converged, so using this window's noise floor as the
% "is this just filter noise" bound for a stability check is consistent
% with what the tracker itself already believes about this track.
HISTORY_WINDOW_FRAMES = 20;         % ~2.0s at dt=0.1s
VELOCITY_NOISE_FLOOR = 0.28;        % [m/s] per-axis std, from the recursion above
MAGNITUDE_TOL = 3 * VELOCITY_NOISE_FLOOR;   % ~0.84 m/s: 3-sigma on tracker's own noise floor
ACCEL_TOL = 2 * sqrt(0.5);          % ~1.41 m/s^2: 2-sigma on objectTracking's own
                                     % PROCESS_NOISE=0.5 unmodeled-acceleration term

% A steady, non-crossing velocity alone cannot rule out a vulnerable road
% user by motion evidence: this project's own scenario ground truth has
% every pedestrian/animal moving at or below 0.90 m/s (scenarios/
% cattleCrossing.m's cow, sqrt(0.5^2+0.75^2)), while every powered-vehicle
% class moves at 1.51 m/s or faster (scenarios/villageRoad.m's
% autoRickshaw, the slowest). Found necessary empirically: without this
% floor, a validation run surfaced villageRoad's animal (velocity
% [0.3, 0.2], speed 0.36 m/s) qualifying as "predictable" during a stretch
% where its own class was lost to "unknown" - a real VRU is exactly what
% must never be relaxed. The floor sits at the mid-point of that gap.
MIN_RELAXATION_SPEED = (0.90 + 1.51) / 2; % ~1.2 m/s

persistent trackHistory
if isempty(trackHistory)
    trackHistory = containers.Map('KeyType', 'double', 'ValueType', 'any');
end

predictedTrajectories = cell(1, numel(trackedAgents));
for i = 1:numel(trackedAgents)
    agent = trackedAgents(i);
    behaviorCategory = classifyBehavior(agent, egoState.yaw);

    trackHistory = updateTrackHistory(trackHistory, agent, HISTORY_WINDOW_FRAMES);

    isIrregularClass = any(strcmp(agent.class, irregularClasses));
    isUnpredictableMotion = behaviorCategory == "crossing" || behaviorCategory == "merging";

    relaxToConstantVelocity = false;
    if agent.class == "unknown" && ~isUnpredictableMotion && behaviorCategory ~= "stopped"
        relaxToConstantVelocity = isPredictableUnknown(trackHistory(agent.id), ...
            HISTORY_WINDOW_FRAMES, MAGNITUDE_TOL, ACCEL_TOL, MIN_RELAXATION_SPEED, dt);
    end

    if behaviorCategory ~= "stopped" && (isIrregularClass || isUnpredictableMotion) && ~relaxToConstantVelocity
        predictedTrajectories{i} = irregularMotionModel(agent, horizon, dt);
    else
        predictedTrajectories{i} = constantVelocityPrediction(agent, horizon, dt);
    end
end

end

function trackHistory = updateTrackHistory(trackHistory, agent, windowFrames)
% updateTrackHistory - appends this frame's (position, velocity, timestamp)
% to agent.id's rolling buffer, capped at windowFrames entries. A
% non-increasing timestamp (a new scenario run reusing small ids in the
% same MATLAB session, or - defensively - a re-seeded id) discards any
% stale buffer and starts fresh, since objectTracking.m drops rather than
% coasts unmatched tracks, so a live id's history is otherwise guaranteed
% frame-to-frame continuous by construction.
entry = struct('position', agent.position, 'velocity', agent.velocity, 'timestamp', agent.timestamp);
if isKey(trackHistory, agent.id)
    buf = trackHistory(agent.id);
    if isempty(buf) || agent.timestamp <= buf(end).timestamp
        buf = entry;
    else
        buf(end + 1) = entry;
        if numel(buf) > windowFrames
            buf = buf(end - windowFrames + 1:end);
        end
    end
else
    buf = entry;
end
trackHistory(agent.id) = buf;
end

function isPredictable = isPredictableUnknown(hist, minFrames, magnitudeTol, accelTol, minSpeed, dt)
% isPredictableUnknown - true only when an "unknown"-class track's recent
% history gives positive evidence of steady, predictable, vehicle-like
% motion. Any missing or ambiguous evidence returns false, keeping the
% existing conservative irregularMotionModel - this function only ever
% relaxes conservatism, never adds to it. See trajectoryPrediction.m's
% header for how minFrames/magnitudeTol/accelTol/minSpeed are derived.
%
% Requires ALL of:
%   - at least minFrames of history, each consecutive frame apart (a gap
%     means the buffer is stale/incomplete, not that motion is erratic)
%   - mean speed at or above minSpeed - a steady walking-pace track cannot
%     be distinguished from a pedestrian/animal by motion alone, so it is
%     never eligible regardless of how stable it looks
%   - velocity magnitude stable within magnitudeTol across the window
%   - heading stable within a magnitudeTol-derived, speed-scaled bound (a
%     fixed-degree bound would be too tight at low speed and too loose at
%     high speed for the same underlying velocity-vector noise)
%   - no single-step acceleration beyond accelTol (a real swerve/brake,
%     not filter noise)

isPredictable = false;

if numel(hist) < minFrames
    return;
end

timestamps = [hist.timestamp];
gaps = diff(timestamps);
if any(gaps <= 0) || any(gaps > 1.5 * dt)
    return;
end

velocities = reshape([hist.velocity], 2, [])'; % Nx2, row k = [vx_k, vy_k]
speeds = vecnorm(velocities, 2, 2);

if max(speeds) - min(speeds) > magnitudeTol
    return;
end

meanVelocity = mean(velocities, 1);
meanSpeed = norm(meanVelocity);
if meanSpeed < minSpeed
    return; % walking-pace or slower - cannot rule out a vulnerable road user
end

meanHeading = atan2(meanVelocity(2), meanVelocity(1));
headings = atan2(velocities(:, 2), velocities(:, 1));
headingError = atan2(sin(headings - meanHeading), cos(headings - meanHeading));
headingTol = atan2(magnitudeTol, meanSpeed);
if max(abs(headingError)) > headingTol
    return;
end

accel = diff(velocities, 1, 1) / dt; % (N-1)x2
if any(vecnorm(accel, 2, 2) > accelTol)
    return;
end

isPredictable = true;
end
