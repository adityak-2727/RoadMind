function [trackedAgents, trackInfo, trackerState] = carlaTrackingStep(fusedAgents, trackerState, dt, trackingCfg)
% carlaTrackingStep - Phase 12: wraps the frozen perception/objectTracking.m
% with the missed-observation coasting/lifecycle bookkeeping it does not
% provide, WITHOUT modifying it. objectTracking.m is called exactly as
% main.m already calls it (same three arguments, same base createAgent()
% schema in and out) - this file only adds a layer immediately around
% that call.
%
% WHY A WRAPPER INSTEAD OF EDITING objectTracking.m: it is frozen, and by
% its own documented design it DROPS (never coasts) a track with no
% matching detection this frame. Phase 12 requires the opposite - coast
% through a temporary gap, drop only after a configured tolerance. Rather
% than redesigning the frozen file, this wrapper re-injects a Kalman-
% PREDICTED (not measurement-updated) version of a just-dropped track
% back into the NEXT call's trackedAgentsPrev, giving objectTracking.m's
% own nearest-neighbour gating (GATING_DIST=3.0m, unchanged) the chance
% to re-associate a returning real detection to the same id.
%
% The predict-only step (x_pred = F*x, P_pred = F*P*F' + Q) necessarily
% duplicates objectTracking.m's own F/Q construction (PROCESS_NOISE=0.5,
% same dt convention) - objectTracking.m does not expose a standalone
% predict primitive, and adding one would itself be a change to the
% frozen file. The values are copied verbatim from objectTracking.m's own
% header/body so this cannot silently drift without both files being
% inspected side by side.
%
% IDENTITY: objectTracking.m assigns its own sequential track id
% (persisted via trackedAgentsPrev), completely independent of whatever
% id a fused agent arrived with (Phase 11's fused agents often already
% carry a CARLA actor id as their base .id - see createFusedAgent.m). The
% output trackedAgents(i).id is therefore the TRACKER's own identity, per
% the Phase 12 spec's explicit requirement to keep these distinct. The
% CARLA actor id (when one exists) is recovered separately in trackInfo
% by re-associating each output track to the nearest raw fused
% observation within identityReassociationGateMeters - a best-effort,
% documented re-derivation, since objectTracking.m's internal association
% decisions are not exposed to callers.
%
% Inputs:
%   fusedAgents  - struct array of config/createFusedAgent.m agents (this
%                  tick's real observations from carlaPerceptionStep.m),
%                  or [] / empty when nothing was observed this tick.
%   trackerState - the previous call's returned trackerState, or []/empty
%                  on the first call (initializes fresh).
%   dt           - [s] time since the previous call.
%   trackingCfg  - optional, from config/carlaTrackingConfig.m.
% Outputs:
%   trackedAgents - base config/createAgent.m struct array (real +
%                   coasted tracks), safe to pass UNMODIFIED into
%                   trajectoryPrediction.m / behaviorDecision.m /
%                   localPlanner.m / adaptivePlanner.m / collisionCheck.m
%                   / vehicleController.m exactly as main.m already does.
%   trackInfo     - struct array, SAME ORDER as trackedAgents, one entry
%                   each: .trackId, .carlaActorId ([] if none),
%                   .age, .missedCount, .state ("new"|"confirmed"|"coasted")
%   trackerState  - opaque state to pass into the next call; also carries
%                   trackerState.diagnostics.lostTrackIds /
%                   .coastedTrackIds for this tick (metrics use).

if nargin < 4 || isempty(trackingCfg)
    trackingCfg = carlaTrackingConfig();
end

if isempty(trackerState) || ~isstruct(trackerState)
    trackerState = struct( ...
        'trackedAgentsPrev', repmat(createAgent(), 0, 0), ...
        'meta', containers.Map('KeyType', 'double', 'ValueType', 'any'), ...
        'nowTimestamp', 0);
end

baseFused = reduceFusedAgentsToBase(fusedAgents);
now = trackerState.nowTimestamp + dt;

% --- The one frozen call, unmodified: identical signature/semantics to
% main.m's `trackedAgents = objectTracking(fusedAgents, trackedAgentsPrev, dt);`
rawTracked = objectTracking(baseFused, trackerState.trackedAgentsPrev, dt);

prevIds = idsOf(trackerState.trackedAgentsPrev);
rawIds  = idsOf(rawTracked);
missingIds = setdiff(prevIds, rawIds);

lostTrackIds = [];
coastedTrackIds = [];

for k = 1:numel(missingIds)
    mid = missingIds(k);
    if ~isKey(trackerState.meta, mid)
        continue; % defensive: should not happen, every prevId has a meta entry
    end
    m = trackerState.meta(mid);
    withinTickTolerance = (m.missedCount + 1) <= trackingCfg.missedObservationToleranceTicks;
    withinTimeTolerance = (now - m.lastRealTimestamp) <= trackingCfg.missedObservationTimeoutSeconds;

    if withinTickTolerance && withinTimeTolerance
        prevAgent = findById(trackerState.trackedAgentsPrev, mid);
        coasted = kalmanPredictOnly(prevAgent, dt);
        rawTracked(end + 1) = coasted; %#ok<AGROW>

        m.missedCount = m.missedCount + 1;
        m.age = m.age + 1;
        m.state = "coasted";
        trackerState.meta(mid) = m;
        coastedTrackIds(end + 1) = mid; %#ok<AGROW>
    else
        remove(trackerState.meta, mid);
        lostTrackIds(end + 1) = mid; %#ok<AGROW>
    end
end

% --- Update/create meta for every id now present in rawTracked (both
% freshly-associated/new tracks from objectTracking.m, and the coasted
% entries just re-injected above).
trackInfo = repmat(struct('trackId', 0, 'carlaActorId', [], 'age', 0, 'missedCount', 0, 'state', "new"), 0, 0);

for i = 1:numel(rawTracked)
    tid = rawTracked(i).id;

    if any(coastedTrackIds == tid)
        m = trackerState.meta(tid); % already updated above
    else
        carlaActorId = reassociateCarlaActorId(rawTracked(i), fusedAgents, baseFused, trackingCfg.identityReassociationGateMeters);
        if isKey(trackerState.meta, tid)
            m = trackerState.meta(tid);
            m.age = m.age + 1;
            m.missedCount = 0;
            m.state = "confirmed";
            if ~isempty(carlaActorId)
                m.carlaActorId = carlaActorId; % refresh only when this tick actually saw the actor
            end
        else
            m = struct('age', 1, 'missedCount', 0, 'state', "new", 'carlaActorId', carlaActorId);
        end
        m.lastRealTimestamp = rawTracked(i).timestamp;
        trackerState.meta(tid) = m;
    end

    trackInfo(end + 1) = struct('trackId', tid, 'carlaActorId', m.carlaActorId, ...
        'age', m.age, 'missedCount', m.missedCount, 'state', m.state); %#ok<AGROW>
end

trackerState.trackedAgentsPrev = rawTracked;
trackerState.nowTimestamp = now;
trackerState.diagnostics = struct('lostTrackIds', lostTrackIds, 'coastedTrackIds', coastedTrackIds);

trackedAgents = rawTracked;

end

% ---------------------------------------------------------------------

function ids = idsOf(agents)
if isempty(agents)
    ids = [];
else
    ids = [agents.id];
end
end

function agent = findById(agents, id)
agent = [];
for i = 1:numel(agents)
    if agents(i).id == id
        agent = agents(i);
        return;
    end
end
end

function coasted = kalmanPredictOnly(prevAgent, dt)
% Predict-only Kalman step (no measurement update), for one tick of
% coasting through a missed observation. F/Q duplicated verbatim from
% perception/objectTracking.m's own construction (PROCESS_NOISE=0.5) -
% see this file's header for why a copy, not a shared call, is used.
PROCESS_NOISE = 0.5;
F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Q = PROCESS_NOISE * [dt^4/4 0       dt^3/2 0; ...
                     0      dt^4/4  0      dt^3/2; ...
                     dt^3/2 0       dt^2   0; ...
                     0      dt^3/2  0      dt^2];

x_prev = [prevAgent.position(1); prevAgent.position(2); prevAgent.velocity(1); prevAgent.velocity(2)];
P_prev = prevAgent.covariance;
if isempty(P_prev) || any(size(P_prev) ~= [4, 4])
    P_prev = diag([0.5, 0.5, 10, 10]); % matches objectTracking.m's own INITIAL_COVARIANCE
end

x_pred = F * x_prev;
P_pred = F * P_prev * F' + Q;

coasted = prevAgent;
coasted.position = [x_pred(1), x_pred(2)];
coasted.velocity = [x_pred(3), x_pred(4)];
coasted.covariance = P_pred;
coasted.timestamp = prevAgent.timestamp + dt;
coasted.confidence = max(0.1, prevAgent.confidence * 0.85); % honest decay while unconfirmed - never silently held at full confidence
end

function actorId = reassociateCarlaActorId(trackedAgent, fusedAgents, baseFused, gate)
% Re-derives which raw fused observation (and therefore CARLA actor id,
% if any) this tracker output corresponds to, by nearest position within
% `gate`. Needed because objectTracking.m's internal nearest-neighbour
% association is not exposed to callers. Returns [] when nothing is close
% enough (e.g. a track that only ever came from LiDAR/radar and has no
% CARLA actor correspondence).
actorId = [];
if isempty(fusedAgents) || isempty(baseFused)
    return;
end
best = gate;
bestIdx = [];
for k = 1:numel(baseFused)
    d = norm(baseFused(k).position - trackedAgent.position);
    if d < best
        best = d;
        bestIdx = k;
    end
end
if ~isempty(bestIdx)
    actorId = fusedAgents(bestIdx).simulatorActorId;
end
end
