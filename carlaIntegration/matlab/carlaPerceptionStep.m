function [fusedAgents, obs] = carlaPerceptionStep(perceptionCfg, debugDedup, previousFusedAgents)
% carlaPerceptionStep - Phase 11 main entry point: one complete
% CARLA-sensors -> unified-agents perception tick.
%
%   CARLA camera / LiDAR / radar
%            |
%     time synchronization        (carlaGetSynchronizedObservations.m)
%            |
%     coordinate transformation   (Phase 10 converters, already verified)
%            |
%     multi-sensor association    (perception/sensorFusion.m - REUSED)
%            |
%     duplicate suppression       (perception/sensorFusion.m - REUSED)
%            |
%     confidence / uncertainty    (this file)
%            |
%     unified agents              (config/createFusedAgent.m)
%
% REUSE, NOT REIMPLEMENTATION. Association and duplicate suppression are
% NOT rewritten here: perception/sensorFusion.m already implements
% camera-anchored nearest-neighbour association with distance gating, plus
% a hardened duplicate-suppression pass (class-compatibility guard,
% velocity gating, and history corroboration for ambiguous
% unknown<->known merges). It is called UNMODIFIED, and its slot
% semantics line up exactly with what the three CARLA streams provide:
%
%   sensorFusion slot 1 ("camera")  <- CARLA-grounded actor metadata:
%                                      supplies CLASS. See honesty note.
%   sensorFusion slot 2 ("lidar")   <- LiDAR clusters: supply POSITION.
%   sensorFusion slot 3 ("radar")   <- radar detections: supply VELOCITY.
%
% HONESTY NOTE (carried forward from Phase 10, do not weaken): the class
% information in slot 1 is CARLA simulator ground truth read from the
% actor list - it is NOT the output of an image-based detector. The RGB
% frame is genuinely captured and is carried through this pipeline (and
% displayed by carlaPerceptionDemo.m), but no classification is performed
% on its pixels. The correct description of slot 1 is "camera image +
% CARLA-grounded actor correspondence", never "the camera detected a
% car". Fused agents record this as source "carla_ground_truth", never
% "camera".
%
% Inputs:
%   perceptionCfg - optional, from config/carlaPerceptionConfig.m
%   debugDedup    - optional, default false; forwarded to sensorFusion.m's
%                   existing diagnostic flag
%   previousFusedAgents - optional, the fused agents returned by the
%                   PREVIOUS call. Forwarded (reduced to the base
%                   createAgent() schema) to sensorFusion.m's existing
%                   trackedAgentsPrev argument, which its duplicate-
%                   suppression pass uses to corroborate ambiguous
%                   unknown<->known merges. Without it, sensorFusion
%                   deliberately refuses those merges ("if uncertain,
%                   keep separate"), which in live CARLA leaves a large
%                   vehicle's extra radar returns as separate duplicate
%                   objects - measured: a truck produced 2 fused objects
%                   until this was threaded through. Passing [] is safe
%                   and simply restores the conservative behaviour.
% Outputs:
%   fusedAgents - struct array of config/createFusedAgent.m unified agents
%                 (empty 0x0 array when nothing usable was observed)
%   obs         - the synchronization bundle from
%                 carlaGetSynchronizedObservations.m, so callers can see
%                 exactly which sensors contributed and why

if nargin < 1 || isempty(perceptionCfg)
    perceptionCfg = carlaPerceptionConfig();
end
if nargin < 2 || isempty(debugDedup)
    debugDedup = false;
end
if nargin < 3
    previousFusedAgents = [];
end

fusedAgents = repmat(createFusedAgent(), 0, 0);

obs = carlaGetSynchronizedObservations(perceptionCfg);
if ~obs.anyUsable
    return; % nothing arrived, or everything was stale - clean empty result
end

% --- Convert only the IN-SYNC streams into the common createAgent()
% representation, using the Phase 10 converters unmodified. A stale or
% missing stream contributes nothing this tick rather than being fused
% across simulation instants.
%
% COMMON COORDINATE FRAME (the "coordinate transformation" stage of the
% Phase 11 architecture). The three streams do NOT natively share a
% frame, and fusing them without this step silently compares
% incommensurate numbers - an early version of this file did exactly
% that and could never associate a LiDAR cluster with an actor:
%
%   LiDAR clusters  - sensor-local (origin at the LiDAR mount)
%   radar detections- sensor-local (origin at the radar mount, 2m forward)
%   actor metadata  - CARLA WORLD coordinates
%
% Everything is therefore normalised to ONE frame: the EGO-RELATIVE
% project frame (origin at the ego vehicle origin, +x along the ego's
% heading, +y to the ego's left, radians, right-handed - the project's
% own handedness as established in carlaToProjectState.m). Sensor streams
% only need their mount offset added; the actor stream needs a full
% world->ego pose transform. The verified Phase 10 helpers
% (carlaCoordToProject/carlaYawToProject) are reused for handedness -
% this file does not introduce a second, conflicting convention.
carlaCfg = carlaConfig();

groundTruthAgents = repmat(createAgent(), 0, 0);
lidarAgents       = repmat(createAgent(), 0, 0);
radarAgents       = repmat(createAgent(), 0, 0);

if obs.status.lidar.usable
    lidarAgents = carlaLidarPointsToAgents(obs.lidarPoints);
    lidarAgents = shiftSensorLocalToEgoFrame(lidarAgents, carlaCfg.lidar);
    lidarAgents = filterByRange(lidarAgents, perceptionCfg.maxSensorRangeMeters);
end
if obs.status.radar.usable
    radarAgents = carlaRadarToAgents(obs.radarDetections);
    radarAgents = shiftSensorLocalToEgoFrame(radarAgents, carlaCfg.radar);
    radarAgents = filterByRange(radarAgents, perceptionCfg.maxSensorRangeMeters);
end

% The actor stream additionally needs the ego pose to leave world
% coordinates at all - without it, it cannot join the common frame and
% must not contribute (rather than contributing wrong numbers).
actorsUsable = obs.status.actors.usable && ~isempty(obs.egoState);
if actorsUsable
    groundTruthAgents = carlaActorObjectsToAgents(obs.actorObjects);
    groundTruthAgents = compensateActorMotion(groundTruthAgents, obs.referenceTime, obs.actorObjects.timestamp);
    groundTruthAgents = worldToEgoFrame(groundTruthAgents, obs.egoState);
    groundTruthAgents = filterByRange(groundTruthAgents, perceptionCfg.maxSensorRangeMeters);
elseif obs.status.actors.usable
    obs.status.actors.usable = false;
    obs.status.actors.state  = "missing"; % in sync, but unusable without an ego pose
end

if isempty(groundTruthAgents) && isempty(lidarAgents) && isempty(radarAgents)
    return; % sensors were in sync but observed nothing
end

% --- Association + duplicate suppression: perception/sensorFusion.m,
% called unmodified. It returns plain createAgent() structs.
baseFused = sensorFusion(groundTruthAgents, lidarAgents, radarAgents, debugDedup, ...
    reduceFusedAgentsToBase(previousFusedAgents));

% --- Phase 11 enrichment: everything sensorFusion.m does not do.
nextLocalId = 1;
for i = 1:numel(baseFused)
    base = baseFused(i);

    agent = createFusedAgent();
    agent.class      = base.class;
    agent.position   = base.position;
    agent.velocity   = base.velocity;
    agent.heading    = base.heading;
    agent.timestamp  = base.timestamp;
    agent.covariance = base.covariance;

    % Map sensorFusion.m's generic slot names back to the actual CARLA
    % stream that filled each slot (a faithful renaming of the slot
    % semantics documented above, not new information).
    agent.sources = mapSourcesToCarlaStreams(base.source);
    agent.source  = strjoin(agent.sources, "+");

    % Confidence from sensor agreement - the transparent ladder in
    % config/carlaPerceptionConfig.m (1 sensor -> 0.50, 2 -> 0.75,
    % 3 -> 0.90). Documented heuristic, not a probabilistic claim.
    nSources = max(1, min(3, numel(agent.sources)));
    agent.confidence = perceptionCfg.confidenceBySourceCount(nSources);

    % Uncertainty: nominal 1-sigma from the BEST contributing sensor,
    % plus the MEASURED disagreement between the contributing sensors'
    % own reported positions for this object.
    agent.uncertainty.positionStd = bestNominalStd(agent.sources, perceptionCfg);
    agent.uncertainty.sensorDisagreement = measureSensorDisagreement( ...
        base.position, agent.sources, groundTruthAgents, lidarAgents, radarAgents, perceptionCfg.identityGateMeters);

    % Synchronization provenance, so a fused agent can always be traced
    % back to the instant it belongs to.
    agent.syncFrame     = obs.referenceFrame;
    agent.syncMaxOffset = obs.maxOffset;

    % Identity + dimensions from the CARLA actor this fused object
    % corresponds to, when one can be found. See createFusedAgent.m for
    % why simulator identity is labelled distinctly from a tracking id.
    [actorId, dims] = matchToCarlaActor(base.position, groundTruthAgents, obs.actorObjects, perceptionCfg.identityGateMeters);
    if ~isempty(actorId)
        agent.id               = actorId;
        agent.simulatorActorId = actorId;
        agent.identitySource   = "carla_actor_id";
        agent.dimensions       = dims;
    else
        agent.id             = nextLocalId;
        agent.identitySource = "local_sequential";
        nextLocalId = nextLocalId + 1;
    end

    fusedAgents(end + 1) = agent; %#ok<AGROW>
end

end

% ---------------------------------------------------------------------

% reduceToBaseAgents was moved out to its own file, reduceFusedAgentsToBase.m,
% in Phase 12 so carlaTrackingStep.m could reuse it without a second,
% potentially drifting copy - identical logic, only the location changed.
%
% NOTE / known limitation (kept from the original inline version): the
% previous tick's agents are in the PREVIOUS tick's ego-relative frame.
% While the ego is stationary (as in the Phase 11 validation scene) the
% two frames coincide, but under ego motion the history positions drift
% relative to the current frame, which weakens - never falsifies - the
% corroboration check (a failed corroboration only makes dedup MORE
% conservative, i.e. it keeps objects separate). Proper frame-consistent
% history is Phase 12 tracking's job.

function agents = shiftSensorLocalToEgoFrame(agents, sensorCfg)
% Sensor-local -> ego-relative. The Phase 10 converters return positions
% relative to the SENSOR's own origin; the sensor is mounted at a known
% offset on the vehicle (config/carlaConfig.m, CARLA's attach_to
% convention: x forward, y right, z up, metres). Adding that offset puts
% the observation in the ego frame.
%
% The y offset is negated because the mount is specified in CARLA's
% left-handed convention while the converters have already produced
% project-handed (y-left) coordinates - the same single y-mirror that
% carlaCoordToProject.m applies, kept consistent rather than re-derived.
% With the default mounts (mountY = 0) the y term is zero; it is handled
% anyway so a side-mounted sensor would still be correct.
%
% Sensor yaw is assumed 0 (the Phase 10 defaults). A non-zero mountYaw
% would additionally require rotating the sensor-local points, which is
% deliberately NOT done here rather than done wrongly - see the Phase 11
% known limitations.
if isempty(agents)
    return;
end
dx = sensorCfg.mountX;
dy = -sensorCfg.mountY;
for i = 1:numel(agents)
    agents(i).position = agents(i).position + [dx, dy];
end
end

function agents = compensateActorMotion(agents, referenceTime, actorTime)
% First-order motion compensation: the actor metadata was queried at
% actorTime, but is being fused against observations from referenceTime.
% Each actor's position is advanced by its OWN reported velocity over
% that gap, so the looser actor-query tolerance (see
% carlaPerceptionConfig.m) does not silently introduce a position error
% that scales with the gap.
%
% This is a correction computed from measured velocity, not an invented
% measurement. It is exact for constant velocity and leaves only a
% second-order (acceleration) residual over a sub-second gap. A
% stationary actor is unaffected.
if isempty(agents) || isempty(referenceTime) || isempty(actorTime)
    return;
end
dt = referenceTime - actorTime;
if abs(dt) < 1e-6
    return;
end
for i = 1:numel(agents)
    agents(i).position = agents(i).position + agents(i).velocity * dt;
    agents(i).timestamp = referenceTime;
end
end

function agents = worldToEgoFrame(agents, egoState)
% CARLA world (project-handed) -> ego-relative project frame. Position is
% translated then rotated into the ego's heading; velocity and heading are
% rotated only. This is the same rotation already validated live in Phase
% 10's coordinate tests (ahead/left/right/moving).
if isempty(agents)
    return;
end
c = cos(egoState.yaw);
s = sin(egoState.yaw);
for i = 1:numel(agents)
    d = agents(i).position - [egoState.x, egoState.y];
    agents(i).position = [c * d(1) + s * d(2), -s * d(1) + c * d(2)];

    v = agents(i).velocity;
    agents(i).velocity = [c * v(1) + s * v(2), -s * v(1) + c * v(2)];

    agents(i).heading = agents(i).heading - egoState.yaw;
end
end

function agents = filterByRange(agents, maxRange)
% Sensor-intrinsic range gate only. Deliberately NOT informed by ground
% truth in any way - filtering sensor returns by "is there a known actor
% there" would make the sensors look better than they are, which is
% exactly the kind of self-deception Phase 11 must avoid.
if isempty(agents)
    return;
end
keep = false(1, numel(agents));
for i = 1:numel(agents)
    keep(i) = norm(agents(i).position) <= maxRange;
end
agents = agents(keep);
end

function sources = mapSourcesToCarlaStreams(sourceStr)
% sensorFusion.m labels its slots with the project's generic sensor names
% ("camera"/"lidar"/"radar"). Phase 11 fills those slots with specific
% CARLA streams, so the labels are mapped back to what actually produced
% the data - in particular "camera" -> "carla_ground_truth", preserving
% Phase 10's explicit distinction between a captured RGB frame and
% simulator-grounded actor metadata.
parts = strsplit(string(sourceStr), "+");
sources = string.empty(1, 0);
for i = 1:numel(parts)
    switch parts(i)
        case "camera"
            sources(end + 1) = "carla_ground_truth"; %#ok<AGROW>
        case "lidar"
            sources(end + 1) = "carla_lidar"; %#ok<AGROW>
        case "radar"
            sources(end + 1) = "carla_radar"; %#ok<AGROW>
        otherwise
            sources(end + 1) = parts(i); %#ok<AGROW>
    end
end
sources = unique(sources, 'stable');
end

function s = bestNominalStd(sources, perceptionCfg)
% Nominal 1-sigma of the most precise contributing sensor. These are
% documented engineering estimates (see carlaPerceptionConfig.m), not
% calibrated measurements - createFusedAgent.m says so on the field.
s = [];
for i = 1:numel(sources)
    name = char(sources(i));
    if isfield(perceptionCfg.nominalPositionStd, name)
        v = perceptionCfg.nominalPositionStd.(name);
        if isempty(s) || v < s
            s = v;
        end
    end
end
end

function d = measureSensorDisagreement(fusedPosition, sources, gtAgents, lidarAgents, radarAgents, gate)
% MEASURED (not estimated): the largest distance between the positions
% that the contributing sensors themselves reported for this object. For
% each contributing stream, the nearest observation to the fused position
% within `gate` is taken as that stream's own reading; the max pairwise
% distance among those readings is returned. 0 when fewer than two
% streams contributed.
positions = zeros(0, 2);
for i = 1:numel(sources)
    switch sources(i)
        case "carla_ground_truth"
            p = nearestPosition(fusedPosition, gtAgents, gate);
        case "carla_lidar"
            p = nearestPosition(fusedPosition, lidarAgents, gate);
        case "carla_radar"
            p = nearestPosition(fusedPosition, radarAgents, gate);
        otherwise
            p = [];
    end
    if ~isempty(p)
        positions(end + 1, :) = p; %#ok<AGROW>
    end
end

d = 0;
for a = 1:size(positions, 1)
    for b = (a + 1):size(positions, 1)
        d = max(d, norm(positions(a, :) - positions(b, :)));
    end
end
end

function p = nearestPosition(position, agents, gate)
p = [];
best = gate;
for i = 1:numel(agents)
    dist = norm(agents(i).position - position);
    if dist < best
        best = dist;
        p = agents(i).position;
    end
end
end

function [actorId, dims] = matchToCarlaActor(position, groundTruthAgents, actorObjects, gate)
% Re-associates a fused object back to the CARLA actor it corresponds to,
% by nearest position within `gate`, to recover simulator-grounded
% identity and bounding-box dimensions. Returns [] when no actor is close
% enough - e.g. a LiDAR cluster on a building, which legitimately has no
% actor identity and must not be given a fabricated one.
%
% Matching is done against the ALREADY-TRANSFORMED ground-truth agents
% (ego-relative frame, carrying CARLA's actor id in .id), not against the
% raw world-frame actor rows - comparing an ego-frame fused position to a
% world-frame actor position is exactly the frame error this pipeline
% exists to avoid. Dimensions are then looked up from the raw rows by
% actor id.
actorId = [];
dims = [];
if isempty(groundTruthAgents)
    return;
end

best = gate;
bestIdx = [];
for i = 1:numel(groundTruthAgents)
    dist = norm(groundTruthAgents(i).position - position);
    if dist < best
        best = dist;
        bestIdx = i;
    end
end

if isempty(bestIdx)
    return;
end
actorId = groundTruthAgents(bestIdx).id;

if isempty(actorObjects) || actorObjects.numObjects == 0
    return;
end
raw = actorObjects.raw;
row = find(raw(:, 1) == actorId, 1);
if ~isempty(row)
    % CARLA reports bounding-box half-extents; double them for full
    % [length, width, height].
    dims = 2 * [raw(row, 10), raw(row, 11), raw(row, 12)];
end
end
