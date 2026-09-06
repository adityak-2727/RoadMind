function fusedAgents = sensorFusion(cameraAgents, lidarAgents, radarAgents, debugDedup, trackedAgentsPrev)
% sensorFusion - associates detections across modalities by spatial nearest-
% neighbor (within a fixed gating distance) and merges them per each
% sensor's simulated strength: camera contributes class, lidar contributes
% position, radar contributes velocity. Detections seen by only one sensor
% are still included using whatever fields that sensor provided, rather
% than being dropped. A conservative post-fusion deduplication pass then
% catches the case where the SAME physical object produced two separate
% fused entries because one sensor's detection fell just outside the
% camera-anchored gate above (see the local function deduplicateFusedAgents
% for the full rationale - this is a distinct, additional pass, not a
% change to the association logic above it).
%
% Inputs:
%   cameraAgents, lidarAgents, radarAgents - struct arrays from each modality
%   debugDedup - optional, default false. When true, prints per-merge and
%                per-rejected-close-pair diagnostics to the console. Kept
%                as an optional trailing argument (not a new required
%                parameter) so every existing 3-argument call site is
%                unaffected.
%   trackedAgentsPrev - optional, default empty. Previous frame's tracked
%                agents (the same struct array main.m already threads into
%                objectTracking.m) - used only as corroborating evidence
%                when this frame's own velocity data can't establish
%                whether an unknown-class detection and a known-class
%                detection are the same physical object; see
%                deduplicateFusedAgents' doc comment for why this was
%                added and what it fixes. Passing [] disables this check
%                (falls back to rejecting the ambiguous merge, per the
%                safety principle "if uncertain, keep separate").
% Output:
%   fusedAgents - struct array of merged agents. agent.source records
%                 which sensor(s) contributed, e.g. "camera+lidar" or
%                 "radar" alone - not always the literal string "fused"
%                 anymore; see config/createAgent.m's updated schema note.

if nargin < 4 || isempty(debugDedup)
    debugDedup = false;
end
if nargin < 5
    trackedAgentsPrev = repmat(createAgent(), 0, 0);
end

GATING_DIST = 2.5; % [m] max distance to consider two detections the same object

usedLidar = false(1, numel(lidarAgents));
usedRadar = false(1, numel(radarAgents));
fusedAgents = repmat(createAgent(), 0, 0);

for i = 1:numel(cameraAgents)
    cam = cameraAgents(i);

    lidarIdx = nearestUnused(cam.position, lidarAgents, usedLidar, GATING_DIST);
    radarIdx = nearestUnused(cam.position, radarAgents, usedRadar, GATING_DIST);

    fused = createAgent();
    fused.class = cam.class; % camera -> class

    contributingSources = "camera";

    if ~isempty(lidarIdx)
        fused.position = lidarAgents(lidarIdx).position; % lidar -> position
        usedLidar(lidarIdx) = true;
        contributingSources = contributingSources + "+lidar";
    else
        fused.position = cam.position;
    end

    if ~isempty(radarIdx)
        fused.velocity = radarAgents(radarIdx).velocity; % radar -> velocity
        usedRadar(radarIdx) = true;
        contributingSources = contributingSources + "+radar";
    else
        fused.velocity = cam.velocity;
    end

    fused.heading = atan2(fused.velocity(2), fused.velocity(1));
    fused.confidence = cam.confidence;
    fused.source = contributingSources;
    fused.timestamp = cam.timestamp;

    fusedAgents(end + 1) = fused; %#ok<AGROW>
end

% Detections only lidar or only radar saw are still real objects - include
% them rather than silently dropping information a single-sensor view had.
for i = 1:numel(lidarAgents)
    if ~usedLidar(i)
        a = lidarAgents(i);
        a.source = "lidar";
        fusedAgents(end + 1) = a; %#ok<AGROW>
    end
end
for i = 1:numel(radarAgents)
    if ~usedRadar(i)
        a = radarAgents(i);
        a.source = "radar";
        fusedAgents(end + 1) = a; %#ok<AGROW>
    end
end

fusedAgents = deduplicateFusedAgents(fusedAgents, debugDedup, trackedAgentsPrev);

end

function idx = nearestUnused(position, candidates, used, gatingDist)
% Returns the index of the closest not-yet-claimed candidate within
% gatingDist, or [] if none qualifies.
idx = [];
bestDist = gatingDist;
for k = 1:numel(candidates)
    if used(k)
        continue;
    end
    d = norm(candidates(k).position - position);
    if d < bestDist
        bestDist = d;
        idx = k;
    end
end
end

function fusedAgents = deduplicateFusedAgents(fusedAgents, debugDedup, trackedAgentsPrev)
% deduplicateFusedAgents - conservative post-fusion pass that catches one
% physical object producing two separate fusedAgents entries because one
% sensor's raw detection fell just outside GATING_DIST from the others (the
% confirmed villageRoad t=2.00s case: a radar detection missed the 2.5m
% camera-anchor gate by 0.235m and survived as a second "unknown"-class
% entry for the same parked car already represented, correctly, as
% class=car by the camera+lidar match). This is deliberately a separate,
% conservative pass, not a change to GATING_DIST or the association loop
% above - enlarging that gate directly would risk merging two genuinely
% close but distinct objects at the sensor-association stage, before any
% class/velocity corroboration is available to guard against it.
%
% DEDUP_DISTANCE derivation: the dominant noise source in the confirmed
% failure is the gap between camera (positionNoiseStd=0.4m) and radar
% (positionNoiseStd=1.2m) readings of the same point. Their combined
% (root-sum-square) position-noise std is sqrt(0.4^2+1.2^2)=1.265m; the 2D
% radial 95%/99% confidence bounds for that combined noise are
% 1.265*sqrt(chi2inv(0.95,2))=3.10m and 1.265*sqrt(chi2inv(0.99,2))=3.84m.
% DEDUP_DISTANCE=3.5m sits between those two - comfortably above the
% confirmed 2.735m failure case, comfortably below the closest real
% distinct-object separation observed in any of the five scenarios.
%
% Merges only when ALL of the following hold:
%   - position distance < DEDUP_DISTANCE
%   - class compatible: NEVER merges two different confident (non-"unknown")
%     classes, regardless of distance - this is the primary false-merge guard
%   - velocity consistent - see the two-tier check below
%
% HARDENING (this revision): validating the first version against
% urbanIntersection found a genuine false merge - a car's stray radar echo
% (velocity ~(5.17,-0.11), i.e. clearly the car) merged into a motorcycle's
% fused entry at 3.40m, because the motorcycle's OWN fused velocity that
% tick was exactly [0,0] (no radar had matched its camera detection), which
% bypassed the velocity check entirely (it only ran when BOTH sides exceeded
% MEANINGFUL_VELOCITY). The true motorcycle-duplicate case, for comparison,
% merged at the practically-identical distance of 3.39m - so tightening
% DEDUP_DISTANCE cannot separate these two cases; something other than
% distance has to.
%
% That something is history. For every unknown<->known merge where the
% within-frame check above was inconclusive (at least one side's current
% velocity is uninformative), the known side's position is looked up in
% trackedAgentsPrev (gated at 3.0m, matching objectTracking.m's own
% convention) for a prior velocity estimate to corroborate against. Checked
% against real measured data before writing this: the true motorcycle
% duplicate's candidate velocity (4.17,0.16) differs from the prior
% motorcycle track's velocity (3.99,0.55) by only 0.43 m/s (allowed); the
% false merge's candidate velocity (5.17,-0.11) differs from the prior
% motorcycle track's velocity (-4.01,0.06) by 9.18 m/s (correctly
% rejected). The confirmed car/pushcart static-duplicate cases are also
% covered: when BOTH current-frame velocities are near-zero, history is
% still consulted, and a prior track that is ALSO near-zero corroborates a
% genuinely static object; no prior track at all is treated as ambiguous
% and the merge is rejected for this tick (it can still merge once history
% exists) - "if uncertain whether two nearby objects are the same object,
% keep them separate" is applied literally here, not just in spirit.
% This is scoped to the unknown<->known direction only (the "known-class-
% priority" reason) - the confirmed false merge was of that shape, and nothing
% in validation showed the same<->same-class ("higher-confidence") path
% needed the same treatment.
%
% Diagnostics (kept, not fabricated, gated behind debugDedup so normal runs
% are not spammed): every merge, every rejected-but-close pair, and every
% ambiguous-evidence rejection is logged with index, classes, sources,
% distance, and reason. A summary line reports input/output counts, merge
% count, and rejected-close-pair count - an invariant check surfaced as
% information only; nothing is auto-deleted beyond what the merge rule
% itself decided.

DEDUP_DISTANCE = 3.5; % [m] - see derivation above
VELOCITY_INCONSISTENCY_THRESHOLD = 2.0; % [m/s]
MEANINGFUL_VELOCITY = 0.3; % [m/s] - below this, velocity is treated as uninformative (likely an unpopulated default)
HISTORY_GATING_DIST = 3.0; % [m] - matches objectTracking.m's own GATING_DIST

n = numel(fusedAgents);
if n < 2
    return;
end

pairDistances = [];
for i = 1:n
    for j = (i + 1):n
        d = norm(fusedAgents(i).position - fusedAgents(j).position);
        if d < DEDUP_DISTANCE
            pairDistances(end + 1, :) = [d, i, j]; %#ok<AGROW>
        end
    end
end

if isempty(pairDistances)
    return;
end

pairDistances = sortrows(pairDistances, 1); % closest pairs merge first

removed = false(1, n);
mergeCount = 0;
rejectedClosePairCount = 0;

for p = 1:size(pairDistances, 1)
    d = pairDistances(p, 1);
    i = pairDistances(p, 2);
    j = pairDistances(p, 3);

    if removed(i) || removed(j)
        continue; % one side already absorbed into an earlier, closer merge this pass
    end

    classI = fusedAgents(i).class;
    classJ = fusedAgents(j).class;
    iUnknown = (classI == "unknown");
    jUnknown = (classJ == "unknown");

    if ~iUnknown && ~jUnknown && classI ~= classJ
        rejectedClosePairCount = rejectedClosePairCount + 1;
        if debugDedup
            fprintf('[dedup] NOT merged: index %d (%s) and index %d (%s), distance=%.2fm - different confident classes\n', ...
                i, classI, j, classJ, d);
        end
        continue;
    end

    velI = fusedAgents(i).velocity;
    velJ = fusedAgents(j).velocity;
    bothVelocitiesMeaningful = norm(velI) > MEANINGFUL_VELOCITY && norm(velJ) > MEANINGFUL_VELOCITY;

    if bothVelocitiesMeaningful
        velDiff = norm(velI - velJ);
        if velDiff > VELOCITY_INCONSISTENCY_THRESHOLD
            rejectedClosePairCount = rejectedClosePairCount + 1;
            if debugDedup
                fprintf('[dedup] NOT merged: index %d and index %d, distance=%.2fm - velocity mismatch %.2fm/s\n', ...
                    i, j, d, velDiff);
            end
            continue;
        end
    end

    isUnknownKnownPair = iUnknown ~= jUnknown;

    if isUnknownKnownPair && ~bothVelocitiesMeaningful
        % The within-frame check above couldn't rule this out (at least one
        % side's velocity is uninformative) - require history corroboration
        % before allowing an unknown detection to be absorbed into a known
        % track. See the function-level doc comment for why this exists.
        if iUnknown
            unknownVel = velI; knownIdx = j;
        else
            unknownVel = velJ; knownIdx = i;
        end

        priorVel = [];
        bestPriorDist = HISTORY_GATING_DIST;
        for k = 1:numel(trackedAgentsPrev)
            pd = norm(trackedAgentsPrev(k).position - fusedAgents(knownIdx).position);
            if pd < bestPriorDist
                bestPriorDist = pd;
                priorVel = trackedAgentsPrev(k).velocity;
            end
        end

        if isempty(priorVel)
            rejectedClosePairCount = rejectedClosePairCount + 1;
            if debugDedup
                fprintf('[dedup] NOT merged: index %d and index %d, distance=%.2fm - ambiguous (no velocity evidence this frame, no prior track to corroborate)\n', ...
                    i, j, d);
            end
            continue;
        end

        if norm(unknownVel) > MEANINGFUL_VELOCITY || norm(priorVel) > MEANINGFUL_VELOCITY
            historyDiff = norm(unknownVel - priorVel);
            if historyDiff > VELOCITY_INCONSISTENCY_THRESHOLD
                rejectedClosePairCount = rejectedClosePairCount + 1;
                if debugDedup
                    fprintf('[dedup] NOT merged: index %d and index %d, distance=%.2fm - candidate velocity (%.2f,%.2f) inconsistent with prior track velocity (%.2f,%.2f), diff=%.2fm/s\n', ...
                        i, j, d, unknownVel(1), unknownVel(2), priorVel(1), priorVel(2), historyDiff);
                end
                continue;
            end
        end
        % else: both candidate and prior velocity are near-zero - consistent
        % with a genuinely static/slow object, allow the merge to proceed.
    end

    if ~iUnknown && jUnknown
        survivorIdx = i; mergedIdx = j; reason = "known-class-priority";
    elseif iUnknown && ~jUnknown
        survivorIdx = j; mergedIdx = i; reason = "known-class-priority";
    elseif fusedAgents(i).confidence >= fusedAgents(j).confidence
        survivorIdx = i; mergedIdx = j; reason = "higher-confidence";
    else
        survivorIdx = j; mergedIdx = i; reason = "higher-confidence";
    end

    survivor = fusedAgents(survivorIdx);
    mergedAway = fusedAgents(mergedIdx);

    % Adopt the merged-away entry's real velocity if the survivor's own is
    % just an unpopulated default (camera/lidar always report [0,0]) - a
    % measured data-quality improvement, not a guess, since it only fires
    % when the merged-away entry actually carries real (radar-derived) motion.
    if norm(survivor.velocity) < 1e-6 && norm(mergedAway.velocity) > MEANINGFUL_VELOCITY
        survivor.velocity = mergedAway.velocity;
        survivor.heading = atan2(mergedAway.velocity(2), mergedAway.velocity(1));
    end

    combinedSources = unique([strsplit(survivor.source, "+"), strsplit(mergedAway.source, "+")], 'stable');
    survivor.source = strjoin(combinedSources, "+");

    if debugDedup
        fprintf('[dedup] MERGED: index %d (%s, %s) into index %d (%s, %s), distance=%.2fm, reason=%s\n', ...
            mergedIdx, mergedAway.class, mergedAway.source, survivorIdx, survivor.class, survivor.source, d, reason);
    end

    fusedAgents(survivorIdx) = survivor;
    removed(mergedIdx) = true;
    mergeCount = mergeCount + 1;
end

fusedAgents = fusedAgents(~removed);

if debugDedup
    fprintf('[dedup] summary: %d input agents -> %d output agents, %d merges, %d rejected close pairs\n', ...
        n, numel(fusedAgents), mergeCount, rejectedClosePairCount);
end

end
