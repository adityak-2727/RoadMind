function trackedAgents = objectTracking(fusedAgents, trackedAgentsPrev, dt) %#ok<INUSD>
% objectTracking - assigns persistent IDs by nearest-neighbor association
% against the previous frame's tracks (within a fixed gating distance), and
% lightly smooths velocity for matched tracks (simple exponential smoothing,
% not a full Kalman filter - matches the "use a simple tracking approach
% first" guidance in the project brief). Unmatched detections become new
% tracks; tracks with no matching detection this frame are dropped rather
% than coasted (no "presumed still there" logic yet).
%
% Inputs:
%   fusedAgents       - struct array, this frame's fused detections
%   trackedAgentsPrev - struct array, previous frame's tracked agents (with ids)
%   dt                - [s] time since previous frame (unused directly;
%                       kept for a future velocity-from-position-delta
%                       estimate, since fused detections already carry a
%                       radar-derived velocity)
% Output:
%   trackedAgents     - struct array with stable agent.id across frames

GATING_DIST = 3.0; % [m]
VELOCITY_SMOOTHING = 0.5; % weight on this frame's velocity vs. the track's prior velocity

if isempty(trackedAgentsPrev)
    nextId = 1;
else
    nextId = max([trackedAgentsPrev.id]) + 1;
end

usedPrev = false(1, numel(trackedAgentsPrev));
trackedAgents = repmat(createAgent(), 0, 0);

for i = 1:numel(fusedAgents)
    agent = fusedAgents(i);

    bestIdx = [];
    bestDist = GATING_DIST;
    for j = 1:numel(trackedAgentsPrev)
        if usedPrev(j)
            continue;
        end
        d = norm(agent.position - trackedAgentsPrev(j).position);
        if d < bestDist
            bestDist = d;
            bestIdx = j;
        end
    end

    if ~isempty(bestIdx)
        agent.id = trackedAgentsPrev(bestIdx).id;
        agent.velocity = VELOCITY_SMOOTHING * agent.velocity + (1 - VELOCITY_SMOOTHING) * trackedAgentsPrev(bestIdx).velocity;
        usedPrev(bestIdx) = true;
    else
        agent.id = nextId;
        nextId = nextId + 1;
    end

    trackedAgents(end + 1) = agent; %#ok<AGROW>
end

end
