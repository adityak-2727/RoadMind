function trackedAgents = objectTracking(fusedAgents, trackedAgentsPrev, dt)
% objectTracking - assigns persistent IDs by nearest-neighbor association
% against the previous frame's tracks (within a fixed gating distance), then
% runs a constant-velocity Kalman filter per matched track over state
% [x, y, vx, vy], observing position only - i.e. velocity is estimated from
% consecutive position measurements the way the project brief's tracking
% diagram describes (current position + previous position -> velocity
% estimate), refined properly through the filter rather than a raw delta.
% A brand-new track is seeded with whatever velocity sensorFusion already
% supplied (radar-derived, or zero) and a large initial velocity
% uncertainty, so it starts informed but the filter still owns refining it.
% Tracks with no matching detection this frame are dropped, not coasted.
%
% Inputs:
%   fusedAgents       - struct array, this frame's fused detections
%   trackedAgentsPrev - struct array, previous frame's tracked agents (with ids)
%   dt                - [s] time since previous frame
% Output:
%   trackedAgents     - struct array with stable agent.id and Kalman-filtered
%                       position/velocity/covariance across frames

GATING_DIST = 3.0; % [m]
PROCESS_NOISE = 0.5;  % tunable: how much unmodeled acceleration to expect
MEASUREMENT_NOISE = diag([0.3, 0.3]); % assumed post-fusion position uncertainty [m^2]
INITIAL_COVARIANCE = diag([0.5, 0.5, 10, 10]); % new track: confident in position, not velocity

F = [1 0 dt 0; 0 1 0 dt; 0 0 1 0; 0 0 0 1];
Q = PROCESS_NOISE * [dt^4/4 0       dt^3/2 0; ...
                     0      dt^4/4  0      dt^3/2; ...
                     dt^3/2 0       dt^2   0; ...
                     0      dt^3/2  0      dt^2];
H = [1 0 0 0; 0 1 0 0];

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
        prevTrack = trackedAgentsPrev(bestIdx);
        usedPrev(bestIdx) = true;

        x_prev = [prevTrack.position(1); prevTrack.position(2); prevTrack.velocity(1); prevTrack.velocity(2)];
        P_prev = prevTrack.covariance;

        x_pred = F * x_prev;
        P_pred = F * P_prev * F' + Q;

        z = [agent.position(1); agent.position(2)];
        innovation = z - H * x_pred;
        S = H * P_pred * H' + MEASUREMENT_NOISE;
        K = P_pred * H' / S;
        x_updated = x_pred + K * innovation;
        P_updated = (eye(4) - K * H) * P_pred;

        agent.position = [x_updated(1), x_updated(2)];
        agent.velocity = [x_updated(3), x_updated(4)];
        agent.covariance = P_updated;
        agent.id = prevTrack.id;
    else
        agent.covariance = INITIAL_COVARIANCE;
        agent.id = nextId;
        nextId = nextId + 1;
    end

    trackedAgents(end + 1) = agent; %#ok<AGROW>
end

end
