function agents = carlaActorObjectsToAgents(objects)
% carlaActorObjectsToAgents - Phase 10: converts CARLA's simulator-
% grounded nearby-actor metadata (carlaGetNearbyActorObjects.m's output)
% into config/createAgent.m-schema agents.
%
% IMPORTANT / HONESTY NOTE: this data is CARLA's own ground-truth actor
% state (true position, velocity, heading, class - read directly from
% the simulator), NOT the output of an RGB-image-based object detector.
% It is used to satisfy the "camera object/class information"
% requirement honestly, per the Phase 10 spec's explicit instruction not
% to misrepresent simulator ground truth as a trained detector's output.
% See carla_adapter.py's get_nearby_actor_objects() docstring for the
% same disclosure at the source.
%
% Columns of objects.raw (see carlaGetNearbyActorObjects.m):
%   [actorId, classCode, x, y, z, vx, vy, vz, yawDeg, extentX, extentY,
%    extentZ, distanceM] - all in CARLA's world frame.
%
% Position/velocity/heading are converted to the project frame via
% carlaCoordToProject.m / carlaYawToProject.m (the same verified
% left-handed -> right-handed mirror used by carlaToProjectState.m for
% the ego vehicle). class is looked up from objects.classNames by
% classCode (ground-truth class, not inferred).
%
% confidence is set to 1.0: this is exact simulator state, not a noisy
% detection - there is no detection uncertainty to report (unlike every
% other perception source in this project, which is either a real
% imperfect sensor or a synthetic noise model of one).
%
% Input:
%   objects - struct from carlaGetNearbyActorObjects.m (.raw Nx13,
%             .classNames, .frame, .timestamp), or a struct with
%             .numObjects == 0, or []. Each emitted agent is stamped
%             with objects.timestamp - the query's own CARLA world-
%             snapshot time.
% Output:
%   agents - struct array of createAgent()-schema agents, agent.source =
%            "carla_ground_truth" (deliberately distinct from "camera" -
%            never call this "camera" output, since it was not derived
%            from image pixels).

agents = repmat(createAgent(), 0, 0);
if isempty(objects) || objects.numObjects == 0
    return;
end

raw = objects.raw;
classNames = objects.classNames;

for i = 1:size(raw, 1)
    actorId = raw(i, 1);
    classCode = raw(i, 2);
    x = raw(i, 3); y = raw(i, 4);
    vx = raw(i, 6); vy = raw(i, 7);
    yawDeg = raw(i, 9);

    a = createAgent();
    a.id = actorId;
    classIdx = classCode + 1; % classCode is 0-based (Python side); MATLAB cell is 1-based
    if classIdx >= 1 && classIdx <= numel(classNames)
        a.class = string(classNames{classIdx});
    else
        a.class = "unknown";
    end
    [xProj, yProj] = carlaCoordToProject(x, y);
    a.position = [xProj, yProj];
    [vxProj, vyProj] = carlaCoordToProject(vx, vy);
    a.velocity = [vxProj, vyProj];
    a.heading = carlaYawToProject(yawDeg);
    a.confidence = 1.0; % exact simulator ground truth, not a noisy detection
    a.source = "carla_ground_truth";
    a.timestamp = objects.timestamp;
    agents(end + 1) = a; %#ok<AGROW>
end

end
