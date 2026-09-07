function base = reduceFusedAgentsToBase(fusedAgents)
% reduceFusedAgentsToBase - converts config/createFusedAgent.m structs
% back to the plain config/createAgent.m schema, which is possible
% precisely because the fused schema is a strict superset (see
% createFusedAgent.m's header). This exists because MATLAB struct arrays
% require every element to share the identical field set - a base
% createAgent()-schema array pre-typed by frozen functions like
% perception/objectTracking.m (which literally does
% `trackedAgents = repmat(createAgent(), 0, 0);` internally) errors if
% assigned an element with extra fields. Every frozen function this
% project's pipeline calls (objectTracking, trajectoryPrediction,
% behaviorDecision, localPlanner, adaptivePlanner, collisionCheck,
% vehicleController) must therefore only ever receive plain createAgent()
% structs, never the extended Phase 11/12 schemas.
%
% Originally written inline in carlaPerceptionStep.m (Phase 11); moved
% here unchanged so carlaTrackingStep.m (Phase 12) can reuse the exact
% same conversion instead of a second, potentially drifting copy.
%
% Input:
%   fusedAgents - struct array of config/createFusedAgent.m agents, or []
% Output:
%   base - struct array of config/createAgent.m agents (0x0 if empty input)

base = repmat(createAgent(), 0, 0);
if isempty(fusedAgents)
    return;
end
for i = 1:numel(fusedAgents)
    f = fusedAgents(i);
    a = createAgent();
    a.id         = f.id;
    a.class      = f.class;
    a.position   = f.position;
    a.velocity   = f.velocity;
    a.heading    = f.heading;
    a.confidence = f.confidence;
    a.source     = f.source;
    a.timestamp  = f.timestamp;
    a.covariance = f.covariance;
    base(end + 1) = a; %#ok<AGROW>
end
end
