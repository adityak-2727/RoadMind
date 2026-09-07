function agent = createFusedAgent()
% createFusedAgent - Phase 11 unified fused-agent schema.
%
% This is a STRICT SUPERSET of config/createAgent.m, not a competing
% schema: every createAgent() field keeps its exact name, meaning and
% units, and Phase 11 only ADDS fields that the base schema has no place
% for. createAgent.m itself is untouched (it is used by the five frozen
% MATLAB scenarios, the whole synthetic perception pipeline, and the
% planner), so extending it in place would have been a frozen-file change
% for no benefit.
%
% Consequence of the superset relationship: a fused agent can always be
% reduced back to a plain createAgent() (drop the extra fields), which is
% what keeps this compatible with everything downstream that expects the
% base schema. It is NOT interchangeable in the other direction - do not
% assign a createFusedAgent() into a createAgent() struct array.
%
% INHERITED FROM createAgent() (identical meaning - see that file):
%   id, class, position, velocity, heading, confidence, source,
%   timestamp, covariance
%
% ADDED BY PHASE 11:
%   agent.acceleration      [ax, ay] [m/s^2], or [] when not observable.
%                            NOT populated by Phase 11: none of the three
%                            CARLA sensors provides acceleration directly
%                            and differentiating a single frame's velocity
%                            would be fabrication. Left [] deliberately;
%                            Phase 12 (tracking) is where this becomes
%                            estimable.
%   agent.dimensions        [length, width, height] [m] from CARLA's own
%                            actor bounding-box extents (doubled - CARLA
%                            reports half-extents), or [] when the object
%                            was seen only by LiDAR/radar (no bounding box
%                            is available from those in this phase).
%   agent.sources           string array of the contributing sensor
%                            sources, e.g. ["carla_ground_truth",
%                            "carla_lidar", "carla_radar"]. The inherited
%                            scalar `source` field keeps the project's
%                            existing '+'-joined convention
%                            ("carla_ground_truth+carla_lidar") so
%                            anything reading the base schema still works;
%                            `sources` is the structured form.
%   agent.uncertainty       struct:
%                              .positionStd        [m] 1-sigma estimate,
%                                                   from the best (lowest
%                                                   sigma) contributing
%                                                   sensor - see
%                                                   carlaPerceptionConfig.m
%                                                   (engineering estimates,
%                                                   NOT calibrated)
%                              .sensorDisagreement [m] MEASURED max
%                                                   distance between the
%                                                   contributing sensors'
%                                                   own reported positions
%                                                   for this object (0 when
%                                                   only one sensor
%                                                   contributed). This one
%                                                   is a real measurement,
%                                                   not an estimate.
%   agent.behavior          string, reserved for Phase 12's behaviour
%                            classification (prediction/classifyBehavior.m
%                            already owns that logic for the MATLAB
%                            pipeline). Phase 11 leaves it "unknown" -
%                            it does not classify behaviour.
%   agent.simulatorActorId  CARLA's own actor id when this fused object
%                            could be traced back to a specific CARLA
%                            actor, else [].
%   agent.identitySource    string explaining where `id` came from:
%                              "carla_actor_id" - simulator-grounded
%                                identity (stable across frames because
%                                CARLA guarantees it, NOT because a
%                                tracking algorithm established it)
%                              "local_sequential" - assigned by this
%                                phase for an object with no traceable
%                                CARLA actor (e.g. a LiDAR cluster on a
%                                building). Stable only within one call.
%                            This distinction is deliberate and load-
%                            bearing: Phase 11 does NOT implement
%                            tracking, and a simulator actor id must
%                            never be presented as a perception-generated
%                            track id. Phase 12 owns real tracking
%                            (perception/objectTracking.m already
%                            implements Kalman tracking + persistent ids
%                            for the synthetic pipeline and is the natural
%                            starting point).
%   agent.syncFrame         CARLA frame number of the reference sensor
%                            observation this fused agent belongs to.
%   agent.syncMaxOffset     [s] MEASURED max timestamp offset among the
%                            contributing observations - 0 when a single
%                            sensor contributed.

agent = struct( ...
    'id',                 0, ...
    'class',              "unknown", ...
    'position',           [0, 0], ...
    'velocity',           [0, 0], ...
    'acceleration',       [], ...
    'heading',            0, ...
    'dimensions',         [], ...
    'confidence',         0, ...
    'source',             "unknown", ...
    'sources',            string.empty(1, 0), ...
    'uncertainty',        struct('positionStd', [], 'sensorDisagreement', 0), ...
    'behavior',           "unknown", ...
    'timestamp',          0, ...
    'covariance',         zeros(4), ...
    'simulatorActorId',   [], ...
    'identitySource',     "local_sequential", ...
    'syncFrame',          [], ...
    'syncMaxOffset',      0 ...
);

end
