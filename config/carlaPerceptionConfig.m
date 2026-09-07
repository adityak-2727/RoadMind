function cfg = carlaPerceptionConfig()
% carlaPerceptionConfig - Phase 11 (CARLA -> MATLAB perception + sensor
% fusion) tunables. Same struct-config style as sensorConfig.m /
% carlaConfig.m. This is the single place Phase 11's thresholds live.
%
% Every value below that could have been guessed was instead DERIVED FROM
% MEASURED CARLA BEHAVIOUR - see the derivations inline. Phase 11's spec
% explicitly required not choosing a synchronization tolerance arbitrarily.
%
% ---------------------------------------------------------------------
% syncToleranceSeconds derivation (MEASURED, not assumed)
% ---------------------------------------------------------------------
% Measured live against CARLA 0.9.16 (Town10HD_Opt, headless), 60
% complete camera+LiDAR+radar polls, with the Phase 10 default sensor
% configuration (config/carlaConfig.m):
%
%   Timestamp spread across the 3 sensors, per poll:
%       mean=0.0319s  median=0.0339s  p90=0.0372s  p99=0.0969s  max=0.1025s
%   Per-sensor update interval (consecutive distinct frames):
%       camera ~0.064s, lidar ~0.063s, radar ~0.063s  (i.e. ~16 Hz)
%   Pairwise |dt|:
%       lidar<->radar  mean=0.0010s  max=0.0128s   (these tick together)
%       camera<->lidar mean=0.0308s  max=0.1025s
%       camera<->radar mean=0.0319s  max=0.1025s
%
% Chosen: 0.075 s. Rationale:
%   - It is ~1 sensor update interval (0.063s). With genuinely
%     asynchronous sensors you cannot resolve "the same instant" more
%     finely than one update period, so a tolerance below that would
%     reject observations that are in fact as simultaneous as this
%     sensor set can produce.
%   - It sits ABOVE the measured p90 spread (0.0372s), so the large
%     majority of normal polls pass without being needlessly rejected.
%   - It sits BELOW the measured p99/max (0.097-0.103s), so a genuinely
%     anomalous lag is still flagged as stale rather than silently fused.
%   - Physical cost bound: at 15 m/s relative speed, 0.075s corresponds
%     to ~1.1 m of position disagreement - comfortably inside
%     perception/sensorFusion.m's 2.5 m association gate, so a
%     within-tolerance mismatch cannot by itself break association.
%
% ---------------------------------------------------------------------
% staleTimeoutSeconds
% ---------------------------------------------------------------------
% Separate, deliberately longer bound for "this sensor has stopped
% producing at all" (as opposed to "this frame is slightly offset").
% 0.5s is ~8 missed update intervals - well beyond any spread observed
% above, so it only fires on a real stall, never on normal jitter. Phase
% 11 requires that stale data is never silently reused indefinitely.
%
% ---------------------------------------------------------------------
% Association / fusion gates
% ---------------------------------------------------------------------
% Association and duplicate suppression themselves are NOT reimplemented
% here - Phase 11 reuses perception/sensorFusion.m unmodified, which owns
% its own GATING_DIST (2.5m) and DEDUP_DISTANCE (3.5m) with their own
% documented derivations. The gates below apply only to the Phase 11
% steps that sensorFusion.m does not perform: recovering simulator-
% grounded identity after fusion, and bounding raw sensor input.
%
% Schema:
%   cfg.syncToleranceSeconds   [s]  max timestamp offset from the newest
%                                    sensor for an observation to count as
%                                    the same simulation instant
%   cfg.staleTimeoutSeconds    [s]  beyond this, a sensor is reported
%                                    missing rather than reused
%   cfg.identityGateMeters     [m]  gate for re-associating a fused agent
%                                    back to the CARLA actor it came from
%   cfg.maxSensorRangeMeters   [m]  raw LiDAR/radar observations beyond
%                                    this are dropped before fusion
%   cfg.minLidarClusterPoints  [-]  reserved; see note below
%   cfg.confidenceBySourceCount [-] transparent confidence ladder

% ---------------------------------------------------------------------
% actorQueryToleranceSeconds - a SEPARATE, deliberately looser tolerance
% for the CARLA-grounded actor-metadata stream, and the reason it differs
% ---------------------------------------------------------------------
% The three real sensors deliver asynchronously and their mutual spread
% is a property of CARLA (~0.03s, measured above). The actor metadata is
% not a sensor at all - it is an on-demand world query whose timestamp is
% whenever MATLAB asked. Its offset from the sensor instant is therefore
% dominated by how long MATLAB spends pulling data across the py.*
% boundary (measured: ~0.28-0.40s for a 640x480 RGB frame + LiDAR sweep +
% radar sweep + actor list), NOT by anything CARLA does.
%
% Holding that stream to the 0.075s sensor tolerance made it fail on
% every tick, which stripped all class and identity information from the
% fused output. Holding the SENSORS to a ~0.4s tolerance instead would
% have been the wrong fix - it would silently permit fusing genuinely
% unrelated sensor frames, which Phase 11 explicitly forbids.
%
% So the two are gated separately, and the actor stream is additionally
% MOTION-COMPENSATED to the reference instant using each actor's own
% reported velocity (see carlaPerceptionStep.m) - a first-order
% correction that makes the looser window honest rather than hand-waved.
% Residual error after compensation is second-order (acceleration over
% <0.4s), and is reported per agent in agent.syncMaxOffset.
%
% 0.6s is set above the worst measured acquisition cost (~0.40s) with
% margin, and below staleTimeoutSeconds' meaning of "this stream has
% actually stopped".

cfg = struct( ...
    'syncToleranceSeconds',       0.075, ...
    'actorQueryToleranceSeconds', 0.6, ...
    'staleTimeoutSeconds',        1.0, ...
    'identityGateMeters',         3.0, ...
    'maxSensorRangeMeters',       60.0, ...
    'minLidarClusterPoints',      1 ...
);

% Confidence ladder (Phase 11 "confidence should reflect sensor
% evidence"). Deliberately a simple, transparent, documented mapping from
% "how many independent sensors contributed to this fused object" to a
% confidence value - NOT a probabilistic guarantee, and not fitted to any
% data. Index = number of contributing sensors (1, 2 or 3).
cfg.confidenceBySourceCount = [0.50, 0.75, 0.90];

% Nominal per-sensor position uncertainty (1-sigma, metres), used only to
% report an honest uncertainty estimate on the fused output. These mirror
% the ORDERING already documented in config/sensorConfig.m for the
% synthetic pipeline (LiDAR precise, radar poor angular resolution) and
% are order-of-magnitude engineering estimates for the CARLA sensors, not
% calibrated measurements - labelled as such in createFusedAgent.m.
cfg.nominalPositionStd = struct( ...
    'carla_ground_truth', 0.0, ...  % exact simulator state (see honesty note in carlaActorObjectsToAgents.m)
    'carla_lidar',        0.15, ... % cluster-centroid spread
    'carla_radar',        1.20 ...  % radar angular resolution dominates
);

end
