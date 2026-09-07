function cfg = carlaTrackingConfig()
% carlaTrackingConfig - Phase 12 (Tracking + Trajectory Prediction on
% CARLA) tunables. Same struct-config style as carlaPerceptionConfig.m.
%
% ---------------------------------------------------------------------
% Why this file exists instead of changing perception/objectTracking.m
% ---------------------------------------------------------------------
% objectTracking.m is frozen and, by its own documented design, DROPS
% (not coasts) any track with no matching detection this frame - see its
% own header comment: "Tracks with no matching detection this frame are
% dropped, not coasted." Phase 12 requires the opposite: the tracker must
% "predict through the temporary gap and retain the track for the
% configured missed-observation tolerance", but must not become
% immortal. Rather than redesigning the frozen file, carlaTrackingStep.m
% wraps it: objectTracking.m is called exactly as main.m already calls
% it (unmodified), and a coasting layer around that call re-injects a
% dead-reckoned version of a just-dropped track back into the next
% call's trackedAgentsPrev, giving objectTracking.m's own nearest-
% neighbour gating (GATING_DIST=3.0m, unchanged) a chance to re-associate
% a returning real observation to the SAME id. This file holds that
% wrapper's tunables only - it does not duplicate or override anything
% inside objectTracking.m itself.
%
% Schema:
%   cfg.missedObservationToleranceTicks   [-] consecutive ticks a track
%                                          may be coasted (no real
%                                          detection matched) before it
%                                          is dropped for good
%   cfg.missedObservationTimeoutSeconds   [s] a parallel, tick-rate-
%                                          independent bound - whichever
%                                          of the two limits is reached
%                                          first ends the coast. Needed
%                                          because a live CARLA polling
%                                          loop's tick rate is not fixed
%                                          the way main.m's simCfg.dt is.
%   cfg.identityReassociationGateMeters   [m] gate used to re-derive
%                                          which raw fused observation
%                                          (and therefore which CARLA
%                                          actor id, if any) corresponds
%                                          to each of objectTracking.m's
%                                          OUTPUT tracks - see
%                                          carlaTrackingStep.m for why
%                                          this re-derivation is needed
%                                          (objectTracking.m's own
%                                          internal association is not
%                                          exposed to callers)
%
% ---------------------------------------------------------------------
% missedObservationToleranceTicks / TimeoutSeconds derivation
% ---------------------------------------------------------------------
% Phase 10/11 measured live CARLA sensor updates at ~16 Hz (~0.063s per
% update) and a full carlaPerceptionStep() poll (all 3 sensors + actor
% query) costing up to ~0.4s of wall-clock time when run from MATLAB.
% A "temporary missed observation" demo (Phase 12 spec) needs a gap long
% enough to be a clearly visible, deliberate interruption on a live demo
% (not indistinguishable from ordinary frame-to-frame jitter), but short
% enough that the tracker is not effectively immortal. 5 ticks at a
% typical closed-loop demo rate of ~2-3 Hz (carlaClosedLoopStep.m's own
% measured pacing, dominated by the same sensor-transfer cost) is
% ~1.5-2.5s of real coasting - long enough to be an obvious, deliberate
% gap in a jury demo, short enough that a track that never returns is
% dropped well within one demo scenario's run.
cfg = struct( ...
    'missedObservationToleranceTicks', 5, ...
    'missedObservationTimeoutSeconds', 2.5, ...
    'identityReassociationGateMeters', 2.0 ...
);

end
