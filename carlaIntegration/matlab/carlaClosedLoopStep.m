function [loopState, report] = carlaClosedLoopStep(loopState)
% carlaClosedLoopStep - Phase 12: one full tick of the REAL closed loop
% driven by REAL CARLA sensor/fusion observations:
%
%   carlaPerceptionStep (Phase 11, unmodified)
%           |
%   carlaTrackingStep (wraps objectTracking.m, unmodified, + coasting)
%           |
%   carlaPredictionStep (trajectoryPrediction.m, UNMODIFIED - K1 lives
%           |            entirely inside it)
%   behaviorDecision / decisionStateMachine / behaviorSeverity (UNMODIFIED,
%           |            same call sequence as main.m)
%   localPlanner / adaptivePlanner (K2, UNMODIFIED) / collisionCheck
%           |            (UNMODIFIED) / pathSmoothing (UNMODIFIED)
%   vehicleController (UNMODIFIED)
%           |
%   carlaApplyControl -> REAL CARLA ego vehicle
%
% Every one of those calls uses EXACTLY the same function, arguments, and
% call order main.m already uses for the five synthetic scenarios - this
% file only supplies real CARLA data in place of main.m's synthetic
% perception/scenario ground truth, plus the two additions Phase 12 was
% scoped to add (tracking-gap coasting, behavior-label reporting).
%
% NEVER CARLA autopilot, never a scripted/waypoint trajectory - the ego
% is driven ONLY by controlCommand computed above, applied via
% carlaApplyControl exactly like Phase 9's control channel.
%
% ONE MINIMAL INTERFACE FIX (documented, not a redesign): CARLA's raw
% vehicle state does not report the currently-applied steering angle
% (Phase 9's carlaToProjectState.m already documents this and leaves
% egoState.steering at its schema default). vehicleController.m needs
% that value to rate-limit its own output. Rather than modifying
% vehicleController.m or carlaToProjectState.m, this file - which already
% knows the exact steering angle IT commanded last tick - injects that
% known value into egoState.steering immediately before calling
% vehicleController, the same way carlaGetEgoState()'s caller would if
% the field were populated. No frozen file was changed to make this work.
%
% STEERING SIGN (loopState.steerSign, verified empirically against a
% live server, not assumed): controlCommand.steeringAngle is in this
% project's right-handed frame (radians, positive = turn left / CCW yaw
% increases, the standard bicycle-model convention purePursuitController.m
% already uses); CARLA's VehicleControl.steer is left-handed, normalized
% to [-1, 1]. Measured live: commanding raw CARLA steer=+0.3 with
% throttle=0.4 for 3s changed the PROJECT-frame yaw from -0.0028 rad to
% -0.9485 rad (delta -0.946 rad, i.e. a right turn) - so a positive raw
% CARLA steer corresponds to a NEGATIVE project-frame yaw change.
% Therefore carlaSteer = steerSign * steeringAngle with steerSign = -1
% (set in carlaClosedLoopInit.m) is required to make a positive
% project-frame steeringAngle (turn left) produce the correct real CARLA
% turn direction - the same single-axis mirror every other CARLA<->
% project conversion in this codebase uses (carlaToProjectState.m's yaw,
% carlaYawToProject.m), now confirmed for the control direction too,
% not just position/heading.
%
% Input/Output:
%   loopState - from carlaClosedLoopInit.m (first call) or the previous
%               call's returned loopState.
% Output:
%   report - struct with everything the visualization/metrics layer
%            needs: egoState, fusedAgents, obs, trackedAgents, trackInfo,
%            predictedTrajectories, behaviorInfo, decisionState,
%            targetSpeed, selectedTrajectory, candidateTrajectories,
%            isColliding, minTTC, controlCommand, dt, tickCount

PLANNING_DT = 0.1; % must match localPlanner.m/collisionCheck.m's own hardcoded PLANNING_DT

% PACING (fixed after a real bug found live in Phase 12 development):
% trajectoryPrediction.m's single `dt` parameter serves TWO purposes -
% the output trajectory's discretization step (which must equal
% PLANNING_DT for index alignment with localPlanner.m/collisionCheck.m),
% AND the divisor K1's internal acceleration/history-gap analysis uses to
% interpret REAL timestamp deltas in its own trackHistory buffer. main.m
% never has to reconcile these because its whole simulation runs on one
% fixed dt throughout. An earlier version of this file passed carlaTrackingStep
% the REAL measured wall-clock dt but carlaPredictionStep a separate fixed
% PLANNING_DT=0.1 - which silently fed K1's accel check the WRONG divisor
% for real velocity deltas (measured live: real ticks ran at ~0.03-0.05s,
% understating true acceleration by ~2-3x, making K1 MORE permissive than
% its own calibration intends - the wrong direction for a safety check).
% Fixed by pacing this loop to real ~0.1s ticks (matching main.m's own
% dt, which K1's PROCESS_NOISE=0.5/ACCEL_TOL derivation was calibrated
% against) and using that SAME dt for both the tracker and the predictor -
% eliminating the mismatch instead of papering over it.
MIN_TICK_SECONDS = 0.1;
if loopState.firstTick
    dt = MIN_TICK_SECONDS; % nominal first-tick value; no prior tick to measure a real interval from
    loopState.firstTick = false;
else
    elapsed = toc(loopState.lastTickTic);
    if elapsed < MIN_TICK_SECONDS
        pause(MIN_TICK_SECONDS - elapsed);
        elapsed = toc(loopState.lastTickTic);
    end
    dt = elapsed;
end
loopState.lastTickTic = tic;
loopState.tickCount = loopState.tickCount + 1;

% --- 1/2. Perception + fusion (Phase 11, unmodified) ---
[fusedAgents, obs] = carlaPerceptionStep(loopState.perceptionCfg, false, loopState.previousFusedAgents);
loopState.previousFusedAgents = fusedAgents;

% --- Phase 13 fix: carlaPerceptionStep.m deliberately returns fusedAgents
% in the EGO-RELATIVE frame (needed for its own internal sensor-range
% gate) - but every frozen function downstream of tracking
% (objectTracking/trajectoryPrediction/behaviorDecision/localPlanner/
% adaptivePlanner/collisionCheck) is validated against, and assumes,
% agent positions in the SAME GLOBAL frame as egoState.x/y and
% globalPath (confirmed directly from main.m and from
% behaviorDecision.m's own `relVec = agent.position - [egoState.x,
% egoState.y]`). Converting back to global frame here, before tracking
% even runs, is what carlaFusedAgentsToGlobal.m's header documents in
% full - this single missing conversion was silently defeating every
% distance/TTC-based safety check in the CARLA closed loop. Applied
% unconditionally, using THIS tick's egoState (not obs.egoState from a
% stale previous cycle).
if ~isempty(obs.egoState)
    fusedAgents = carlaFusedAgentsToGlobal(fusedAgents, obs.egoState);
end

% --- Phase 14 mandatory frame-safety assertion: fail LOUDLY rather than
% silently mix frames again. carlaFusedAgentsToGlobal.m above is applied
% exactly ONCE per tick, and only here - no other call site in this file
% touches fusedAgents' frame. As a runtime invariant check (not a second
% conversion), every agent's GLOBAL position must lie within a bounded
% multiple of the sensor's own configured range of the ego's GLOBAL
% position; a violation can only mean the conversion was skipped, applied
% twice, or applied with the wrong egoState - exactly the class of defect
% Phase 13 found. This never fires in normal operation (sensor-range
% filtering inside carlaPerceptionStep.m already bounds ego-relative
% distance to maxSensorRangeMeters before the conversion), so a firing
% here is treated as a hard integration defect, not a degraded-sensor
% case - it throws rather than silently skipping the tick.
if ~isempty(obs.egoState) && ~isempty(fusedAgents)
    FRAME_ASSERT_MARGIN = 3.0; % generous multiple, not a tight tolerance - this guards against gross frame errors (~doubling/omission), not normal sensor noise
    egoPosCheck = [obs.egoState.x, obs.egoState.y];
    maxAllowedDist = FRAME_ASSERT_MARGIN * loopState.perceptionCfg.maxSensorRangeMeters;
    for fi = 1:numel(fusedAgents)
        distFromEgo = norm(fusedAgents(fi).position - egoPosCheck);
        if distFromEgo > maxAllowedDist
            error('carlaClosedLoopStep:frameMismatch', ...
                ['Coordinate-frame invariant violated: fused agent %d is %.1fm from ego ' ...
                 '(egoPos=[%.2f,%.2f]), exceeding %.1fx the configured sensor range (%.1fm). ' ...
                 'This indicates carlaFusedAgentsToGlobal.m was skipped, double-applied, or given ' ...
                 'the wrong egoState - see carlaFusedAgentsToGlobal.m and the Phase 13 frame-bug ' ...
                 'writeup in docs/carla_integration.md before investigating further.'], ...
                fi, distFromEgo, egoPosCheck(1), egoPosCheck(2), FRAME_ASSERT_MARGIN, loopState.perceptionCfg.maxSensorRangeMeters);
        end
    end
end

report = struct('tickCount', loopState.tickCount, 'dt', dt, 'obs', obs, 'fusedAgents', fusedAgents);

if isempty(obs.egoState)
    % No ego state this tick (e.g. a transient CARLA hiccup) - skip
    % safely rather than acting on stale/fabricated state. Matches Phase
    % 10/11's "never crash, never fabricate" discipline.
    report.skipped = true;
    return;
end
report.skipped = false;

egoState = obs.egoState;
egoState.steering = loopState.lastSteeringAngle; % see header: the one documented interface fix

% --- 3. Tracking (wraps objectTracking.m, unmodified) ---
[trackedAgents, trackInfo, loopState.trackerState] = carlaTrackingStep(fusedAgents, loopState.trackerState, dt, loopState.trackingCfg);

% --- 4. Prediction (trajectoryPrediction.m, UNMODIFIED - K1 untouched) ---
[predictedTrajectories, behaviorInfo] = carlaPredictionStep(trackedAgents, egoState, loopState.planHorizon, PLANNING_DT);

% --- 5. Decision (UNMODIFIED, same sequence as main.m) ---
behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, loopState.planCfg, loopState.decisionState);
proposedState = decisionStateMachine(loopState.decisionState, behaviorCommand, struct());
isEscalation = behaviorSeverity(proposedState) > behaviorSeverity(loopState.decisionState);
dwellSatisfied = (egoState.timestamp - loopState.lastTransitionTime) >= loopState.minDwellTime;
if proposedState ~= loopState.decisionState && (isEscalation || dwellSatisfied)
    loopState.decisionState = proposedState;
    loopState.lastTransitionTime = egoState.timestamp;
    loopState.stateTransitionCount = loopState.stateTransitionCount + 1;
end
if isfield(loopState.decisionSpeedFactors, loopState.decisionState)
    targetSpeed = loopState.baseSpeed * loopState.decisionSpeedFactors.(char(loopState.decisionState));
else
    targetSpeed = 0; % unrecognized state -> stop, never guess a speed
end

% --- 6. Planning (K2, UNMODIFIED) + collision check (UNMODIFIED) ---
planningTic = tic; % Phase 13: planner-only wall-clock time, NOT the fixed
                    % simulation/model timestep - measured separately from
                    % the perception/tracking/prediction work above.
candidateTrajectories = localPlanner(egoState, loopState.globalPath, predictedTrajectories, loopState.planCfg);
priorIdx = loopState.previousCandidateIndex;
[selectedTrajectory, loopState.previousCandidateIndex] = adaptivePlanner( ...
    egoState, candidateTrajectories, predictedTrajectories, loopState.vehCfg, loopState.planCfg, "carlaClosedLoop", priorIdx);
[isColliding, minTTC] = collisionCheck(selectedTrajectory, predictedTrajectories, loopState.vehCfg);
smoothPath = pathSmoothing(selectedTrajectory, struct());
planningElapsedS = toc(planningTic);

% --- Phase 13 planning diagnostics (reporting only - does not affect
% which candidate is selected, that decision is made entirely inside the
% frozen adaptivePlanner.m call above). Re-runs collisionCheck.m (frozen,
% unmodified) on EVERY candidate exactly as adaptivePlanner.m already
% does internally, purely to expose the feasible/collision-rejected/TTC-
% rejected breakdown that function computes but does not return. This is
% observing the frozen function's own ground truth from outside, not a
% second decision-making implementation. ---
nCandidates = numel(candidateTrajectories);
candidateMinTTC = Inf(1, nCandidates);
candidateColliding = false(1, nCandidates);
for ci = 1:nCandidates
    [candidateColliding(ci), candidateMinTTC(ci)] = collisionCheck(candidateTrajectories{ci}, predictedTrajectories, loopState.vehCfg);
end
collisionRejectedCount = sum(candidateColliding);
ttcRejectedCount = sum(~candidateColliding & candidateMinTTC < loopState.planCfg.ttcThresholds.critical);
feasibleCount = sum(~candidateColliding & candidateMinTTC >= loopState.planCfg.ttcThresholds.critical);
usedFallback = feasibleCount == 0 && nCandidates > 0;

% --- 7. Control (UNMODIFIED) ---
controlCommand = vehicleController(egoState, smoothPath, loopState.vehCfg, targetSpeed, dt);

% --- Phase 14 failsafe: a NaN/Inf control command must never reach CARLA
% actuation. This is a pure integration-layer guard (vehicleController.m
% itself is untouched) - if its output is ever invalid for any reason
% (e.g. a NaN propagating from an edge-case upstream), degrade to a
% controlled stop (steer centered, throttle off, full brake) rather than
% sending an undefined command to the vehicle, matching the project's
% "prefer controlled braking over uncontrolled motion" failsafe rule.
% loopState.lastSteeringAngle intentionally still latches the SAFE value
% below, not the invalid one, so next tick's rate-limiter is not fed NaN.
isCommandInvalid = ~isfinite(controlCommand.steeringAngle) || ~isfinite(controlCommand.throttle) || ~isfinite(controlCommand.brake);
if isCommandInvalid
    warning('carlaClosedLoopStep:invalidControlCommand', ...
        'vehicleController produced a non-finite command (steer=%g throttle=%g brake=%g) at tick %d - applying a controlled stop instead.', ...
        controlCommand.steeringAngle, controlCommand.throttle, controlCommand.brake, loopState.tickCount);
    controlCommand.steeringAngle = 0;
    controlCommand.throttle = 0;
    controlCommand.brake = 1;
end
loopState.lastSteeringAngle = controlCommand.steeringAngle;

% --- 8. Apply to the REAL CARLA ego - never autopilot, never a scripted trajectory ---
carlaSteer = loopState.steerSign * controlCommand.steeringAngle / loopState.vehCfg.maxSteerAngle;
carlaSteer = max(-1, min(1, carlaSteer));
carlaApplyControl(carlaSteer, controlCommand.throttle, controlCommand.brake);

report.egoState               = egoState;
report.trackedAgents          = trackedAgents;
report.trackInfo              = trackInfo;
report.predictedTrajectories  = predictedTrajectories;
report.behaviorInfo           = behaviorInfo;
report.decisionState          = loopState.decisionState;
report.targetSpeed            = targetSpeed;
report.candidateTrajectories  = candidateTrajectories;
report.selectedTrajectory     = selectedTrajectory;
report.smoothPath             = smoothPath;
report.isColliding             = isColliding;
report.minTTC                 = minTTC;
report.controlCommand         = controlCommand;
report.carlaSteer              = carlaSteer;
report.stateTransitionCount   = loopState.stateTransitionCount;
report.goalDistance            = norm([egoState.x, egoState.y] - loopState.goalPosition);

% --- Phase 13 additions (all purely additive reporting - see the block
% above for how these are computed; none of it feeds back into
% selection) ---
report.planningElapsedS        = planningElapsedS;
report.totalCandidates         = nCandidates;
report.feasibleCandidateCount  = feasibleCount;
report.collisionRejectedCount  = collisionRejectedCount;
report.ttcRejectedCount        = ttcRejectedCount;
report.candidateMinTTC         = candidateMinTTC;
report.candidateColliding      = candidateColliding;
report.usedFallback            = usedFallback;
report.selectedCandidateIndex  = loopState.previousCandidateIndex;
report.candidateChanged        = ~isequal(loopState.previousCandidateIndex, priorIdx);

% --- Phase 14 addition ---
report.commandOverridden       = isCommandInvalid;

end
