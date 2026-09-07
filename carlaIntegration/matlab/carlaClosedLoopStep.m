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
candidateTrajectories = localPlanner(egoState, loopState.globalPath, predictedTrajectories, loopState.planCfg);
priorIdx = loopState.previousCandidateIndex;
[selectedTrajectory, loopState.previousCandidateIndex] = adaptivePlanner( ...
    egoState, candidateTrajectories, predictedTrajectories, loopState.vehCfg, loopState.planCfg, "carlaClosedLoop", priorIdx);
[isColliding, minTTC] = collisionCheck(selectedTrajectory, predictedTrajectories, loopState.vehCfg);
smoothPath = pathSmoothing(selectedTrajectory, struct());

% --- 7. Control (UNMODIFIED) ---
controlCommand = vehicleController(egoState, smoothPath, loopState.vehCfg, targetSpeed, dt);
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

end
