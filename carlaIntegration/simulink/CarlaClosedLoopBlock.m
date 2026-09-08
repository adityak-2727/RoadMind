classdef CarlaClosedLoopBlock < matlab.System
    % CarlaClosedLoopBlock - Phase 14: hosts the REAL, unmodified CARLA
    % closed loop (carlaClosedLoopStep.m - perception -> fusion -> global-
    % frame conversion (carlaFusedAgentsToGlobal.m, Phase 13) -> tracking
    % -> prediction (K1) -> decision -> planning (K2) -> collisionCheck ->
    % control -> CARLA actuation) as one Simulink block, so the live CARLA
    % ego vehicle is genuinely driven from within a running Simulink
    % model - mirrors simulink/AutonomyPipelineBlock.m's already-
    % established pattern (same project, same MATLAB System object
    % approach, same "Interpreted Execution" requirement for the same
    % reason - struct/cell arrays throughout), but drives the real Indian
    % hero scene through CARLA instead of the five synthetic scenarios.
    %
    % This block does not reimplement any pipeline stage. Every tick it
    % calls exactly carlaClosedLoopStep.m - Phase 12/13's already-
    % validated wrapper around the frozen algorithm stack - and nothing
    % else. setupImpl builds the APPROACH -> curved TURN -> EXIT path
    % (carlaGenerateIntersectionTurnPath.m, Phase 13) using the ego's real
    % starting pose and ApproachLengthM, which defaults to 45.0m - the
    % live-measured distance (via CARLA's own waypoint API, walking
    % wp.next() from the ego's spawn point) from the hero scene's ego
    % staging point to the actual junction (id=103) entry, NOT the 5.0m
    % Phase 13's Demo H used (that value only needed to prove the curved
    % path could be consumed by the planner without crashing - Phase 13
    % explicitly deferred the physical turn, and never validated the
    % turn's placement against the visual scene). Using the wrong,
    % shorter distance here would make the vehicle arc through the
    % approach road well before the real intersection.
    %
    % Closed-loop feedback: LoopState is a persistent System object
    % property (carlaClosedLoopInit.m's opaque struct), updated at the
    % end of each stepImpl call and read again at the start of the next -
    % each Simulink tick is exactly one call to carlaClosedLoopStep.m,
    % with state genuinely carried tick-to-tick the same way
    % AutonomyPipelineBlock.m carries its own EgoState/TrackedAgentsPrev/
    % DecisionState. Once GoalToleranceM is reached, the block applies one
    % final controlled stop (steer=0, throttle=0, brake=1) and then holds
    % state, mirroring AutonomyPipelineBlock.m's own goal-reached freeze
    % and main.m's `break`.
    %
    % Traffic actors in the hero scene remain scripted via
    % carlaIndianSceneTrafficStep.m (established, non-autopilot, velocity-
    % commanded) exactly as every other Phase 11.5-13 demo already does -
    % this block only ever actuates the EGO through carlaClosedLoopStep.m.

    properties (Nontunable)
        TurnRadiusM = 12.0      % arc radius, m - matches the Phase 13-validated value; do not tighten just to force a turn
        ApproachLengthM = 45.0  % live-measured ego-spawn-to-junction-103-entry distance, m
        ExitLengthM = 25.0
        ExitHeadingDeg = 89.64  % West approach -> North exit, left turn (Phase 11.5's resolved junction geometry)
        BaseSpeedMps = 4.0
        SensorRangeM = 25.0
        GoalToleranceM = 3.0
    end

    properties (Access = private)
        LoopState
        SceneState
        SceneCfg
        TrafficState
        GoalReachedFlag
        LastEgoX
        LastEgoY
        LastEgoYaw
        LastEgoVelocity
        LastDecisionSeverity
        LastMinTTC
        LastIsColliding
        LastDistToGoal
        LastSteeringDeg
        LastThrottle
        LastBrake
        LastFeasibleCount
    end

    methods (Access = protected)
        function setupImpl(obj)
            obj.SceneCfg = carlaIndianSceneConfig();
            carlaCfg = carlaConfig();
            carlaConnect(carlaCfg);
            carlaLoadMap(obj.SceneCfg.mapName);
            obj.SceneState = carlaBuildIndianHeroScene(obj.SceneCfg);
            carlaAttachCamera(carlaCfg.camera);
            carlaAttachLidar(carlaCfg.lidar);
            carlaAttachRadar(carlaCfg.radar);
            carlaAttachCollisionSensor(); % Phase 14: authoritative ground-truth collision log, see its own header
            pause(2.0);

            egoState0 = carlaGetEgoState();
            globalPath = carlaGenerateIntersectionTurnPath( ...
                [egoState0.x, egoState0.y], egoState0.yaw, deg2rad(obj.ExitHeadingDeg), ...
                obj.TurnRadiusM, obj.ApproachLengthM, obj.ExitLengthM);

            obj.LoopState = carlaClosedLoopInit(60, obj.BaseSpeedMps, obj.SensorRangeM, globalPath);
            obj.TrafficState = [];
            obj.GoalReachedFlag = false;

            obj.LastEgoX = egoState0.x;
            obj.LastEgoY = egoState0.y;
            obj.LastEgoYaw = egoState0.yaw;
            obj.LastEgoVelocity = 0;
            obj.LastDecisionSeverity = behaviorSeverity("cruise");
            obj.LastMinTTC = Inf;
            obj.LastIsColliding = false;
            obj.LastDistToGoal = norm(globalPath(end, :) - [egoState0.x, egoState0.y]);
            obj.LastSteeringDeg = 0;
            obj.LastThrottle = 0;
            obj.LastBrake = 0;
            obj.LastFeasibleCount = 0;
        end

        function [egoX, egoY, egoYaw, egoVelocity, decisionSeverityOut, minTTCOut, ...
                  isCollidingOut, distToGoal, goalReachedOut, steeringDeg, throttleCmd, ...
                  brakeCmd, feasibleCandidateCount, tickCountOut, realCollisionCount] = stepImpl(obj)

            if ~obj.GoalReachedFlag
                % Phase 14: LastEgoX/LastEgoY (updated at the end of the
                % previous tick, or the spawn pose on the first tick) is
                % used instead of an extra carlaGetEgoState() RPC call -
                % see carlaIndianSceneTrafficStep.m's own header for why
                % this ego-position argument exists (preventing scripted
                % traffic from driving through a stationary ego).
                obj.TrafficState = carlaIndianSceneTrafficStep(obj.SceneState, obj.SceneCfg, obj.TrafficState, [obj.LastEgoX, obj.LastEgoY]);
                [obj.LoopState, report] = carlaClosedLoopStep(obj.LoopState);

                if ~report.skipped
                    obj.LastEgoX = report.egoState.x;
                    obj.LastEgoY = report.egoState.y;
                    obj.LastEgoYaw = report.egoState.yaw;
                    obj.LastEgoVelocity = report.egoState.velocity;
                    obj.LastDecisionSeverity = behaviorSeverity(report.decisionState);
                    obj.LastMinTTC = report.minTTC;
                    obj.LastIsColliding = report.isColliding;
                    obj.LastDistToGoal = report.goalDistance;
                    obj.LastSteeringDeg = rad2deg(report.controlCommand.steeringAngle);
                    obj.LastThrottle = report.controlCommand.throttle;
                    obj.LastBrake = report.controlCommand.brake;
                    obj.LastFeasibleCount = report.feasibleCandidateCount;

                    if report.goalDistance < obj.GoalToleranceM
                        obj.GoalReachedFlag = true;
                        carlaApplyControl(0, 0, 1); % final controlled stop, mirrors main.m's goal-reached handling
                        obj.LastThrottle = 0;
                        obj.LastBrake = 1;
                    end
                end
            end

            egoX = obj.LastEgoX;
            egoY = obj.LastEgoY;
            egoYaw = obj.LastEgoYaw;
            egoVelocity = obj.LastEgoVelocity;
            decisionSeverityOut = obj.LastDecisionSeverity;
            minTTCOut = obj.LastMinTTC;
            isCollidingOut = double(obj.LastIsColliding);
            distToGoal = obj.LastDistToGoal;
            goalReachedOut = double(obj.GoalReachedFlag);
            steeringDeg = obj.LastSteeringDeg;
            throttleCmd = obj.LastThrottle;
            brakeCmd = obj.LastBrake;
            feasibleCandidateCount = obj.LastFeasibleCount;
            tickCountOut = obj.LoopState.tickCount;
            realCollisionCount = numel(carlaGetCollisionEvents()); % ground-truth physics contacts, independent of collisionCheck.m's predicted TTC above
        end

        function releaseImpl(~)
            carlaDisconnect();
        end

        function resetImpl(~)
        end

        function varargout = getOutputSizeImpl(~)
            varargout = repmat({1}, 1, 15);
        end

        function varargout = getOutputDataTypeImpl(~)
            varargout = repmat({'double'}, 1, 15);
        end

        function varargout = isOutputComplexImpl(~)
            varargout = repmat({false}, 1, 15);
        end

        function varargout = isOutputFixedSizeImpl(~)
            varargout = repmat({true}, 1, 15);
        end
    end
end
