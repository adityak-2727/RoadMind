classdef testCarlaPhase14Turning < matlab.unittest.TestCase
% testCarlaPhase14Turning - Phase 14 tests: MATLAB/Simulink genuinely
% driving the CARLA ego vehicle through APPROACH -> OBSERVE -> DECIDE ->
% PLAN -> TURN -> EXIT via carlaClosedLoopStep.m (extended this phase
% with a coordinate-frame runtime assertion and a NaN/Inf control-command
% failsafe) - never CARLA autopilot, never a scripted/prerecorded ego
% trajectory, never teleportation. Same discipline as every prior CARLA
% test file: every live test SKIPS (assumeTrue, never a fabricated pass)
% when no CARLA server is reachable, and never mocks the behavior under
% test. A real carlaAttachCollisionSensor()/carlaGetCollisionEvents()
% (new this phase) is used wherever a test needs authoritative ground-
% truth collision data - collisionCheck.m's own predicted/geometric flag
% is validated separately and is not treated as proof of zero physical
% contact (see docs/carla_integration.md's Phase 14 section for why both
% are necessary).

    properties
        CarlaCfg
        SceneCfg
        VehCfg
        CarlaReady logical
    end

    methods (TestClassSetup)
        function addProjectToPath(testCase) %#ok<INUSD>
            root = fileparts(fileparts(fileparts(mfilename('fullpath'))));
            addpath(genpath(root));
        end

        function checkCarlaAvailability(testCase)
            testCase.CarlaCfg = carlaConfig();
            testCase.SceneCfg = carlaIndianSceneConfig();
            testCase.VehCfg = vehicleConfig();
            testCase.CarlaReady = isCarlaAvailable(testCase.CarlaCfg);
            if ~testCase.CarlaReady
                fprintf('[testCarlaPhase14Turning] No live CARLA server reachable - LIVE tests SKIPPED (not failed). Pure-math tests still run without CARLA.\n');
            end
        end
    end

    methods (TestMethodTeardown)
        function cleanupAfterEachTest(testCase)
            if testCase.CarlaReady
                carlaDisconnect();
            end
        end
    end

    methods (Access = private)
        function [sceneState, egoState0] = connectAndBuildScene(testCase)
            carlaConnect(testCase.CarlaCfg);
            carlaLoadMap(testCase.SceneCfg.mapName);
            sceneState = carlaBuildIndianHeroScene(testCase.SceneCfg);
            carlaAttachCamera(testCase.CarlaCfg.camera);
            carlaAttachLidar(testCase.CarlaCfg.lidar);
            carlaAttachRadar(testCase.CarlaCfg.radar);
            carlaAttachCollisionSensor();
            pause(2.0);
            egoState0 = carlaGetEgoState();
        end

        function [globalPath, turnInfo] = buildTurnPath(~, egoState0)
            [globalPath, turnInfo] = carlaGenerateIntersectionTurnPath( ...
                [egoState0.x, egoState0.y], egoState0.yaw, deg2rad(89.64), 12.0, 45.0, 25.0);
        end
    end

    methods (Test)
        %% A. Controller interface test - live, short
        function testControllerInterfaceOutputStructure(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            for k = 1:20
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                cc = report.controlCommand;
                testCase.verifyTrue(isfield(cc, 'steeringAngle') && isfield(cc, 'throttle') && isfield(cc, 'brake'), ...
                    'controlCommand must expose steeringAngle/throttle/brake.');
                testCase.verifyTrue(isfinite(cc.steeringAngle) && isfinite(cc.throttle) && isfinite(cc.brake), ...
                    'controlCommand fields must always be finite.');
                testCase.verifyLessThanOrEqual(abs(cc.steeringAngle), testCase.VehCfg.maxSteerAngle + 1e-6);
                testCase.verifyGreaterThanOrEqual(cc.throttle, 0);
                testCase.verifyLessThanOrEqual(cc.throttle, 1);
                testCase.verifyGreaterThanOrEqual(cc.brake, 0);
                testCase.verifyLessThanOrEqual(cc.brake, 1);
            end
        end

        %% B. Steering direction test - the empirically-measured sign
        % (documented in carlaClosedLoopStep.m's header: positive project-
        % frame steeringAngle must produce a LEFT turn in real CARLA,
        % requiring carlaSteer = steerSign * steeringAngle with
        % steerSign=-1) must not silently drift. Live: commands a fixed
        % positive raw CARLA steer with throttle and measures the actual
        % project-frame yaw change sign, exactly reproducing the
        % measurement carlaClosedLoopStep.m's header documents.
        function testSteeringDirectionSign(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            testCase.verifyEqual(loopState.steerSign, -1, ...
                'steerSign must remain -1 - see carlaClosedLoopStep.m header for the live measurement this encodes.');

            yaw0 = carlaGetEgoState().yaw;
            for k = 1:20
                carlaApplyControl(0.3, 0.4, 0.0); % positive raw CARLA steer (right, per the documented measurement)
                pause(0.1);
            end
            yaw1 = carlaGetEgoState().yaw;
            yawChange = atan2(sin(yaw1 - yaw0), cos(yaw1 - yaw0));
            testCase.verifyLessThan(yawChange, 0, ...
                'A positive raw CARLA steer must produce a NEGATIVE project-frame yaw change (a right turn) - matching carlaClosedLoopStep.m''s documented measurement. If this fails, the steering sign convention has drifted and steerSign must be re-derived, not blindly flipped.');
        end

        %% C. Steering-rate-limit test - pure math, calls the frozen
        % vehicleController.m directly with a large desired-steering jump
        % and confirms the per-tick change never exceeds maxSteerRate*dt.
        function testSteeringRateLimit(testCase)
            vehCfg = vehicleConfig();
            egoState = createEgoState();
            egoState.x = 0; egoState.y = 0; egoState.yaw = 0; egoState.velocity = 3.0; egoState.steering = 0;
            % A path sharply off to one side forces purePursuitController
            % to want a large steering angle in one step.
            sharpPath = [1, 0; 2, 5; 3, 8];
            dt = 0.1;
            cc = vehicleController(egoState, sharpPath, vehCfg, 3.0, dt);
            maxAllowedStep = vehCfg.maxSteerRate * dt;
            testCase.verifyLessThanOrEqual(abs(cc.steeringAngle - egoState.steering), maxAllowedStep + 1e-9, ...
                'vehicleController must never change steering faster than vehicleConfig.maxSteerRate.');
        end

        %% D. Curvature feasibility test - pure math, using the REAL
        % corrected approach length (45.0m, not Phase 13 Demo H's 5.0m -
        % see docs/carla_integration.md's Phase 14 section for the live
        % measurement that corrected it).
        function testCurvatureFeasibility(testCase)
            approachStart = [-63.90, -135.42];
            approachHeading = deg2rad(1.30);
            exitHeading = deg2rad(89.64);
            [globalPath, turnInfo] = carlaGenerateIntersectionTurnPath(approachStart, approachHeading, exitHeading, 12.0, 45.0, 25.0);
            vehCfg = vehicleConfig();
            feas = carlaCheckTurnFeasibility(globalPath, turnInfo, vehCfg);
            testCase.verifyTrue(feas.isFeasible, 'The Phase 14 turn path (radius=12.0m) must be feasible against vehicleConfig limits.');
            testCase.verifyEqual(feas.minRadiusM, 12.0, 'AbsTol', 0.1, 'Measured curvature-derived min radius must match the requested arc radius.');
            testCase.verifyGreaterThan(feas.pathLengthM, 45.0 + 25.0, 'Path length must be at least the straight approach+exit segments combined.');
        end

        %% E. Straight approach tracking test - live: lateral deviation
        % from the planned corridor must stay bounded during ordinary
        % straight-line driving (not oscillating wildly).
        function testStraightApproachTracking(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            lateralErrors = [];
            for k = 1:40
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                p = loopState.globalPath;
                d = vecnorm(p - [report.egoState.x, report.egoState.y], 2, 2);
                lateralErrors(end+1) = min(d); %#ok<AGROW>
            end
            testCase.verifyLessThan(mean(lateralErrors), 2.5, ...
                'Mean distance from the ego to its own planned corridor should stay well within the candidate lattice half-width during ordinary driving.');
        end

        %% F. Curved turn tracking test - live: heading must change
        % CONTINUOUSLY (no discontinuous jumps beyond the rate limit)
        % while traversing the arc.
        function testCurvedTurnTracking(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            [~, egoState0] = testCase.connectAndBuildScene();
            globalPath = testCase.buildTurnPath(egoState0);
            loopState = carlaClosedLoopInit(60, 4.0, 25, globalPath);
            yaws = []; dts = [];
            for k = 1:150
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                yaws(end+1) = report.egoState.yaw; %#ok<AGROW>
                dts(end+1) = report.dt; %#ok<AGROW>
            end
            unwrapped = unwrap(yaws);
            maxAllowedYawStepPerTick = testCase.VehCfg.maxSteerRate * max(dts) * 3; % generous multiple - checking for discontinuities, not exact rate-limit equality (yaw response also depends on speed/wheelbase, not just steering angle)
            yawSteps = abs(diff(unwrapped));
            testCase.verifyLessThan(max(yawSteps), maxAllowedYawStepPerTick, ...
                'Heading must change continuously tick-to-tick - no discontinuous jump consistent with a teleport or a broken control loop.');
        end

        %% G. Speed-control test - live: speed must never exceed
        % vehicleConfig.maxSpeed and must respond to targetSpeed changes.
        function testSpeedControlStability(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            speeds = [];
            for k = 1:50
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                speeds(end+1) = report.egoState.velocity; %#ok<AGROW>
                testCase.verifyGreaterThanOrEqual(report.egoState.velocity, -1e-6, 'Speed must never be negative.');
                testCase.verifyLessThanOrEqual(report.egoState.velocity, testCase.VehCfg.maxSpeed + 1e-6, 'Speed must never exceed vehicleConfig.maxSpeed.');
            end
            testCase.verifyGreaterThan(max(speeds), 0.5, 'The ego must actually accelerate from rest during a normal approach.');
        end

        %% H. Brake-response test - live: a very close staged conflict
        % must produce a meaningful brake command at some point.
        function testBrakeResponse(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('walker.pedestrian.0001', 10, -3, 0.5, 0.0);
            testCase.assumeFalse(isempty(actorId), 'Spawn collided at this offset - inconclusive.');
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            maxBrake = 0;
            for k = 1:60
                carlaSetActorVelocityRelativeToEgo(actorId, 0.0, 0.6, 0.0);
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                maxBrake = max(maxBrake, report.controlCommand.brake);
            end
            testCase.verifyGreaterThan(maxBrake, 0.3, 'A pedestrian closing to within a few meters of the ego''s path must eventually produce a meaningful brake command.');
        end

        %% I. Crossing-vehicle avoidance test - live, with REAL collision
        % ground-truth check (not just the geometric flag).
        function testCrossingVehicleAvoidance(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 12, 6, 1.5, -90.0);
            testCase.assumeFalse(isempty(actorId), 'Spawn collided at this offset - inconclusive.');
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            statesSeen = strings(1,0);
            for k = 1:60
                carlaSetActorVelocityRelativeToEgo(actorId, 0.0, -1.0, 0.0);
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                statesSeen(end+1) = report.decisionState; %#ok<AGROW>
            end
            testCase.verifyTrue(any(statesSeen ~= "cruise"), 'A genuine crossing conflict must produce at least one non-cruise decision.');
            realCollisions = carlaGetCollisionEvents();
            testCase.verifyEqual(numel(realCollisions), 0, 'The crossing-vehicle scenario must complete with zero REAL physical collisions.');
        end

        %% J. Pedestrian avoidance test - live, VRU-specific: must reach a
        % VRU-tier decision (brake/wait/emergency_stop/avoid), not just
        % "replan"/"merge", per behaviorDecision.m's VULNERABLE_CLASSES
        % handling.
        function testPedestrianAvoidance(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('walker.pedestrian.0001', 12, -4, 0.5, 0.0);
            testCase.assumeFalse(isempty(actorId), 'Spawn collided at this offset - inconclusive.');
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            vruStates = ["avoid", "brake", "wait", "emergency_stop"];
            sawVruTier = false;
            for k = 1:60
                carlaSetActorVelocityRelativeToEgo(actorId, 0.0, 0.7, 0.0);
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                if any(report.decisionState == vruStates)
                    sawVruTier = true;
                end
            end
            testCase.verifyTrue(sawVruTier, 'A pedestrian crossing into the ego''s path must produce at least one VRU-sensitive decision tier (avoid/brake/wait/emergency_stop).');
            realCollisions = carlaGetCollisionEvents();
            testCase.verifyEqual(numel(realCollisions), 0, 'The pedestrian scenario must complete with zero REAL physical collisions.');
        end

        %% K. Motorcycle/bicycle avoidance test - live: correct
        % class/behavior recognition through the full pipeline.
        function testMotorcycleBicycleAvoidance(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('vehicle.bh.crossbike', 12, 3, 1.5, -90.0);
            testCase.assumeFalse(isempty(actorId), 'Spawn collided at this offset - inconclusive.');
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            sawBicycleClass = false;
            for k = 1:60
                carlaSetActorVelocityRelativeToEgo(actorId, 0.0, -0.6, 0.0);
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                for i = 1:numel(report.trackedAgents)
                    if ~isempty(report.trackInfo(i).carlaActorId) && report.trackInfo(i).carlaActorId == actorId ...
                            && report.trackedAgents(i).class == "bicycle"
                        sawBicycleClass = true;
                    end
                end
            end
            testCase.verifyTrue(sawBicycleClass, 'The staged bicycle must be tracked and correctly classified as class="bicycle" at least once.');
        end

        %% L. Parked-obstacle test - live: the hero scene's own parked
        % vehicles must be tracked as real, near-stationary agents.
        function testParkedObstacleHandling(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            sawSlowCarClass = false;
            for k = 1:60
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                for i = 1:numel(report.trackedAgents)
                    ta = report.trackedAgents(i);
                    if (ta.class == "car" || ta.class == "unknown") && norm(ta.velocity) < 0.5
                        sawSlowCarClass = true;
                    end
                end
            end
            testCase.verifyTrue(sawSlowCarClass, 'At least one near-stationary vehicle-like agent (a parked car or unclassified static roadside object) must be tracked during a normal approach through the hero scene.');
        end

        %% M. Dynamic replanning test - live: candidateChanged must
        % actually fire at least once against a genuine moving hazard -
        % never fabricated, only measured.
        function testDynamicReplanning(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 14, -8, 1.5, atan2d(1.2, 1.0));
            testCase.assumeFalse(isempty(actorId), 'Spawn collided at this offset - inconclusive.');
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            replanCount = 0;
            for k = 1:60
                carlaSetActorVelocityRelativeToEgo(actorId, 1.0, 1.2, 0.0);
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                if report.candidateChanged
                    replanCount = replanCount + 1;
                end
            end
            testCase.verifyGreaterThan(replanCount, 0, 'A moving merge-style hazard must cause the planner to genuinely change its selected candidate at least once - this is measured, not assumed.');
        end

        %% N. Full approach -> turn -> exit test - live, the primary
        % Phase 14 acceptance test: the ego must PHYSICALLY complete the
        % maneuver (goal reached) with a substantial, continuous heading
        % change, and zero REAL collisions.
        function testFullApproachTurnExit(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            [~, egoState0] = testCase.connectAndBuildScene();
            globalPath = testCase.buildTurnPath(egoState0);
            loopState = carlaClosedLoopInit(60, 4.0, 25, globalPath);
            GOAL_TOL = 3.0;
            yaw0 = egoState0.yaw;
            goalReached = false;
            lastYaw = yaw0;
            for k = 1:900
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                lastYaw = report.egoState.yaw;
                if report.goalDistance < GOAL_TOL
                    carlaApplyControl(0, 0, 1);
                    goalReached = true;
                    break;
                end
            end
            testCase.verifyTrue(goalReached, 'The ego must physically reach the post-intersection goal within 900 ticks.');
            headingChange = abs(atan2(sin(lastYaw - yaw0), cos(lastYaw - yaw0)));
            testCase.verifyGreaterThan(rad2deg(headingChange), 45, 'A genuine turn must produce a substantial heading change (>45deg), not just wandering forward.');
            realCollisions = carlaGetCollisionEvents();
            testCase.verifyEqual(numel(realCollisions), 0, 'The full approach->turn->exit maneuver must complete with zero REAL physical collisions.');
        end

        %% O. Closed-loop CARLA integration test via Simulink - live,
        % short: confirms the MATLAB System block genuinely drives CARLA
        % from within a running Simulink model, not just via a plain
        % MATLAB loop.
        function testSimulinkClosedLoopIntegration(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            buildCarlaClosedLoopModel();
            load_system('carlaClosedLoopPipeline');
            cleanupObj = onCleanup(@() close_system('carlaClosedLoopPipeline', 0)); %#ok<NASGU>
            simOut = sim('carlaClosedLoopPipeline', 'StopTime', '3');
            x = squeeze(simOut.carlaEgoX);
            y = squeeze(simOut.carlaEgoY);
            testCase.verifyGreaterThan(numel(x), 1, 'The Simulink model must run for more than one tick.');
            testCase.verifyGreaterThan(norm([x(end)-x(1), y(end)-y(1)]), 0, 'The ego must have physically moved during the Simulink-hosted run.');
        end

        %% P. Cleanup/orphan-actor test - mirrors the established pattern
        % from testCarlaHeroSceneSensorFusion.m's own testCleanupLeavesNoOrphans,
        % applied with the Phase 14 additions (collision sensor, turn-path
        % conflict actors) in play.
        function testCleanupLeavesNoOrphans(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 12, 6, 1.5, -90.0); %#ok<NASGU>
            pause(1.0);
            carlaDisconnect();

            py.importlib.import_module('carla');
            freshClient = py.carla.Client(testCase.CarlaCfg.host, int32(testCase.CarlaCfg.port));
            freshClient.set_timeout(testCase.CarlaCfg.timeoutSeconds);
            worldActors = cell(py.list(freshClient.get_world().get_actors()));
            nonInfra = 0;
            for i = 1:numel(worldActors)
                typeId = char(worldActors{i}.type_id);
                if startsWith(typeId, 'vehicle.') || startsWith(typeId, 'walker.pedestrian') || startsWith(typeId, 'sensor.')
                    nonInfra = nonInfra + 1;
                end
            end
            testCase.verifyEqual(nonInfra, 0, 'No vehicle/pedestrian/sensor actors (including the collision sensor and staged conflict actor) should remain after carlaDisconnect().');
        end
    end
end
