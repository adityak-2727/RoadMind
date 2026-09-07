classdef testCarlaPlanningDecision < matlab.unittest.TestCase
% testCarlaPlanningDecision - Phase 13 tests: the full, real, unmodified
% pipeline (perception -> fusion -> tracking -> prediction -> decision ->
% adaptivePlanner (K2) -> collisionCheck -> controller-ready output)
% driven by carlaClosedLoopStep.m against the Indian hero scene.
%
% The first three tests are pure-math regression guards for the actual
% defect this phase found and fixed: carlaPerceptionStep.m deliberately
% returns fusedAgents in the EGO-RELATIVE frame (needed for its own
% sensor-range gate), but every frozen function downstream of tracking
% (objectTracking/trajectoryPrediction/behaviorDecision/localPlanner/
% adaptivePlanner/collisionCheck) is validated against - and assumes -
% agent positions in the SAME GLOBAL frame as egoState.x/y and globalPath
% (confirmed directly from main.m's usage and from behaviorDecision.m's
% own `relVec = agent.position - [egoState.x, egoState.y]`). Without the
% conversion carlaFusedAgentsToGlobal.m now applies (wired into
% carlaClosedLoopStep.m immediately after perception, before tracking),
% every distance/bearing/TTC computation silently compared numbers from
% two different coordinate origins: minTTC was Inf and every candidate
% trajectory was rated feasible on every tick across 8 independent live
% demo scenarios (470+ ticks total), including scenarios with no staged
% conflict actor at all, even while agents were directly measured within
% 0.3-1.6m of the ego at various points in the same runs. Tests 4+ are
% live end-to-end checks that a genuine conflict actually produces
% escalation, candidate rejection, and a finite TTC - the exact behavior
% that bug was silently suppressing.
%
% Same discipline as every prior CARLA test file: every live test SKIPS
% (assumeTrue, never a fabricated pass) when no CARLA server is
% reachable, and never mocks the behavior under test.

    properties
        CarlaCfg
        SceneCfg
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
            testCase.CarlaReady = isCarlaAvailable(testCase.CarlaCfg);
            if ~testCase.CarlaReady
                fprintf('[testCarlaPlanningDecision] No live CARLA server reachable - LIVE tests SKIPPED (not failed). Frame-conversion LOGIC tests still run without CARLA.\n');
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
            pause(2.0);
            egoState0 = carlaGetEgoState();
        end
    end

    methods (Test)
        %% 1. Frame round-trip is the exact mathematical inverse of
        % carlaPerceptionStep.m's internal worldToEgoFrame - no CARLA
        % needed, this is pure geometry.
        function testFrameRoundTripRecoversGlobalPosition(testCase)
            egoState = struct('x', -63.90, 'y', -135.42, 'yaw', deg2rad(1.30));
            trueGlobalPos = [-48.91, -135.08]; % a real hero-scene actor position
            c = cos(egoState.yaw); s = sin(egoState.yaw);
            d = trueGlobalPos - [egoState.x, egoState.y];
            egoRelativePos = [c*d(1) + s*d(2), -s*d(1) + c*d(2)]; % worldToEgoFrame's own transform

            agent = createAgent();
            agent.position = egoRelativePos;
            agent.velocity = [1.5, -0.5];
            recovered = carlaFusedAgentsToGlobal(agent, egoState);

            testCase.verifyEqual(recovered.position, trueGlobalPos, 'AbsTol', 1e-9, ...
                'carlaFusedAgentsToGlobal must exactly invert worldToEgoFrame''s position transform.');
        end

        %% 2. Velocity is rotated consistently with position (same yaw,
        % no translation component - velocity round-trip must also hold).
        function testFrameRoundTripRecoversGlobalVelocity(testCase)
            egoState = struct('x', 10, 'y', -20, 'yaw', deg2rad(37));
            trueGlobalVel = [2.0, -3.0];
            c = cos(egoState.yaw); s = sin(egoState.yaw);
            egoRelativeVel = [c*trueGlobalVel(1) + s*trueGlobalVel(2), -s*trueGlobalVel(1) + c*trueGlobalVel(2)];

            agent = createAgent();
            agent.position = [5, 5];
            agent.velocity = egoRelativeVel;
            recovered = carlaFusedAgentsToGlobal(agent, egoState);

            testCase.verifyEqual(recovered.velocity, trueGlobalVel, 'AbsTol', 1e-9, ...
                'Velocity must round-trip through the same rotation as position.');
        end

        %% 3. Empty input never errors (matches every other CARLA
        % integration helper's "never crash, never fabricate" discipline).
        function testFrameConversionHandlesEmptyInput(testCase)
            egoState = struct('x', 0, 'y', 0, 'yaw', 0);
            result = carlaFusedAgentsToGlobal(repmat(createAgent(), 0, 0), egoState);
            testCase.verifyEqual(numel(result), 0, 'Empty input must return empty output without error.');
        end

        %% 4. LIVE: a pedestrian genuinely crossing into the ego's path
        % must escalate the decision state beyond cruise. This is the
        % single strongest regression guard for the frame bug: before the
        % fix, this scenario (and every other staged conflict) produced
        % decisionState="cruise" for 100% of ticks regardless of how
        % close the actor came, because behaviorDecision.m was comparing
        % an ego-relative agent position against the ego's own global
        % position.
        function testCrossingPedestrianEscalatesDecision(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('walker.pedestrian.0001', 12, -4, 0.5, 0.0);
            testCase.assumeFalse(isempty(actorId), 'Spawn collided at this offset - inconclusive.');

            loopState = carlaClosedLoopInit(60, 4.0, 25);
            statesSeen = strings(1, 0);
            for k = 1:60
                carlaSetActorVelocityRelativeToEgo(actorId, 0.0, 0.7, 0.0);
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                statesSeen(end+1) = report.decisionState; %#ok<AGROW>
            end
            testCase.verifyTrue(any(statesSeen ~= "cruise"), ...
                'A pedestrian genuinely crossing into the ego''s path must produce at least one non-cruise decision.');
        end

        %% 5. LIVE: the same genuine conflict must cause the planner to
        % reject at least one candidate trajectory (feasibleCandidateCount
        % < totalCandidates on at least one tick) - before the fix this
        % was 15/15 feasible on every single tick, always.
        function testGenuineConflictRejectsAtLeastOneCandidate(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 12, 6, 1.5, -90.0);
            testCase.assumeFalse(isempty(actorId), 'Spawn collided at this offset - inconclusive.');

            loopState = carlaClosedLoopInit(60, 4.0, 25);
            anyRejected = false;
            for k = 1:60
                carlaSetActorVelocityRelativeToEgo(actorId, 0.0, -1.0, 0.0);
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                if report.feasibleCandidateCount < report.totalCandidates
                    anyRejected = true;
                end
            end
            testCase.verifyTrue(anyRejected, ...
                'A crossing vehicle actually entering the corridor must cause the planner to reject at least one candidate on at least one tick.');
        end

        %% 6. LIVE: minTTC must become finite for a genuine close-range
        % conflict (Inf on every tick, always, was the bug's signature).
        function testGenuineConflictProducesFiniteTTC(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            actorId = carlaSpawnActorRelativeToEgo('vehicle.bh.crossbike', 12, 3, 1.5, -90.0);
            testCase.assumeFalse(isempty(actorId), 'Spawn collided at this offset - inconclusive.');

            loopState = carlaClosedLoopInit(60, 4.0, 25);
            minTTCObserved = Inf;
            for k = 1:60
                carlaSetActorVelocityRelativeToEgo(actorId, 0.0, -0.6, 0.0);
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                minTTCObserved = min(minTTCObserved, report.minTTC);
            end
            testCase.verifyLessThan(minTTCObserved, Inf, ...
                'A genuine close-range conflict must produce a finite minTTC on at least one tick.');
        end

        %% 7. LIVE: the curved intersection turn path (carlaGenerateIntersectionTurnPath.m)
        % is genuinely consumed by the real closed loop without error,
        % feeding a custom globalPath through carlaClosedLoopInit.m's
        % Phase 13 extension.
        function testCurvedTurnPathRunsThroughClosedLoopWithoutError(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            [~, egoState0] = testCase.connectAndBuildScene();
            [globalPath, turnInfo] = carlaGenerateIntersectionTurnPath( ...
                [egoState0.x, egoState0.y], egoState0.yaw, deg2rad(89.64), 12.0, 5.0, 25.0);
            vehCfg = vehicleConfig();
            feasibility = carlaCheckTurnFeasibility(globalPath, turnInfo, vehCfg);
            testCase.verifyTrue(feasibility.isFeasible, ...
                'The generated turn path must be within the vehicle''s curvature limits.');

            loopState = carlaClosedLoopInit(60, 4.0, 25, globalPath);
            for k = 1:15
                [loopState, report] = carlaClosedLoopStep(loopState); %#ok<NASGU>
            end
            testCase.verifyTrue(true, 'Reaching this line means 15 ticks ran with the curved path and no exception was thrown.');
        end

        %% 8. LIVE: controller-interface output stays within vehicleConfig
        % limits at all times (Phase 14 will consume these commands
        % directly - they must never exceed the physical vehicle limits
        % regardless of how the planner/decision layer reacts).
        function testControllerOutputStaysWithinVehicleLimits(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndBuildScene();
            vehCfg = vehicleConfig();
            loopState = carlaClosedLoopInit(60, 4.0, 25);
            for k = 1:40
                [loopState, report] = carlaClosedLoopStep(loopState);
                if report.skipped; continue; end
                testCase.verifyLessThanOrEqual(abs(report.controlCommand.steeringAngle), vehCfg.maxSteerAngle + 1e-6, ...
                    'Commanded steering angle must never exceed vehicleConfig.maxSteerAngle.');
                testCase.verifyGreaterThanOrEqual(report.controlCommand.throttle, 0, 'Throttle must be non-negative.');
                testCase.verifyLessThanOrEqual(report.controlCommand.throttle, 1, 'Throttle must not exceed 1.');
                testCase.verifyGreaterThanOrEqual(report.controlCommand.brake, 0, 'Brake must be non-negative.');
                testCase.verifyLessThanOrEqual(report.controlCommand.brake, 1, 'Brake must not exceed 1.');
            end
        end
    end
end
