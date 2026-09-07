classdef testCarlaIntegration < matlab.unittest.TestCase
% testCarlaIntegration - Phase 9 integration tests (Tests 1-6 from the
% phase spec). Every test SKIPS (via assumeTrue, marked "Filtered"/
% incomplete by matlab.unittest - not failed, and never a fabricated
% pass) when no live CARLA server is reachable. That is the honest,
% expected result on this development machine (no CARLA installation was
% found - see docs/carla_integration.md for the full inspection). These
% tests exercise the real carla/matlab/*.m adapter functions end-to-end
% against an actual CARLA server; they intentionally never mock or stub
% CARLA, since a mocked pass would not prove the integration works.
%
% "Test 7 - MATLAB regression" from the phase spec is not duplicated
% here: it is the EXISTING tests/testCollisionCheck.m, testPlanner.m,
% testPrediction.m suite (Phase 9 - Automated Regression Testing, already
% complete and frozen), re-run as part of this phase's own validation
% step rather than reimplemented in this file.

    properties
        CarlaCfg
        CarlaReady logical
    end

    methods (TestClassSetup)
        function addProjectToPath(testCase) %#ok<INUSD>
            root = fileparts(fileparts(fileparts(mfilename('fullpath')))); % carla/tests -> carla -> repo root
            addpath(genpath(root));
        end

        function checkCarlaAvailability(testCase)
            testCase.CarlaCfg = carlaConfig();
            testCase.CarlaReady = isCarlaAvailable(testCase.CarlaCfg);
            if ~testCase.CarlaReady
                fprintf(['[testCarlaIntegration] No live CARLA server reachable at %s:%d - every test ' ...
                    'below will be SKIPPED (not failed). Expected on this development machine; see ' ...
                    'docs/carla_integration.md.\n'], testCase.CarlaCfg.host, testCase.CarlaCfg.port);
            end
        end
    end

    methods (TestMethodTeardown)
        function cleanupAfterEachTest(testCase)
            if testCase.CarlaReady
                carlaDisconnect(); % Task 3/9: safe to call even if this test didn't spawn anything
            end
        end
    end

    methods (Test)
        function testConnection(testCase)
            % Test 1 - CARLA connection: expected "connection successful".
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping (see docs/carla_integration.md).');

            carlaConnect(testCase.CarlaCfg);
            session = getCarlaSession();
            testCase.verifyTrue(session.Connected, 'carlaConnect() completed without error but session.Connected is false.');
        end

        function testEgoVehicleSpawn(testCase)
            % Test 2 - ego vehicle spawn: expected "valid CARLA actor".
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping (see docs/carla_integration.md).');

            carlaConnect(testCase.CarlaCfg);
            actorId = carlaSpawnEgoVehicle(testCase.CarlaCfg);

            testCase.verifyClass(actorId, 'double', 'Expected a numeric CARLA actor id.');
            testCase.verifyGreaterThan(actorId, 0, 'A valid CARLA actor id must be positive.');
        end

        function testStateRetrieval(testCase)
            % Test 3 - state retrieval: expected "valid position/velocity/heading".
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping (see docs/carla_integration.md).');

            carlaConnect(testCase.CarlaCfg);
            carlaSpawnEgoVehicle(testCase.CarlaCfg);
            egoState = carlaGetEgoState();

            testCase.verifyTrue(isfinite(egoState.x) && isfinite(egoState.y), 'Ego position must be finite.');
            testCase.verifyTrue(isfinite(egoState.yaw), 'Ego heading must be finite.');
            testCase.verifyGreaterThanOrEqual(egoState.velocity, 0, 'Ego speed (scalar) must be non-negative.');
        end

        function testControlCommandReachesVehicle(testCase)
            % Test 4 - control command: expected "command reaches vehicle".
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping (see docs/carla_integration.md).');

            carlaConnect(testCase.CarlaCfg);
            carlaSpawnEgoVehicle(testCase.CarlaCfg);

            applied = carlaApplyControl(0.0, 0.3, 0.0);
            testCase.verifyEqual(applied.throttle, 0.3, 'AbsTol', 1e-6, ...
                'The throttle command echoed back from carla_adapter.py does not match what was sent.');
        end

        function testVehicleRespondsToControl(testCase)
            % Test 5 - vehicle response: expected "vehicle state changes
            % consistently with applied test input". Applies a small,
            % controlled throttle for a short, fixed duration and confirms
            % speed increased - an integration-channel check only, NOT a
            % scripted autonomous trajectory.
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping (see docs/carla_integration.md).');

            carlaConnect(testCase.CarlaCfg);
            carlaSpawnEgoVehicle(testCase.CarlaCfg);

            stateBefore = carlaGetEgoState();
            carlaApplyControl(0.0, 0.5, 0.0);
            pause(1.5);
            stateAfter = carlaGetEgoState();
            carlaApplyControl(0.0, 0.0, 1.0); % brake to a stop before cleanup

            testCase.verifyGreaterThan(stateAfter.velocity, stateBefore.velocity, ...
                'Applying throttle did not measurably increase ego speed - the control channel may not be reaching the vehicle.');
        end

        function testCleanup(testCase)
            % Test 6 - cleanup: expected "CARLA actor is removed and
            % connection closes cleanly".
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping (see docs/carla_integration.md).');

            carlaConnect(testCase.CarlaCfg);
            carlaSpawnEgoVehicle(testCase.CarlaCfg);

            carlaDisconnect();
            session = getCarlaSession();
            testCase.verifyFalse(session.Connected, 'Session must report disconnected after carlaDisconnect().');

            carlaDisconnect(); % must not error when called a second time (idempotent cleanup)
            testCase.verifyTrue(true, 'Second carlaDisconnect() call completed without error.');
        end
    end
end
