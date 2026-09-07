classdef testCarlaSensors < matlab.unittest.TestCase
% testCarlaSensors - Phase 10 (CARLA Sensor Simulation) integration
% tests: camera/LiDAR/radar acquisition, the common-representation
% converters, sensor-interface robustness, coordinate-transform
% correctness, and sensor lifecycle/cleanup. Same discipline as
% testCarlaIntegration.m (Phase 9): every test SKIPS (via assumeTrue,
% never a fabricated pass) when no live CARLA server is reachable - it
% never mocks or stubs CARLA.
%
% These test the SENSOR INTERFACE only (Phase 10 scope) - not sensor
% fusion, not the planner/decision stack (Phase 11, explicitly deferred).
%
% Does not modify or duplicate testCarlaIntegration.m's Phase 9 tests
% (connection/spawn/state/control/response/cleanup) - this file adds
% only the Phase 10 sensor-specific coverage.

    properties
        CarlaCfg
        CarlaReady logical
    end

    methods (TestClassSetup)
        function addProjectToPath(testCase) %#ok<INUSD>
            root = fileparts(fileparts(fileparts(mfilename('fullpath')))); % carlaIntegration/tests -> carlaIntegration -> repo root
            addpath(genpath(root));
        end

        function checkCarlaAvailability(testCase)
            testCase.CarlaCfg = carlaConfig();
            testCase.CarlaReady = isCarlaAvailable(testCase.CarlaCfg);
            if ~testCase.CarlaReady
                fprintf(['[testCarlaSensors] No live CARLA server reachable at %s:%d - every test ' ...
                    'below will be SKIPPED (not failed). Expected on this development machine; see ' ...
                    'docs/carla_integration.md.\n'], testCase.CarlaCfg.host, testCase.CarlaCfg.port);
            end
        end
    end

    methods (TestMethodTeardown)
        function cleanupAfterEachTest(testCase)
            if testCase.CarlaReady
                carlaDisconnect(); % safe to call even if this test didn't spawn anything
            end
        end
    end

    methods (Access = private)
        function connectAndSpawnEgo(testCase)
            carlaConnect(testCase.CarlaCfg);
            carlaSpawnEgoVehicle(testCase.CarlaCfg);
        end

        function waitForData(~, getterFn, timeoutSeconds)
            % Polls getterFn() (a function handle returning [] until data
            % arrives) until non-empty or timeoutSeconds elapses.
            t0 = tic;
            while toc(t0) < timeoutSeconds
                result = getterFn();
                if ~isempty(result)
                    return;
                end
                pause(0.2);
            end
        end
    end

    methods (Test)
        function testCameraProducesRealFrame(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndSpawnEgo();
            carlaAttachCamera(testCase.CarlaCfg.camera);
            testCase.waitForData(@carlaGetCameraFrame, 5.0);

            frame = carlaGetCameraFrame();
            testCase.verifyNotEmpty(frame, 'No camera frame arrived within the timeout.');
            testCase.verifyEqual(size(frame.image), [testCase.CarlaCfg.camera.height, testCase.CarlaCfg.camera.width, 3], ...
                'Camera image dimensions do not match the configured width/height.');
            testCase.verifyClass(frame.image, 'uint8', 'Camera image must be uint8 RGB.');
            testCase.verifyTrue(isfinite(frame.timestamp) && frame.timestamp >= 0, 'Camera timestamp must be a finite, non-negative real value.');
        end

        function testLidarProducesRealPoints(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndSpawnEgo();
            carlaAttachLidar(testCase.CarlaCfg.lidar);
            testCase.waitForData(@carlaGetLidarPoints, 5.0);

            points = carlaGetLidarPoints();
            testCase.verifyNotEmpty(points, 'No LiDAR sweep arrived within the timeout.');
            testCase.verifyGreaterThan(points.numPoints, 0, 'A LiDAR sweep in an open scene should contain points.');
            testCase.verifySize(points.xyzi, [points.numPoints, 4], 'LiDAR points must be an Nx4 [x,y,z,intensity] array.');
            testCase.verifyTrue(all(isfinite(points.xyzi(:))), 'LiDAR points must be finite.');

            agents = carlaLidarPointsToAgents(points);
            for i = 1:numel(agents)
                testCase.verifyEqual(agents(i).class, "unknown", 'LiDAR-only agents must never be assigned an invented class.');
                testCase.verifyEqual(agents(i).source, "carla_lidar");
            end
        end

        function testRadarProducesRealDetectionsWithVelocity(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndSpawnEgo();
            carlaAttachRadar(testCase.CarlaCfg.radar);
            testCase.waitForData(@carlaGetRadarDetections, 5.0);

            radar = carlaGetRadarDetections();
            testCase.verifyNotEmpty(radar, 'No radar sweep arrived within the timeout.');
            if radar.numDetections > 0
                testCase.verifySize(radar.raw, [radar.numDetections, 4], 'Radar detections must be an Nx4 [depth,azimuth,altitude,velocity] array.');
                testCase.verifyTrue(all(isfinite(radar.raw(:))), 'Radar detections must be finite.');
                agents = carlaRadarToAgents(radar);
                testCase.verifyNumElements(agents, radar.numDetections);
                for i = 1:numel(agents)
                    testCase.verifyTrue(isfinite(agents(i).velocity(1)) && isfinite(agents(i).velocity(2)), ...
                        'Radar-derived agent velocity must be finite (radar is the one sensor here that reports real velocity).');
                end
            end
        end

        function testCameraOnlyRobustness(testCase)
            % Sensor-interface robustness: camera attached alone (no
            % LiDAR/radar) must work standalone and not require the
            % other sensors to be present.
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndSpawnEgo();
            carlaAttachCamera(testCase.CarlaCfg.camera);
            testCase.waitForData(@carlaGetCameraFrame, 5.0);

            frame = carlaGetCameraFrame();
            testCase.verifyNotEmpty(frame, 'Camera-only attach should still deliver frames.');
        end

        function testLidarOnlyRobustness(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndSpawnEgo();
            carlaAttachLidar(testCase.CarlaCfg.lidar);
            testCase.waitForData(@carlaGetLidarPoints, 5.0);

            points = carlaGetLidarPoints();
            testCase.verifyNotEmpty(points, 'LiDAR-only attach should still deliver sweeps.');
        end

        function testWeakOrMissingObservationsReturnEmptyNotError(testCase)
            % Immediately after attaching (before the first async
            % callback has fired), getters must return [] gracefully -
            % never throw, never fabricate a frame.
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndSpawnEgo();
            carlaAttachCamera(testCase.CarlaCfg.camera);
            carlaAttachLidar(testCase.CarlaCfg.lidar);
            carlaAttachRadar(testCase.CarlaCfg.radar);

            frame = carlaGetCameraFrame();
            points = carlaGetLidarPoints();
            radar = carlaGetRadarDetections();
            % Immediately after attach, data may or may not have arrived
            % yet depending on scheduling - the contract under test is
            % only that a missing observation is [] and never an error.
            testCase.verifyTrue(isempty(frame) || isstruct(frame));
            testCase.verifyTrue(isempty(points) || isstruct(points));
            testCase.verifyTrue(isempty(radar) || isstruct(radar));

            testCase.verifyEmpty(carlaLidarPointsToAgents([]), 'Converter must handle a missing ([]) LiDAR sweep gracefully.');
            testCase.verifyEmpty(carlaRadarToAgents([]), 'Converter must handle a missing ([]) radar sweep gracefully.');
            testCase.verifyEmpty(carlaActorObjectsToAgents([]), 'Converter must handle missing ([]) actor objects gracefully.');
        end

        function testDuplicateObservationsDoNotCrash(testCase)
            % Reading a sensor twice in a row, faster than a new frame
            % can arrive, must return the same latest frame both times
            % without erroring - the "duplicate observation" robustness
            % case for a bounded depth-1 latest-frame interface.
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndSpawnEgo();
            carlaAttachCamera(testCase.CarlaCfg.camera);
            testCase.waitForData(@carlaGetCameraFrame, 5.0);

            frameA = carlaGetCameraFrame();
            frameB = carlaGetCameraFrame();
            testCase.verifyNotEmpty(frameA);
            testCase.verifyNotEmpty(frameB);
            testCase.verifyEqual(frameB.frame, frameA.frame, ...
                'Reading the camera twice back-to-back (faster than a new async frame can arrive) should return the same frame number, not error or silently mix frames.');
        end

        function testCoordinateTransformAheadAndRight(testCase)
            % Coordinate-system verification: an actor spawned directly
            % ahead of the ego must land on the ego's local +x axis;
            % spawned to CARLA's "right" must land on the ego's local
            % -y axis (project frame), confirming the axis mirror in
            % carlaCoordToProject.m end-to-end against a live server.
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            testCase.connectAndSpawnEgo();

            egoState = carlaGetEgoState();
            aheadId = carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 10.0, 0.0, 0.5, 0.0);
            testCase.assumeTrue(~isempty(aheadId), 'Ahead-actor spawn collided at this spawn point - inconclusive, not a failure.');
            pause(0.2);
            rawAhead = carlaGetActorState(aheadId);
            [xP, yP] = carlaCoordToProject(rawAhead.location.x, rawAhead.location.y);
            dx = xP - egoState.x; dy = yP - egoState.y;
            localX = cos(egoState.yaw) * dx + sin(egoState.yaw) * dy;
            localY = -sin(egoState.yaw) * dx + cos(egoState.yaw) * dy;
            testCase.verifyEqual(localX, 10.0, 'AbsTol', 0.5, 'An actor spawned 10m ahead must land ~10m ahead in the project frame.');
            testCase.verifyEqual(localY, 0.0, 'AbsTol', 0.5, 'An actor spawned directly ahead must have ~zero lateral offset.');

            rightId = carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 0.0, 5.0, 0.5, 0.0);
            testCase.assumeTrue(~isempty(rightId), 'Right-actor spawn collided at this spawn point - inconclusive, not a failure.');
            pause(0.2);
            rawRight = carlaGetActorState(rightId);
            [xP2, yP2] = carlaCoordToProject(rawRight.location.x, rawRight.location.y);
            dx2 = xP2 - egoState.x; dy2 = yP2 - egoState.y;
            localX2 = cos(egoState.yaw) * dx2 + sin(egoState.yaw) * dy2;
            localY2 = -sin(egoState.yaw) * dx2 + cos(egoState.yaw) * dy2;
            testCase.verifyEqual(localX2, 0.0, 'AbsTol', 0.5, 'An actor spawned to the side must have ~zero forward offset.');
            testCase.verifyEqual(localY2, -5.0, 'AbsTol', 0.5, 'An actor spawned 5m to CARLA''s right must land at project-frame lateral -5m (verified sign convention).');
        end

        function testSensorLifecycleNoOrphanActors(testCase)
            % Full lifecycle: spawn -> attach sensors -> data flows ->
            % disconnect -> zero orphan sensor/vehicle actors remain.
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            carlaConnect(testCase.CarlaCfg);
            carlaSpawnEgoVehicle(testCase.CarlaCfg);
            carlaAttachCamera(testCase.CarlaCfg.camera);
            carlaAttachLidar(testCase.CarlaCfg.lidar);
            carlaAttachRadar(testCase.CarlaCfg.radar);
            testId = carlaSpawnActorRelativeToEgo('vehicle.audi.tt', 8.0, 0.0, 0.5, 0.0);
            pause(1.0);

            carlaDisconnect();

            session = getCarlaSession();
            testCase.verifyFalse(session.Connected, 'Session must report disconnected after carlaDisconnect().');

            % Independent orphan check: a fresh raw py.carla.Client query
            % (bypassing CarlaSession entirely) confirms no sensor/vehicle
            % actor from this test survives cleanup - matching the manual
            % verification already performed during Phase 10 development.
            py.importlib.import_module('carla');
            freshClient = py.carla.Client(testCase.CarlaCfg.host, int32(testCase.CarlaCfg.port));
            freshClient.set_timeout(testCase.CarlaCfg.timeoutSeconds);
            worldActors = cell(py.list(freshClient.get_world().get_actors()));

            sensorCount = 0; vehicleCount = 0;
            for i = 1:numel(worldActors)
                typeId = char(worldActors{i}.type_id);
                if startsWith(typeId, 'sensor.')
                    sensorCount = sensorCount + 1;
                elseif startsWith(typeId, 'vehicle.')
                    vehicleCount = vehicleCount + 1;
                end
            end
            testCase.verifyEqual(sensorCount, 0, 'No sensor actors from this test should remain after carlaDisconnect().');
            testCase.verifyEqual(vehicleCount, 0, 'No vehicle actors (ego or test actor) from this test should remain after carlaDisconnect().');
        end
    end
end
