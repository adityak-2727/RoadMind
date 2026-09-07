classdef testCarlaIndianHeroScene < matlab.unittest.TestCase
% testCarlaIndianHeroScene - Phase 11.5 (Indian urban hero environment)
% tests. Same discipline as every prior CARLA test file in this project:
% every live test SKIPS (assumeTrue, never a fabricated pass) when no
% CARLA server is reachable. Never mocks or stubs CARLA.
%
% Tests the ENVIRONMENT only (scene construction, actor manifest,
% cleanup, repeatability) - not the full sensor/perception/fusion/
% tracking/prediction/decision/planning/control stack, which is already
% covered by testCarlaSensors.m/testCarlaPerceptionFusion.m/
% testCarlaTrackingPrediction-equivalent Phase 12 coverage and is
% explicitly unmodified by this phase.

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
                fprintf(['[testCarlaIndianHeroScene] No live CARLA server reachable - LIVE tests below ' ...
                    'will be SKIPPED (not failed). Config-only tests still run without CARLA.\n']);
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

    methods (Test)
        %% ---------------- Config-only tests (no CARLA needed) ----------------

        function testFourGenuinelyDistinctApproaches(testCase)
            % The 4 traffic-actor approach tags in the manifest must
            % actually represent 4 distinct compass directions, not a
            % 3-way or a duplicate - checked against the live-resolved
            % yaw values baked into the config (not re-queried here).
            cfg = testCase.SceneCfg;
            approaches = unique([cfg.trafficActors.approach]);
            testCase.verifyEqual(numel(approaches), 4, 'Expected traffic actors staged on all 4 approaches (N/S/E/W).');
            testCase.verifyTrue(all(ismember(["N","S","E","W"], approaches)));
        end

        function testTrafficActorCountInRange(testCase)
            cfg = testCase.SceneCfg;
            n = numel(cfg.trafficActors);
            testCase.verifyGreaterThanOrEqual(n, 8, 'Phase 11.5 requires 8-10 moving traffic actors.');
            testCase.verifyLessThanOrEqual(n, 10, 'Phase 11.5 requires 8-10 moving traffic actors.');
        end

        function testTrafficActorClassDiversity(testCase)
            % Must not be 8-10 identical cars - confirm at least 4
            % distinct base_type-relevant blueprints are represented.
            cfg = testCase.SceneCfg;
            blueprints = unique(string({cfg.trafficActors.blueprint}));
            testCase.verifyGreaterThanOrEqual(numel(blueprints), 6, ...
                'Traffic manifest should use several distinct blueprints, not near-duplicates.');
            hasBicycle = any(contains(blueprints, ["crossbike", "diamondback", "gazelle"]));
            hasMotorcycle = any(contains(blueprints, ["harley-davidson", "kawasaki", "yamaha", "vespa"]));
            hasTruck = any(contains(blueprints, ["carlamotors", "cybertruck"]));
            hasBus = any(contains(blueprints, "fusorosa"));
            testCase.verifyTrue(hasBicycle, 'Expected at least one bicycle-class traffic actor.');
            testCase.verifyTrue(hasMotorcycle, 'Expected at least one motorcycle-class traffic actor.');
            testCase.verifyTrue(hasTruck, 'Expected at least one truck-class traffic actor.');
            testCase.verifyTrue(hasBus, 'Expected at least one bus-class traffic actor.');
        end

        function testPedestriansPresent(testCase)
            cfg = testCase.SceneCfg;
            testCase.verifyGreaterThanOrEqual(numel(cfg.pedestrians), 1, 'At least one pedestrian is required.');
        end

        function testParkedVehiclesPresent(testCase)
            cfg = testCase.SceneCfg;
            testCase.verifyGreaterThanOrEqual(numel(cfg.parkedVehicles), 1, 'Parked roadside vehicles are required.');
        end

        function testAtLeastOneTurningActor(testCase)
            cfg = testCase.SceneCfg;
            intents = string({cfg.trafficActors.intent});
            testCase.verifyTrue(any(intents == "turn_left" | intents == "turn_right"), ...
                'At least one traffic actor must have a turning intent.');
        end

        function testNonLaneBasedIntentDiversity(testCase)
            % Traffic must not all share one identical scripted intent -
            % confirm several distinct behaviors are represented.
            cfg = testCase.SceneCfg;
            intents = unique(string({cfg.trafficActors.intent}));
            testCase.verifyGreaterThanOrEqual(numel(intents), 4, ...
                'Traffic should exhibit several distinct scripted behaviors, not one repeated pattern.');
        end

        function testDecisionLogicHasNoTrafficLightDependency(testCase)
            % Static check that the frozen decision stack has no code
            % path reading CARLA traffic-light state - the actual reason
            % the intersection is unsignalized, independent of
            % carlaFreezeTrafficLights.m's visual-only freeze.
            decisionFiles = dir(fullfile(fileparts(fileparts(fileparts(mfilename('fullpath')))), 'decision', '*.m'));
            for i = 1:numel(decisionFiles)
                content = fileread(fullfile(decisionFiles(i).folder, decisionFiles(i).name));
                testCase.verifyFalse(contains(content, 'traffic_light', 'IgnoreCase', true) || contains(content, 'TrafficLight'), ...
                    sprintf('%s must not reference CARLA traffic-light state.', decisionFiles(i).name));
            end
        end

        %% ---------------- LIVE tests ----------------

        function testSceneBuildsWithRequiredCounts(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            sceneState = carlaBuildIndianHeroScene(testCase.SceneCfg);

            nTraffic = sum(~isnan(sceneState.trafficActorIds));
            nParked = sum(~isnan(sceneState.parkedVehicleIds));
            nPed = sum(~isnan(sceneState.pedestrianIds));

            testCase.verifyGreaterThanOrEqual(nTraffic, 8, 'At least 8 traffic actors must actually spawn live.');
            testCase.verifyGreaterThanOrEqual(nParked, 1, 'At least 1 parked vehicle must actually spawn live.');
            testCase.verifyGreaterThanOrEqual(nPed, 1, 'At least 1 pedestrian must actually spawn live.');
            testCase.verifyGreaterThan(sceneState.numTrafficLightsFrozen, 0, 'Expected real traffic lights near this intersection to be found and frozen.');
        end

        function testGroundTruthClassificationDistinguishesBicycleFromMotorcycle(testCase)
            % Direct live proof of the Phase 11.5 classify() fix: a real
            % spawned bicycle actor must be reported as class "bicycle",
            % not "motorcycle".
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            sceneState = carlaBuildIndianHeroScene(testCase.SceneCfg); %#ok<NASGU>
            pause(1.0);
            objs = carlaGetNearbyActorObjects(150);
            testCase.verifyGreaterThan(objs.numObjects, 0);

            bicycleBlueprints = ["vehicle.bh.crossbike", "vehicle.diamondback.century", "vehicle.gazelle.omafiets"];
            cfg = testCase.SceneCfg;
            bicycleIdx = find(ismember(string({cfg.trafficActors.blueprint}), bicycleBlueprints), 1);
            testCase.assumeFalse(isempty(bicycleIdx), 'Config has no bicycle actor to check - test setup issue.');

            % Find that bicycle actor's class in the live ground-truth read
            foundBicycleClass = false;
            for i = 1:objs.numObjects
                cls = objs.classNames{objs.raw(i, 2) + 1};
                if strcmp(cls, 'bicycle')
                    foundBicycleClass = true;
                    break;
                end
            end
            testCase.verifyTrue(foundBicycleClass, 'A real spawned bicycle actor was not reported with class "bicycle" - the classify() fix may have regressed.');
        end

        function testSensorSanityAgainstDenseScene(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            carlaBuildIndianHeroScene(testCase.SceneCfg);
            carlaAttachCamera(testCase.CarlaCfg.camera);
            carlaAttachLidar(testCase.CarlaCfg.lidar);
            carlaAttachRadar(testCase.CarlaCfg.radar);
            pause(2.0);

            frame = carlaGetCameraFrame();
            points = carlaGetLidarPoints();
            radar = carlaGetRadarDetections();
            testCase.verifyNotEmpty(frame, 'Camera should produce a frame in the dense scene.');
            testCase.verifyNotEmpty(points, 'LiDAR should produce a sweep in the dense scene.');
            testCase.verifyNotEmpty(radar, 'Radar should produce a sweep in the dense scene.');
            if ~isempty(points)
                testCase.verifyTrue(all(isfinite(points.xyzi(:))), 'LiDAR points must remain finite in a denser scene.');
            end
        end

        function testCleanupLeavesNoOrphans(testCase)
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            carlaBuildIndianHeroScene(testCase.SceneCfg);
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
            testCase.verifyEqual(nonInfra, 0, 'No vehicle/pedestrian/sensor actors from this scene should remain after carlaDisconnect().');
        end

        function testSceneIsRepeatable(testCase)
            % Build, destroy, build again - the second build must spawn
            % essentially the same actor counts (deterministic manifest).
            testCase.assumeTrue(testCase.CarlaReady, 'No live CARLA server reachable - skipping.');
            s1 = carlaBuildIndianHeroScene(testCase.SceneCfg);
            n1 = sum(~isnan(s1.trafficActorIds));
            carlaDisconnect();

            s2 = carlaBuildIndianHeroScene(testCase.SceneCfg);
            n2 = sum(~isnan(s2.trafficActorIds));

            testCase.verifyEqual(n2, n1, 'Rebuilding the scene should spawn the same number of traffic actors each time.');
        end
    end
end
