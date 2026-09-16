classdef testPhase15DemoSafety < matlab.unittest.TestCase
    % Offline validation fixtures only; never used as live sensor evidence.
    methods (Test)
        function acceptsFiniteCurrentStreams(testCase)
            obs = testPhase15DemoSafety.fixture();
            carlaValidateHeroObservation(obs,carlaPerceptionConfig());
            testCase.verifyTrue(true);
        end
        function rejectsMissingRadar(testCase)
            obs = testPhase15DemoSafety.fixture(); obs.radarDetections = [];
            testCase.verifyError(@() carlaValidateHeroObservation(obs,carlaPerceptionConfig()),'Phase15:missingSensor');
        end
        function rejectsStalledStreamsEvenWhenMutuallySynchronized(testCase)
            obs = testPhase15DemoSafety.fixture(); obs.egoState.timestamp = 20;
            testCase.verifyError(@() carlaValidateHeroObservation(obs,carlaPerceptionConfig()),'Phase15:staleSensor');
        end
        function rejectsNonFiniteEgo(testCase)
            obs = testPhase15DemoSafety.fixture(); obs.egoState.x = NaN;
            testCase.verifyError(@() carlaValidateHeroObservation(obs,carlaPerceptionConfig()),'Phase15:invalidEgo');
        end
        function rejectsNonFinitePoints(testCase)
            obs = testPhase15DemoSafety.fixture(); obs.lidarPoints.xyzi(1) = Inf;
            testCase.verifyError(@() carlaValidateHeroObservation(obs,carlaPerceptionConfig()),'Phase15:invalidSensor');
        end
    end
    methods (Static, Access=private)
        function obs = fixture()
            obs.egoState = struct('x',0,'y',0,'yaw',0,'velocity',0,'timestamp',10);
            obs.cameraFrame = struct('timestamp',10);
            obs.lidarPoints = struct('timestamp',10,'xyzi',zeros(1,4));
            obs.radarDetections = struct('timestamp',10,'raw',zeros(0,4));
            obs.actorObjects = struct('timestamp',10,'raw',zeros(0,13));
        end
    end
end
