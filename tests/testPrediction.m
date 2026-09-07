classdef testPrediction < matlab.unittest.TestCase
% testPrediction - regression tests for prediction/trajectoryPrediction.m,
% specifically the frozen K1 "predictable unknown" relaxation gate (see
% docs/architecture.md and the K1 hardening report). Calls the actual,
% unmodified trajectoryPrediction function repeatedly to build up its own
% internal history buffer exactly as main.m's tick loop does - never
% reimplements isPredictableUnknown's checks; the conservative/relaxed
% reference outputs are obtained by calling the real
% irregularMotionModel/constantVelocityPrediction functions directly, not
% by hand-computing their formulas.
%
% Each test uses a distinct, high, arbitrary agent id (900xx) so this
% file's repeated calls can never collide with another test's history,
% regardless of run order - trajectoryPrediction.m keeps its history
% buffer in a `persistent` map scoped to the whole MATLAB session, by
% design (see its own comments), so id isolation is required for
% deterministic, order-independent tests.

    properties (Constant)
        Dt = 0.1;
        Horizon = 4.0;
        HistoryFramesNeeded = 20; % must match trajectoryPrediction.m's HISTORY_WINDOW_FRAMES
    end

    methods (TestClassSetup)
        function addProjectToPath(testCase) %#ok<INUSD>
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(genpath(root));
        end
    end

    methods (Test)
        function testStableUnknownVehicleLikeObjectBecomesPredictable(testCase)
            % A. A track with >=20 consecutive frames of stable,
            % vehicle-speed, non-crossing motion must be eligible for
            % constant-velocity prediction (K1's core relaxation) - and
            % must NOT be relaxed one frame earlier, pinning the exact
            % history-length boundary.
            agentId = 90001;
            velocity = [3.0, 0.0]; % speed 3 m/s, well above K1's ~1.2 m/s floor, parallel to ego heading

            [predictedAt19, predictedAt20] = testCase.runStableHistory(agentId, velocity, testCase.HistoryFramesNeeded);

            agentAt19 = testCase.makeAgent(agentId, "unknown", velocity, (testCase.HistoryFramesNeeded - 1) * testCase.Dt);
            agentAt20 = testCase.makeAgent(agentId, "unknown", velocity, testCase.HistoryFramesNeeded * testCase.Dt);
            expectedConservativeAt19 = irregularMotionModel(agentAt19, testCase.Horizon, testCase.Dt);
            expectedRelaxedAt20 = constantVelocityPrediction(agentAt20, testCase.Horizon, testCase.Dt);

            testCase.verifyEqual(predictedAt19, expectedConservativeAt19, 'AbsTol', 1e-9, ...
                'K1 relaxed a stable unknown track before it had the required 20 frames of history - relaxation must not fire early.');
            testCase.verifyEqual(predictedAt20, expectedRelaxedAt20, 'AbsTol', 1e-9, ...
                'K1 failed to relax a genuinely stable, vehicle-speed unknown track after 20 consistent frames - the core predictable-unknown relaxation is broken.');
        end

        function testSlowUnknownVRUIsNeverRelaxed(testCase)
            % B. K1 VRU protection: a slow unknown object (below K1's
            % MIN_RELAXATION_SPEED) must stay on the conservative model no
            % matter how long/stable its history is - this is the exact
            % protection added after K1 validation found a real VRU
            % (villageRoad's animal, velocity [0.3,0.2]) being relaxed.
            agentId = 90002;
            velocity = [0.3, 0.2]; % speed ~0.36 m/s, well below the ~1.2 m/s floor

            numFrames = testCase.HistoryFramesNeeded + 10; % well past the history requirement
            predicted = [];
            for k = 1:numFrames
                agent = testCase.makeAgent(agentId, "unknown", velocity, k * testCase.Dt);
                predicted = trajectoryPrediction(agent, testCase.makeEgo(), testCase.Horizon, testCase.Dt);
            end

            agentFinal = testCase.makeAgent(agentId, "unknown", velocity, numFrames * testCase.Dt);
            expectedConservative = irregularMotionModel(agentFinal, testCase.Horizon, testCase.Dt);

            testCase.verifyEqual(predicted{1}, expectedConservative, 'AbsTol', 1e-9, ...
                ['K1 VRU protection failed: a slow unknown object (0.36 m/s) was relaxed into ' ...
                 'constant-velocity prediction despite being far below the minimum relaxation speed - ' ...
                 'this could let a real pedestrian/animal be under-predicted.']);
        end

        function testCrossingUnknownStaysConservative(testCase)
            % C. An unknown object currently classified "crossing" must
            % never be relaxed, no matter how stable its history becomes -
            % isUnpredictableMotion must gate relaxation before history is
            % even considered.
            agentId = 90003;
            velocity = [0.0, 3.0]; % perpendicular to ego heading (yaw=0) -> classifyBehavior = "crossing"

            category = classifyBehavior(testCase.makeAgent(agentId, "unknown", velocity, 0), 0);
            testCase.assertEqual(category, "crossing", ...
                'Test fixture error: velocity [0,3] against ego yaw=0 should classify as "crossing".');

            numFrames = testCase.HistoryFramesNeeded + 5;
            predicted = [];
            for k = 1:numFrames
                agent = testCase.makeAgent(agentId, "unknown", velocity, k * testCase.Dt);
                predicted = trajectoryPrediction(agent, testCase.makeEgo(), testCase.Horizon, testCase.Dt);
            end

            agentFinal = testCase.makeAgent(agentId, "unknown", velocity, numFrames * testCase.Dt);
            expectedConservative = irregularMotionModel(agentFinal, testCase.Horizon, testCase.Dt);

            testCase.verifyEqual(predicted{1}, expectedConservative, 'AbsTol', 1e-9, ...
                'K1 relaxed a crossing unknown object despite stable history - crossing motion must always stay conservative regardless of history.');
        end

        function testMergingUnknownStaysConservative(testCase)
            % D. Same protection as C, for "merging" motion.
            agentId = 90004;
            velocity = [3.0, 3.0]; % 45 degrees off ego heading -> classifyBehavior = "merging"

            category = classifyBehavior(testCase.makeAgent(agentId, "unknown", velocity, 0), 0);
            testCase.assertEqual(category, "merging", ...
                'Test fixture error: velocity [3,3] against ego yaw=0 should classify as "merging".');

            numFrames = testCase.HistoryFramesNeeded + 5;
            predicted = [];
            for k = 1:numFrames
                agent = testCase.makeAgent(agentId, "unknown", velocity, k * testCase.Dt);
                predicted = trajectoryPrediction(agent, testCase.makeEgo(), testCase.Horizon, testCase.Dt);
            end

            agentFinal = testCase.makeAgent(agentId, "unknown", velocity, numFrames * testCase.Dt);
            expectedConservative = irregularMotionModel(agentFinal, testCase.Horizon, testCase.Dt);

            testCase.verifyEqual(predicted{1}, expectedConservative, 'AbsTol', 1e-9, ...
                'K1 relaxed a merging unknown object despite stable history - merging motion must always stay conservative regardless of history.');
        end

        function testStoppedUnknownUsesConstantVelocityModel(testCase)
            % E. A stopped unknown agent is routed straight to
            % constantVelocityPrediction by trajectoryPrediction's
            % existing, pre-K1 dispatch (a stopped agent is never treated
            % as erratic, regardless of class) - K1's history-based
            % relaxation is never even consulted for a stopped agent.
            % This test confirms that pre-existing behavior still holds
            % and that K1 did not change it either way. Note: this means
            % a stopped unknown agent already uses the LESS conservative
            % of the two models by original (pre-K1) design - not a K1
            % effect, and not something this task is allowed to change.
            agentId = 90005;
            velocity = [0.0, 0.0]; % stopped

            agent = testCase.makeAgent(agentId, "unknown", velocity, testCase.Dt);
            category = classifyBehavior(agent, 0);
            testCase.assertEqual(category, "stopped", ...
                'Test fixture error: zero velocity should classify as "stopped".');

            predicted = trajectoryPrediction(agent, testCase.makeEgo(), testCase.Horizon, testCase.Dt);
            expected = constantVelocityPrediction(agent, testCase.Horizon, testCase.Dt);

            testCase.verifyEqual(predicted{1}, expected, 'AbsTol', 1e-9, ...
                ['A stopped unknown agent must use constantVelocityPrediction (trajectoryPrediction''s ' ...
                 'existing stopped-agent dispatch, unrelated to and unaffected by K1).']);
        end
    end

    methods (Access = private)
        function agent = makeAgent(~, id, class, velocity, timestamp)
            agent = createAgent();
            agent.id = id;
            agent.class = class;
            agent.position = velocity * timestamp; % consistent with constant velocity from the origin
            agent.velocity = velocity;
            agent.timestamp = timestamp;
        end

        function ego = makeEgo(~)
            ego = createEgoState();
            ego.yaw = 0; % reference "parallel to the road" direction for classifyBehavior
        end

        function [predictedAt19, predictedAt20] = runStableHistory(testCase, agentId, velocity, framesNeeded)
            for k = 1:(framesNeeded - 1)
                agent = testCase.makeAgent(agentId, "unknown", velocity, k * testCase.Dt);
                predicted = trajectoryPrediction(agent, testCase.makeEgo(), testCase.Horizon, testCase.Dt);
            end
            predictedAt19 = predicted{1};

            agent = testCase.makeAgent(agentId, "unknown", velocity, framesNeeded * testCase.Dt);
            predicted = trajectoryPrediction(agent, testCase.makeEgo(), testCase.Horizon, testCase.Dt);
            predictedAt20 = predicted{1};
        end
    end
end
