classdef testCollisionCheck < matlab.unittest.TestCase
% testCollisionCheck - regression tests for planning/collisionCheck.m.
% Calls the actual, unmodified collisionCheck function against small,
% hand-built trajectories - never reimplements its distance/threshold
% logic. See planning/collisionCheck.m for the real signature/behavior
% these tests protect:
%   [isColliding, minTTC] = collisionCheck(egoTrajectory,
%     predictedTrajectories, vehicleConfig)
% egoRadius=1.1m and PLANNING_DT=0.1s are collisionCheck's own internal
% constants (not passed in) - this file does not duplicate them, it only
% engineers fixtures whose expected minTTC follows from a known row index
% times 0.1s.
%
% Note: collisionCheck itself has no concept of a "critical TTC
% threshold" - it only returns minTTC. The critical-threshold comparison
% (plannerConfig.ttcThresholds.critical) lives in adaptivePlanner, so
% that behavior is regression-tested in tests/testPlanner.m instead, not
% here.

    methods (TestClassSetup)
        function addProjectToPath(testCase) %#ok<INUSD>
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(genpath(root));
        end
    end

    methods (Test)
        function testSeparatedTrajectoryIsNotColliding(testCase)
            % A candidate and a predicted agent trajectory that never come
            % within threshold distance must not be flagged as colliding.
            ego = [(1:10)', zeros(10, 1)]; % straight line along y=0
            pred = {[(1:10)', 50 * ones(10, 1), 0.5 * ones(10, 1)]}; % far away, y=50
            vehCfg = vehicleConfig();

            [isColliding, minTTC] = collisionCheck(ego, pred, vehCfg);

            testCase.verifyFalse(isColliding, ...
                'collisionCheck flagged a collision for trajectories 50m apart - separated trajectories must not be classified as colliding.');
            testCase.verifyEqual(minTTC, Inf, ...
                'minTTC should be Inf when no predicted overlap exists.');
        end

        function testOverlappingTrajectoryIsColliding(testCase)
            % A predicted agent trajectory that coincides with the ego
            % candidate's path must be flagged as colliding.
            ego = [(1:10)', zeros(10, 1)];
            pred = {[(1:10)', zeros(10, 1), 0.1 * ones(10, 1)]}; % same path, small uncertainty radius
            vehCfg = vehicleConfig();

            [isColliding, minTTC] = collisionCheck(ego, pred, vehCfg);

            testCase.verifyTrue(isColliding, ...
                'collisionCheck did not flag a collision for two trajectories occupying the same points - overlapping trajectories must be classified as colliding.');
            testCase.verifyLessThan(minTTC, Inf, ...
                'minTTC must be finite when a collision is flagged.');
        end

        function testMinTTCMatchesEarliestViolation(testCase)
            % TTC behavior: minTTC must equal the timestep index of the
            % FIRST predicted overlap, at PLANNING_DT=0.1s per step -
            % built from a fixture where the violation is engineered to
            % start at a known row (row 5), not by recomputing
            % collisionCheck's own search internally.
            PLANNING_DT = 0.1;
            violationRow = 5;
            ego = [(1:10)', zeros(10, 1)];
            predPositions = 100 * ones(10, 2); % far away everywhere...
            predPositions(violationRow:end, :) = ego(violationRow:end, :); % ...except from row 5 onward, right on top of ego
            pred = {[predPositions, 0.05 * ones(10, 1)]};
            vehCfg = vehicleConfig();

            [isColliding, minTTC] = collisionCheck(ego, pred, vehCfg);

            testCase.verifyTrue(isColliding, 'Expected a collision once the predicted trajectory reaches the ego path at row 5.');
            testCase.verifyEqual(minTTC, violationRow * PLANNING_DT, 'AbsTol', 1e-9, ...
                sprintf('minTTC should equal the first-violation row (%d) times PLANNING_DT (%.2fs) = %.2fs.', ...
                violationRow, PLANNING_DT, violationRow * PLANNING_DT));
        end

        function testEarliestViolationAcrossMultipleAgents(testCase)
            % When multiple predicted agents are checked, minTTC must be
            % the EARLIEST violation across all of them, not just the
            % first agent in the list.
            ego = [(1:10)', zeros(10, 1)];
            farAgent = [100 * ones(10, 1), 100 * ones(10, 1), 0.1 * ones(10, 1)]; % never close
            nearAgent = 100 * ones(10, 3);
            nearAgent(3:end, 1:2) = ego(3:end, :); % collides starting row 3
            nearAgent(:, 3) = 0.05;
            pred = {farAgent, nearAgent};
            vehCfg = vehicleConfig();

            [isColliding, minTTC] = collisionCheck(ego, pred, vehCfg);

            testCase.verifyTrue(isColliding, 'Expected a collision from the near agent.');
            testCase.verifyEqual(minTTC, 3 * 0.1, 'AbsTol', 1e-9, ...
                'minTTC must reflect the earliest violation among all predicted agents (row 3), not be skewed by the unrelated far agent.');
        end

        function testEmptyPredictedTrajectoriesIsNotColliding(testCase)
            % Documented edge case in collisionCheck.m: no predicted
            % trajectories at all must never be treated as a collision.
            ego = [(1:10)', zeros(10, 1)];
            vehCfg = vehicleConfig();

            [isColliding, minTTC] = collisionCheck(ego, {}, vehCfg);

            testCase.verifyFalse(isColliding, 'An empty predictedTrajectories input must never be classified as a collision.');
            testCase.verifyEqual(minTTC, Inf, 'minTTC must be Inf with no predicted trajectories.');
        end
    end
end
