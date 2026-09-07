classdef testPlanner < matlab.unittest.TestCase
% testPlanner - regression tests for planning/adaptivePlanner.m (the
% frozen K2 feasibility-aware fallback). Calls the actual, unmodified
% adaptivePlanner function against small, hand-built candidate/predicted
% trajectory fixtures - never reimplements its cost function, safety
% screen, or fallback ranking. See planning/adaptivePlanner.m for the
% real signature these tests protect:
%   [selectedTrajectory, selectedIndex] = adaptivePlanner(egoState,
%     candidateTrajectories, predictedTrajectories, vehicleConfig,
%     plannerConfig, scenarioContext, previousIndex, debugFallback)

    methods (TestClassSetup)
        function addProjectToPath(testCase) %#ok<INUSD>
            root = fileparts(fileparts(mfilename('fullpath')));
            addpath(genpath(root));
        end
    end

    methods (Test)
        function testNormalSafeCandidateSelectionUnaffectedByK2(testCase)
            % A. With no obstacles, every candidate is safe; the planner
            % must use the normal min-cost-among-safe branch and pick the
            % lowest-cost (here: zero lateral-deviation centerline)
            % candidate - K2 only touches the no-safe-candidate fallback
            % branch and must never influence this path.
            numSteps = 10;
            xs = (1:numSteps)';
            candidates = { ...
                [xs, -2 * ones(numSteps, 1)], ... % offset -2
                [xs,  0 * ones(numSteps, 1)], ... % centerline
                [xs,  2 * ones(numSteps, 1)]  ...  % offset +2
            };
            ego = createEgoState();
            ego.velocity = 5;
            vehCfg = vehicleConfig();
            planCfg = plannerConfig();
            previousIndex = 2; % centerline, removes any consistency-cost tiebreak ambiguity

            [selectedTrajectory, selectedIndex] = adaptivePlanner(ego, candidates, {}, vehCfg, planCfg, "unitTest", previousIndex, false);

            testCase.verifyEqual(selectedIndex, 2, ...
                ['With all candidates safe and equal except lateral deviation, the zero-deviation ' ...
                 'centerline candidate must be selected - normal safe-candidate selection may have been altered.']);
            [isColliding, ~] = collisionCheck(selectedTrajectory, {}, vehCfg);
            testCase.verifyFalse(isColliding, 'The selected trajectory from the normal (safe-candidate) branch must not be colliding.');
        end

        function testUnsafeCandidateNeverReportedSafe(testCase)
            % B. Every candidate collides well below the critical TTC
            % threshold, so the planner must fall back - and the
            % trajectory it returns must still independently verify as
            % colliding when re-checked with the real collisionCheck. K2
            % must never let a candidate that failed the safety screen
            % come back out looking safe.
            numSteps = 10;
            xs = (1:numSteps)';
            candidates = {[xs, zeros(numSteps, 1)], [xs, 0.1 * ones(numSteps, 1)]};
            pred = {[xs, zeros(numSteps, 1), 0.05 * ones(numSteps, 1)]}; % sits on/near both candidates the whole time
            ego = createEgoState();
            ego.velocity = 5;
            vehCfg = vehicleConfig();
            planCfg = plannerConfig();

            [selectedTrajectory, selectedIndex] = adaptivePlanner(ego, candidates, pred, vehCfg, planCfg, "unitTest", 1, false);

            testCase.verifyNotEmpty(selectedIndex, 'adaptivePlanner must still return a fallback selection, not nothing, when no candidate is safe.');
            [isColliding, minTTC] = collisionCheck(selectedTrajectory, pred, vehCfg);
            testCase.verifyTrue(isColliding, ...
                ['K2 fallback safety-screen violation: the selected fallback trajectory was independently ' ...
                 're-checked with the real collisionCheck and came back as NOT colliding - a candidate that ' ...
                 'failed the safety screen must never be classified as safe.']);
            testCase.verifyLessThan(minTTC, planCfg.ttcThresholds.critical, ...
                'The selected fallback trajectory''s minTTC should be below the critical threshold, consistent with why fallback triggered.');
        end

        function testFallbackPrefersFeasibleCandidateOverHigherTTCInfeasibleOne(testCase)
            % C. Core K2 regression test. Two candidates, both fail the
            % safety screen (both collide below critical TTC):
            %   candidate 1: straight (curvature ~0, feasible), collides
            %                early -> LOWER minTTC
            %   candidate 2: sharp ~90-degree corner (curvature far above
            %                the vehicle's feasible limit), collides later
            %                -> HIGHER minTTC
            % The pre-K2 rule (max(minTTC) alone) would have picked
            % candidate 2. K2 must pick candidate 1 instead, since
            % candidate 2 is kinematically infeasible.
            n = 15;
            k = (1:n)';

            candidateFeasible = [k, zeros(n, 1)]; % straight along y=0
            candidateInfeasible = [[(1:8)', 5 * ones(8, 1)]; [8 * ones(7, 1), (6:12)']]; % straight, then a sharp ~90-degree corner

            obstacleNearFeasible = 100 * ones(n, 3);
            obstacleNearFeasible(3:end, 1:2) = candidateFeasible(3:end, :); % collides with candidate 1 from row 3
            obstacleNearFeasible(:, 3) = 0.05;

            obstacleNearInfeasible = -100 * ones(n, 3);
            obstacleNearInfeasible(10:end, 1:2) = candidateInfeasible(10:end, :); % collides with candidate 2 from row 10
            obstacleNearInfeasible(:, 3) = 0.05;

            ego = createEgoState();
            ego.velocity = 5;
            vehCfg = vehicleConfig();
            planCfg = plannerConfig();
            pred = {obstacleNearFeasible, obstacleNearInfeasible};

            [~, ttcFeasible] = collisionCheck(candidateFeasible, pred, vehCfg);
            [~, ttcInfeasible] = collisionCheck(candidateInfeasible, pred, vehCfg);
            testCase.assertGreaterThan(ttcInfeasible, ttcFeasible, ...
                ['Test fixture error: the infeasible candidate must have the HIGHER minTTC for this to be a ' ...
                 'meaningful K2 regression test (otherwise the old max(minTTC) rule would agree with K2 by coincidence).']);

            [~, selectedIndex] = adaptivePlanner(ego, {candidateFeasible, candidateInfeasible}, pred, vehCfg, planCfg, "unitTest", 1, false);

            testCase.verifyEqual(selectedIndex, 1, ...
                ['K2 feasibility-aware fallback regression: with candidate 2 kinematically infeasible but having ' ...
                 'a higher minTTC, and candidate 1 feasible with a lower minTTC, the planner must select the ' ...
                 'feasible candidate 1. Selecting candidate 2 would mean K2''s feasibility preference has been ' ...
                 'lost, reverting to the old max(minTTC)-only rule.']);
        end

        function testNoFeasibleCandidateKeepsOriginalLeastBadFallback(testCase)
            % D. When NO candidate is kinematically feasible, K2 must fall
            % back to the original, pre-K2 least-bad rule (max minTTC
            % among all candidates) rather than refusing to select
            % anything or inventing a new rule.
            n = 15;
            sharpTurn = @(yOffset) [[(1:8)', yOffset * ones(8, 1)]; [8 * ones(7, 1), (yOffset + 2:yOffset + 8)']];
            candidateA = sharpTurn(0);   % infeasible sharp corner
            candidateB = sharpTurn(20);  % infeasible sharp corner, far from candidateA

            obstacleNearA = 1000 * ones(n, 3);
            obstacleNearA(3:end, 1:2) = candidateA(3:end, :); % collides with A from row 3 -> lower minTTC
            obstacleNearA(:, 3) = 0.05;
            obstacleNearB = -1000 * ones(n, 3);
            obstacleNearB(8:end, 1:2) = candidateB(8:end, :); % collides with B from row 8 -> higher minTTC
            obstacleNearB(:, 3) = 0.05;

            ego = createEgoState();
            ego.velocity = 5;
            vehCfg = vehicleConfig();
            planCfg = plannerConfig();
            pred = {obstacleNearA, obstacleNearB};

            [~, ttcA] = collisionCheck(candidateA, pred, vehCfg);
            [~, ttcB] = collisionCheck(candidateB, pred, vehCfg);
            testCase.assertGreaterThan(ttcB, ttcA, 'Test fixture error: candidate B must have the higher minTTC.');

            maxFeasibleCurvature = tan(vehCfg.maxSteerAngle) / vehCfg.wheelbase;
            testCase.assertGreaterThan(maxFeasibleCurvature, 0, 'Test fixture sanity check failed.');

            [~, selectedIndex] = adaptivePlanner(ego, {candidateA, candidateB}, pred, vehCfg, planCfg, "unitTest", 1, false);

            testCase.verifyEqual(selectedIndex, 2, ...
                ['With no candidate kinematically feasible, K2 must retain the original least-bad fallback ' ...
                 '(max minTTC among all candidates, here candidate B) rather than leaving the fallback ' ...
                 'undefined or picking the lower-TTC candidate.']);
        end
    end
end
