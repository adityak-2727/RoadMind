function feasibility = carlaCheckTurnFeasibility(globalPath, turnInfo, vehCfg)
% carlaCheckTurnFeasibility - Phase 13: numerically validates a
% carlaGenerateIntersectionTurnPath.m path against the EXISTING,
% unmodified vehicle constraints (config/vehicleConfig.m) - the same
% wheelbase/maxSteerAngle-derived minimum-turn-radius formula
% planning/adaptivePlanner.m's own K2 fallback already uses
% internally (`tan(vehicleConfig.maxSteerAngle) / vehicleConfig.wheelbase`
% as the max feasible curvature), applied here to the GENERATED PATH
% itself before it is ever handed to the planner - so an infeasible turn
% is caught explicitly rather than discovered only via K2's fallback
% machinery at runtime.
%
% Inputs:
%   globalPath - Nx2 waypoints from carlaGenerateIntersectionTurnPath.m
%   turnInfo   - struct from the same function
%   vehCfg     - struct from config/vehicleConfig.m (unmodified)
% Output:
%   feasibility - struct:
%     .pathLengthM         total arc-length of globalPath
%     .maxCurvature        max |heading change| / segment length, 1/m,
%                          measured directly from the waypoints (not
%                          assumed from turnInfo.arcRadius) - a genuine
%                          numerical measurement of the path actually
%                          produced
%     .minRadiusM           1/maxCurvature (Inf if maxCurvature ~ 0)
%     .headingChangeRad     from turnInfo, the net heading change through
%                          the turn
%     .maxFeasibleCurvature tan(vehCfg.maxSteerAngle)/vehCfg.wheelbase -
%                          IDENTICAL formula to adaptivePlanner.m's own
%                          K2 fallback feasibility check, not a
%                          reimplementation with different numbers
%     .isFeasible           logical: maxCurvature <= maxFeasibleCurvature
%     .requiredSteerDeg     the steering angle the measured maxCurvature
%                          would require (rad2deg(atan(curvature*wheelbase)))
%     .maxFeasibleSteerDeg  rad2deg(vehCfg.maxSteerAngle)
%     .lateralContinuity    max consecutive-waypoint lateral jump (m) -
%                          a coarse continuity sanity check (a real
%                          discontinuity/jump would show up as an outlier)

diffs = diff(globalPath, 1, 1);
segLen = hypot(diffs(:,1), diffs(:,2));
feasibility.pathLengthM = sum(segLen);

heading = atan2(diffs(:,2), diffs(:,1));
headingDiff = atan2(sin(diff(heading)), cos(diff(heading))); % wrapped, per-segment heading change
midSegLen = (segLen(1:end-1) + segLen(2:end)) / 2;
validSeg = midSegLen > 1e-6;
curvatures = zeros(size(headingDiff));
curvatures(validSeg) = abs(headingDiff(validSeg)) ./ midSegLen(validSeg);

feasibility.maxCurvature = max(curvatures);
if feasibility.maxCurvature > 1e-9
    feasibility.minRadiusM = 1 / feasibility.maxCurvature;
else
    feasibility.minRadiusM = Inf;
end

feasibility.headingChangeRad = turnInfo.headingChangeRad;

feasibility.maxFeasibleCurvature = tan(vehCfg.maxSteerAngle) / vehCfg.wheelbase;
feasibility.isFeasible = feasibility.maxCurvature <= feasibility.maxFeasibleCurvature;

feasibility.requiredSteerDeg = rad2deg(atan(feasibility.maxCurvature * vehCfg.wheelbase));
feasibility.maxFeasibleSteerDeg = rad2deg(vehCfg.maxSteerAngle);

feasibility.lateralContinuity = max(segLen); % largest single waypoint-to-waypoint jump

end
