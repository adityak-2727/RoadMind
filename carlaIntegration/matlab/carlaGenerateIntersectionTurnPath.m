function [globalPath, turnInfo] = carlaGenerateIntersectionTurnPath(approachStartXY, approachHeadingRad, exitHeadingRad, turnRadiusM, approachLengthM, exitLengthM)
% carlaGenerateIntersectionTurnPath - Phase 13: builds a continuous
% APPROACH -> curved TURN -> EXIT path through the Indian hero scene's
% real 4-way junction, as an Nx2 waypoint array compatible with the
% EXISTING, frozen path-consuming chain (localPlanner.m -> adaptivePlanner.m
% (K2) -> collisionCheck.m -> pathSmoothing.m -> purePursuitController.m),
% exactly the same "globalPath" type globalPlanner.m already produces for
% carlaClosedLoopInit.m's straight-line demo corridor.
%
% This is a PATH GENERATION / PATH CONFIGURATION layer addition, per the
% explicit Phase 13 instruction ("If the existing global path is
% straight-only, modify the PATH CONFIGURATION / PATH GENERATION layer...
% Do not replace adaptivePlanner.m"). No frozen file is touched:
% localPlanner.m already parameterizes its candidate lattice by ARC
% LENGTH along whatever globalPath it is given (confirmed by inspection
% during the Phase 11.5 audit - it uses interp1 against cumulative
% segment length and local tangent/normal vectors, with no straight-line
% assumption anywhere), so a curved globalPath is consumed exactly the
% same way a straight one is - this function only needed to exist
% because nothing previously CONSTRUCTED a curved one.
%
% GEOMETRY: a circular arc of radius turnRadiusM connects the approach
% heading to the exit heading (whichever direction - left, right, or any
% other turn angle - naturally results from the two headings given), with
% a straight approach segment before it and a straight exit segment
% after it. The arc's centre and sweep angle are derived directly from
% the two headings (not hand-tuned): for a heading change of
% (exitHeadingRad - approachHeadingRad), a circular arc of the given
% radius is the direction-consistent solution - a real vehicle's minimum-
% radius circular arc IS the correct model for what "physically feasible
% curvature" means (matches vehicleConfig.m's own
% wheelbase/tan(maxSteerAngle) minimum-radius definition, checked by
% carlaCheckTurnFeasibility.m).
%
% Inputs:
%   approachStartXY     - [x, y] where the approach segment begins
%                          (typically the ego's current/staging position)
%   approachHeadingRad  - heading (project-frame radians) of the approach
%   exitHeadingRad      - heading (project-frame radians) of the exit
%   turnRadiusM         - optional, default 12.0 - arc radius. Must be
%                          checked against the vehicle's minimum feasible
%                          radius by carlaCheckTurnFeasibility.m; this
%                          function does not clamp it itself, so an
%                          infeasible radius is caught explicitly rather
%                          than silently corrected.
%   approachLengthM     - optional, default 25.0 - straight distance
%                          before the arc begins
%   exitLengthM         - optional, default 25.0 - straight distance
%                          after the arc ends
% Outputs:
%   globalPath - Nx2 array of [x, y] waypoints, continuous, ordered from
%                approach start to exit end - drop-in replacement for
%                globalPlanner.m's output wherever a curved route is
%                needed instead of a straight one
%   turnInfo   - struct: .arcCenter, .arcRadius, .headingChangeRad,
%                .arcStartXY, .arcEndXY, .approachHeadingRad,
%                .exitHeadingRad, .turnDirection ("left"|"right"|"straight")

if nargin < 4 || isempty(turnRadiusM)
    turnRadiusM = 12.0;
end
if nargin < 5 || isempty(approachLengthM)
    approachLengthM = 25.0;
end
if nargin < 6 || isempty(exitLengthM)
    exitLengthM = 25.0;
end

WAYPOINT_SPACING_M = 1.0;

% --- Approach segment: straight line ending where the arc begins ---
approachDir = [cos(approachHeadingRad), sin(approachHeadingRad)];
arcStartXY = approachStartXY + approachLengthM * approachDir;
nApproach = max(2, round(approachLengthM / WAYPOINT_SPACING_M));
approachSeg = approachStartXY + (0:nApproach)' / nApproach * approachLengthM .* approachDir;

% --- Heading change, wrapped to (-pi, pi] so the sign gives the turn
% direction directly (positive = CCW = "left" in this project's
% right-handed, CCW-positive convention - the same convention
% purePursuitController.m/vehicleController.m already use). ---
headingChange = atan2(sin(exitHeadingRad - approachHeadingRad), cos(exitHeadingRad - approachHeadingRad));
if abs(headingChange) < deg2rad(5)
    turnDirection = "straight";
elseif headingChange > 0
    turnDirection = "left";
else
    turnDirection = "right";
end

% --- Circular arc: center is turnRadiusM to the LEFT of the approach
% heading if turning left (headingChange > 0), to the RIGHT if turning
% right - i.e. center = arcStartXY + turnRadiusM * sign(headingChange) *
% leftNormal(approachHeading). This is the standard circular-arc
% construction: the vehicle's path curves toward whichever side the
% center is on. ---
leftNormal = [-sin(approachHeadingRad), cos(approachHeadingRad)];
turnSign = sign(headingChange);
if turnSign == 0
    turnSign = 1; % "straight" case: arc degenerates to a point, direction is irrelevant
end
arcCenter = arcStartXY + turnRadiusM * turnSign * leftNormal;

% Parametric angle of the start point relative to the center, then sweep
% by headingChange (a CCW sweep for a left turn, CW for a right turn -
% the tangent direction of a point moving along a circle always leads the
% radius-vector angle by +90 degrees for CCW motion, -90 for CW, which is
% exactly consistent with turnSign here).
phi0 = atan2(arcStartXY(2) - arcCenter(2), arcStartXY(1) - arcCenter(1));
nArc = max(3, round(turnRadiusM * abs(headingChange) / WAYPOINT_SPACING_M));
phis = phi0 + turnSign * (0:nArc)' / nArc * abs(headingChange);
arcSeg = arcCenter + turnRadiusM * [cos(phis), sin(phis)];
arcEndXY = arcSeg(end, :);

% --- Exit segment: straight line continuing along the exit heading ---
exitDir = [cos(exitHeadingRad), sin(exitHeadingRad)];
nExit = max(2, round(exitLengthM / WAYPOINT_SPACING_M));
exitSeg = arcEndXY + (0:nExit)' / nExit * exitLengthM .* exitDir;

globalPath = [approachSeg; arcSeg(2:end, :); exitSeg(2:end, :)];

turnInfo = struct( ...
    'arcCenter', arcCenter, 'arcRadius', turnRadiusM, 'headingChangeRad', headingChange, ...
    'arcStartXY', arcStartXY, 'arcEndXY', arcEndXY, ...
    'approachHeadingRad', approachHeadingRad, 'exitHeadingRad', exitHeadingRad, ...
    'turnDirection', turnDirection);

end
