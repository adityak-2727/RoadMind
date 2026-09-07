function agents = carlaRadarToAgents(radarDetections)
% carlaRadarToAgents - Phase 10: converts a real CARLA radar sweep
% (carlaGetRadarDetections.m's output) into config/createAgent.m-schema
% agents. Sensor-INTERFACE-level conversion only (Phase 10 scope) - not
% a tracker, not a classifier, not sensor fusion (Phase 11).
%
% CARLA's RadarDetection gives [depth, azimuth, altitude, velocity] per
% detection, all in the radar sensor's own local frame (azimuth/altitude
% in RADIANS, 0 = straight ahead, positive azimuth = to the right -
% verified against CARLA's own python_api docs; velocity is the RADIAL
% component along the sensor-to-point line: positive = moving away from
% the sensor, negative = approaching).
%
% Conversion:
%   1. Spherical -> sensor-local Cartesian:
%        x = depth * cos(altitude) * cos(azimuth)
%        y = depth * cos(altitude) * sin(azimuth)
%      (z dropped - this project's agents are 2D, matching every other
%      perception module here).
%   2. Axis convert: carlaCoordToProject's x,-y mirror (sensor-local
%      CARLA frame -> project frame), same as the LiDAR converter.
%   3. Velocity vector: radar only measures the RADIAL speed, not a full
%      2D velocity - the minimum-necessary, still-correct way to expose
%      it as a 2D vector (config/createAgent.m's schema) is to project
%      the scalar radial velocity onto the known bearing direction
%      (sensor -> detection), which is exactly the direction the radial
%      component was measured along:
%        velocity = radialVelocity * unitDirection(sensor -> point)
%      This carries no lateral-velocity information (radar alone cannot
%      observe it) - a documented limitation, not an invented value.
%
% class is always "unknown" (radar alone gives no semantic class - same
% documented limitation as perception/radarDetection.m's synthetic
% counterpart and the LiDAR converter).
%
% Input:
%   radarDetections - struct from carlaGetRadarDetections.m (.raw Nx4:
%                     [depth_m, azimuth_rad, altitude_rad,
%                     velocity_mps], .timestamp), or [] (no sweep yet ->
%                     returns an empty agent array). Each emitted agent
%                     is stamped with radarDetections.timestamp - the
%                     sweep's own CARLA capture time.
% Output:
%   agents - struct array of createAgent()-schema agents, agent.source =
%            "carla_radar" (distinguishable from the synthetic "radar"
%            source used elsewhere in the project).

agents = repmat(createAgent(), 0, 0);
if isempty(radarDetections) || radarDetections.numDetections == 0
    return;
end

raw = radarDetections.raw;
depth = raw(:, 1);
azimuth = raw(:, 2);
altitude = raw(:, 3);
radialVelocity = raw(:, 4);

xLocal = depth .* cos(altitude) .* cos(azimuth);
yLocal = depth .* cos(altitude) .* sin(azimuth);

[xProj, yProj] = carlaCoordToProject(xLocal, yLocal);

horizontalRange = hypot(xProj, yProj);
horizontalRange(horizontalRange < eps) = eps; % avoid divide-by-zero for a detection exactly at the sensor

for i = 1:size(raw, 1)
    a = createAgent();
    a.id = i;
    a.class = "unknown";
    a.position = [xProj(i), yProj(i)];
    unitDir = [xProj(i), yProj(i)] / horizontalRange(i);
    a.velocity = radialVelocity(i) * unitDir;
    a.confidence = 0.5; % radar alone: fixed moderate confidence, no basis for a data-driven estimate here
    a.source = "carla_radar";
    a.timestamp = radarDetections.timestamp;
    agents(end + 1) = a; %#ok<AGROW>
end

end
