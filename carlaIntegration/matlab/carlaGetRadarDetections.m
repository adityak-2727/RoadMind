function detections = carlaGetRadarDetections()
% carlaGetRadarDetections - Phase 10: retrieves the most recently
% received radar sweep. Requires carlaAttachRadar() first.
%
% Output:
%   detections - [] if no sweep has arrived yet. Otherwise a struct:
%                  .raw            Nx4 double, [depth_m, azimuth_rad,
%                                  altitude_rad, velocity_mps] per
%                                  detection, straight from CARLA's
%                                  RadarDetection (azimuth/altitude are
%                                  RADIANS - distinct from CARLA
%                                  Transform.rotation, which is degrees;
%                                  velocity: positive = moving away from
%                                  the sensor, negative = approaching).
%                  .numDetections  N
%                  .frame          CARLA simulation frame number
%                  .timestamp      CARLA simulation timestamp (seconds)

session = getCarlaSession();
detections = session.getRadarDetections();

end
