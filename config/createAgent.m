function agent = createAgent()
% createAgent - schema/constructor for the common perception "agent" struct.
% Every perception module (camera/lidar/radar/fusion/tracking) must eventually
% produce agents in this shape, regardless of sensor source.
%
% Schema:
%   agent.id          integer, unique track id (persistent across frames once tracked)
%   agent.class       string, one of: 'car','bus','truck','auto','motorcycle',
%                      'bicycle','pedestrian','animal','pushcart','unknown'
%   agent.position    [x, y]      [m] in ego/world frame
%   agent.velocity    [vx, vy]    [m/s]
%   agent.heading     scalar      [rad]
%   agent.confidence  scalar in [0, 1], detection/track confidence
%   agent.source      string; 'camera', 'lidar', or 'radar' for a raw
%                      single-sensor detection, a '+'-joined combination
%                      (e.g. 'camera+lidar', 'camera+lidar+radar') once
%                      perception/sensorFusion.m has merged contributing
%                      sensors for one physical object - see its
%                      deduplicateFusedAgents note for why this isn't
%                      always the literal string 'fused' anymore
%   agent.timestamp   scalar      [s], simulation time of this estimate
%   agent.covariance  4x4 double, Kalman filter state covariance over
%                      [x, y, vx, vy] - only meaningful once
%                      perception/objectTracking.m has taken ownership of a
%                      track; zeros elsewhere (scenarios, raw sensor
%                      detections) since nothing else uses this field.

agent = struct( ...
    'id',          0, ...
    'class',       "unknown", ...
    'position',    [0, 0], ...
    'velocity',    [0, 0], ...
    'heading',     0, ...
    'confidence',  0, ...
    'source',      "unknown", ...
    'timestamp',   0, ...
    'covariance',  zeros(4) ...
);

end
