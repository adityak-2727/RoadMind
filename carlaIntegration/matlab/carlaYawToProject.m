function yawProj = carlaYawToProject(yawDegCarla)
% carlaYawToProject - Phase 10 sensor-data coordinate helper. Applies the
% same yaw formula already verified against a live CARLA server in
% carlaToProjectState.m (frozen, unmodified): degrees->radians AND
% sign-flipped (a left-handed frame's angle must flip sign under the
% single-axis mirror in carlaCoordToProject.m to stay CCW-positive in
% the project's right-handed frame).
%
% Input:
%   yawDegCarla - CARLA-frame yaw (degrees), scalar or array.
% Output:
%   yawProj - project-frame yaw (radians, CCW-positive from +x).

yawProj = deg2rad(-yawDegCarla);

end
