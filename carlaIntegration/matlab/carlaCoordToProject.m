function [xProj, yProj] = carlaCoordToProject(xCarla, yCarla)
% carlaCoordToProject - Phase 10 sensor-data coordinate helper. Applies
% the SAME left-handed -> right-handed axis convention already verified
% end-to-end against a live CARLA server in carlaToProjectState.m
% (frozen, unmodified - see its header for the full derivation): CARLA's
% left-handed Y axis flips sign to become this project's right-handed Y
% axis; X is unchanged.
%
% This is deliberately a plain coordinate-axis mirror, not a full pose
% transform - it is correct for both:
%   - CARLA WORLD-frame coordinates (e.g. carlaGetNearbyActorObjects.m's
%     actor x,y), exactly like carlaToProjectState.m's ego position, and
%   - a CARLA sensor's own LOCAL frame (e.g. LiDAR points relative to
%     the sensor), since the axis convention (x=forward, y=right in a
%     left-handed frame) is identical in both cases - only the origin
%     differs, and this function does not assume or require a
%     particular origin.
%
% Does not duplicate carlaToProjectState.m's logic - reuses its exact
% formula so all CARLA-derived coordinates (ego state, sensor points,
% actor positions) share one consistent, verified convention.
%
% Inputs:
%   xCarla, yCarla - CARLA-frame coordinates (meters), scalar or
%                    same-size arrays (works elementwise, e.g. on a full
%                    LiDAR sweep's x/y columns at once).
% Outputs:
%   xProj, yProj - project-frame coordinates (meters), right-handed.

xProj = xCarla;
yProj = -yCarla;

end
