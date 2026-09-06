function [isColliding, minTTC] = collisionCheck(egoTrajectory, predictedTrajectories, vehicleConfig)
% collisionCheck - stub: will check a candidate ego trajectory against all
% predicted agent trajectories for spatial/temporal overlap and compute the
% minimum time-to-collision. Phase 0: no logic yet.
%
% Inputs:
%   egoTrajectory          - Nx2 array of candidate ego [x, y] waypoints
%   predictedTrajectories  - cell array of predicted agent trajectories
%   vehicleConfig          - struct from config/vehicleConfig.m (for footprint size)
% Outputs:
%   isColliding - logical, true if any predicted overlap found
%   minTTC      - [s] minimum time-to-collision across all agents (Inf if none)

isColliding = false;
minTTC = Inf;

end
