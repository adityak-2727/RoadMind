function smoothnessMetric = calculatePathSmoothness(path)
% calculatePathSmoothness - quantifies path smoothness as the sum of
% absolute curvature change along the path (heading angle change between
% consecutive segments, angle-wrapped to avoid spurious jumps at +/-pi).
% Lower is smoother; 0 for a perfectly straight path.
%
% Inputs:
%   path - Nx2 array of [x, y] waypoints (e.g. the ego's actual driven
%          trajectory, or any candidate/planned path)
% Output:
%   smoothnessMetric - sum(abs(delta curvature)), unitless (radians)

if isempty(path) || size(path, 1) < 3
    smoothnessMetric = 0;
    return;
end

heading = atan2(diff(path(:, 2)), diff(path(:, 1)));
headingChange = diff(heading);
headingChange = atan2(sin(headingChange), cos(headingChange)); % wrap to [-pi, pi]

smoothnessMetric = sum(abs(headingChange));

end
