function plotPredictedTrajectories(predictedTrajectories, axHandle)
% plotPredictedTrajectories - renders each agent's predicted future path.
% Does not clear the axes or manage hold state - see plotPlannedPath for
% the shared frame-composition convention.
%
% Inputs:
%   predictedTrajectories - cell array of Nx3 [x, y, uncertaintyRadius]
%                           predicted waypoint arrays (see prediction/)
%   axHandle               - target axes handle

if isempty(axHandle) || ~ishghandle(axHandle)
    return;
end

for i = 1:numel(predictedTrajectories)
    pred = predictedTrajectories{i};
    if isempty(pred)
        continue;
    end
    plot(axHandle, pred(:, 1), pred(:, 2), ':', 'Color', [0.9, 0.4, 0.1], 'LineWidth', 1);
end

end
