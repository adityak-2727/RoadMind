function plotDetectedObjects(agents, egoState, axHandle) %#ok<INUSD>
% plotDetectedObjects - renders agents around the ego vehicle, colored/
% labeled by class. Does not clear the axes or manage hold state - see
% plotPlannedPath for the shared frame-composition convention.
%
% Inputs:
%   agents   - struct array of agents to plot
%   egoState - current ego state struct (unused directly; kept for a
%              future relative-frame rendering mode)
%   axHandle - target axes handle

if isempty(axHandle) || ~ishghandle(axHandle) || isempty(agents)
    return;
end

for i = 1:numel(agents)
    agent = agents(i);
    plot(axHandle, agent.position(1), agent.position(2), 's', ...
         'MarkerFaceColor', [0.9, 0.5, 0.1], 'MarkerEdgeColor', 'k', 'MarkerSize', 7);
    text(axHandle, agent.position(1) + 0.5, agent.position(2) + 0.5, char(agent.class), ...
         'FontSize', 8, 'Color', [0.4, 0.2, 0]);
end

end
