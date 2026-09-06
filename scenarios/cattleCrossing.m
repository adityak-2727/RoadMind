function scenarioData = cattleCrossing()
% cattleCrossing - straight rural stretch with a roadside stall and a cow
% that suddenly crosses the ego's path, meant to drive TTC down sharply
% and exercise emergency braking/replanning.
%
% The cow's velocity is tuned so its road-crossing moment (y: -6 -> 0)
% lands at roughly t=8s, x=50 - matching where the ego actually is at that
% time under main.m's default cruise speed for this scenario (verified by
% simulation, not just geometry: at the previous vy=2.5 the cow crossed
% and cleared the road by t=3s while the ego was still under 10m in, so
% the "sudden crossing" never actually coincided with the ego at all).
%
% Output:
%   scenarioData - struct with fields: map, egoStart, egoGoal, agents

x = (0:2:100)';
centerline = [x, zeros(size(x))];

egoStart = createEgoState();
egoStart.x = centerline(1, 1);
egoStart.y = centerline(1, 2);
egoStart.yaw = 0;

egoGoal = centerline(end, :);

roadsideStall = createAgent();
roadsideStall.id = 1;
roadsideStall.class = "pushcart";
roadsideStall.position = [35, 3.0];
roadsideStall.velocity = [0, 0];
roadsideStall.heading = 0;
roadsideStall.confidence = 1.0;
roadsideStall.source = "fused";

cow = createAgent();
cow.id = 2;
cow.class = "animal";
cow.position = [50, -6.0];
cow.velocity = [0.5, 0.75]; % crosses y=0 near t=8s, x~54 - timed to meet the ego, not miss it
cow.heading = atan2(0.75, 0.5);
cow.confidence = 1.0;
cow.source = "fused";

scenarioData = struct( ...
    'name',     "cattleCrossing", ...
    'map',      struct('centerline', centerline, 'width', 5.0), ...
    'egoStart', egoStart, ...
    'egoGoal',  egoGoal, ...
    'agents',   [roadsideStall, cow] ...
);

end
