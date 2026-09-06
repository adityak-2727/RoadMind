function scenarioData = cattleCrossing()
% cattleCrossing - straight rural stretch with a roadside stall and a cow
% that suddenly crosses the ego's path, meant to drive TTC down sharply
% and exercise emergency braking/replanning. Phase 2: real geometry +
% agents, no perception/decision logic yet.
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
cow.velocity = [0.5, 2.5]; % suddenly crossing perpendicular to the road near x=50
cow.heading = atan2(2.5, 0.5);
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
