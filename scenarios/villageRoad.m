function scenarioData = villageRoad()
% villageRoad - unmarked/narrow village road scenario: a gently winding
% single-lane road with a parked pushcart and a slow-moving animal near
% the roadside. Phase 1: real geometry + a couple of agents, no dynamic
% traffic logic yet (agents are simple placeholders for later prediction).
%
% Output:
%   scenarioData - struct with fields: map, egoStart, egoGoal, agents

x = (0:2:120)';
y = 5 * sin(x / 30);
centerline = [x, y];

startYaw = atan2(centerline(2, 2) - centerline(1, 2), centerline(2, 1) - centerline(1, 1));

egoStart = createEgoState();
egoStart.x = centerline(1, 1);
egoStart.y = centerline(1, 2);
egoStart.yaw = startYaw;

egoGoal = centerline(end, :);

pushcart = createAgent();
pushcart.id = 1;
pushcart.class = "pushcart";
pushcart.position = [40, 5 * sin(40 / 30) + 2.0];
pushcart.velocity = [0, 0];
pushcart.heading = 0;
pushcart.confidence = 1.0;
pushcart.source = "fused";

animal = createAgent();
animal.id = 2;
animal.class = "animal";
animal.position = [80, 5 * sin(80 / 30) - 1.5];
animal.velocity = [0.3, 0.2];
animal.heading = atan2(0.2, 0.3);
animal.confidence = 1.0;
animal.source = "fused";

scenarioData = struct( ...
    'name',     "villageRoad", ...
    'map',      struct('centerline', centerline, 'width', 5.0), ...
    'egoStart', egoStart, ...
    'egoGoal',  egoGoal, ...
    'agents',   [pushcart, animal] ...
);

end
