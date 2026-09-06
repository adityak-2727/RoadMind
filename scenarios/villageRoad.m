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

parkedVehicle = createAgent();
parkedVehicle.id = 3;
parkedVehicle.class = "car";
parkedVehicle.position = [15, 5 * sin(15 / 30) + 1.8];
parkedVehicle.velocity = [0, 0];
parkedVehicle.heading = 0;
parkedVehicle.confidence = 1.0;
parkedVehicle.source = "fused";

pedestrian = createAgent();
pedestrian.id = 4;
pedestrian.class = "pedestrian";
pedestrian.position = [60, 5 * sin(60 / 30) - 1.0];
pedestrian.velocity = [0, 0.3]; % ambling toward the road edge
pedestrian.heading = pi / 2;
pedestrian.confidence = 1.0;
pedestrian.source = "fused";

motorcycle = createAgent();
motorcycle.id = 5;
motorcycle.class = "motorcycle";
motorcycle.position = [25, 5 * sin(25 / 30) + 0.3];
motorcycle.velocity = [4.0, 0.5]; % overtaking along the road, slight weave
motorcycle.heading = atan2(0.5, 4.0);
motorcycle.confidence = 1.0;
motorcycle.source = "fused";

autoRickshaw = createAgent();
autoRickshaw.id = 6;
autoRickshaw.class = "auto";
autoRickshaw.position = [100, 5 * sin(100 / 30) + 1.5];
autoRickshaw.velocity = [1.5, -0.2];
autoRickshaw.heading = atan2(-0.2, 1.5);
autoRickshaw.confidence = 1.0;
autoRickshaw.source = "fused";

pothole = createAgent();
pothole.id = 7;
pothole.class = "unknown"; % static road-surface hazard, not a road user
pothole.position = [70, 5 * sin(70 / 30) + 1.0];
pothole.velocity = [0, 0];
pothole.heading = 0;
pothole.confidence = 1.0;
pothole.source = "fused";

scenarioData = struct( ...
    'name',     "villageRoad", ...
    'map',      struct('centerline', centerline, 'width', 5.0), ...
    'egoStart', egoStart, ...
    'egoGoal',  egoGoal, ...
    'agents',   [pushcart, animal, parkedVehicle, pedestrian, motorcycle, autoRickshaw, pothole] ...
);

end
