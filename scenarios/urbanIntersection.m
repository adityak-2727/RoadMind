function scenarioData = urbanIntersection()
% urbanIntersection - busy unsignalized four-way intersection: ego travels
% straight through while unregulated cross traffic (car, motorcycle) and a
% crossing pedestrian pass through the same junction with no right-of-way
% rules. Phase 2: real geometry + agents, no perception/decision logic yet.
%
% Output:
%   scenarioData - struct with fields: map, egoStart, egoGoal, agents

y = (-40:2:40)';
centerline = [zeros(size(y)), y]; % ego travels straight north through the junction at x=0

egoStart = createEgoState();
egoStart.x = centerline(1, 1);
egoStart.y = centerline(1, 2);
egoStart.yaw = pi / 2; % heading north

egoGoal = centerline(end, :);

crossCar = createAgent();
crossCar.id = 1;
crossCar.class = "car";
crossCar.position = [-30, 0];
crossCar.velocity = [5, 0]; % approaching the junction from the west
crossCar.heading = 0;
crossCar.confidence = 1.0;
crossCar.source = "fused";

crossMotorcycle = createAgent();
crossMotorcycle.id = 2;
crossMotorcycle.class = "motorcycle";
crossMotorcycle.position = [30, 4];
crossMotorcycle.velocity = [-4, 0]; % approaching from the east, offset lane
crossMotorcycle.heading = pi;
crossMotorcycle.confidence = 1.0;
crossMotorcycle.source = "fused";

crossingPedestrian = createAgent();
crossingPedestrian.id = 3;
crossingPedestrian.class = "pedestrian";
crossingPedestrian.position = [6, -6];
crossingPedestrian.velocity = [-0.4, 0.6]; % crossing diagonally near the junction
crossingPedestrian.heading = atan2(0.6, -0.4);
crossingPedestrian.confidence = 1.0;
crossingPedestrian.source = "fused";

mergingAutoRickshaw = createAgent();
mergingAutoRickshaw.id = 4;
mergingAutoRickshaw.class = "auto";
mergingAutoRickshaw.position = [12, -20];
mergingAutoRickshaw.velocity = [-3, 4]; % cutting diagonally into the junction, no defined lane to merge from
mergingAutoRickshaw.heading = atan2(4, -3);
mergingAutoRickshaw.confidence = 1.0;
mergingAutoRickshaw.source = "fused";

scenarioData = struct( ...
    'name',     "urbanIntersection", ...
    'map',      struct('centerline', centerline, 'width', 6.0), ...
    'egoStart', egoStart, ...
    'egoGoal',  egoGoal, ...
    'agents',   [crossCar, crossMotorcycle, crossingPedestrian, mergingAutoRickshaw] ...
);

end
