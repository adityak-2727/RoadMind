function scenarioData = marketArea()
% marketArea - dense mixed-traffic market street: narrow corridor with
% parked vehicles, a pushcart, a crossing pedestrian, and a weaving
% motorcycle. Phase 2: real geometry + agents, no perception/decision
% logic yet.
%
% Output:
%   scenarioData - struct with fields: map, egoStart, egoGoal, agents

x = (0:2:80)';
centerline = [x, zeros(size(x))];

egoStart = createEgoState();
egoStart.x = centerline(1, 1);
egoStart.y = centerline(1, 2);
egoStart.yaw = 0;

egoGoal = centerline(end, :);

parkedAuto = createAgent();
parkedAuto.id = 1;
parkedAuto.class = "auto";
parkedAuto.position = [18, 1.4];
parkedAuto.velocity = [0, 0];
parkedAuto.heading = 0;
parkedAuto.confidence = 1.0;
parkedAuto.source = "fused";

pushcart = createAgent();
pushcart.id = 2;
pushcart.class = "pushcart";
pushcart.position = [34, -1.0];
pushcart.velocity = [0, 0];
pushcart.heading = 0;
pushcart.confidence = 1.0;
pushcart.source = "fused";

crossingPedestrian = createAgent();
crossingPedestrian.id = 3;
crossingPedestrian.class = "pedestrian";
crossingPedestrian.position = [50, -1.5];
crossingPedestrian.velocity = [0, 0.35]; % slowly crossing the narrow corridor
crossingPedestrian.heading = pi / 2;
crossingPedestrian.confidence = 1.0;
crossingPedestrian.source = "fused";

weavingMotorcycle = createAgent();
weavingMotorcycle.id = 4;
weavingMotorcycle.class = "motorcycle";
weavingMotorcycle.position = [64, 0.8];
weavingMotorcycle.velocity = [2.0, -0.8]; % weaving diagonally through traffic
weavingMotorcycle.heading = atan2(-0.8, 2.0);
weavingMotorcycle.confidence = 1.0;
weavingMotorcycle.source = "fused";

scenarioData = struct( ...
    'name',     "marketArea", ...
    'map',      struct('centerline', centerline, 'width', 4.0), ...
    'egoStart', egoStart, ...
    'egoGoal',  egoGoal, ...
    'agents',   [parkedAuto, pushcart, crossingPedestrian, weavingMotorcycle] ...
);

end
