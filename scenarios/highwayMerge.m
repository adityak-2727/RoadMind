function scenarioData = highwayMerge()
% highwayMerge - ego enters from an on-ramp and merges onto a highway
% carrying a slow-moving truck and a faster car in the adjacent lane.
% Phase 2: real geometry + agents, no perception/decision logic yet.
%
% Output:
%   scenarioData - struct with fields: map, egoStart, egoGoal, agents

rampX = (0:2:40)';
rampY = -10 * (1 - rampX / 40).^2; % ramp curves up from y=-10 to y=0 by x=40

highwayX = (42:2:150)';
highwayY = zeros(size(highwayX));

centerline = [[rampX, rampY]; [highwayX, highwayY]];

egoStart = createEgoState();
egoStart.x = centerline(1, 1);
egoStart.y = centerline(1, 2);
egoStart.yaw = atan2(centerline(2, 2) - centerline(1, 2), centerline(2, 1) - centerline(1, 1));

egoGoal = centerline(end, :);

slowTruck = createAgent();
slowTruck.id = 1;
slowTruck.class = "truck";
slowTruck.position = [60, 0];
slowTruck.velocity = [3, 0]; % well below highway cruise speed, in the merge lane
slowTruck.heading = 0;
slowTruck.confidence = 1.0;
slowTruck.source = "fused";

fastCar = createAgent();
fastCar.id = 2;
fastCar.class = "car";
fastCar.position = [20, 3.5];
fastCar.velocity = [11, 0]; % passing lane, normal highway speed
fastCar.heading = 0;
fastCar.confidence = 1.0;
fastCar.source = "fused";

scenarioData = struct( ...
    'name',     "highwayMerge", ...
    'map',      struct('centerline', centerline, 'width', 3.5), ...
    'egoStart', egoStart, ...
    'egoGoal',  egoGoal, ...
    'agents',   [slowTruck, fastCar] ...
);

end
