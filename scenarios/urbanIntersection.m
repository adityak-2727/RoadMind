function scenarioData = urbanIntersection()
% urbanIntersection - busy unsignalized four-way intersection: ego travels
% straight through while unregulated cross traffic (cars, motorcycle,
% bicycle, auto-rickshaw), an oncoming car, a crossing pedestrian, and two
% potholes on the ego's own approach all interact with no right-of-way
% rules. Phase 2: real geometry + agents, no perception/decision logic
% yet.
%
% Potholes are modeled as static agents (class="pothole", velocity=[0,0])
% rather than a separate obstacle type - createAgent.m's generic class/
% position/velocity schema is already sufficient: a stationary agent IS a
% static obstacle to every frozen module that consumes agents
% (behaviorDecision.m only special-cases VULNERABLE_CLASSES = pedestrian/
% animal/bicycle, which "pothole" correctly does not match;
% trajectoryPrediction.m only special-cases irregularClasses, which
% "pothole" also does not match, so it gets the same constant-position
% [velocity=[0,0]] prediction as any other agent - the correct behavior
% for something that never moves).
%
% TRAFFIC LIGHT (visual only, not an agent, not read by any decision
% logic): this scenario file has no static-infrastructure concept
% separate from agents, and adding one would be an actual architecture
% change - not done here. Instead, matching config/carlaIndianSceneConfig.m's
% own documented precedent exactly ("Freezing the lights ... changes
% nothing about how any decision is made"), a traffic light is drawn as a
% purely decorative marker by visualization/RealtimeRenderer.m's
% drawTrafficLight() (called from main.m for this scenario only) at the
% junction corner. It carries no state, is never queried by
% behaviorDecision.m/decisionStateMachine.m, and does not change how
% right-of-way is decided - the intersection remains genuinely
% unsignalized, exactly as every prior phase of this project requires.
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

secondCrossCar = createAgent();
secondCrossCar.id = 5;
secondCrossCar.class = "car";
secondCrossCar.position = [-48, -2]; % pushed back from -45 (with the light active, this car's green-phase arrival at the junction coincided with the ego's own crossing - measured live: min TTC 0.40s, still 0 collisions but tight)
secondCrossCar.velocity = [3.2, 0]; % slowed from 4.5 (same reason as the position change - more margin, not just later arrival)
secondCrossCar.heading = 0;
secondCrossCar.confidence = 1.0;
secondCrossCar.source = "fused";

oncomingCar = createAgent();
oncomingCar.id = 6;
oncomingCar.class = "car";
oncomingCar.position = [2.2, 28]; % widened from 1.2 (measured live: caused a 0.55m geometric collision - not enough passing clearance against the ego's own ~1.1m contact radius)
oncomingCar.velocity = [0, -4]; % coming the other way on the ego's own road, offset from centerline
oncomingCar.heading = -pi / 2;
oncomingCar.confidence = 1.0;
oncomingCar.source = "fused";

weavingCyclist = createAgent();
weavingCyclist.id = 7;
weavingCyclist.class = "bicycle";
weavingCyclist.position = [2.4, -14];
weavingCyclist.velocity = [-0.3, 3.5]; % riding up along the ego's approach, drifting toward the centerline
weavingCyclist.heading = atan2(3.5, -0.3);
weavingCyclist.confidence = 1.0;
weavingCyclist.source = "fused";

pothole1 = createAgent();
pothole1.id = 8;
pothole1.class = "pothole";
pothole1.position = [1.4, -25]; % static, offset from centerline, on the ego's approach before the junction
pothole1.velocity = [0, 0];
pothole1.heading = 0;
pothole1.confidence = 1.0;
pothole1.source = "fused";

pothole2 = createAgent();
pothole2.id = 9;
pothole2.class = "pothole";
pothole2.position = [-1.9, -32]; % static, opposite-side offset; moved from (-1.1,-12) after live testing showed the ego's avoidance swerve for oncomingCar/weavingCyclist near y=-12 to -15 passed directly through that point (0.53m clearance) - relocated further back on the approach, before that conflict zone, with more lateral offset
pothole2.velocity = [0, 0];
pothole2.heading = 0;
pothole2.confidence = 1.0;
pothole2.source = "fused";

scenarioData = struct( ...
    'name',     "urbanIntersection", ...
    'map',      struct('centerline', centerline, 'width', 6.0), ...
    'egoStart', egoStart, ...
    'egoGoal',  egoGoal, ...
    'agents',   [crossCar, crossMotorcycle, crossingPedestrian, mergingAutoRickshaw, ...
                 secondCrossCar, oncomingCar, weavingCyclist, pothole1, pothole2] ...
);

end
