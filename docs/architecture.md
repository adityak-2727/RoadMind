# Architecture — SIH 26037 Adaptive Path Planning & Collision Avoidance

Phase 0 document. Every signature below matches an actual stub file in the
repo (no logic implemented yet). This is the contract Phase 1+ implements
against — inputs/outputs should not change shape even when MATLAB Automated
Driving Toolbox objects (e.g. `drivingScenario`, `multiObjectTracker`,
`vehicleDynamics`) are swapped in underneath later.

## Closed-loop pipeline (text diagram)

```
                      ┌─────────────────────────────────────────────────────────┐
                      │                                                         │
                      ▼                                                         │
   ┌───────────┐   ┌────────────┐   ┌───────────┐   ┌────────────┐   ┌────────┐│
   │  SENSORS  │   │ PERCEPTION │   │ PREDICTION│   │  DECISION  │   │PLANNING││
   │ camera    │──▶│ detection  │──▶│ per-agent │──▶│  behavior  │──▶│ global ││
   │ lidar     │   │ + fusion   │   │ motion    │   │  state     │   │ local  ││
   │ radar     │   │ + tracking │   │ models    │   │  machine   │   │ adaptive│
   └───────────┘   └────────────┘   └───────────┘   └────────────┘   │ collision│
         ▲                                                            │ check  │
         │                                                            │ smooth ││
         │                                                            └───┬────┘│
         │                                                                │     │
         │               ┌────────────┐   ┌───────────────┐              │     │
         │               │  VEHICLE   │   │    CONTROL    │              │     │
         └───────────────│   MODEL    │◀──│  controller   │◀─────────────┘     │
                          │ (bicycle)  │   │ pure pursuit  │                    │
                          └─────┬──────┘   └───────────────┘                    │
                                │                                               │
                                └───────────────────────────────────────────────┘
                                   ego state feeds back to sensors next tick
```

Data flow per simulation step `t`:

1. **Sensors → Perception**: raw sensor data (image/point cloud/radar returns)
   at time `t` is converted into `agent` structs.
2. **Perception fusion/tracking**: per-sensor agent lists are fused into one
   list, then tracked (stable IDs, filtered state) against `t-1`'s tracks.
3. **Prediction**: each tracked agent's future trajectory over the planning
   horizon is predicted (model chosen by `agent.class`).
4. **Decision**: current ego state + tracked agents + predictions → a
   high-level behavior command, arbitrated by a state machine.
5. **Planning**: global reference path (computed once/rarely) + local
   candidate trajectories (resampled each step) are scored by the adaptive
   planner's cost function and checked for collision; result is smoothed.
6. **Control**: the smoothed trajectory is tracked by a controller producing
   throttle/brake/steering.
7. **Vehicle model**: integrates one timestep forward (kinematic bicycle
   model) to produce the next ego state.
8. **Loop back**: the new ego state is what sensors perceive relative to at
   `t+1` — closing the loop.

## Function signatures by stage

### 1. Perception (`perception/`)

```matlab
agents = cameraDetection(image, cameraParams, timestamp)
% Runs camera-based object detection; returns agent structs, agent.source = "camera".

agents = lidarDetection(pointCloud, lidarParams, timestamp)
% Clusters/segments a lidar point cloud into agent detections, agent.source = "lidar".

agents = radarDetection(radarData, radarParams, timestamp)
% Converts raw radar range/doppler returns into agent detections, agent.source = "radar".

fusedAgents = sensorFusion(cameraAgents, lidarAgents, radarAgents)
% Associates and merges detections from all sensors into one fused agent list.
```

### 2. Tracking (`perception/objectTracking.m`)

```matlab
trackedAgents = objectTracking(fusedAgents, trackedAgentsPrev, dt)
% Assigns persistent IDs and filters/predicts agent state across frames.
```

### 3. Prediction (`prediction/`)

```matlab
predictedTrajectory = constantVelocityPrediction(agent, horizon, dt)
% Projects an agent forward assuming constant velocity/heading (cars/buses/trucks).

predictedTrajectory = irregularMotionModel(agent, horizon, dt)
% Models erratic motion with wider uncertainty (pedestrian/animal/pushcart/bicycle).

predictedTrajectories = trajectoryPrediction(trackedAgents, horizon, dt)
% Dispatches each tracked agent to the right motion model by agent.class and collects results.
```

### 4. Decision (`decision/`)

```matlab
behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, decisionState)
% Maps current context into a high-level behavior intent (cruise/yield/stop/crawl/overtake).

nextState = decisionStateMachine(currentState, behaviorCommand, sensorEvents)
% Holds the FSM governing high-level behavior transitions, guarded by sensor/perception events.
```

### 5. Planning (`planning/`)

```matlab
globalPath = globalPlanner(startPose, goalPose, mapData)
% Computes a coarse start-to-goal route over the scenario map.

candidateTrajectories = localPlanner(egoState, globalPath, predictedTrajectories, plannerConfig)
% Samples a set of candidate short-horizon trajectories around the global path.

selectedTrajectory = adaptivePlanner(egoState, candidateTrajectories, predictedTrajectories, plannerConfig, scenarioContext)
% Scores candidates via the weighted cost function and selects the best one.
% This is where behavior adapts to unstructured-road context (mixed traffic, unmarked lanes).

[isColliding, minTTC] = collisionCheck(egoTrajectory, predictedTrajectories, vehicleConfig)
% Checks a candidate ego trajectory against predicted agent trajectories; returns min TTC.

smoothPath = pathSmoothing(rawPath, smoothingParams)
% Smooths a raw waypoint path into a curvature-continuous path for the controller.
```

### 6. Control (`control/`)

```matlab
controlCommand = vehicleController(egoState, selectedTrajectory, vehicleConfig)
% Combines lateral + longitudinal control into a single command for the vehicle model.

steeringAngle = purePursuitController(egoState, path, lookaheadDist, vehicleConfig)
% Computes steering angle to track a path using pure pursuit geometry.
```

### 7. Vehicle model (`control/bicycleModel.m`)

```matlab
nextEgoState = bicycleModel(egoState, controlCommand, vehicleConfig, dt)
% Integrates the kinematic bicycle model one timestep forward; closes the loop to perception.
```

### 8. Evaluation (`evaluation/`)

```matlab
ttc = calculateTTC(egoState, agent)
% Computes time-to-collision between ego and one agent.

smoothnessMetric = calculatePathSmoothness(path)
% Quantifies path smoothness (curvature-based) for ride-comfort scoring.

latencyStats = calculateReplanningLatency(replanTimestamps)
% Computes mean/max/count latency between successive replanning events.

completionRate = calculateCompletionRate(simLog)
% Computes fraction of runs/route completed without collision or timeout.

metricsReport = evaluateScenario(simLog, scenarioConfig)
% Runs all evaluation metrics over one scenario's log and assembles a report.
```

### 9. Visualization (`visualization/`)

```matlab
plotDetectedObjects(agents, egoState, axHandle)
plotPredictedTrajectories(predictedTrajectories, axHandle)
plotPlannedPath(path, egoState, axHandle)
figHandle = createDemoFigure(simLog, scenarioName)
```

### 10. Scenarios (`scenarios/`)

Each returns a `scenarioData` struct: `name`, `map`, `egoStart`, `egoGoal`,
`agents`.

```matlab
scenarioData = villageRoad()
scenarioData = urbanIntersection()
scenarioData = highwayMerge()
scenarioData = marketArea()
scenarioData = cattleCrossing()
```

## Toolbox-swap note

Every function above takes/returns plain structs and matrices, not toolbox
objects. When Automated Driving Toolbox is introduced (e.g. `drivingScenario`
for `mapData`, `multiObjectTracker` inside `objectTracking`, `vehicleDynamics`
blocks under `bicycleModel`), only the *inside* of each stub changes — callers
never need to change because the struct/array contract at each boundary stays
fixed.
