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

There is no real image/point-cloud/radar-return simulator in this project.
`image`/`pointCloud`/`radarData` are each the ground-truth agents already
filtered to that sensor's range/FOV by `main.m`'s `filterAgentsForSensor`
(main.m plays "the sensor capturing a frame"; these functions play "the
detector interpreting it"). Each function then genuinely degrades that
input to mimic its sensor's real characteristics (camera: accurate class,
noisy position, no velocity; lidar: precise position, no class, no
velocity; radar: accurate velocity, poor position, no class) using
`config/sensorConfig.m` (another file not in the original Phase 0 list, same
justification as `createAgent.m`/`createEgoState.m`). This is simulated
detection, explicitly labeled as such throughout - never presented as real
AI detection output, per the project brief's requirement to keep ground
truth and perception output clearly distinct.

### 2. Tracking (`perception/objectTracking.m`)

```matlab
trackedAgents = objectTracking(fusedAgents, trackedAgentsPrev, dt)
% Assigns persistent IDs and filters/predicts agent state across frames.
```

### 3. Prediction (`prediction/`)

```matlab
predictedTrajectory = constantVelocityPrediction(agent, horizon, dt)
% Projects an agent forward assuming constant velocity/heading (cars/buses/trucks).
% Returns Nx3 [x, y, uncertaintyRadius] - see Phase 2 interface note below.

predictedTrajectory = irregularMotionModel(agent, horizon, dt)
% Models erratic motion with wider uncertainty (pedestrian/animal/pushcart/bicycle).
% Returns Nx3 [x, y, uncertaintyRadius], radius growing faster than the constant-velocity model.

predictedTrajectories = trajectoryPrediction(trackedAgents, horizon, dt)
% Dispatches each tracked agent to the right motion model by agent.class and collects results.
```

### 4. Decision (`decision/`)

```matlab
behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, plannerConfig, decisionState)
% Maps current context into a high-level behavior intent: "cruise", "yield", or "emergency_stop",
% by classifying the worst-case calculateTTC() across trackedAgents against plannerConfig.ttcThresholds.
% plannerConfig added in Phase 4 - see interface note below.

nextState = decisionStateMachine(currentState, behaviorCommand, sensorEvents)
% Holds the FSM governing high-level behavior transitions: escalates immediately, de-escalates
% one severity level per tick so risk clearing doesn't snap the vehicle straight back to full speed.
```

### 5. Planning (`planning/`)

```matlab
globalPath = globalPlanner(startPose, goalPose, mapData)
% Computes a coarse start-to-goal route over the scenario map.

candidateTrajectories = localPlanner(egoState, globalPath, predictedTrajectories, plannerConfig)
% Samples a set of candidate short-horizon trajectories around the global path.

selectedTrajectory = adaptivePlanner(egoState, candidateTrajectories, predictedTrajectories, vehicleConfig, plannerConfig, scenarioContext)
% Scores candidates via the weighted cost function and selects the best one.
% This is where behavior adapts to unstructured-road context (mixed traffic, unmarked lanes).
% vehicleConfig added in Phase 2 - see interface note below.

[isColliding, minTTC] = collisionCheck(egoTrajectory, predictedTrajectories, vehicleConfig)
% Checks a candidate ego trajectory against predicted agent trajectories; returns min TTC.

smoothPath = pathSmoothing(rawPath, smoothingParams)
% Smooths a raw waypoint path into a curvature-continuous path for the controller.
```

### 6. Control (`control/`)

```matlab
controlCommand = vehicleController(egoState, selectedTrajectory, vehicleConfig, targetSpeed)
% Combines lateral (pure pursuit) + longitudinal (P control to targetSpeed) into a single
% command for the vehicle model. targetSpeed added in Phase 4 - see interface note below.

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

## Interface change log

Signatures that changed from their original Phase 0 draft once the function
actually had to do its documented job. Each was made while the function had
no callers besides `main.m` (also updated each time), so there was no
breakage to manage - noted here so the reason isn't lost.

**Phase 2** (collision-aware local planning):

- `constantVelocityPrediction`/`irregularMotionModel`/`trajectoryPrediction`
  return **Nx3** `[x, y, uncertaintyRadius]` instead of Nx2. `collisionCheck`
  needs a per-timestep safety radius per agent and there was nowhere else to
  carry it without breaking the "plain matrices, not structs" rule.
- `adaptivePlanner` gained a `vehicleConfig` parameter. It calls
  `collisionCheck` internally to reject unsafe candidates, and
  `collisionCheck` requires `vehicleConfig` - the original draft signature
  had no way to thread it through.

## Known behavior: decision dwell time lives in main.m, not decisionStateMachine

`decisionStateMachine`'s signature was never changed, but its transitions
are gated by a minimum dwell time (`MIN_DWELL_TIME`, currently 1.5s) that
`main.m` enforces around the call, not inside the function itself. Reason:
`calculateTTC` depends on `egoState.velocity`, and the decision layer sets
that same velocity via `targetSpeed` - braking hard for `emergency_stop`
can itself make the very next instant's TTC estimate look briefly safe,
which without a dwell time flips the state straight back to `yield`,
re-accelerates, and drops TTC again, oscillating every 0.5-0.9s (observed
directly in `highwayMerge`). Escalating to more caution is applied
immediately regardless of dwell time (safety-first); only de-escalating is
rate-limited. This lives in `main.m` because it is a scheduling/orchestration
concern about how often to *commit* a transition, not part of what
transitions are valid - `decisionStateMachine` stays a pure function.

**Phase 4** (decision layer / speed control):

- `behaviorDecision` gained a `plannerConfig` parameter, needed for
  `plannerConfig.ttcThresholds` to classify risk into a behavior command.
- `vehicleController` gained a `targetSpeed` parameter. It previously
  hardcoded a fixed cruise speed internally, which made it structurally
  impossible for the vehicle to ever yield or stop - lateral planning alone
  can't resolve a perpendicular crossing (see `urbanIntersection`'s near-miss
  in the Phase 3 validation run). The caller (`main.m`) now derives
  `targetSpeed` from `decisionState` via `behaviorDecision`/`decisionStateMachine`.

## Toolbox-swap note

Every function above takes/returns plain structs and matrices, not toolbox
objects. When Automated Driving Toolbox is introduced (e.g. `drivingScenario`
for `mapData`, `multiObjectTracker` inside `objectTracking`, `vehicleDynamics`
blocks under `bicycleModel`), only the *inside* of each stub changes — callers
never need to change because the struct/array contract at each boundary stays
fixed.
