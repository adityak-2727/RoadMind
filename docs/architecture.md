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

fusedAgents = sensorFusion(cameraAgents, lidarAgents, radarAgents, debugDedup)
% Associates and merges detections from all sensors into one fused agent list, then runs a
% conservative post-fusion deduplication pass (local function deduplicateFusedAgents) that
% catches one physical object producing two fused entries when a sensor's raw detection missed
% the camera-anchored gate by a small margin. debugDedup is optional (default false) - see
% deduplicateFusedAgents' own doc comment for the full threshold derivation and merge rules.
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
% Assigns persistent IDs by nearest-neighbor gating, then runs a constant-velocity
% Kalman filter per track over [x, y, vx, vy] (position-only measurement), per the
% project brief's explicit preference. Tracks with no match this frame are dropped,
% not coasted. Requires agent.covariance (config/createAgent.m) - see schema note below.
```

### 3. Prediction (`prediction/`)

```matlab
predictedTrajectory = constantVelocityPrediction(agent, horizon, dt)
% Projects an agent forward assuming constant velocity/heading (cars/buses/trucks).
% Returns Nx3 [x, y, uncertaintyRadius] - see Phase 2 interface note below.

predictedTrajectory = irregularMotionModel(agent, horizon, dt)
% Models erratic motion with wider uncertainty (pedestrian/animal/pushcart/bicycle).
% Returns Nx3 [x, y, uncertaintyRadius], radius growing faster than the constant-velocity model.

predictedTrajectories = trajectoryPrediction(trackedAgents, egoState, horizon, dt)
% Dispatches each tracked agent to the right motion model using agent.class AND its
% current behavior category (classifyBehavior.m): irregular class, or currently
% "crossing"/"merging" relative to egoState.yaw, gets the higher-uncertainty model
% regardless of class - a car cutting across an intersection is as hard to predict
% as a pedestrian doing the same. egoState added in Phase 4 - see interface note below.

category = classifyBehavior(agent, referenceHeading)
% Classifies one agent's current motion into "stopped"/"normal"/"crossing"/"merging"
% relative to referenceHeading (main.m passes egoState.yaw). "erratic" (the project
% brief's fifth category) needs multi-frame heading history not currently tracked -
% irregular-class agents get conservative treatment regardless of category instead.
```

### 4. Decision (`decision/`)

```matlab
behaviorCommand = behaviorDecision(egoState, trackedAgents, predictedTrajectories, plannerConfig, decisionState)
% Maps current context into a high-level behavior intent by evaluating the project brief's
% priority hierarchy in strict order - first tier to fire wins, so safety dominates efficiency:
%   1. emergency collision avoidance -> "emergency_stop"
%   2. pedestrian/animal safety      -> "brake" (or "wait")
%   3. dynamic obstacle avoidance    -> "avoid"
%   4. merge/interaction             -> "merge" / "follow"
%   5. normal navigation             -> "cruise"
% Tier 2 gives vulnerable road users (pedestrian/animal/bicycle) 1.5x the TTC threshold and a
% wider proximity radius than vehicles. plannerConfig added in Phase 4 - see interface note below.

nextState = decisionStateMachine(currentState, behaviorCommand, sensorEvents)
% FSM over the brief's full state set. Escalates immediately (safety-first); relaxes at most one
% behaviorSeverity rank per transition and never below what was proposed, walking the recovery
% ramp emergency_stop -> brake -> avoid -> replan -> follow/merge -> cruise.

rank = behaviorSeverity(state)
% Single source of truth for how restrictive each state is (1 cruise .. 7 emergency_stop), kept
% monotonic with main.m's decisionSpeedFactors. decisionStateMachine holds the inverse map.
```

### 5. Planning (`planning/`)

```matlab
globalPath = globalPlanner(startPose, goalPose, mapData)
% Computes a coarse start-to-goal route over the scenario map.

candidateTrajectories = localPlanner(egoState, globalPath, predictedTrajectories, plannerConfig)
% Samples a set of candidate short-horizon trajectories around the global path.

[selectedTrajectory, selectedIndex] = adaptivePlanner(egoState, candidateTrajectories, predictedTrajectories, vehicleConfig, plannerConfig, scenarioContext, previousIndex)
% Scores candidates via the weighted cost function (including a small consistency
% cost against previousIndex, to avoid flickering between near-tied candidates -
% see interface note below) and selects the best one. This is where behavior adapts
% to unstructured-road context (mixed traffic, unmarked lanes).
% vehicleConfig added in Phase 2, previousIndex/selectedIndex in Phase 4 - see interface note below.

[isColliding, minTTC] = collisionCheck(egoTrajectory, predictedTrajectories, vehicleConfig)
% Checks a candidate ego trajectory against predicted agent trajectories; returns min TTC.

smoothPath = pathSmoothing(rawPath, smoothingParams)
% Smooths a raw waypoint path into a curvature-continuous path for the controller.
```

### 6. Control (`control/`)

```matlab
controlCommand = vehicleController(egoState, selectedTrajectory, vehicleConfig, targetSpeed, dt)
% Combines lateral (pure pursuit + rate-limited against egoState.steering) + longitudinal
% (P control to targetSpeed) into a single command for the vehicle model. targetSpeed added
% in Phase 4, dt in Phase 7 - see interface note below.

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
% sum(abs(delta curvature)) over the path (angle-wrapped) - main.m calls this on the
% ego's actual driven history (egoHistory) at the end of a run, not just one candidate.

latencyStats = calculateReplanningLatency(replanLatencies)
% Computes mean/median/max/count over a list of already-measured per-event latencies
% (see "Replanning trigger/latency" note below) - renamed from replanTimestamps in the
% original Phase 0 draft, which couldn't have been what this consumes (see interface note).

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

**Phase 5** (Kalman-filter tracking + behavior-aware prediction):

- `trajectoryPrediction` gained an `egoState` parameter. Classifying an
  agent's motion as "crossing"/"merging" (new: `classifyBehavior.m`) needs a
  reference direction to measure against, and `egoState.yaw` is the only
  one available without also threading through road geometry.
- `adaptivePlanner` gained `previousIndex` (input) and `selectedIndex`
  (output). Without them, the cost function had no notion of continuity
  between steps: when several candidates were near-tied in cost near an
  obstacle, the argmin flipped between them every step, producing wild
  steering swings (found live in `highwayMerge`: candidate index bounced
  2→3→2→2→3→2→3, later 5→8→8→8→8→4→4→6→8, alongside +/-40 degree yaw
  swings, driving minimum clearance down to 1.17m from 4.86m before that
  specific validation pass). A small consistency cost against the previous
  index fixed it (2.38m after) - a genuinely large safety cost difference
  still dominates it.

**Phase 6** (six-weight cost function, evaluation metric rename):

- `adaptivePlanner`'s cost function now uses all six `plannerConfig.costWeights`
  terms (`obstacleClearance`, `speedChange`, `uncertainty` were previously
  contributing zero) - see the function's own doc comment for what each
  measures, including the `speedChange` curvature-implied-speed proxy this
  decoupled lateral/speed architecture needed instead of a literal
  per-candidate speed cost.
- `calculateReplanningLatency`'s parameter renamed `replanTimestamps` ->
  `replanLatencies`. The project brief's own diagram
  (`new_path_time - obstacle_detection_time`) needs two timestamps per event
  to produce one latency value, so a single list of raw timestamps could
  never have been what this function consumes - it must already be a list
  of per-event latency values, computed by the caller (`main.m`).

## Replanning trigger/latency instrumentation (main.m)

`main.m` replans every simulation tick unconditionally already (continuous,
not periodic/event-triggered), so "replanning latency" measures something
real but narrower than the project brief's diagram implies: how long from a
`plannerConfig.replanTriggers` condition first becoming active until the
system's *output* actually changes (`adaptivePlanner`'s selected candidate
index differs, or `decisionState` escalates) - not literally
"obstacle-detected timestamp to new-path-generated timestamp," since a new
path is technically generated every tick regardless.

Only `onTTCBelowWarning` drives this measurement, despite
`onNewObstacle`/`onTrackLost` also being flags in `plannerConfig`. Both were
tried and dropped: validating this found that tracked agent IDs churn
almost every frame even with only 1-2 real agents nearby (sensor position
noise - radar's especially, +/-1.2m - intermittently fails
`sensorFusion`'s/`objectTracking`'s nearest-neighbor gating for the *same*
physical object, minting a fresh ID). That's a real perception-layer
accuracy issue worth fixing separately, but it made ID-based triggers fire
almost every step, which is useless as a signal. TTC is a continuous
numeric value and isn't affected by it.

In practice, measured latency comes back ~0.00s across all five scenarios,
for two honest, non-buggy reasons: (1) there is no artificial computation
delay in this simulation - a real deployed system's actual sensor/compute
pipeline latency would show up here and this one doesn't model it; (2) the
TTC being monitored is the *already-avoiding* selected path's TTC
(`collisionCheck`'s output), which a working planner keeps above the
warning threshold proactively, so the trigger condition itself rarely
fires in scenarios where avoidance is working well (`urbanIntersection`,
`marketArea`, `cattleCrossing` all showed zero trigger events in
validation). This confirms the desired safety property (immediate
response, proactive avoidance) rather than producing a rich statistical
distribution - report it as such, not as a metric with unresolved dynamic
range.

**Phase 10** (post-fusion deduplication):

- `sensorFusion` gained an optional trailing `debugDedup` parameter
  (default false). Every existing 3-argument call site (`main.m` included)
  is unaffected. Needed to give a validation harness visibility into
  per-merge decisions without spamming normal demo output.
- `agent.source`'s value space changed from a fixed enum
  (`camera`/`lidar`/`radar`/`fused`) to a `+`-joined provenance string
  (e.g. `camera+lidar+radar`) - confirmed safe by searching the whole
  codebase for `.source` reads first: nothing branches on its value
  anywhere, it is write-only metadata. See config/createAgent.m's updated
  schema note.
- **Found during the required false-merge validation, not hidden**: the
  dedup rule produced one confirmed false merge in `urbanIntersection`
  (a car and a motorcycle, genuinely different objects, merged for one
  tick at their 3.40m closest approach near the junction) out of 46 merge
  events; zero false merges in 92 events in `marketArea`. The true-duplicate
  motorcycle case in the same run merged at 3.39m - essentially identical
  distance to the false case at 3.40m - so tightening `DEDUP_DISTANCE`
  cannot cleanly separate them; a future pass would need a different
  signal (e.g. mandatory velocity-consistency for any unknown-into-known
  merge, not only when both sides exceed the meaningful-velocity floor).
  Not fixed in this pass - reported as a known, measured, low-frequency
  limitation.

**Phase 7** (steering-rate limiting):

- `vehicleController` gained a `dt` parameter, and `vehicleConfig` gained
  `maxSteerRate`. Measured, not precautionary: validating the Phase 6
  decision layer found `adaptivePlanner`'s selected candidate index can
  still jump substantially between ticks despite its consistency cost (a
  soft cost, not a hard constraint) - e.g. `urbanIntersection` candidate
  15->1 produced a steering command swinging 70 degrees in one 0.1s tick
  (700 deg/s), and 4 of 5 scenarios saturated the absolute +-35 degree
  limit repeatedly. The absolute clamp (still in force) bounds *where*
  steering can be; it did nothing to bound *how fast* it gets there. Now
  rate-limited to 60 deg/s (full lock-to-lock in ~1.2s) against
  `egoState.steering` (the angle actually applied last tick, already
  carried in the schema). Confirmed by measurement: max steering jump
  dropped from 343-700 deg/s to exactly 60 deg/s (the limit) across all
  five scenarios, with no geometric collisions in any of them.

## Decision layer: state set, Stateflow, and known oscillation

Stateflow **is** installed and licensed in this environment (checked with
`license('test','Stateflow')`), but it is not practical for this
architecture: charts execute inside Simulink models, so driving one from
this plain per-tick MATLAB loop would mean a `sim()` call every 0.1s step
(compile/init overhead per call) or restructuring the whole simulation into
Simulink. The brief explicitly permits implementing equivalent MATLAB logic
first and connecting Stateflow later, which is the path taken. The FSM is
`decision/decisionStateMachine.m` + `decision/behaviorSeverity.m`.

Two bugs were found and fixed while validating this layer:

- **Recovery overshoot.** The first version de-escalated `brake` straight
  to a fixed `replan` hop regardless of what `behaviorDecision` proposed.
  That dropped three severity ranks at once, far below the actual hazard
  level, so the next tick immediately re-escalated (escalation being
  immediate by design) - a sustained `brake<->replan` oscillation with a
  ~1.8s period and 0.3s flip-backs whenever a hazard persisted. Fixed by
  relaxing at most one rank per transition and never below the proposal.
- **`wait` on the recovery ladder.** `wait` ranks between `brake` and
  `emergency_stop` but shares `emergency_stop`'s 0.0 speed factor, so
  every recovery from an emergency stop was forced through an extra dwell
  period at a dead stop. On `highwayMerge` that meant sitting stopped
  while an 11 m/s car closed. `wait` is now skipped on the way down and
  only entered when `behaviorDecision` explicitly proposes it.

**Known remaining behavior:** with a persistent borderline hazard the state
can still alternate `brake <-> avoid` on roughly the dwell period (~1.5s),
because leaving a state requires clearing `RECOVERY_MARGIN` but re-entering
it is immediate (escalation is deliberately never delayed). This is a
comfort/smoothness issue, not a safety one - measured clearance improved on
every scenario - and closing it would mean weakening the
escalate-immediately property, which is not a trade worth making silently.

## Schema change: agent.covariance (config/createAgent.m)

Added a `covariance` field (4x4 double) to the `agent` struct for
`objectTracking`'s Kalman filter to carry state uncertainty across frames
between calls (a struct array requires every element to share the same
fields, so this couldn't live only on tracked agents). It defaults to
`zeros(4)` and is meaningless everywhere except inside `objectTracking` -
scenarios, sensor detections, and everything downstream of tracking just
carry the placeholder and never read it.

## Toolbox-swap note

Every function above takes/returns plain structs and matrices, not toolbox
objects. When Automated Driving Toolbox is introduced (e.g. `drivingScenario`
for `mapData`, `multiObjectTracker` inside `objectTracking`, `vehicleDynamics`
blocks under `bicycleModel`), only the *inside* of each stub changes — callers
never need to change because the struct/array contract at each boundary stays
fixed.
